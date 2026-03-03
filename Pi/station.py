#!/usr/bin/env python3
"""
Ground Station Controller — orchestrates a full satellite pass.

Coordinates all three subsystems simultaneously:
  1. Rotator   — tracking via rotctld (hamlib) over TCP
  2. UHF RX    — CC1200 packet capture at 433 MHz via RF HAT UART
  3. VHF TX    — CC1200 uplink at 145 MHz (optional, for commanding)
  4. Doppler   — real-time frequency correction on both radios

Data flow during a pass:

    CelesTrak TLE ─→ PyEphem orbit prediction
                          │
                          ├─→ AZ/EL  ─→ rotctld ─→ Rotator Pico ─→ motors
                          │
                          └─→ range rate ─→ Doppler shift
                                              │
                     ┌────────────────────────┤
                     │                        │
              UHF RX freq adjust       VHF TX freq adjust
              (433 MHz ± Δf)           (145 MHz ± Δf)
                     │                        │
                     └───── RF HAT Pico ──────┘
                              │
                         /dev/serial0
                              │
                     ┌────────┴────────┐
                     │  Packet log     │
                     │  KISS/SiDS out  │
                     └─────────────────┘

Usage:
    python3 station.py                              # auto-pick next pass
    python3 station.py --sat "ISS"                  # track specific satellite
    python3 station.py --list                       # list upcoming passes
    python3 station.py --sat "ISS" --no-rf          # rotator only, no RF HAT
    python3 station.py --sat "ISS" --tx cmd.bin     # also TX a command packet
    python3 station.py --doppler                    # enable Doppler correction
"""

import os
import sys
import time
import math
import json
import signal
import socket
import argparse
import datetime
import urllib.request
from pathlib import Path
from typing import Optional, Tuple

try:
    import ephem
except ImportError:
    print("ERROR: PyEphem required. Install with: pip install ephem")
    sys.exit(1)

sys.path.insert(0, str(Path(__file__).parent))
from rf_hat import (
    CC1200Link, RADIO_UHF, RADIO_VHF,
    STATE_RX, STATE_IDLE,
    EVT_RX_DATA, EVT_ERROR,
    doppler_shift, freq_to_regs,
)
from buzzer import Buzzer

# ---------------------------------------------------------------------------
# Station config (defaults — override with --lat/--lon/--elev or ~/station.conf)
# ---------------------------------------------------------------------------
STATION_CONF = os.path.expanduser("~/station.conf")
STATION_LAT  = "51.1"       # degrees N  (KU Leuven Campus Geel)
STATION_LON  = "4.97"       # degrees E
STATION_ELEV = 25            # meters ASL

ROTCTLD_HOST = "localhost"
ROTCTLD_PORT = 4533

RF_HAT_PORT  = "/dev/serial0"
UHF_FREQ_MHZ = 433.0        # AetherSpace downlink
VHF_FREQ_MHZ = 145.9        # AetherSpace uplink
UHF_PROFILE  = str(Path(__file__).parent / "configs" / "smartrf_uhf_435.txt")
VHF_PROFILE  = str(Path(__file__).parent / "configs" / "smartrf_vhf_145.txt")

UPDATE_HZ     = 10           # rotator update rate
DOPPLER_HZ    = 2            # Doppler update rate (don't thrash CC1200 regs)
MIN_ELEV      = 5.0          # minimum pass elevation
STATUS_FILE   = os.path.expanduser("~/.station_status.json")  # read by dashboard.py
PREDICT_HOURS = 12
METRICS_INTERVAL_S = 10.0

# TLE sources
TLE_URLS = [
    ("https://celestrak.org/NORAD/elements/gp.php?GROUP=stations&FORMAT=TLE", "Space Stations"),
    ("https://celestrak.org/NORAD/elements/gp.php?GROUP=amateur&FORMAT=TLE", "Amateur"),
    ("https://celestrak.org/NORAD/elements/gp.php?GROUP=weather&FORMAT=TLE", "Weather"),
    ("https://celestrak.org/NORAD/elements/gp.php?GROUP=noaa&FORMAT=TLE", "NOAA"),
]


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------
def load_station_conf():
    """Load station location from ~/station.conf if it exists.
    Returns (lat, lon, elev) or (None, None, None) if not found."""
    try:
        if os.path.exists(STATION_CONF):
            with open(STATION_CONF, "r") as f:
                cfg = json.load(f)
            return cfg.get("lat"), cfg.get("lon"), cfg.get("elev")
    except Exception:
        pass
    return None, None, None


def write_dashboard_status(data: dict):
    """Write status JSON for dashboard.py to read."""
    try:
        with open(STATUS_FILE, "w") as f:
            json.dump(data, f)
    except Exception:
        pass


def clear_dashboard_status():
    """Remove status file when not actively tracking."""
    try:
        if os.path.exists(STATUS_FILE):
            os.remove(STATUS_FILE)
    except Exception:
        pass


def ts() -> str:
    return datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]


def log(msg: str, logfile=None):
    line = f"[{ts()}] {msg}"
    print(line)
    if logfile:
        logfile.write(line + "\n")
        logfile.flush()


def fetch_tles():
    """Fetch TLE data from CelesTrak."""
    tles = []
    for url, group in TLE_URLS:
        try:
            req = urllib.request.Request(url, headers={"User-Agent": "SatNOGS-Basestation/1.0"})
            resp = urllib.request.urlopen(req, timeout=10)
            lines = resp.read().decode().strip().split("\n")
            lines = [l.strip() for l in lines if l.strip()]
            for i in range(0, len(lines) - 2, 3):
                name = lines[i].strip()
                l1 = lines[i+1].strip()
                l2 = lines[i+2].strip()
                if l1.startswith("1 ") and l2.startswith("2 "):
                    tles.append((name, l1, l2))
            log(f"  TLE: {len(lines)//3} sats from {group}")
        except Exception as e:
            log(f"  TLE WARNING: failed to fetch {group}: {e}")
    return tles


def make_observer(lat=None, lon=None, elev=None):
    obs = ephem.Observer()
    obs.lat = str(lat if lat is not None else STATION_LAT)
    obs.lon = str(lon if lon is not None else STATION_LON)
    obs.elevation = elev if elev is not None else STATION_ELEV
    obs.horizon = str(MIN_ELEV)
    return obs


def find_passes(tles, hours=12, max_results=20, lat=None, lon=None, elev=None):
    """Find upcoming passes sorted by start time."""
    obs = make_observer(lat, lon, elev)
    now = ephem.now()
    passes = []

    for name, l1, l2 in tles:
        try:
            sat = ephem.readtle(name, l1, l2)
            obs.date = now
            for _ in range(3):
                try:
                    info = obs.next_pass(sat)
                    if info[0] is None or info[4] is None:
                        break
                    rise_t = info[0]
                    max_el = math.degrees(float(info[3]))
                    set_t = info[4]
                    duration = (set_t - rise_t) * 24 * 3600

                    if max_el >= MIN_ELEV and duration > 10:
                        hours_away = (rise_t - now) * 24
                        if hours_away <= hours:
                            passes.append({
                                "name": name,
                                "tle": (name, l1, l2),
                                "rise_time": ephem.Date(rise_t).datetime(),
                                "set_time": ephem.Date(set_t).datetime(),
                                "max_el": max_el,
                                "duration": duration,
                                "rise_az": math.degrees(float(info[1])),
                                "set_az": math.degrees(float(info[5])),
                            })
                    obs.date = info[4] + ephem.minute
                except Exception:
                    break
            obs.date = now
        except Exception:
            continue

    passes.sort(key=lambda p: p["rise_time"])
    return passes[:max_results]


# ---------------------------------------------------------------------------
# Rotctld interface
# ---------------------------------------------------------------------------
class RotctlClient:
    """TCP client for rotctld (hamlib)."""

    def __init__(self, host: str = "localhost", port: int = 4533):
        self.host = host
        self.port = port
        self.sock: Optional[socket.socket] = None

    def connect(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.settimeout(5)
        self.sock.connect((self.host, self.port))

    def close(self):
        if self.sock:
            try:
                self.sock.close()
            except Exception:
                pass
            self.sock = None

    def _cmd(self, cmd: str) -> str:
        if not self.sock:
            return ""
        self.sock.sendall((cmd + "\n").encode())
        time.sleep(0.05)
        try:
            return self.sock.recv(1024).decode().strip()
        except socket.timeout:
            return ""

    def set_position(self, az: float, el: float):
        az = az % 360
        el = max(0, min(90, el))
        self._cmd(f"P {az:.1f} {el:.1f}")

    def get_position(self) -> Tuple[Optional[float], Optional[float]]:
        resp = self._cmd("p")
        lines = resp.split("\n")
        if len(lines) >= 2:
            try:
                return float(lines[0]), float(lines[1])
            except ValueError:
                pass
        return None, None

    def park(self):
        self.set_position(0.0, 0.0)


# ---------------------------------------------------------------------------
# RF HAT manager
# ---------------------------------------------------------------------------
class RfHatManager:
    """Manages CC1200 UHF RX and optional VHF TX via the RF HAT Pico."""

    def __init__(self, port: str = "/dev/serial0"):
        self.port = port
        self.link = CC1200Link()
        self.uhf_freq_hz = 0.0
        self.vhf_freq_hz = 0.0
        self.pkt_count = 0
        self.total_bytes = 0
        self.packets = []  # list of (timestamp, data_bytes)

    def open(self) -> bool:
        try:
            self.link.open(self.port)
        except Exception as e:
            log(f"RF HAT: Cannot open {self.port}: {e}")
            return False

        time.sleep(0.1)
        pong = self.link.ping()
        if pong is None:
            log("RF HAT: No PING response")
            self.link.close()
            return False
        log(f"RF HAT: PING OK")

        info = self.link.get_info()
        if info:
            log(f"RF HAT: radios={info.count} part=0x{info.part:02X} ver=0x{info.ver:02X}")
        return True

    def close(self):
        try:
            self.link.set_streaming(False)
            self.link.set_state(STATE_IDLE)
        except Exception:
            pass
        self.link.close()

    def setup_uhf_rx(self, freq_mhz: float, profile_path: str) -> bool:
        """Configure UHF radio for RX at the given frequency."""
        if not self.link.select_radio(RADIO_UHF):
            log("RF HAT: Failed to select UHF radio")
            return False
        log("RF HAT: Selected UHF radio")

        log(f"RF HAT: Loading UHF profile: {profile_path}")
        ok, msg = self.link.apply_smartrf_config(profile_path)
        if not ok:
            log(f"RF HAT: UHF profile failed: {msg}")
            return False
        log(f"RF HAT: {msg}")

        self.uhf_freq_hz = freq_mhz * 1e6
        if not self.link.set_frequency(self.uhf_freq_hz):
            log("RF HAT: Failed to set UHF frequency")
            return False

        actual = self.link.get_frequency()
        if actual:
            log(f"RF HAT: UHF freq = {actual/1e6:.6f} MHz")

        return True

    def start_rx(self) -> bool:
        """Enter RX mode and enable streaming on the currently selected radio."""
        # Make sure UHF is selected
        self.link.select_radio(RADIO_UHF)
        if not self.link.set_state(STATE_RX):
            log("RF HAT: Failed to enter RX")
            return False
        if not self.link.set_streaming(True):
            log("RF HAT: Failed to enable streaming")
            return False
        log("RF HAT: UHF RX streaming enabled")
        return True

    def stop_rx(self):
        self.link.select_radio(RADIO_UHF)
        self.link.set_streaming(False)
        self.link.set_state(STATE_IDLE)
        log("RF HAT: UHF RX stopped")

    def update_uhf_doppler(self, freq_hz: float) -> bool:
        """Update UHF RX frequency for Doppler correction."""
        if abs(freq_hz - self.uhf_freq_hz) < 50:  # skip if < 50 Hz change
            return True
        self.link.select_radio(RADIO_UHF)
        ok = self.link.set_frequency(freq_hz)
        if ok:
            self.uhf_freq_hz = freq_hz
        return ok

    def poll_packets(self, logfile=None) -> int:
        """Process pending RX events. Returns number of new packets."""
        count = 0
        for evt_type, body in self.link.pop_events():
            if evt_type == EVT_RX_DATA and body:
                n = body[0]
                data = body[1:1 + n]
                self.pkt_count += 1
                self.total_bytes += n
                count += 1
                hex_str = data.hex(" ")
                self.packets.append((datetime.datetime.now(), data))
                log(f"RX [{self.pkt_count}] {n} bytes: {hex_str}", logfile)
            elif evt_type == EVT_ERROR and body:
                code = body[0]
                log(f"RF HAT: EVT_ERROR 0x{code:02X}", logfile)
        return count

    def get_metrics_str(self) -> str:
        self.link.select_radio(RADIO_UHF)
        m = self.link.get_metrics()
        if not m:
            return "no metrics"
        rssi = f"{m.rssi_dbm:.1f} dBm" if m.rssi_dbm is not None else "N/A"
        return f"MARC=0x{m.marc:02X} RXBYTES={m.rxbytes} RSSI={rssi}"

    def tx_vhf_packet(self, data: bytes, freq_mhz: float,
                      profile_path: str) -> bool:
        """Switch to VHF, transmit a packet, switch back to UHF RX."""
        # Stop UHF streaming first
        self.link.select_radio(RADIO_UHF)
        self.link.set_streaming(False)
        self.link.set_state(STATE_IDLE)

        # Configure VHF
        if not self.link.select_radio(RADIO_VHF):
            log("RF HAT: Failed to select VHF radio")
            return False

        ok, msg = self.link.apply_smartrf_config(profile_path)
        if not ok:
            log(f"RF HAT: VHF profile failed: {msg}")
            return False

        vhf_hz = freq_mhz * 1e6
        self.link.set_frequency(vhf_hz)

        # Transmit
        ok = self.link.tx_packet(data)
        if ok:
            log(f"RF HAT: VHF TX OK — {len(data)} bytes at {freq_mhz:.3f} MHz")
        else:
            log("RF HAT: VHF TX FAILED")

        # Return to UHF RX
        self.link.select_radio(RADIO_UHF)
        self.link.set_state(STATE_RX)
        self.link.set_streaming(True)
        return ok


# ---------------------------------------------------------------------------
# Pass tracker
# ---------------------------------------------------------------------------
def track_pass(rotctl: RotctlClient, rf: Optional[RfHatManager],
               tle, pass_info, args, logfile=None, buzzer: Optional[Buzzer] = None,
               lat=None, lon=None, elev=None):
    """Execute a full satellite pass with tracking + RF."""
    name, l1, l2 = tle
    sat = ephem.readtle(name, l1, l2)
    obs = make_observer(lat, lon, elev)
    obs.horizon = "0"

    log(f"========== PASS: {name} ==========", logfile)
    log(f"  Max EL: {pass_info['max_el']:.1f} deg", logfile)
    log(f"  Duration: {pass_info['duration']:.0f}s", logfile)
    log(f"  Rise AZ: {pass_info['rise_az']:.1f} → Set AZ: {pass_info['set_az']:.1f}", logfile)
    if rf:
        log(f"  UHF RX: {args.uhf_freq:.3f} MHz", logfile)
        if args.doppler:
            log(f"  Doppler correction: ENABLED", logfile)

    # Pre-position rotator
    rise_az = pass_info["rise_az"]
    log(f"Slewing to AOS: AZ {rise_az:.1f} EL 0.0", logfile)
    rotctl.set_position(rise_az, 0)

    # Wait for AOS
    now_utc = datetime.datetime.now(datetime.timezone.utc)
    rise_utc = pass_info["rise_time"].replace(tzinfo=datetime.timezone.utc)
    wait = (rise_utc - now_utc).total_seconds()
    if wait > 0:
        log(f"Waiting {wait:.0f}s for AOS...", logfile)
        while wait > 0.5:
            time.sleep(min(wait, 1.0))
            now_utc = datetime.datetime.now(datetime.timezone.utc)
            wait = (rise_utc - now_utc).total_seconds()
            if not running:
                return

    log("** AOS — pass started **", logfile)
    if buzzer:
        buzzer.beep_aos()

    # Start RX if RF is available
    if rf:
        rf.start_rx()
        if rf:
            rf.pkt_count = 0
            rf.total_bytes = 0

    set_utc = pass_info["set_time"].replace(tzinfo=datetime.timezone.utc)
    dt = 1.0 / UPDATE_HZ
    doppler_dt = 1.0 / DOPPLER_HZ
    last_doppler = 0.0
    last_metrics = 0.0
    tick = 0
    c = 299_792_458.0  # speed of light

    try:
        while running:
            now_utc = datetime.datetime.now(datetime.timezone.utc)
            if now_utc >= set_utc:
                break

            # Compute satellite position
            obs.date = ephem.Date(now_utc)
            sat.compute(obs)
            az = math.degrees(float(sat.az))
            el = math.degrees(float(sat.alt))

            # Send to rotator
            if el >= 0:
                rotctl.set_position(az, el)

            # Doppler correction
            now_mono = time.time()
            if rf and args.doppler and (now_mono - last_doppler) >= doppler_dt:
                last_doppler = now_mono
                # range_rate is velocity in m/s (positive = receding)
                range_rate = float(sat.range_velocity)
                uhf_doppler = doppler_shift(args.uhf_freq * 1e6, range_rate)
                rf.update_uhf_doppler(uhf_doppler)

            # Poll RF packets
            if rf:
                new_pkts = rf.poll_packets(logfile)
                if new_pkts > 0 and buzzer:
                    buzzer.beep_packet()

            # Periodic metrics
            if rf and (now_mono - last_metrics) >= METRICS_INTERVAL_S:
                last_metrics = now_mono
                remaining = (set_utc - now_utc).total_seconds()
                mstr = rf.get_metrics_str()
                log(f"METRICS: {mstr} | pkts={rf.pkt_count} "
                    f"bytes={rf.total_bytes} T-{remaining:.0f}s", logfile)

            # Status line
            remaining = (set_utc - now_utc).total_seconds()
            total_dur = pass_info["duration"]
            progress = max(0, min(100, (1 - remaining / total_dur) * 100)) if total_dur > 0 else 0
            bar_el = int(max(0, el) / 90 * 20)
            bar = "|" + "#" * bar_el + " " * (20 - bar_el) + "|"
            rf_info = f"pkts={rf.pkt_count}" if rf else "no-rf"
            print(f"\r  AZ{az:>7.1f} EL{el:>5.1f} {bar} {rf_info:<12s} T-{remaining:>5.0f}s",
                  end="", flush=True)

            # Update dashboard status file (~1 Hz, not every tick)
            if tick % UPDATE_HZ == 0:
                rssi = None
                if rf:
                    m = rf.link.get_metrics()
                    if m:
                        rssi = m.rssi_dbm
                write_dashboard_status({
                    "streaming": rf is not None,
                    "rssi": rssi,
                    "packets": rf.pkt_count if rf else 0,
                    "freq_mhz": args.uhf_freq,
                    "satellite": name,
                    "pass_progress": round(progress, 1),
                })

            tick += 1
            next_tick = time.time() + dt
            sleep_time = next_tick - time.time()
            if sleep_time > 0:
                time.sleep(sleep_time)

    finally:
        print()
        log("** LOS — pass complete **", logfile)
        if buzzer:
            buzzer.beep_los()
        if rf:
            rf.stop_rx()
            log(f"Pass summary: {rf.pkt_count} packets, {rf.total_bytes} bytes", logfile)
        clear_dashboard_status()

    return rf.pkt_count if rf else 0


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
running = True

def on_signal(sig, frame):
    global running
    running = False

signal.signal(signal.SIGINT, on_signal)
signal.signal(signal.SIGTERM, on_signal)


def _run_daemon(args, logfile):
    """Daemon mode: wait for location, loop passes continuously.

    Designed to run as a systemd service on the Pi. Workflow:
      1. Wait for ~/station.conf (set from phone GPS via dashboard)
      2. Connect to rotctld
      3. Fetch TLEs, find upcoming passes
      4. Track all passes in sequence
      5. Sleep, then repeat from step 2
    """
    log("==========================================", logfile)
    log(" SatNOGS Station — DAEMON MODE", logfile)
    log("==========================================", logfile)

    # Wait for station.conf (set via phone GPS on dashboard)
    log("Waiting for station location (set GPS via dashboard)...", logfile)
    while running:
        conf_lat, conf_lon, conf_elev = load_station_conf()
        if conf_lat is not None:
            break
        time.sleep(5)

    if not running:
        return

    sta_lat = conf_lat
    sta_lon = conf_lon
    sta_elev = conf_elev or STATION_ELEV
    log(f"Location set: {sta_lat}N, {sta_lon}E, {sta_elev:.0f}m", logfile)

    buzzer = Buzzer()  # no link yet — beeps are no-ops until RF opens

    try:
        while running:
            # Re-read location (user may update GPS mid-session)
            new_lat, new_lon, new_elev = load_station_conf()
            if new_lat is not None:
                sta_lat, sta_lon = new_lat, new_lon
                sta_elev = new_elev or sta_elev

            # Connect to rotctld
            rotctl = RotctlClient(args.rot_host, args.rot_port)
            try:
                rotctl.connect()
                log(f"Rotctld connected", logfile)
            except Exception as e:
                log(f"Cannot connect to rotctld: {e} — retrying in 30s", logfile)
                time.sleep(30)
                continue

            # Open RF HAT
            rf = None
            if not args.no_rf:
                rf = RfHatManager(args.rf_port)
                if not rf.open():
                    log("RF HAT failed — continuing without RF", logfile)
                    rf = None
                elif not rf.setup_uhf_rx(args.uhf_freq, args.uhf_profile):
                    log("UHF setup failed — continuing without RF", logfile)
                    rf.close()
                    rf = None

            # Update buzzer link (buzzer lives on the RF HAT Pico)
            buzzer.link = rf.link if rf else None
            buzzer.beep_ready()

            # Fetch TLEs
            log("Fetching TLE data...", logfile)
            tles = fetch_tles()
            if not tles:
                log("No TLE data — retrying in 60s", logfile)
                rotctl.close()
                if rf:
                    rf.close()
                time.sleep(60)
                continue

            # Filter by satellite name if specified
            if args.sat:
                search = args.sat.upper()
                tles_to_search = [(n, l1, l2) for n, l1, l2 in tles if search in n.upper()]
            else:
                tles_to_search = tles

            # Find passes
            passes = find_passes(tles_to_search, hours=PREDICT_HOURS,
                                 lat=sta_lat, lon=sta_lon, elev=sta_elev)

            if passes:
                log(f"Found {len(passes)} passes", logfile)
                for i, p in enumerate(passes):
                    if not running:
                        break
                    log(f"Queued: {p['name']} max EL {p['max_el']:.1f}° in "
                        f"{(p['rise_time'].replace(tzinfo=datetime.timezone.utc) - datetime.datetime.now(datetime.timezone.utc)).total_seconds()/60:.0f} min",
                        logfile)
                    track_pass(rotctl, rf, p["tle"], p, args, logfile,
                               buzzer=buzzer,
                               lat=sta_lat, lon=sta_lon, elev=sta_elev)
                    if i < len(passes) - 1 and running:
                        rotctl.park()
            else:
                log("No passes found in prediction window", logfile)

            # Cleanup this cycle
            rotctl.park()
            rotctl.close()
            if rf:
                rf.close()
            clear_dashboard_status()

            if not running:
                break

            log(f"Sleeping {args.loop_delay}s before next cycle...", logfile)
            for _ in range(args.loop_delay):
                if not running:
                    break
                time.sleep(1)

    finally:
        clear_dashboard_status()
        buzzer.cleanup()
        if logfile:
            logfile.close()
        log("Daemon stopped.")


def main():
    parser = argparse.ArgumentParser(
        description="SatNOGS Ground Station Controller — rotator + RF HAT")
    parser.add_argument("--sat", default=None,
                        help="Satellite name to track (partial match)")
    parser.add_argument("--list", action="store_true",
                        help="List upcoming passes and exit")

    # Rotator
    parser.add_argument("--rot-host", default=ROTCTLD_HOST,
                        help="rotctld host (default: localhost)")
    parser.add_argument("--rot-port", type=int, default=ROTCTLD_PORT,
                        help="rotctld port (default: 4533)")

    # RF HAT
    parser.add_argument("--rf-port", default=RF_HAT_PORT,
                        help="RF HAT serial port (default: /dev/serial0)")
    parser.add_argument("--no-rf", action="store_true",
                        help="Disable RF HAT (rotator tracking only)")
    parser.add_argument("--uhf-freq", type=float, default=UHF_FREQ_MHZ,
                        help="UHF RX frequency in MHz (default: 433.0)")
    parser.add_argument("--uhf-profile", default=UHF_PROFILE,
                        help="UHF SmartRF profile path")
    parser.add_argument("--doppler", action="store_true",
                        help="Enable real-time Doppler correction")

    # TX
    parser.add_argument("--tx", default=None,
                        help="TX a binary file on VHF during the pass")
    parser.add_argument("--vhf-freq", type=float, default=VHF_FREQ_MHZ,
                        help="VHF TX frequency in MHz (default: 145.9)")
    parser.add_argument("--vhf-profile", default=VHF_PROFILE,
                        help="VHF SmartRF profile path")

    # Station location
    parser.add_argument("--lat", type=float, default=None,
                        help="Station latitude in degrees N (default: from station.conf or 51.1)")
    parser.add_argument("--lon", type=float, default=None,
                        help="Station longitude in degrees E (default: from station.conf or 4.97)")
    parser.add_argument("--elev", type=float, default=None,
                        help="Station elevation in meters ASL (default: from station.conf or 25)")

    # Daemon mode
    parser.add_argument("--daemon", action="store_true",
                        help="Daemon mode: wait for station.conf, loop passes continuously")
    parser.add_argument("--loop-delay", type=int, default=300,
                        help="Seconds between TLE refresh cycles in daemon mode (default: 300)")

    # Output
    parser.add_argument("--log", default=None,
                        help="Log file path (appended)")

    args = parser.parse_args()

    logfile = None
    if args.log:
        logfile = open(args.log, "a", encoding="utf-8")

    # Daemon mode: wait for station.conf, then loop passes continuously
    if args.daemon:
        _run_daemon(args, logfile)
        return

    # Resolve station location: CLI > station.conf > hardcoded defaults
    conf_lat, conf_lon, conf_elev = load_station_conf()
    sta_lat = args.lat or conf_lat or float(STATION_LAT)
    sta_lon = args.lon or conf_lon or float(STATION_LON)
    sta_elev = args.elev or conf_elev or STATION_ELEV

    log("==========================================", logfile)
    log(" SatNOGS Ground Station Controller", logfile)
    log("==========================================", logfile)
    log(f"Station: {sta_lat}N, {sta_lon}E, {sta_elev:.0f}m", logfile)
    log(f"Rotator: {args.rot_host}:{args.rot_port}", logfile)
    if not args.no_rf:
        log(f"RF HAT:  {args.rf_port}", logfile)
        log(f"UHF RX:  {args.uhf_freq:.3f} MHz", logfile)
        log(f"Doppler: {'ON' if args.doppler else 'OFF'}", logfile)

    # Fetch TLEs
    log("Fetching TLE data...", logfile)
    tles = fetch_tles()
    if not tles:
        log("FATAL: No TLE data!", logfile)
        sys.exit(1)
    log(f"Loaded {len(tles)} satellites", logfile)

    # Filter by name
    if args.sat:
        search = args.sat.upper()
        filtered = [(n, l1, l2) for n, l1, l2 in tles if search in n.upper()]
        if not filtered:
            log(f"No satellite matching '{args.sat}'", logfile)
            sys.exit(1)
        tles_to_search = filtered
        log(f"Filtered to {len(filtered)} satellite(s)", logfile)
    else:
        tles_to_search = tles

    # Find passes
    log(f"Computing passes for next {PREDICT_HOURS}h...", logfile)
    passes = find_passes(tles_to_search, hours=PREDICT_HOURS,
                         lat=sta_lat, lon=sta_lon, elev=sta_elev)

    if not passes:
        log("No passes found in prediction window.", logfile)
        sys.exit(0)

    # List mode
    if args.list:
        print()
        fmt = f"  {'#':>3s}  {'Satellite':<24s}  {'Rise (UTC)':>12s}  {'Max EL':>7s}  {'Dur':>5s}  {'AZ range'}"
        print(fmt)
        print(f"  {'---':>3s}  {'------------------------':<24s}  {'----------':>12s}  {'------':>7s}  {'-----':>5s}  {'--------'}")
        for i, p in enumerate(passes):
            rise_str = p["rise_time"].strftime("%H:%M:%S")
            az_range = f"{p['rise_az']:.0f}->{p['set_az']:.0f}"
            print(f"  {i+1:>3d}  {p['name']:<24s}  {rise_str:>12s}  {p['max_el']:>6.1f}°  {p['duration']:>4.0f}s  {az_range}")
        print()
        sys.exit(0)

    # Connect to rotctld
    log(f"Connecting to rotctld...", logfile)
    rotctl = RotctlClient(args.rot_host, args.rot_port)
    try:
        rotctl.connect()
        cur_az, cur_el = rotctl.get_position()
        log(f"Rotctld connected — AZ={cur_az} EL={cur_el}", logfile)
    except Exception as e:
        log(f"FATAL: Cannot connect to rotctld: {e}", logfile)
        sys.exit(1)

    # Open RF HAT
    rf: Optional[RfHatManager] = None
    if not args.no_rf:
        log("Opening RF HAT...", logfile)
        rf = RfHatManager(args.rf_port)
        if not rf.open():
            log("WARNING: RF HAT failed to open — continuing without RF", logfile)
            rf = None
        else:
            if not rf.setup_uhf_rx(args.uhf_freq, args.uhf_profile):
                log("WARNING: UHF setup failed — continuing without RF", logfile)
                rf.close()
                rf = None

    # Load TX data if specified
    tx_data = None
    if args.tx:
        try:
            with open(args.tx, "rb") as f:
                tx_data = f.read()
            log(f"TX payload loaded: {len(tx_data)} bytes from {args.tx}", logfile)
        except Exception as e:
            log(f"WARNING: Failed to load TX file: {e}", logfile)

    # Initialize buzzer (uses RF HAT UART; no-op if rf is None)
    buzzer = Buzzer(rf.link if rf else None)
    buzzer.beep_ready()

    # Track passes
    try:
        for i, p in enumerate(passes):
            if not running:
                break

            now_utc = datetime.datetime.now(datetime.timezone.utc)
            rise_utc = p["rise_time"].replace(tzinfo=datetime.timezone.utc)
            wait = (rise_utc - now_utc).total_seconds()

            log(f"Next: {p['name']} — max EL {p['max_el']:.1f}°, "
                f"{p['duration']:.0f}s, in {max(0,wait)/60:.1f} min", logfile)

            pkt_count = track_pass(rotctl, rf, p["tle"], p, args, logfile,
                                   buzzer=buzzer,
                                   lat=sta_lat, lon=sta_lon, elev=sta_elev)

            # TX on VHF after pass if requested
            if tx_data and rf and running:
                log("Transmitting VHF uplink packet...", logfile)
                rf.tx_vhf_packet(tx_data, args.vhf_freq, args.vhf_profile)

            # Park between passes
            if i < len(passes) - 1 and running:
                log("Parking rotator...", logfile)
                rotctl.park()

    finally:
        log("Shutting down...", logfile)
        clear_dashboard_status()
        rotctl.park()
        rotctl.close()
        if rf:
            rf.close()
        buzzer.cleanup()
        if logfile:
            logfile.close()
        log("Done.")


if __name__ == "__main__":
    main()
