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
    python3 station.py --no-satnogs                 # disable SatNOGS DB submission
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
import tempfile
import urllib.request
from pathlib import Path
from typing import Optional, Tuple, List

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
    EXT_FREQOFF_EST1, EXT_FREQOFF_EST0, EXT_LQI_VAL,
    doppler_shift, freq_to_regs,
)
from rf_backend import RfBackend, RfMetrics
from buzzer import Buzzer
from satnogs import SatNOGSClient
from sat_library import SatLibrary, SatProfile, ModulationConfig, pick_best_uhf_transmitter


def lookup_with_satnogs_fallback(sat_library: SatLibrary, norad_id: int,
                                  name: str) -> Optional[SatProfile]:
    """Look up satellite profile locally, then fall back to SatNOGS DB API.

    If the local profile DB has no entry, queries db.satnogs.org for the
    satellite's transmitter info and builds a SatProfile on the fly.
    """
    profile = sat_library.lookup(norad_id=norad_id, name=name)
    if profile:
        return profile

    # Local lookup failed — try SatNOGS DB API
    if not norad_id:
        return None

    try:
        txs = sat_library.fetch_satnogs_transmitters(norad_id)
    except Exception:
        return None

    if not txs:
        return None

    # Pick the best CC1200-compatible UHF transmitter (closest to 435 MHz)
    tx = pick_best_uhf_transmitter(txs)
    if not tx:
        return None
    mod = None
    baud = tx.get("baud")
    cc_fmt = tx.get("cc1200_format", "2-GFSK")
    if baud and baud > 0:
        mode_str = tx.get("mode", "").upper()
        if mode_str in ("GMSK", "MSK", "MSK AX.100 MODE 5", "MSK AX.100 MODE 6"):
            deviation = int(baud / 4)  # h=0.5
        else:
            deviation = int(baud * 0.35)  # h~0.7 typical amateur FSK
        rx_bw = max(12.5, (baud + 2 * deviation) * 1.2 / 1000)

        # Infer sync word and protocol from mode when possible
        sync_word = None
        protocol = ""
        use_serial = False
        if "AX.100" in mode_str or "AX100" in mode_str:
            sync_word = "930B51DE"  # GOMspace AX.100 Mode 5/6
            protocol = "ax100_mode5"
        elif "G3RUH" in mode_str or "AX.25" in mode_str or "AX25" in mode_str:
            protocol = "ax25_g3ruh"
            # Serial mode disabled — PIO outputs zeros, breaks packet reception
            # TODO: re-enable when serial mode is verified with real signal
            use_serial = False
        elif "USP" in mode_str:
            protocol = "usp"
            use_serial = False  # disabled until PIO verified

        mod = ModulationConfig(
            format=cc_fmt,
            symbol_rate_bps=int(baud),
            deviation_hz=deviation,
            rx_bw_khz=round(rx_bw, 1),
            sync_word=sync_word,
        )

    return SatProfile(
        freq_mhz=tx["freq_mhz"],
        modulation=mod,
        description=f"SatNOGS DB: {tx.get('description', tx.get('mode', ''))}",
        protocol=protocol,
        use_serial_mode=use_serial,
    )


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
HISTORY_FILE  = os.path.expanduser("~/.station_pass_history.json")  # recent pass log
PREDICT_HOURS = 12
METRICS_INTERVAL_S = 1.0

# TLE sources
TLE_URLS = [
    # Primary: AMSAT (reliable, no rate limiting)
    ("https://www.amsat.org/tle/current/nasabare.txt", "AMSAT"),
    # Fallback: CelesTrak (may 403 under rate limiting)
    ("https://celestrak.org/NORAD/elements/gp.php?GROUP=stations&FORMAT=TLE", "Space Stations"),
    ("https://celestrak.org/NORAD/elements/gp.php?GROUP=amateur&FORMAT=TLE", "Amateur"),
    ("https://celestrak.org/NORAD/elements/gp.php?GROUP=weather&FORMAT=TLE", "Weather"),
    ("https://celestrak.org/NORAD/elements/gp.php?GROUP=noaa&FORMAT=TLE", "NOAA"),
]

def _load_extra_norad_ids():
    """Extract NORAD IDs from sat_profiles.json for individual TLE fetch."""
    try:
        p = Path(__file__).parent / "sat_profiles.json"
        with open(p, "r") as f:
            data = json.load(f)
        return [s["norad_id"] for s in data.get("satellites", [])
                if isinstance(s.get("norad_id"), int)]
    except Exception:
        return []


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


def norad_id_from_tle(l1: str) -> int:
    """Extract the NORAD catalogue number from TLE line 1.
    Returns 0 if parsing fails."""
    try:
        return int(l1[2:7].strip())
    except Exception:
        return 0


def write_dashboard_status(data: dict):
    """Write status JSON atomically for dashboard.py to read."""
    try:
        dir_name = os.path.dirname(STATUS_FILE)
        fd, tmp_path = tempfile.mkstemp(dir=dir_name, suffix=".tmp")
        try:
            with os.fdopen(fd, "w") as f:
                json.dump(data, f)
            os.rename(tmp_path, STATUS_FILE)
        except Exception:
            # Clean up temp file on failure
            try:
                os.unlink(tmp_path)
            except OSError:
                pass
            raise
    except Exception:
        pass


def clear_dashboard_status():
    """Remove status file when not actively tracking."""
    try:
        if os.path.exists(STATUS_FILE):
            os.remove(STATUS_FILE)
    except Exception:
        pass


def record_pass_history(name, pass_info, packets=0, total_bytes=0, peak_rssi=None):
    """Append a completed pass to the history file (keeps last 20).
    Deduplicates by name + max_el within 60s to prevent scan mode from
    recording the same visit multiple times."""
    try:
        history = []
        if os.path.exists(HISTORY_FILE):
            with open(HISTORY_FILE, "r") as f:
                history = json.load(f)
        now_str = datetime.datetime.utcnow().strftime("%Y-%m-%dT%H:%M:%SZ")
        max_el = round(pass_info.get("max_el", 0), 1)
        # Dedup: skip if same satellite + similar max_el recorded in last 60s
        if history:
            last = history[-1]
            if (last.get("name") == name and
                abs(last.get("max_el", 0) - max_el) < 1.0):
                try:
                    last_t = datetime.datetime.strptime(last["time"], "%Y-%m-%dT%H:%M:%SZ")
                    now_t = datetime.datetime.strptime(now_str, "%Y-%m-%dT%H:%M:%SZ")
                    if (now_t - last_t).total_seconds() < 60:
                        return  # duplicate — skip
                except Exception:
                    pass
        entry = {
            "name": name,
            "time": now_str,
            "duration": round(pass_info.get("duration", 0)),
            "max_el": max_el,
            "packets": packets,
            "bytes": total_bytes,
            "peak_rssi": peak_rssi,
        }
        history.append(entry)
        history = history[-20:]  # keep last 20
        with open(HISTORY_FILE, "w") as f:
            json.dump(history, f)
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


_tle_cache_station = []
_tle_cache_station_time = 0.0

def fetch_tles():
    """Fetch TLEs. Uses dashboard's cache first, then AMSAT as fallback. Cached 30 min."""
    global _tle_cache_station, _tle_cache_station_time
    now = time.time()
    if _tle_cache_station and (now - _tle_cache_station_time) < 1800:
        return _tle_cache_station

    tles = []
    seen = set()

    # Primary: dashboard's cached TLE file (shared data source, no extra API calls)
    tle_file = os.path.expanduser("~/.station_tles.json")
    try:
        if os.path.exists(tle_file):
            mtime = os.path.getmtime(tle_file)
            if now - mtime < 7200:  # use if less than 2 hours old
                with open(tle_file, "r") as f:
                    data = json.load(f)
                for name, lines in data.items():
                    if len(lines) == 2 and lines[0].startswith("1 ") and lines[1].startswith("2 "):
                        tles.append((name, lines[0], lines[1]))
                        seen.add(name)
                log(f"  TLE: {len(tles)} sats from dashboard cache")
                del data
    except Exception as e:
        log(f"  TLE: dashboard cache failed: {e}")

    # Only use AMSAT + CelesTrak for station.py (lightweight, ~80 sats)
    # Dashboard.py handles the full SatNOGS DB catalog separately
    for url, group in TLE_URLS:
        try:
            req = urllib.request.Request(url, headers={"User-Agent": "SatNOGS-Basestation/1.0"})
            resp = urllib.request.urlopen(req, timeout=10)
            lines = resp.read().decode().strip().split("\n")
            lines = [l.strip() for l in lines if l.strip()]
            added = 0
            for i in range(0, len(lines) - 2, 3):
                name = lines[i].strip()
                l1 = lines[i+1].strip()
                l2 = lines[i+2].strip()
                if l1.startswith("1 ") and l2.startswith("2 ") and name not in seen:
                    tles.append((name, l1, l2))
                    seen.add(name)
                    added += 1
            if added:
                log(f"  TLE: +{added} sats from {group}")
        except Exception as e:
            log(f"  TLE WARNING: failed to fetch {group}: {e}")
    # Fetch individual satellites by NORAD ID (MEO, non-standard orbits)
    existing_norads = set()
    for _, l1, _ in tles:
        try:
            existing_norads.add(int(l1[2:7].strip()))
        except Exception:
            pass
    missing = [n for n in _load_extra_norad_ids() if n not in existing_norads]
    if missing:
        ids_str = ",".join(str(n) for n in missing)
        try:
            url = f"https://celestrak.org/NORAD/elements/gp.php?CATNR={ids_str}&FORMAT=TLE"
            req = urllib.request.Request(url, headers={"User-Agent": "SatNOGS-Basestation/1.0"})
            resp = urllib.request.urlopen(req, timeout=10)
            lines = resp.read().decode().strip().split("\n")
            lines = [l.strip() for l in lines if l.strip()]
            added = 0
            for i in range(0, len(lines) - 2, 3):
                name = lines[i].strip()
                l1 = lines[i+1].strip()
                l2 = lines[i+2].strip()
                if l1.startswith("1 ") and l2.startswith("2 ") and name not in seen:
                    tles.append((name, l1, l2))
                    seen.add(name)
                    added += 1
            if added:
                log(f"  TLE: +{added} sats from individual NORAD IDs")
        except Exception as e:
            log(f"  TLE WARNING: individual NORAD fetch failed: {e}")

    if tles:
        _tle_cache_station = tles
        _tle_cache_station_time = time.time()
        # Save to disk so next startup is instant
        try:
            tle_file = os.path.expanduser("~/.station_tles.json")
            data = {name: [l1, l2] for name, l1, l2 in tles}
            with open(tle_file, "w") as f:
                json.dump(data, f)
        except Exception:
            pass
    return tles


def make_observer(lat=None, lon=None, elev=None):
    obs = ephem.Observer()
    obs.lat = str(lat if lat is not None else STATION_LAT)
    obs.lon = str(lon if lon is not None else STATION_LON)
    obs.elevation = elev if elev is not None else STATION_ELEV
    obs.horizon = str(MIN_ELEV)
    return obs


def find_passes(tles, hours=12, max_results=20, lat=None, lon=None, elev=None):
    """Find upcoming passes sorted by start time. Includes currently overhead passes."""
    obs = make_observer(lat, lon, elev)
    now = ephem.now()
    # Start search 20 min in the past to catch passes already overhead
    search_start = now - 20.0 / (24 * 60)
    passes = []

    for name, l1, l2 in tles:
        try:
            sat = ephem.readtle(name, l1, l2)
            obs.date = search_start
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
                        # Skip passes that already ended
                        if set_t < now:
                            obs.date = set_t + ephem.minute
                            continue
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
        self.close()  # clean up any previous socket before reconnecting
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.settimeout(0.5)
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
class RfHatManager(RfBackend):
    """Manages CC1200 UHF RX and optional VHF TX via the RF HAT Pico."""

    def __init__(self, port: str = "/dev/serial0"):
        self.port = port
        self.link = CC1200Link()
        self.uhf_freq_hz = 0.0
        self.uhf_base_freq_hz = 0.0  # satellite's nominal freq (before Doppler)
        self.vhf_freq_hz = 0.0
        self.pkt_count = 0
        self.total_bytes = 0
        self.packets: List[Tuple] = []  # list of (timestamp, data_bytes)
        self.radio_config = {
            "freq_mhz": 0.0,
            "modulation": "2-GFSK",
            "symbol_rate_bps": 2400,
            "profile": "default",
        }
        # Raw data dump
        self._raw_file = None
        self._raw_bytes = 0
        self._use_serial_mode = False
        # RSSI tracking for pass statistics
        self.rssi_min: Optional[float] = None
        self.rssi_max: Optional[float] = None
        self.rssi_sum: float = 0.0
        self.rssi_count: int = 0
        self._prev_rx_total: int = 0
        self._prev_rx_time: float = 0.0
        self._rx_rate_bps: float = 0.0

    def name(self) -> str:
        return "CC1200"

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

    def set_frequency(self, freq_hz: float) -> bool:
        return self.update_uhf_doppler(freq_hz)

    def get_metrics(self) -> RfMetrics:
        # Don't call select_radio() here — it resets streaming and desired_state
        # in the firmware. UHF is already selected during tracking.
        m = self.link.get_metrics()
        rssi = m.rssi_dbm if m else None
        return RfMetrics(
            signal_dbm=rssi,
            signal_label="RSSI",
            signal_unit="dBm",
            freq_hz=self.uhf_freq_hz,
            streaming=True,  # only called while streaming
            backend_name="CC1200",
            packets=self.pkt_count,
            total_bytes=self.total_bytes,
        )

    def get_status_dict(self) -> dict:
        m = self.link.get_metrics()
        rssi = m.rssi_dbm if m else None

        # Track RSSI min/max/avg over the pass
        if rssi is not None:
            if self.rssi_min is None or rssi < self.rssi_min:
                self.rssi_min = rssi
            if self.rssi_max is None or rssi > self.rssi_max:
                self.rssi_max = rssi
            self.rssi_sum += rssi
            self.rssi_count += 1

        # Compute RX data rate (bytes/sec)
        now = time.time()
        if m and self._prev_rx_time > 0:
            dt = now - self._prev_rx_time
            if dt > 0.1:
                dbytes = m.rx_total_bytes - self._prev_rx_total
                self._rx_rate_bps = max(0.0, dbytes / dt)
                self._prev_rx_total = m.rx_total_bytes
                self._prev_rx_time = now
        elif m:
            self._prev_rx_total = m.rx_total_bytes
            self._prev_rx_time = now

        # Read FREQOFF_EST (CC1200's measured freq error — 16-bit signed)
        freqoff_est = None
        fo1 = self.link.read_ext(EXT_FREQOFF_EST1)
        fo0 = self.link.read_ext(EXT_FREQOFF_EST0)
        if fo1 is not None and fo0 is not None:
            raw = (fo1 << 8) | fo0
            if raw & 0x8000:
                raw -= 0x10000
            # Convert to Hz: FREQOFF_EST has same resolution as FREQOFF
            # LSB = f_xosc / (2^18 * LO_DIV) = 40e6 / (262144 * 8) ≈ 19.07 Hz
            freqoff_est_hz = round(raw * 19.07, 0)
            freqoff_est = int(freqoff_est_hz)

        # Read LQI (link quality indicator — lower = better)
        lqi = self.link.read_ext(EXT_LQI_VAL)

        d = {
            "streaming": bool(m.streaming) if m else False,
            "rssi": rssi,
            "packets": self.pkt_count,
            "freq_mhz": self.uhf_freq_hz / 1e6 if self.uhf_freq_hz else 0,
            "radio_config": self.radio_config,
            "rf_backend": "cc1200",
        }
        if m:
            d["cc1200"] = {
                "marc": m.marc,
                "desired_state": m.desired_state,
                "flags": m.flags,
                "rx_overflow": m.rx_overflow_count,
                "rx_fifo_err": m.rx_fifo_err_count,
                "tx_fifo_err": m.tx_fifo_err_count,
                "rx_total_bytes": m.rx_total_bytes,
                "uptime_ms": m.uptime_ms,
                "rx_rate_bps": round(self._rx_rate_bps, 0),
                "freqoff_est_hz": freqoff_est,
                "lqi": lqi,
                "rssi_min": self.rssi_min,
                "rssi_max": self.rssi_max,
                "rssi_avg": round(self.rssi_sum / self.rssi_count, 1) if self.rssi_count > 0 else None,
            }
        return d

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
        self.uhf_base_freq_hz = self.uhf_freq_hz
        if not self.link.set_frequency(self.uhf_freq_hz, force_cal=True):
            log("RF HAT: Failed to set UHF frequency")
            return False

        actual = self.link.get_frequency()
        if actual:
            log(f"RF HAT: UHF freq = {actual/1e6:.6f} MHz")

        return True

    def configure_for_satellite(self, profile: SatProfile, default_profile: str = "") -> bool:
        """Reconfigure the CC1200 for a specific satellite's radio parameters.

        Three tiers:
          1. Custom SmartRF profile → full apply_smartrf_config() + set_frequency()
          2. Modulation override → configure_modulation() + set_frequency()
          3. Frequency-only → just set_frequency()
        """
        # Always clear serial mode before reconfiguring (prevents stale flag
        # from previous sat like SONATE-2 blocking FIFO reads)
        if self.link.is_open():
            self.link.set_serial_mode(False)
        self._use_serial_mode = getattr(profile, 'use_serial_mode', False)
        self.link.select_radio(RADIO_UHF)

        if profile.smartrf_profile:
            # Tier C: full SmartRF profile
            profile_path = str(Path(__file__).parent / profile.smartrf_profile)
            log(f"RF HAT: Loading custom profile: {profile.smartrf_profile}")
            ok, msg = self.link.apply_smartrf_config(profile_path)
            if not ok:
                log(f"RF HAT: Custom profile failed: {msg}")
                return False
            log(f"RF HAT: {msg}")
            ok = self.link.set_frequency(profile.freq_mhz * 1e6)
            self.radio_config = {
                "freq_mhz": profile.freq_mhz,
                "modulation": "custom",
                "symbol_rate_bps": 0,
                "profile": profile.description,
            }

        elif profile.modulation:
            # Tier B: modulation register override
            m = profile.modulation
            sync_str = f", sync={m.sync_word}" if m.sync_word else ", sync=default"
            log(f"RF HAT: Configuring {m.format} {m.symbol_rate_bps} bps, "
                f"dev={m.deviation_hz} Hz, BW={m.rx_bw_khz} kHz{sync_str}")
            ok = self.link.configure_modulation(
                mod_format=m.format,
                symbol_rate_bps=m.symbol_rate_bps,
                deviation_hz=m.deviation_hz,
                rx_bw_khz=m.rx_bw_khz,
                sync_word=m.sync_word,
                freq_hz=profile.freq_mhz * 1e6,
            )
            if not ok:
                log("RF HAT: Modulation reconfiguration failed")
                return False
            self.radio_config = {
                "freq_mhz": profile.freq_mhz,
                "modulation": m.format,
                "symbol_rate_bps": m.symbol_rate_bps,
                "profile": profile.description,
            }

        else:
            # Tier A: frequency-only override
            log(f"RF HAT: Frequency-only retune to {profile.freq_mhz:.3f} MHz")
            ok = self.link.set_frequency(profile.freq_mhz * 1e6)
            self.radio_config = {
                "freq_mhz": profile.freq_mhz,
                "modulation": "2-GFSK",
                "symbol_rate_bps": 2400,
                "profile": profile.description,
            }

        if ok:
            self.uhf_freq_hz = profile.freq_mhz * 1e6
            self.uhf_base_freq_hz = self.uhf_freq_hz
            actual = self.link.get_frequency()
            if actual:
                log(f"RF HAT: UHF freq = {actual/1e6:.6f} MHz")
        return ok

    def start_rx(self) -> bool:
        """Enter RX mode and enable streaming on the currently selected radio."""
        # Clear serial mode if left over from previous sat
        self.link.set_serial_mode(False)
        # Make sure UHF is selected
        self.link.select_radio(RADIO_UHF)
        # Flush RX FIFO before entering RX (prevents stale data / FIFO errors)
        self.link.set_state(STATE_IDLE)
        self.link.strobe(0x3A)  # SFRX — flush RX FIFO
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
        self.link.strobe(0x3A)  # SFRX — flush RX FIFO
        log("RF HAT: UHF RX stopped")

    def start_serial_rx(self) -> bool:
        """Enable CC1200 synchronous serial mode + PIO raw bit streaming."""
        self.link.select_radio(RADIO_UHF)
        if not self.link.set_serial_mode(True):
            log("RF HAT: Failed to enable serial mode")
            return False
        log("RF HAT: Serial mode enabled (raw bit streaming via PIO)")
        return True

    def stop_serial_rx(self):
        """Disable serial mode, return to FIFO/IDLE."""
        self.link.select_radio(RADIO_UHF)
        self.link.set_serial_mode(False)
        log("RF HAT: Serial mode disabled")

    def update_uhf_doppler(self, freq_hz: float) -> bool:
        """Update UHF RX frequency for Doppler correction.

        Uses set_frequency_word() directly for minimal latency — avoids
        redundant select_radio() (which kills streaming) and FS_CFG writes
        (band doesn't change during Doppler updates).
        """
        if abs(freq_hz - self.uhf_freq_hz) < 50:  # skip if < 50 Hz change
            return True
        _, freq2, freq1, freq0 = freq_to_regs(freq_hz)
        word = (freq2 << 16) | (freq1 << 8) | freq0
        ok = self.link.set_frequency_word(word)
        if ok:
            self.uhf_freq_hz = freq_hz
        return ok

    def poll_packets(self, logfile=None) -> int:
        """Process pending RX events. Returns number of new packets.

        ALL RX data is written to self._raw_file (if open) for post-processing.
        Packets >= 4 bytes are counted and logged separately.
        """
        MIN_PACKET_BYTES = 4  # minimum real frame size
        count = 0
        for evt_type, body in self.link.pop_events():
            if evt_type == EVT_RX_DATA and body:
                n = body[0]
                data = body[1:1 + n]
                # Write ALL raw data to binary dump file (including noise)
                if self._raw_file and data:
                    self._raw_file.write(data)
                    self._raw_bytes += len(data)
                if n < MIN_PACKET_BYTES:
                    continue
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

    def open_raw_dump(self, path: str, metadata: dict = None):
        """Open a binary file to capture ALL raw RX data for post-processing.
        Also writes a .json metadata file alongside the .raw file."""
        self._raw_file = open(path, "wb")
        self._raw_bytes = 0
        self._raw_path = path
        log(f"RF HAT: Raw dump → {path}")
        # Write metadata JSON alongside the raw file
        if metadata:
            meta_path = path.replace('.raw', '.json')
            try:
                import json as _json
                with open(meta_path, "w") as mf:
                    _json.dump(metadata, mf, indent=2)
            except Exception:
                pass

    def close_raw_dump(self) -> int:
        """Close raw dump file. Returns total bytes written.
        Updates .json metadata with final capture stats."""
        n = self._raw_bytes
        if self._raw_file:
            self._raw_file.close()
            self._raw_file = None
        if n > 0:
            log(f"RF HAT: Raw dump closed — {n} bytes")
            # Update metadata with final stats
            meta_path = getattr(self, '_raw_path', '').replace('.raw', '.json')
            if meta_path and os.path.exists(meta_path):
                try:
                    import json as _json
                    with open(meta_path, "r") as mf:
                        meta = _json.load(mf)
                    meta["capture_bytes"] = n
                    meta["packets"] = self.pkt_count
                    meta["rssi_min"] = self.rssi_min
                    meta["rssi_max"] = self.rssi_max
                    meta["rssi_avg"] = round(self.rssi_sum / self.rssi_count, 1) if self.rssi_count > 0 else None
                    meta["end_time"] = datetime.datetime.now(datetime.timezone.utc).isoformat()
                    with open(meta_path, "w") as mf:
                        _json.dump(meta, mf, indent=2)
                except Exception:
                    pass
        return n

    def get_metrics_str(self) -> str:
        # Don't call select_radio() — it kills streaming
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
# RF backend factory
# ---------------------------------------------------------------------------
def create_rf_backend(args, logfile=None) -> Optional[RfBackend]:
    """Create and open an RF backend based on --rf-mode.

    Priority for 'auto': CC1200 -> RTL-SDR -> None.
    """
    mode = getattr(args, 'rf_mode', 'auto')

    if mode == "none":
        log("RF mode: none (disabled)", logfile)
        return None

    if mode in ("cc1200", "auto"):
        rf = RfHatManager(args.rf_port)
        if rf.open():
            if rf.setup_uhf_rx(args.uhf_freq, args.uhf_profile):
                log(f"RF backend: CC1200 on {args.rf_port}", logfile)
                return rf
            else:
                log("CC1200: UHF setup failed", logfile)
                rf.close()
        else:
            if mode == "cc1200":
                log("CC1200: failed to open (explicit mode)", logfile)
            else:
                log("CC1200: not available, trying RTL-SDR...", logfile)

        if mode == "cc1200":
            return None

    if mode in ("rtlsdr", "auto"):
        try:
            from rtlsdr_backend import RtlSdrBackend
            device_index = getattr(args, 'rtlsdr_device', 0)
            gain = getattr(args, 'rtlsdr_gain', 'auto')
            sample_rate = getattr(args, 'rtlsdr_rate', 240000)
            rtl = RtlSdrBackend(
                device_index=device_index,
                gain=gain,
                sample_rate=sample_rate,
            )
            if rtl.open():
                rtl.set_frequency(args.uhf_freq * 1e6)
                log(f"RF backend: RTL-SDR (device {device_index})", logfile)
                return rtl
            else:
                log("RTL-SDR: failed to open", logfile)
        except ImportError:
            if mode == "rtlsdr":
                log("RTL-SDR: pyrtlsdr not installed", logfile)
            else:
                log("RTL-SDR: not available (pyrtlsdr not installed)", logfile)
        except Exception as e:
            log(f"RTL-SDR: error: {e}", logfile)

    log("RF backend: none available", logfile)
    return None


# ---------------------------------------------------------------------------
# Pass tracker
# ---------------------------------------------------------------------------
def track_pass(rotctl: RotctlClient, rf: Optional[RfBackend],
               tle, pass_info, args, logfile=None, buzzer: Optional[Buzzer] = None,
               lat=None, lon=None, elev=None,
               satnogs: Optional[SatNOGSClient] = None):
    """Execute a full satellite pass with tracking + RF."""
    name, l1, l2 = tle
    norad_id = norad_id_from_tle(l1)
    sat = ephem.readtle(name, l1, l2)
    obs = make_observer(lat, lon, elev)
    obs.horizon = "0"

    log(f"========== PASS: {name} ==========", logfile)
    log(f"  Max EL: {pass_info['max_el']:.1f} deg", logfile)
    log(f"  Duration: {pass_info['duration']:.0f}s", logfile)
    log(f"  Rise AZ: {pass_info['rise_az']:.1f} → Set AZ: {pass_info['set_az']:.1f}", logfile)
    if rf:
        log(f"  RF backend: {rf.name()}", logfile)
        _actual_mhz = (getattr(rf, 'uhf_base_freq_hz', 0) or 0) / 1e6 or args.uhf_freq
        log(f"  UHF RX: {_actual_mhz:.3f} MHz", logfile)
        if args.doppler:
            log(f"  Doppler correction: ENABLED", logfile)

    # Pre-position rotator
    rise_az = pass_info["rise_az"]
    log(f"Slewing to AOS: AZ {rise_az:.1f} EL 0.0", logfile)
    rotctl.set_position(rise_az, 0)

    # Write status during pre-AOS slew so dashboard knows we're active
    _rc = getattr(rf, 'radio_config', None) if rf else None
    _rf_name = rf.name() if rf and hasattr(rf, 'name') else "none"
    write_dashboard_status({
        "satellite": name, "pass_progress": 0,
        "freq_mhz": (getattr(rf, 'uhf_base_freq_hz', 0) or 0) / 1e6 or args.uhf_freq,
        "streaming": False, "rssi": None, "packets": 0,
        "rf_backend": _rf_name, "radio_config": _rc,
        "cmd_az": round(rise_az, 2), "cmd_el": 0.0,
        "max_el": round(pass_info["max_el"], 1),
        "rise_az": round(pass_info["rise_az"], 1),
        "set_az": round(pass_info["set_az"], 1),
        "los_seconds": round(pass_info["duration"], 0),
        "duration": round(pass_info["duration"], 0),
    })

    # Wait for AOS
    now_utc = datetime.datetime.now(datetime.timezone.utc)
    rise_utc = pass_info["rise_time"].replace(tzinfo=datetime.timezone.utc)
    wait = (rise_utc - now_utc).total_seconds()
    if wait > 0:
        log(f"Waiting {wait:.0f}s for AOS...", logfile)
        stop_file = os.path.expanduser("~/.station_stop")
        track_file_check = os.path.expanduser("~/.station_track")
        _last_keepalive = time.monotonic()
        while wait > 0.5:
            time.sleep(min(wait, 3.0))
            now_utc = datetime.datetime.now(datetime.timezone.utc)
            wait = (rise_utc - now_utc).total_seconds()
            # Re-send position periodically during AOS wait (keeps rotctld alive)
            if time.monotonic() - _last_keepalive >= 15.0:
                try:
                    rotctl.set_position(rise_az, 0)
                except Exception:
                    pass
                _last_keepalive = time.monotonic()
            # Keep status file fresh so dashboard shows pre-AOS state
            write_dashboard_status({
                "satellite": name, "pass_progress": 0,
                "freq_mhz": (getattr(rf, 'uhf_base_freq_hz', 0) or 0) / 1e6 or args.uhf_freq,
                "streaming": False, "rssi": None, "packets": 0,
                "rf_backend": _rf_name, "radio_config": _rc,
                "cmd_az": round(pass_info["rise_az"], 2), "cmd_el": 0.0,
                "max_el": round(pass_info["max_el"], 1),
                "rise_az": round(pass_info["rise_az"], 1),
                "set_az": round(pass_info["set_az"], 1),
                "los_seconds": round(wait + pass_info["duration"], 0),
                "duration": round(pass_info["duration"], 0),
            })
            if not running:
                return
            # Check for stop or new track during AOS wait
            if os.path.exists(stop_file):
                os.remove(stop_file)
                log("** STOP during AOS wait **", logfile)
                try:
                    with open(os.path.expanduser("~/.station_paused"), "w") as _pf:
                        _pf.write("paused")
                except Exception:
                    pass
                return
            if os.path.exists(track_file_check):
                log("New track request during AOS wait — aborting", logfile)
                return

    log("** AOS — pass started **", logfile)
    if buzzer:
        buzzer.beep_aos()

    # Open CSV pass log for post-analysis
    pass_csv = None
    try:
        log_dir = os.path.expanduser("~/pass_logs")
        os.makedirs(log_dir, exist_ok=True)
        ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        safe_name = name.replace(" ", "_").replace("/", "-")
        csv_path = os.path.join(log_dir, f"{ts}_{safe_name}.csv")
        pass_csv = open(csv_path, "w")
        pass_csv.write("t,az_cmd,el_cmd,az_act,el_act,rssi,freq_hz,doppler_hz,range_km,range_rate,packets\n")
        log(f"Pass log: {csv_path}", logfile)
    except Exception as e:
        log(f"Pass log failed: {e}", logfile)

    # Open raw binary dump for all RX data (post-processing)
    if rf and isinstance(rf, RfHatManager):
        raw_path = os.path.join(log_dir, f"{ts}_{safe_name}.raw")
        _rc = getattr(rf, 'radio_config', {})
        rf.open_raw_dump(raw_path, metadata={
            "satellite": name,
            "norad_id": pass_info.get("norad_id"),
            "start_time": datetime.datetime.now(datetime.timezone.utc).isoformat(),
            "freq_mhz": _rc.get("freq_mhz", 0),
            "modulation": _rc.get("modulation", ""),
            "symbol_rate_bps": _rc.get("symbol_rate_bps", 0),
            "profile": _rc.get("profile", ""),
            "max_el": round(pass_info.get("max_el", 0), 1),
            "rise_az": round(pass_info.get("rise_az", 0), 1),
            "set_az": round(pass_info.get("set_az", 0), 1),
            "duration_s": round(pass_info.get("duration", 0), 0),
            "doppler_enabled": getattr(args, 'doppler', False),
            "serial_mode": getattr(rf, '_use_serial_mode', False),
            "station_lat": sta_lat if 'sta_lat' in dir() else None,
            "station_lon": sta_lon if 'sta_lon' in dir() else None,
        })

    # Start RX if RF is available
    serial_decoder = None
    if rf:
        # Use serial mode if: CLI flag set, OR profile says this sat needs it (G3RUH/USP)
        _profile_serial = getattr(rf, '_use_serial_mode', False)
        use_serial = (getattr(args, 'serial_mode', False) or _profile_serial) and isinstance(rf, RfHatManager)
        if use_serial:
            rf.start_serial_rx()
            # Initialize bitstream decoder
            decoder_name = getattr(args, 'decoder', 'usp')
            if decoder_name == 'usp':
                from usp_decoder import USPDecoder
                serial_decoder = USPDecoder()
                log("Serial mode: USP decoder active", logfile)
            elif decoder_name == 'ax100':
                from ax100_decoder import AX100Decoder
                serial_decoder = AX100Decoder()
                log("Serial mode: AX.100 decoder active", logfile)
            else:
                log("Serial mode: raw capture (no decoder)", logfile)
        else:
            rf.start_rx()
        # Reset packet counters (CC1200 only — RTL-SDR doesn't decode packets)
        if isinstance(rf, RfHatManager):
            rf.pkt_count = 0
            rf.total_bytes = 0
            rf.packets.clear()
            rf.rssi_min = None
            rf.rssi_max = None
            rf.rssi_sum = 0.0
            rf.rssi_count = 0
            rf._prev_rx_total = 0
            rf._prev_rx_time = 0.0
            rf._rx_rate_bps = 0.0

    set_utc = pass_info["set_time"].replace(tzinfo=datetime.timezone.utc)
    dt = 1.0 / UPDATE_HZ
    doppler_dt = 1.0 / DOPPLER_HZ
    last_doppler = 0.0
    last_metrics = 0.0
    tick = 0
    c = 299_792_458.0  # speed of light

    stop_file = os.path.expanduser("~/.station_stop")

    try:
        while running:
            now_utc = datetime.datetime.now(datetime.timezone.utc)
            if now_utc >= set_utc:
                break

            # Check for dashboard stop command
            if os.path.exists(stop_file):
                os.remove(stop_file)
                log("** STOP — user cancelled tracking **", logfile)
                try:
                    with open(os.path.expanduser("~/.station_paused"), "w") as _pf:
                        _pf.write("paused")
                except Exception:
                    pass
                break
            # Check for new track request (user clicked Track on a different satellite)
            track_file_mid = os.path.expanduser("~/.station_track")
            if os.path.exists(track_file_mid):
                try:
                    with open(track_file_mid) as _tf:
                        new_sat = _tf.read().strip()
                    if new_sat.lower() != name.lower():
                        log(f"** New track: {new_sat} — aborting {name} **", logfile)
                        break
                except Exception:
                    pass

            # Compute satellite position
            obs.date = ephem.Date(now_utc)
            sat.compute(obs)
            az = math.degrees(float(sat.az))
            el = math.degrees(float(sat.alt))

            # Send to rotator (reconnect on failure)
            if el >= 0:
                try:
                    rotctl.set_position(az, el)
                except (BrokenPipeError, OSError, ConnectionError):
                    log("Rotctld connection lost — reconnecting...", logfile)
                    try:
                        rotctl.connect()
                        rotctl.set_position(az, el)
                    except Exception:
                        pass  # will retry next tick

            # Doppler correction
            now_mono = time.time()
            if rf and args.doppler and (now_mono - last_doppler) >= doppler_dt:
                last_doppler = now_mono
                range_rate = float(sat.range_velocity)
                # Use the satellite's nominal base frequency (not Doppler-shifted)
                _base_hz = getattr(rf, 'uhf_base_freq_hz', 0) or args.uhf_freq * 1e6
                uhf_doppler = doppler_shift(_base_hz, range_rate)
                rf.set_frequency(uhf_doppler)

            # Poll RF packets (or raw bitstream in serial mode)
            if rf:
                if serial_decoder and isinstance(rf, RfHatManager):
                    # Serial mode: feed raw bytes to decoder
                    new_pkts = 0
                    for evt_type, body in rf.link.pop_events():
                        if evt_type == EVT_RX_DATA and body:
                            n = body[0]
                            raw = body[1:1 + n]
                            rf.total_bytes += n
                            decoded_frames = serial_decoder.feed(raw)
                            for frame in decoded_frames:
                                rf.pkt_count += 1
                                new_pkts += 1
                                rf.packets.append((datetime.datetime.now(), frame))
                                log(f"DECODED [{rf.pkt_count}] {len(frame)} bytes: "
                                    f"{frame[:32].hex()}", logfile)
                else:
                    new_pkts = rf.poll_packets(logfile)
                if new_pkts > 0:
                    if buzzer:
                        buzzer.beep_packet()
                    # Submit newly-received packets to SatNOGS DB (CC1200 only)
                    if satnogs and isinstance(rf, RfHatManager):
                        new_entries = rf.packets[-new_pkts:]
                        for _ts, _data in new_entries:
                            frame_hex = _data.hex()
                            ok = satnogs.submit_frame(norad_id, frame_hex)
                            log(f"SatNOGS: {'submitted' if ok else 'queued'} "
                                f"{len(_data)}B frame (NORAD {norad_id})", logfile)

            # Periodic metrics
            if rf and (now_mono - last_metrics) >= METRICS_INTERVAL_S:
                last_metrics = now_mono
                remaining = (set_utc - now_utc).total_seconds()
                m = rf.get_metrics()
                sig_str = f"{m.signal_dbm:.1f} {m.signal_unit}" if m.signal_dbm is not None else "N/A"
                log(f"METRICS: {m.backend_name} {m.signal_label}={sig_str} "
                    f"pkts={m.packets} bytes={m.total_bytes} T-{remaining:.0f}s", logfile)

            # Status line
            remaining = (set_utc - now_utc).total_seconds()
            total_dur = pass_info["duration"]
            progress = max(0, min(100, (1 - remaining / total_dur) * 100)) if total_dur > 0 else 0
            bar_el = int(max(0, el) / 90 * 20)
            bar = "|" + "#" * bar_el + " " * (20 - bar_el) + "|"
            if rf:
                m = rf.get_metrics()
                rf_info = f"pkts={m.packets}"
            else:
                rf_info = "no-rf"
            print(f"\r  AZ{az:>7.1f} EL{el:>5.1f} {bar} {rf_info:<12s} T-{remaining:>5.0f}s",
                  end="", flush=True)

            # Update dashboard status file (~5 Hz for responsive GUI)
            if tick % 2 == 0:
                if rf:
                    status_data = rf.get_status_dict()
                else:
                    status_data = {"streaming": False, "rssi": None, "packets": 0,
                                   "freq_mhz": 0, "rf_backend": "none"}
                status_data["satellite"] = name
                status_data["pass_progress"] = round(progress, 1)
                # freq_mhz already set by get_status_dict() with Doppler-corrected value
                # Telemetry for focus panel
                status_data["cmd_az"] = round(az, 2)
                status_data["cmd_el"] = round(el, 2)
                status_data["range_km"] = round(float(sat.range) / 1000, 1)
                status_data["range_rate"] = round(float(sat.range_velocity), 1)
                status_data["max_el"] = round(pass_info["max_el"], 1)
                status_data["rise_az"] = round(pass_info["rise_az"], 1)
                status_data["set_az"] = round(pass_info["set_az"], 1)
                status_data["los_seconds"] = round(remaining, 0)
                status_data["duration"] = round(pass_info["duration"], 0)
                # Doppler: compute from range_velocity even if not correcting RF
                _base_hz2 = getattr(rf, 'uhf_base_freq_hz', 0) or args.uhf_freq * 1e6
                _dop_hz = doppler_shift(_base_hz2, float(sat.range_velocity)) - _base_hz2
                status_data["doppler_hz"] = round(_dop_hz, 1)
                if satnogs:
                    status_data["satnogs"] = satnogs.get_stats()
                write_dashboard_status(status_data)

                # Append to CSV pass log
                if pass_csv:
                    try:
                        _t = datetime.datetime.now(datetime.timezone.utc).isoformat()
                        _rssi = status_data.get("rssi", "")
                        _freq = status_data.get("freq_mhz", 0) * 1e6 if status_data.get("freq_mhz") else ""
                        _dop = status_data.get("doppler_hz", "")
                        _rng = status_data.get("range_km", "")
                        _rr = status_data.get("range_rate", "")
                        _pkts = status_data.get("packets", 0)
                        pass_csv.write(f"{_t},{az:.2f},{el:.2f},{status_data.get('cmd_az','')},{status_data.get('cmd_el','')},{_rssi},{_freq},{_dop},{_rng},{_rr},{_pkts}\n")
                        pass_csv.flush()
                    except Exception:
                        pass

            tick += 1
            next_tick = time.time() + dt
            sleep_time = next_tick - time.time()
            if sleep_time > 0:
                time.sleep(sleep_time)

    finally:
        print()
        if pass_csv:
            try:
                pass_csv.close()
            except Exception:
                pass
        log("** LOS — pass complete **", logfile)
        # Close raw binary dump
        if rf and isinstance(rf, RfHatManager):
            raw_bytes = rf.close_raw_dump()
            if raw_bytes > 0:
                log(f"Raw capture: {raw_bytes} bytes saved", logfile)
        if buzzer:
            buzzer.beep_los()
        pkts, nbytes, peak_rssi = 0, 0, None
        if rf:
            use_serial = getattr(args, 'serial_mode', False) and isinstance(rf, RfHatManager)
            if use_serial:
                rf.stop_serial_rx()
                if serial_decoder:
                    log(f"Serial decoder: {serial_decoder.sync_found} syncs, "
                        f"{serial_decoder.frames_decoded} frames decoded", logfile)
            else:
                rf.stop_rx()
            m = rf.get_metrics()
            pkts, nbytes = m.packets, m.total_bytes
            peak_rssi = getattr(m, 'peak_rssi', None)
            log(f"Pass summary: {pkts} packets, {nbytes} bytes ({rf.name()})", logfile)
        record_pass_history(name, pass_info, pkts, nbytes, peak_rssi)
        # Always park after tracking ends (clean exit)
        try:
            rotctl.park()
            log("Parked after pass", logfile)
        except Exception:
            # Retry park with reconnect (rotctld may have restarted)
            try:
                rotctl.connect()
                rotctl.park()
                log("Parked after reconnect", logfile)
            except Exception:
                log("WARNING: could not park rotator", logfile)
        clear_dashboard_status()

    if rf:
        m = rf.get_metrics()
        return m.packets
    return 0


# ---------------------------------------------------------------------------
# Scan mode — cycle through all receivable satellites near their peak
# ---------------------------------------------------------------------------
class _ScanArgs:
    """Minimal args object for scan mode — avoids passing full argparse namespace."""
    def __init__(self, uhf_freq=433.0, doppler=True, mode="scan"):
        self.uhf_freq = uhf_freq
        self.doppler = doppler
        self.mode = mode


def find_all_passes(tles, hours=4, min_el=10.0, lat=None, lon=None, elev=None):
    """Find ALL upcoming passes for all TLE entries.  Returns a list of dicts
    sorted by max-elevation time, each with full orbital data."""
    obs = ephem.Observer()
    obs.lat = str(lat or STATION_LAT)
    obs.lon = str(lon or STATION_LON)
    obs.elevation = float(elev or STATION_ELEV)
    obs.horizon = "0"
    obs.pressure = 0

    now = ephem.now()
    end = now + hours / 24.0
    passes = []

    for name, l1, l2 in tles:
        try:
            sat = ephem.readtle(name, l1, l2)
            obs.date = now
            attempts = 0
            while obs.date < end and attempts < 5:
                attempts += 1
                try:
                    rise_t, rise_az, max_t, max_el, set_t, set_az = obs.next_pass(sat)
                except Exception:
                    break
                if rise_t is None or set_t is None:
                    break
                if rise_t > end:
                    break

                max_el_deg = math.degrees(float(max_el))
                if max_el_deg < min_el:
                    obs.date = set_t + ephem.minute
                    continue
                if set_t < now:
                    obs.date = set_t + ephem.minute
                    continue

                rise_dt = ephem.Date(rise_t).datetime().replace(tzinfo=datetime.timezone.utc)
                max_dt = ephem.Date(max_t).datetime().replace(tzinfo=datetime.timezone.utc)
                set_dt = ephem.Date(set_t).datetime().replace(tzinfo=datetime.timezone.utc)
                duration = (set_dt - rise_dt).total_seconds()

                passes.append({
                    "name": name,
                    "norad_id": norad_id_from_tle(l1),
                    "tle": (name, l1, l2),
                    "rise_time": rise_dt,
                    "max_time": max_dt,
                    "set_time": set_dt,
                    "max_el": max_el_deg,
                    "duration": duration,
                    "rise_az": math.degrees(float(rise_az)),
                    "set_az": math.degrees(float(set_az)),
                })
                obs.date = set_t + ephem.minute
        except Exception:
            continue

    passes.sort(key=lambda p: p["max_time"])
    return passes


def build_schedule(passes, mode="normal", slew_time=8.0, dwell_s=30):
    """Build a non-overlapping visit schedule from candidate passes.

    Each visit is a time window [start, end] centred on the satellite's
    peak elevation.  Greedily assigns visits in max-el-time order,
    skipping overlaps (including slew time).
    """
    schedule = []

    for p in passes:
        max_t = p["max_time"]
        dur = p["duration"]

        if mode in ("test", "scan"):
            half = dwell_s / 2.0
        else:
            half = dur * 0.30

        visit_start = max_t - datetime.timedelta(seconds=half)
        visit_end = max_t + datetime.timedelta(seconds=half)

        if visit_start < p["rise_time"]:
            visit_start = p["rise_time"]
        if visit_end > p["set_time"]:
            visit_end = p["set_time"]

        if schedule:
            prev_end = schedule[-1]["end"]
            gap = (visit_start - prev_end).total_seconds()
            if gap < slew_time:
                continue

        schedule.append({
            "name": p["name"],
            "norad_id": p["norad_id"],
            "tle": p["tle"],
            "start": visit_start,
            "end": visit_end,
            "max_time": max_t,
            "max_el": p["max_el"],
            "rise_az": p["rise_az"],
            "set_az": p["set_az"],
            "duration": (visit_end - visit_start).total_seconds(),
            "pass_duration": dur,
            "pass": p,
        })

    return schedule


def execute_visit(visit, rotctl, rf, args, sat_library, logfile=None,
                  buzzer=None, lat=None, lon=None, elev=None,
                  scan_info=None, check_abort=None, _write_status=None):
    """Track a single satellite for its scheduled visit window."""
    if _write_status is None:
        _write_status = write_dashboard_status

    name, l1, l2 = visit["tle"]
    norad_id = visit["norad_id"]
    sat = ephem.readtle(name, l1, l2)
    obs = make_observer(lat, lon, elev)
    obs.horizon = "0"

    visit_start = visit["start"]
    visit_end = visit["end"]
    dwell = visit["duration"]

    log(f"--- VISIT: {name} (NORAD {norad_id}) ---", logfile)
    log(f"  Window: {dwell:.0f}s, max EL: {visit['max_el']:.1f}°", logfile)

    # Configure CC1200 for this satellite
    _use_serial = False
    if rf:
        profile = lookup_with_satnogs_fallback(sat_library, norad_id, name)
        if profile:
            log(f"  Radio: {profile.freq_mhz:.3f} MHz — {profile.description}", logfile)
            if profile.use_serial_mode:
                log(f"  Serial mode: {profile.protocol} (raw bit stream)", logfile)
                _use_serial = True
            rf.configure_for_satellite(profile, sat_library.default_profile or "")
        else:
            log(f"  No profile — default {args.uhf_freq:.3f} MHz", logfile)
            rf.set_frequency(args.uhf_freq * 1e6)
            if isinstance(rf, RfHatManager):
                rf.radio_config = {
                    "freq_mhz": args.uhf_freq,
                    "modulation": "2-GFSK",
                    "symbol_rate_bps": 2400,
                    "profile": "default",
                }

    # Pre-slew: compute where the sat will be at visit_start
    now_utc = datetime.datetime.now(datetime.timezone.utc)
    target_t = max(visit_start, now_utc)
    obs.date = ephem.Date(target_t)
    sat.compute(obs)
    pre_az = math.degrees(float(sat.az))
    pre_el = max(0, math.degrees(float(sat.alt)))
    log(f"  Pre-slew to AZ {pre_az:.1f} EL {pre_el:.1f}", logfile)
    rotctl.set_position(pre_az, pre_el)

    scan_pos_str = ""
    if scan_info:
        scan_pos_str = f"{scan_info['pos']}/{scan_info['total']}"

    # Wait for visit window to start
    wait = (visit_start - datetime.datetime.now(datetime.timezone.utc)).total_seconds()
    if wait > 0:
        log(f"  Waiting {wait:.0f}s for visit window...", logfile)
        _last_status_write = 0
        while wait > 0.5 and running:
            if check_abort:
                reason = check_abort()
                if reason:
                    log(f"  Abort during wait: {reason}", logfile)
                    return 0

            time.sleep(min(wait, 1.0))
            wait = (visit_start - datetime.datetime.now(datetime.timezone.utc)).total_seconds()
            obs.date = ephem.Date(datetime.datetime.now(datetime.timezone.utc))
            sat.compute(obs)
            az = math.degrees(float(sat.az))
            el = max(0, math.degrees(float(sat.alt)))
            try:
                rotctl.set_position(az, el)
            except Exception:
                pass
            now_mono = time.time()
            if now_mono - _last_status_write >= 2.0:
                _last_status_write = now_mono
                _base_hz = getattr(rf, 'uhf_base_freq_hz', 0) or 0
                _dop_hz = doppler_shift(_base_hz, float(sat.range_velocity)) - _base_hz if _base_hz else 0
                _rc = getattr(rf, 'radio_config', None) if rf else None
                status = {
                    "satellite": name,
                    "pass_progress": 0,
                    "streaming": False, "rssi": None, "packets": 0,
                    "freq_mhz": _base_hz / 1e6 if _base_hz else 0,
                    "rf_backend": rf.name() if rf else "none",
                    "radio_config": _rc,
                    "cmd_az": round(az, 2), "cmd_el": round(el, 2),
                    "range_km": round(float(sat.range) / 1000, 1),
                    "range_rate": round(float(sat.range_velocity), 1),
                    "doppler_hz": round(_dop_hz, 1),
                    "max_el": round(visit["max_el"], 1),
                    "rise_az": round(visit["rise_az"], 1),
                    "set_az": round(visit["set_az"], 1),
                    "los_seconds": round(wait + dwell, 0),
                    "duration": round(visit["pass_duration"], 0),
                    "auto_mode": args.mode,
                }
                if scan_pos_str:
                    status["scan_pos"] = scan_pos_str
                _write_status(status)

    if not running:
        return 0

    log(f"  ** ON TARGET — listening **", logfile)
    if buzzer:
        buzzer.beep_aos()

    # Start RX
    if rf:
        if isinstance(rf, RfHatManager):
            log_dir = os.path.expanduser("~/pass_logs")
            os.makedirs(log_dir, exist_ok=True)
            _ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            safe = name.replace(" ", "_").replace("/", "-")
            _rc = getattr(rf, 'radio_config', {})
            rf.open_raw_dump(os.path.join(log_dir, f"{_ts}_{safe}.raw"), metadata={
                "satellite": name,
                "norad_id": norad_id,
                "start_time": datetime.datetime.now(datetime.timezone.utc).isoformat(),
                "freq_mhz": _rc.get("freq_mhz", 0),
                "modulation": _rc.get("modulation", ""),
                "symbol_rate_bps": _rc.get("symbol_rate_bps", 0),
                "profile": _rc.get("profile", ""),
                "max_el": round(visit.get("max_el", 0), 1),
                "duration_s": round(dwell, 0),
                "mode": args.mode,
                "serial_mode": _use_serial,
            })
        if _use_serial and isinstance(rf, RfHatManager):
            rf.start_serial_rx()
        else:
            rf.start_rx()
        if isinstance(rf, RfHatManager):
            rf.pkt_count = 0
            rf.total_bytes = 0
            rf.packets.clear()
            rf.rssi_min = None
            rf.rssi_max = None
            rf.rssi_sum = 0.0
            rf.rssi_count = 0

    last_doppler = 0.0
    last_metrics = 0.0
    tick = 0

    try:
        while running:
            now_utc = datetime.datetime.now(datetime.timezone.utc)
            if now_utc >= visit_end:
                break

            if check_abort and tick % 10 == 0:
                reason = check_abort()
                if reason:
                    log(f"  Abort during tracking: {reason}", logfile)
                    break

            obs.date = ephem.Date(now_utc)
            sat.compute(obs)
            az = math.degrees(float(sat.az))
            el = math.degrees(float(sat.alt))

            if el >= 0:
                try:
                    rotctl.set_position(az, el)
                except (BrokenPipeError, OSError):
                    try:
                        rotctl.connect()
                        rotctl.set_position(az, el)
                    except Exception:
                        pass

            # Doppler
            now_mono = time.time()
            if rf and args.doppler and (now_mono - last_doppler) >= 0.5:
                last_doppler = now_mono
                range_rate = float(sat.range_velocity)
                base_hz = getattr(rf, 'uhf_base_freq_hz', 0) or args.uhf_freq * 1e6
                rf.set_frequency(doppler_shift(base_hz, range_rate))

            # Poll packets
            if rf:
                new_pkts = rf.poll_packets(logfile)
                if new_pkts > 0 and isinstance(rf, RfHatManager):
                    real_pkts = sum(1 for _, data in rf.packets[-new_pkts:] if len(data) >= 4)
                    if real_pkts > 0 and buzzer:
                        buzzer.beep_packet()

            # Metrics (1 Hz)
            if rf and (now_mono - last_metrics) >= 1.0:
                last_metrics = now_mono
                remaining = (visit_end - now_utc).total_seconds()
                m = rf.get_metrics()
                sig_str = f"{m.signal_dbm:.1f} dBm" if m.signal_dbm is not None else "N/A"
                progress = max(0, min(100, (1 - remaining / dwell) * 100))

                if isinstance(rf, RfHatManager):
                    status_data = rf.get_status_dict()
                else:
                    status_data = {"streaming": False, "rssi": None, "packets": 0,
                                   "freq_mhz": 0, "rf_backend": "none"}
                status_data["streaming"] = True
                status_data["satellite"] = name
                status_data["pass_progress"] = round(progress, 1)
                status_data["cmd_az"] = round(az, 2)
                status_data["cmd_el"] = round(el, 2)
                status_data["range_km"] = round(float(sat.range) / 1000, 1)
                status_data["range_rate"] = round(float(sat.range_velocity), 1)
                status_data["los_seconds"] = round(remaining, 0)
                status_data["duration"] = round(visit["pass_duration"], 0)
                status_data["max_el"] = round(visit["max_el"], 1)
                status_data["rise_az"] = round(visit["rise_az"], 1)
                status_data["set_az"] = round(visit["set_az"], 1)
                status_data["auto_mode"] = args.mode
                if scan_pos_str:
                    status_data["scan_pos"] = scan_pos_str
                _base_hz = getattr(rf, 'uhf_base_freq_hz', 0) or args.uhf_freq * 1e6
                _dop_hz = doppler_shift(_base_hz, float(sat.range_velocity)) - _base_hz
                status_data["doppler_hz"] = round(_dop_hz, 1)
                _write_status(status_data)

                print(f"\r  {name:<20s} AZ{az:>7.1f} EL{el:>5.1f} "
                      f"RSSI={sig_str:<8s} pkts={m.packets:<4d} T-{remaining:>4.0f}s",
                      end="", flush=True)

            tick += 1
            time.sleep(0.1)

    finally:
        print()
        if buzzer:
            buzzer.beep_los()
        pkts = 0
        if rf:
            if _use_serial and isinstance(rf, RfHatManager):
                rf.stop_serial_rx()
            else:
                rf.stop_rx()
            if isinstance(rf, RfHatManager):
                raw_bytes = rf.close_raw_dump()
                if raw_bytes > 0:
                    log(f"  Raw capture: {raw_bytes} bytes saved", logfile)
            m = rf.get_metrics()
            pkts = m.packets
            log(f"  Visit done: {pkts} packets, {m.total_bytes} bytes", logfile)
        record_pass_history(name, visit["pass"], pkts,
                            m.total_bytes if rf else 0,
                            getattr(m, 'peak_rssi', None) if rf else None)

    return pkts


def run_scan_cycle(rotctl, rf, sat_library, buzzer, tles,
                   lat, lon, elev,
                   hours=4, min_el=10.0, dwell_s=30,
                   uhf_freq=433.0, doppler=True,
                   check_abort=None, write_status=None, logfile=None):
    """Run one complete scan cycle: find passes, build schedule, visit each.

    Returns:
        (visited_count, total_packets, abort_reason)
        abort_reason is None if cycle completed normally.
    """
    if write_status is None:
        write_status = write_dashboard_status

    args = _ScanArgs(uhf_freq=uhf_freq, doppler=doppler, mode="scan")

    log("SCAN — Computing passes...", logfile)
    write_status({
        "satellite": "", "pass_progress": 0,
        "streaming": False, "rssi": None, "packets": 0,
        "freq_mhz": 0, "rf_backend": rf.name() if rf else "none",
        "auto_mode": "scan", "scan_pos": "0/0",
    })

    all_passes = find_all_passes(tles, hours=hours, min_el=min_el,
                                  lat=lat, lon=lon, elev=elev)
    log(f"SCAN — Found {len(all_passes)} passes above {min_el}°", logfile)

    if not all_passes:
        log("SCAN — No passes found", logfile)
        return (0, 0, None)

    schedule = build_schedule(all_passes, mode="scan", dwell_s=dwell_s)
    log(f"SCAN — Scheduled {len(schedule)} visits ({dwell_s}s dwell)", logfile)

    if not schedule:
        return (0, 0, None)

    total_pkts = 0
    visited = 0
    abort_reason = None

    for i, visit in enumerate(schedule):
        if not running:
            break

        if check_abort:
            reason = check_abort()
            if reason:
                log(f"SCAN — Aborted before visit {i+1}: {reason}", logfile)
                abort_reason = reason
                break

        now_utc = datetime.datetime.now(datetime.timezone.utc)
        if now_utc >= visit["end"]:
            log(f"SCAN — Skipping {visit['name']} — window already passed", logfile)
            continue

        wait = (visit["start"] - now_utc).total_seconds()
        log(f"SCAN [{i+1}/{len(schedule)}] Next: {visit['name']} "
               f"(EL {visit['max_el']:.0f}°, {visit['duration']:.0f}s dwell) "
               f"in {max(0,wait):.0f}s", logfile)

        scan_info = {"pos": i + 1, "total": len(schedule)}

        write_status({
            "satellite": visit['name'],
            "pass_progress": 0,
            "streaming": False, "rssi": None, "packets": total_pkts,
            "freq_mhz": 0, "rf_backend": rf.name() if rf else "none",
            "auto_mode": "scan",
            "scan_pos": f"{i+1}/{len(schedule)}",
        })

        pkts = execute_visit(
            visit, rotctl, rf, args, sat_library,
            logfile=logfile, buzzer=buzzer,
            lat=lat, lon=lon, elev=elev,
            scan_info=scan_info, check_abort=check_abort,
            _write_status=write_status)

        total_pkts += pkts
        visited += 1
        log(f"SCAN — {visit['name']}: {pkts} pkts (total: {total_pkts})", logfile)

    log(f"SCAN — Cycle done: {visited} visited, {total_pkts} packets", logfile)
    return (visited, total_pkts, abort_reason)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
running = True

def on_signal(sig, frame):
    global running
    running = False

signal.signal(signal.SIGINT, on_signal)
signal.signal(signal.SIGTERM, on_signal)


def _read_auto_mode():
    """Read auto_mode from station.conf.  Returns "off", "track", or "scan".

    Backward-compat: if only bool auto_track exists, maps True→"track", False→"off".
    """
    try:
        if os.path.exists(STATION_CONF):
            with open(STATION_CONF, "r") as f:
                cfg = json.load(f)
            # New 3-state field takes priority
            mode = cfg.get("auto_mode")
            if mode in ("off", "track", "scan"):
                return mode
            # Backward compat: bool auto_track
            at = cfg.get("auto_track")
            if at is False:
                return "off"
            return "track"
    except Exception:
        pass
    return "track"


def _run_daemon(args, logfile):
    """Daemon mode: wait for location, loop passes continuously.

    Designed to run as a systemd service on the Pi. Workflow:
      1. Wait for ~/station.conf (set from phone GPS via dashboard)
      2. Connect to rotctld
      3. Fetch TLEs, find upcoming passes
      4. Branch on auto_mode: off → idle, track → pass-by-pass, scan → scan cycle
      5. Sleep, then repeat from step 2
    """
    log("==========================================", logfile)
    log(" SatNOGS Station — DAEMON MODE", logfile)
    log("==========================================", logfile)

    # Try to open RF HAT early just for buzzer feedback during setup
    early_buzzer = None
    try:
        from buzzer_util import beep as _early_beep, READY as _BZ_READY, SETUP_DONE as _BZ_SETUP_DONE
        _early_beep(_BZ_READY)
        log("RF HAT buzzer: READY beep (hardware alive)", logfile)
        early_buzzer = _early_beep
    except Exception:
        pass

    # Wait for setup wizard completion (GPS, north cal, antenna, RF mode)
    log("Waiting for setup wizard completion (open dashboard on phone)...", logfile)
    while running:
        try:
            if os.path.exists(STATION_CONF):
                with open(STATION_CONF, "r") as f:
                    _sc = json.load(f)
                if _sc.get("setup_complete", False):
                    break
        except Exception:
            pass
        time.sleep(5)

    if not running:
        return
    log("Setup wizard complete", logfile)
    if early_buzzer:
        try:
            early_buzzer(_BZ_SETUP_DONE)
        except Exception:
            pass

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

    # Satellite profile library for per-pass radio reconfiguration
    sat_library = SatLibrary()
    log(f"Satellite library: {len(sat_library.satellites)} profiles loaded", logfile)

    # SatNOGS DB client — optional; station works fine without tokens configured
    satnogs: Optional[SatNOGSClient] = None
    if not args.no_satnogs:
        satnogs = SatNOGSClient()
        if satnogs.is_configured():
            log("SatNOGS DB: enabled — frames will be submitted automatically", logfile)
        else:
            log("SatNOGS DB: no token in station.conf — running without submission", logfile)

    try:
        while running:
            # Re-read location (user may update GPS mid-session)
            new_lat, new_lon, new_elev = load_station_conf()
            if new_lat is not None:
                sta_lat, sta_lon = new_lat, new_lon
                sta_elev = new_elev or sta_elev

            # Check auto_mode + manual track before doing anything expensive
            auto_mode = _read_auto_mode()
            _has_manual = False
            track_file = os.path.expanduser("~/.station_track")
            if os.path.exists(track_file):
                _has_manual = True
            if not args.sat and not _has_manual and auto_mode == "off":
                write_dashboard_status({
                    "streaming": False, "rssi": None, "packets": 0,
                    "freq_mhz": 0, "satellite": "", "pass_progress": 0,
                    "rf_backend": "none", "auto_mode": "off",
                })
                time.sleep(3)
                continue

            # Connect to rotctld
            rotctl = RotctlClient(args.rot_host, args.rot_port)
            try:
                rotctl.connect()
                log(f"Rotctld connected", logfile)
            except Exception as e:
                log(f"Cannot connect to rotctld: {e} — retrying in 30s", logfile)
                time.sleep(30)
                continue

            # Re-read rf_mode from station.conf (dashboard may have changed it)
            try:
                if os.path.exists(STATION_CONF):
                    with open(STATION_CONF, "r") as f:
                        _conf = json.load(f)
                    _rf_mode = _conf.get("rf_mode")
                    if _rf_mode in ("none", "cc1200", "rtlsdr", "auto"):
                        args.rf_mode = _rf_mode
            except Exception:
                pass

            # Open RF backend
            rf = create_rf_backend(args, logfile)

            # Update buzzer link (buzzer lives on the RF HAT Pico)
            buzzer.link = rf.link if isinstance(rf, RfHatManager) else None
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

            # ── SCAN MODE BRANCH ──
            # Re-read auto_mode (may have changed since loop start)
            auto_mode = _read_auto_mode()
            _scan_manual = os.path.exists(os.path.expanduser("~/.station_track"))
            if auto_mode == "scan" and not args.sat and not _scan_manual:
                # Clear any stale paused file from previous stop
                try:
                    os.remove(os.path.expanduser("~/.station_paused"))
                except FileNotFoundError:
                    pass
                log("SCAN mode — starting scan cycle", logfile)
                try:
                    def _scan_check_abort():
                        """Check for mode change, stop, or manual track override."""
                        # Manual track file takes priority (doTrackSat writes
                        # track file only — no stop file)
                        tf = os.path.expanduser("~/.station_track")
                        if os.path.exists(tf):
                            return "track"
                        # Stop file = skip current visit, pause
                        stop_file = os.path.expanduser("~/.station_stop")
                        if os.path.exists(stop_file):
                            try:
                                os.remove(stop_file)
                            except Exception:
                                pass
                            # Create paused file (consistent with track_pass behavior)
                            try:
                                with open(os.path.expanduser("~/.station_paused"), "w") as _pf:
                                    _pf.write("paused")
                            except Exception:
                                pass
                            return "stop"
                        # Mode changed away from scan
                        new_mode = _read_auto_mode()
                        if new_mode != "scan":
                            return "mode_change"
                        return None

                    # Read scan settings from station.conf (dashboard configurable)
                    _scan_hours = PREDICT_HOURS
                    _scan_min_el = MIN_ELEV
                    _scan_dwell = 30
                    try:
                        with open(STATION_CONF, "r") as f:
                            _sc = json.load(f)
                        _scan_hours = int(_sc.get("scan_hours", _scan_hours))
                        _scan_min_el = float(_sc.get("scan_min_el", _scan_min_el))
                        _scan_dwell = int(_sc.get("scan_dwell", _scan_dwell))
                    except Exception:
                        pass

                    visited, pkts, abort_reason = run_scan_cycle(
                        rotctl, rf, sat_library, buzzer, tles,
                        lat=sta_lat, lon=sta_lon, elev=sta_elev,
                        hours=_scan_hours, min_el=_scan_min_el,
                        dwell_s=_scan_dwell,
                        uhf_freq=args.uhf_freq, doppler=args.doppler,
                        check_abort=_scan_check_abort,
                        write_status=write_dashboard_status,
                        logfile=logfile)
                    log(f"SCAN cycle done: {visited} visited, {pkts} pkts, "
                        f"abort={abort_reason}", logfile)
                except Exception as e:
                    log(f"SCAN cycle error: {e}", logfile)

                # After scan cycle, cleanup and loop (will re-check mode)
                rotctl.close()
                if rf:
                    rf.close()
                # If aborted for manual track, skip the sleep
                if abort_reason == "track":
                    continue
                # If stopped by user, wait until paused file is cleared
                # (user must click a mode button or track a sat to resume)
                if abort_reason == "stop":
                    log("SCAN paused by user — waiting for resume...", logfile)
                    write_dashboard_status({
                        "streaming": False, "rssi": None, "packets": 0,
                        "freq_mhz": 0, "satellite": "", "pass_progress": 0,
                        "rf_backend": "none", "auto_mode": "scan",
                    })
                    paused = os.path.expanduser("~/.station_paused")
                    while running and os.path.exists(paused):
                        if os.path.exists(os.path.expanduser("~/.station_track")):
                            try: os.remove(paused)
                            except: pass
                            break
                        if _read_auto_mode() != "scan":
                            try: os.remove(paused)
                            except: pass
                            break
                        time.sleep(2)
                    continue
                # Normal end or mode change — short sleep then re-check
                if not running:
                    break
                log("Waiting for next scan cycle...", logfile)
                _cycle_delay = 60
                wait_end = time.time() + _cycle_delay
                while running and time.time() < wait_end:
                    if os.path.exists(os.path.expanduser("~/.station_track")):
                        break
                    if _read_auto_mode() != "scan":
                        break
                    time.sleep(3)
                continue

            # Read tracking preference — check track file first (most reliable)
            track_sat = args.sat  # CLI arg takes priority
            track_file = os.path.expanduser("~/.station_track")
            if not track_sat and os.path.exists(track_file):
                try:
                    with open(track_file, "r") as f:
                        track_sat = f.read().strip()
                    os.remove(track_file)
                    if track_sat:
                        log(f"Manual track: {track_sat}", logfile)
                        # If satellite not in AMSAT TLEs, fetch from SatNOGS DB
                        search = track_sat.upper()
                        found = any(search in n.upper() for n, l1, l2 in tles)
                        if not found:
                            log(f"Not in AMSAT, fetching from SatNOGS DB...", logfile)
                            try:
                                url = "https://db.satnogs.org/api/tle/?format=json"
                                req = urllib.request.Request(url, headers={"User-Agent": "SatNOGS-Basestation/1.0"})
                                resp = urllib.request.urlopen(req, timeout=20)
                                data = json.loads(resp.read().decode())
                                for sat in data:
                                    name = sat.get("tle0", "").strip()
                                    if name.startswith("0 "):
                                        name = name[2:]
                                    if search in name.upper():
                                        l1 = sat.get("tle1", "").strip()
                                        l2 = sat.get("tle2", "").strip()
                                        if l1.startswith("1 ") and l2.startswith("2 "):
                                            tles.append((name, l1, l2))
                                            log(f"Found {name} in SatNOGS DB", logfile)
                                            break
                                del data  # free memory
                            except Exception as e:
                                log(f"SatNOGS DB fetch failed: {e}", logfile)
                except Exception:
                    pass

            # Filter by satellite name if specified
            if track_sat:
                search = track_sat.upper()
                tles_to_search = [(n, l1, l2) for n, l1, l2 in tles if search in n.upper()]
            else:
                tles_to_search = tles

            # Find passes
            passes = find_passes(tles_to_search, hours=PREDICT_HOURS,
                                 lat=sta_lat, lon=sta_lon, elev=sta_elev)

            _skip_park = False  # set True when breaking for new track request
            if passes:
                log(f"Found {len(passes)} passes", logfile)
                for i, p in enumerate(passes):
                    if not running:
                        break

                    # Check if paused (user pressed Stop)
                    paused_file = os.path.expanduser("~/.station_paused")
                    if os.path.exists(paused_file):
                        # Check if auto mode was re-enabled or a manual track was requested
                        _cur_mode = _read_auto_mode()
                        _new_track = None
                        try:
                            if os.path.exists(STATION_CONF):
                                with open(STATION_CONF, "r") as f:
                                    _pc = json.load(f)
                                if _pc.get("track_mode") == "manual" and _pc.get("track_satellite"):
                                    _new_track = _pc["track_satellite"]
                        except Exception:
                            pass

                        if _new_track:
                            # User selected a new satellite — resume
                            os.remove(paused_file)
                            log(f"Resuming: manual track {_new_track}", logfile)
                            _skip_park = True
                            break  # re-enter outer loop with new target
                        elif _cur_mode == "off":
                            # Auto mode off, stay paused
                            time.sleep(2)
                            continue
                        elif _cur_mode == "scan":
                            # Switched to scan mode — break to outer loop
                            os.remove(paused_file)
                            _skip_park = True
                            break
                        else:
                            # Auto mode track — user pressed stop but track is enabled, resume
                            os.remove(paused_file)

                    # Check for manual track request (file-based, always honored)
                    track_file = os.path.expanduser("~/.station_track")
                    if os.path.exists(track_file):
                        try:
                            with open(track_file, "r") as f:
                                new_target = f.read().strip()
                            os.remove(track_file)
                            if new_target:
                                log(f"New track request: {new_target}", logfile)
                                _skip_park = True
                                break  # break inner loop to re-enter outer with new target
                        except Exception:
                            pass

                    # If auto mode is off and no manual request, stay idle
                    if _read_auto_mode() == "off" and not bool(track_sat):
                        write_dashboard_status({
                            "streaming": False, "rssi": None, "packets": 0,
                            "freq_mhz": 0, "satellite": "", "pass_progress": 0,
                            "rf_backend": "none", "auto_mode": "off",
                        })
                        time.sleep(5)
                        break  # break to outer loop, re-check
                    # If mode switched to scan, break to outer loop
                    if _read_auto_mode() == "scan" and not bool(track_sat):
                        _skip_park = True
                        break

                    # Skip passes already ended
                    now_utc = datetime.datetime.now(datetime.timezone.utc)
                    rise_utc = p["rise_time"].replace(tzinfo=datetime.timezone.utc)
                    set_utc = p["set_time"].replace(tzinfo=datetime.timezone.utc)
                    if now_utc >= set_utc:
                        continue  # pass already ended
                    # In auto mode, skip if more than 50% of pass elapsed
                    # (still worth joining mid-pass for higher-EL portion)
                    # In manual mode (user selected), allow joining at any point
                    is_manual = bool(track_sat)
                    if now_utc > rise_utc and not is_manual:
                        elapsed = (now_utc - rise_utc).total_seconds()
                        if elapsed > p["duration"] * 0.5:
                            log(f"Skipping {p['name']} — already {elapsed:.0f}s in", logfile)
                            continue

                    log(f"Queued: {p['name']} max EL {p['max_el']:.1f}° in "
                        f"{max(0, (rise_utc - now_utc).total_seconds())/60:.0f} min",
                        logfile)

                    # Per-pass radio reconfiguration
                    if rf:
                        pass_name = p["name"]
                        pass_norad = norad_id_from_tle(p["tle"][1])
                        profile = lookup_with_satnogs_fallback(sat_library, pass_norad, pass_name)
                        if profile:
                            tier = "C (SmartRF)" if profile.smartrf_profile else \
                                   "B (modulation)" if profile.modulation else "A (freq-only)"
                            mod_info = ""
                            if profile.modulation:
                                m = profile.modulation
                                mod_info = f" {m.format} {m.symbol_rate_bps}bps dev={m.deviation_hz}Hz BW={m.rx_bw_khz}kHz"
                                if m.sync_word:
                                    mod_info += f" sync=0x{m.sync_word}"
                            log(f"Radio config: {profile.freq_mhz:.3f} MHz — Tier {tier}{mod_info} — {profile.description}", logfile)
                            rf.configure_for_satellite(profile, sat_library.default_profile or "")
                        else:
                            log(f"No profile for {pass_name} — using default {args.uhf_freq:.3f} MHz", logfile)
                            rf.set_frequency(args.uhf_freq * 1e6)
                            if isinstance(rf, RfHatManager):
                                rf.radio_config = {
                                    "freq_mhz": args.uhf_freq,
                                    "modulation": "2-GFSK",
                                    "symbol_rate_bps": 2400,
                                    "profile": "default",
                                }

                    try:
                        track_pass(rotctl, rf, p["tle"], p, args, logfile,
                                   buzzer=buzzer,
                                   lat=sta_lat, lon=sta_lon, elev=sta_elev,
                                   satnogs=satnogs)
                    except Exception as e:
                        log(f"ERROR: pass tracking failed for {p['name']}: {e}", logfile)
                        log("Skipping to next pass", logfile)
                        continue

                    # Check for new manual track request between passes
                    stop_file = os.path.expanduser("~/.station_stop")
                    if os.path.exists(stop_file):
                        os.remove(stop_file)
                    try:
                        if os.path.exists(STATION_CONF):
                            with open(STATION_CONF, "r") as f:
                                _c2 = json.load(f)
                            if _c2.get("track_mode") == "manual" and _c2.get("track_satellite"):
                                log(f"New track request: {_c2['track_satellite']}", logfile)
                                _skip_park = True
                                break  # break inner loop to re-enter outer loop with new target
                    except Exception:
                        pass
                    # Check auto mode between passes
                    _between_mode = _read_auto_mode()
                    if _between_mode == "off":
                        log("Auto mode off — waiting", logfile)
                        break
                    if _between_mode == "scan":
                        log("Switched to scan mode — restarting", logfile)
                        _skip_park = True
                        break

                    if i < len(passes) - 1 and running:
                        rotctl.park()
                        # Flush queued frames between passes (internet may be up now)
                        if satnogs:
                            flushed = satnogs.flush_queue()
                            if flushed:
                                log(f"SatNOGS DB: flushed {flushed} queued frames", logfile)
                        # Try to update satellite library from SatNOGS DB between passes
                        try:
                            added = sat_library.update_from_satnogs()
                            if added:
                                log(f"Satellite library: added {added} new profiles from SatNOGS DB", logfile)
                        except Exception:
                            pass
            else:
                log("No passes found in prediction window", logfile)

            # Cleanup this cycle
            if _skip_park:
                log("Skipping park — new track pending", logfile)
                # Write a transitional status so dashboard knows we're still active
                # (prevents dashboard from re-sending track command during cycle restart)
                write_dashboard_status({
                    "satellite": track_sat or "", "pass_progress": 0,
                    "streaming": False, "rssi": None, "packets": 0,
                    "freq_mhz": 0, "rf_backend": "none",
                })
            else:
                rotctl.park()
                clear_dashboard_status()
            rotctl.close()
            if rf:
                rf.close()

            if not running:
                break

            # Sleep between cycles — check for manual track/mode change every 3s
            # Use shorter delay (60s) if all passes were skipped (frequent recheck)
            _cycle_delay = 60 if not passes or _skip_park else args.loop_delay
            log(f"Waiting {_cycle_delay}s for next cycle...", logfile)
            wait_end = time.time() + _cycle_delay
            track_file = os.path.expanduser("~/.station_track")
            _prev_mode = _read_auto_mode()
            while running and time.time() < wait_end:
                # Check for manual track request
                if os.path.exists(track_file):
                    log("Track request received — waking up", logfile)
                    break
                # Check for mode change (user toggled Off/Track/Scan)
                _cur = _read_auto_mode()
                if _cur != _prev_mode:
                    log(f"Mode changed: {_prev_mode} → {_cur} — waking up", logfile)
                    break
                time.sleep(3)

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

    # RF backend
    parser.add_argument("--rf-port", default=RF_HAT_PORT,
                        help="RF HAT serial port (default: /dev/serial0)")
    parser.add_argument("--rf-mode", default="auto",
                        choices=["none", "cc1200", "rtlsdr", "auto"],
                        help="RF backend: auto|cc1200|rtlsdr|none (default: auto)")
    parser.add_argument("--no-rf", action="store_true",
                        help="(deprecated) Same as --rf-mode none")
    parser.add_argument("--uhf-freq", type=float, default=UHF_FREQ_MHZ,
                        help="UHF RX frequency in MHz (default: 433.0)")
    parser.add_argument("--uhf-profile", default=UHF_PROFILE,
                        help="UHF SmartRF profile path")
    parser.add_argument("--doppler", action="store_true", default=True,
                        help="Enable real-time Doppler correction (default: on)")
    parser.add_argument("--no-doppler", dest="doppler", action="store_false",
                        help="Disable Doppler correction")

    # RTL-SDR options
    parser.add_argument("--rtlsdr-device", type=int, default=0,
                        help="RTL-SDR device index (default: 0)")
    parser.add_argument("--rtlsdr-gain", default="auto",
                        help="RTL-SDR gain: 'auto' or value in dB (default: auto)")
    parser.add_argument("--rtlsdr-rate", type=int, default=240000,
                        help="RTL-SDR sample rate in Hz (default: 240000)")

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

    # SatNOGS DB
    parser.add_argument("--no-satnogs", action="store_true",
                        help="Disable SatNOGS DB frame submission")

    # Serial mode (raw bit streaming for USP/AX.100 protocols)
    parser.add_argument("--serial-mode", action="store_true",
                        help="Use CC1200 synchronous serial mode + PIO for raw "
                             "bitstream capture (required for USP/AX.100 decoding)")
    parser.add_argument("--decoder", default="usp",
                        choices=["usp", "ax100", "none"],
                        help="Bitstream decoder for --serial-mode (default: usp)")

    # Output
    parser.add_argument("--log", default=None,
                        help="Log file path (appended)")

    args = parser.parse_args()

    # Deprecated --no-rf alias
    if args.no_rf:
        args.rf_mode = "none"

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
    log(f"RF mode: {args.rf_mode}", logfile)
    if args.rf_mode != "none":
        log(f"UHF RX:  {args.uhf_freq:.3f} MHz", logfile)
        log(f"Doppler: {'ON' if args.doppler else 'OFF'}", logfile)

    # Satellite profile library
    sat_library = SatLibrary()
    log(f"Satellite library: {len(sat_library.satellites)} profiles loaded", logfile)

    # SatNOGS DB client
    satnogs: Optional[SatNOGSClient] = None
    if not args.no_satnogs:
        satnogs = SatNOGSClient()
        if satnogs.is_configured():
            log("SatNOGS DB: enabled — frames will be submitted automatically", logfile)
        else:
            log("SatNOGS DB: no token in station.conf — running without submission", logfile)

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

    # Open RF backend
    rf: Optional[RfBackend] = create_rf_backend(args, logfile)

    # Load TX data if specified
    tx_data = None
    if args.tx:
        try:
            with open(args.tx, "rb") as f:
                tx_data = f.read()
            log(f"TX payload loaded: {len(tx_data)} bytes from {args.tx}", logfile)
        except Exception as e:
            log(f"WARNING: Failed to load TX file: {e}", logfile)

    # Initialize buzzer (uses RF HAT UART; no-op if rf is not CC1200)
    cc1200_link = rf.link if isinstance(rf, RfHatManager) else None
    buzzer = Buzzer(cc1200_link)
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

            # Per-pass radio reconfiguration
            if rf:
                pass_name = p['name']
                pass_norad = norad_id_from_tle(p['tle'][1])
                profile = lookup_with_satnogs_fallback(sat_library, pass_norad, pass_name)
                if profile:
                    tier = "C (SmartRF)" if profile.smartrf_profile else \
                           "B (modulation)" if profile.modulation else "A (freq-only)"
                    mod_info = ""
                    if profile.modulation:
                        m = profile.modulation
                        mod_info = f" {m.format} {m.symbol_rate_bps}bps dev={m.deviation_hz}Hz BW={m.rx_bw_khz}kHz"
                        if m.sync_word:
                            mod_info += f" sync=0x{m.sync_word}"
                    log(f"Radio config: {profile.freq_mhz:.3f} MHz — Tier {tier}{mod_info} — {profile.description}", logfile)
                    rf.configure_for_satellite(profile, sat_library.default_profile or "")
                else:
                    log(f"No profile for {pass_name} — using default {args.uhf_freq:.3f} MHz", logfile)
                    rf.set_frequency(args.uhf_freq * 1e6)
                    if isinstance(rf, RfHatManager):
                        rf.radio_config = {
                            "freq_mhz": args.uhf_freq,
                            "modulation": "2-GFSK",
                            "symbol_rate_bps": 2400,
                            "profile": "default",
                        }

            pkt_count = track_pass(rotctl, rf, p["tle"], p, args, logfile,
                                   buzzer=buzzer,
                                   lat=sta_lat, lon=sta_lon, elev=sta_elev,
                                   satnogs=satnogs)

            # TX on VHF after pass if requested (CC1200 only)
            if tx_data and isinstance(rf, RfHatManager) and running:
                log("Transmitting VHF uplink packet...", logfile)
                rf.tx_vhf_packet(tx_data, args.vhf_freq, args.vhf_profile)

            # Park between passes + flush offline queue
            if i < len(passes) - 1 and running:
                log("Parking rotator...", logfile)
                rotctl.park()
                if satnogs:
                    flushed = satnogs.flush_queue()
                    if flushed:
                        log(f"SatNOGS DB: flushed {flushed} queued frames", logfile)

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
