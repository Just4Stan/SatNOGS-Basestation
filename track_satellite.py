#!/usr/bin/env python3
"""
Real satellite tracker — computes live AZ/EL from TLE data and commands
the rotator via direct USB serial or rotctld (hamlib) over the network.

Usage (USB serial — laptop directly to Pico, no Pi needed):
    python3 track_satellite.py --serial auto                # auto-detect port
    python3 track_satellite.py --serial /dev/cu.usbmodem1401 --sat "ISS"
    python3 track_satellite.py --serial auto --list         # show upcoming passes

Usage (rotctld — via Pi):
    python3 track_satellite.py                    # auto-pick next visible pass
    python3 track_satellite.py --sat "ISS"        # track ISS specifically
    python3 track_satellite.py --host 10.72.3.105 # rotctld on remote Pi

Requires: pip install ephem pyserial
"""

import ephem
import socket
import time
import math
import sys
import os
import argparse
import urllib.request
import json
from datetime import datetime, timezone

try:
    import serial as pyserial
except ImportError:
    pyserial = None

# --- Station location (KU Leuven Campus Geel area) ---
STATION_LAT  = "51.1"      # degrees N
STATION_LON  = "4.97"      # degrees E
STATION_ELEV = 25           # meters ASL

# --- rotctld connection ---
ROTCTLD_HOST = "localhost"
ROTCTLD_PORT = 4533

# --- Tracking parameters ---
UPDATE_HZ    = 10           # target update rate
MIN_ELEV     = 5.0          # minimum elevation to consider a pass (degrees)
PREDICT_HOURS = 12          # how far ahead to search for passes

# --- TLE sources ---
TLE_GROUPS = {
    "stations":  "Space Stations",
    "weather":   "Weather",
    "amateur":   "Amateur",
    "noaa":      "NOAA",
    "active":    "Active (all)",
    "visual":    "Visual",
    "science":   "Science",
    "starlink":  "Starlink",
    "oneweb":    "OneWeb",
    "resource":  "Earth Resources",
    "sarsat":    "Search & Rescue",
    "geo":       "Geostationary",
    "gpz":       "GPS Operational",
    "galileo":   "Galileo",
    "cubesat":   "CubeSats",
    "military":  "Military",
    "radar":     "Radar Calibration",
    "education": "Education",
}
DEFAULT_GROUPS = ["stations", "weather", "amateur", "noaa"]
CELESTRAK_URL = "https://celestrak.org/NORAD/elements/gp.php?GROUP={}&FORMAT=TLE"


def fetch_tles(groups=None):
    """Fetch TLE data from CelesTrak. Returns list of (name, line1, line2)."""
    if groups is None:
        groups = DEFAULT_GROUPS
    tles = []
    seen = set()  # deduplicate by NORAD catalog number (line1[2:7])
    for group in groups:
        label = TLE_GROUPS.get(group, group)
        url = CELESTRAK_URL.format(group)
        try:
            req = urllib.request.Request(url, headers={"User-Agent": "SatNOGS-Tracker/1.0"})
            resp = urllib.request.urlopen(req, timeout=15)
            lines = resp.read().decode().strip().split("\n")
            lines = [l.strip() for l in lines if l.strip()]
            count = 0
            for i in range(0, len(lines) - 2, 3):
                name = lines[i].strip()
                l1 = lines[i+1].strip()
                l2 = lines[i+2].strip()
                if l1.startswith("1 ") and l2.startswith("2 "):
                    norad_id = l1[2:7].strip()
                    if norad_id not in seen:
                        seen.add(norad_id)
                        tles.append((name, l1, l2))
                        count += 1
            print(f"  Fetched {count} sats from {label}")
        except Exception as e:
            print(f"  Warning: failed to fetch {label}: {e}")
    return tles


def make_observer(lat=None, lon=None, elev=None):
    """Create a PyEphem observer for our station."""
    obs = ephem.Observer()
    obs.lat = str(lat if lat is not None else STATION_LAT)
    obs.lon = str(lon if lon is not None else STATION_LON)
    obs.elevation = elev if elev is not None else STATION_ELEV
    obs.horizon = str(MIN_ELEV)
    return obs


def find_passes(tles, hours=12, max_results=20, lat=None, lon=None, elev=None):
    """Find upcoming satellite passes sorted by start time."""
    obs = make_observer(lat, lon, elev)
    now = ephem.now()
    passes = []

    for name, l1, l2 in tles:
        try:
            sat = ephem.readtle(name, l1, l2)
            obs.date = now
            # Search for up to 3 passes per satellite
            for _ in range(3):
                try:
                    info = obs.next_pass(sat)
                    # info = (rise_time, rise_az, max_alt_time, max_alt, set_time, set_az)
                    if info[0] is None or info[4] is None:
                        break
                    rise_t = info[0]
                    max_el = math.degrees(float(info[3]))
                    set_t = info[4]
                    duration = (set_t - rise_t) * 24 * 3600  # seconds

                    if max_el >= MIN_ELEV and duration > 10:
                        # Check if within our prediction window
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

                    # Move past this pass to find the next one
                    obs.date = info[4] + ephem.minute
                except Exception:
                    break
            obs.date = now  # Reset for next satellite
        except Exception:
            continue

    passes.sort(key=lambda p: p["rise_time"])
    return passes[:max_results]


class RotctlBackend:
    """Rotator control via rotctld TCP."""
    def __init__(self, host, port):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.settimeout(5)
        self.sock.connect((host, port))

    def _cmd(self, cmd):
        self.sock.sendall((cmd + "\n").encode())
        time.sleep(0.05)
        try:
            return self.sock.recv(1024).decode().strip()
        except socket.timeout:
            return ""

    def set_position(self, az, el):
        az = az % 360
        el = max(0, min(90, el))
        self._cmd(f"P {az:.1f} {el:.1f}")

    def park(self):
        self._cmd("P 0.0 0.0")

    def get_position(self):
        resp = self._cmd("p")
        lines = resp.split("\n")
        if len(lines) >= 2:
            try:
                return float(lines[0]), float(lines[1])
            except ValueError:
                pass
        return None, None

    def close(self):
        try:
            self.sock.close()
        except Exception:
            pass


class SerialBackend:
    """Rotator control via direct USB serial (EasyComm)."""
    def __init__(self, port):
        if pyserial is None:
            print("  ERROR: pyserial required for --serial mode")
            print("  Install with: pip install pyserial")
            sys.exit(1)
        self.ser = pyserial.Serial(port, 115200, timeout=0.05)
        time.sleep(2)  # wait for Pico to reset
        while self.ser.in_waiting:
            self.ser.readline()

    def set_position(self, az, el):
        az = az % 360
        el = max(0, min(90, el))
        self.ser.reset_input_buffer()
        self.ser.write(f"AZ{az:.1f} EL{el:.1f}\n".encode())

    def park(self):
        self.ser.reset_input_buffer()
        self.ser.write(b"PARK\n")

    def get_position(self):
        self.ser.reset_input_buffer()
        self.ser.write(b"AZ\n")
        deadline = time.time() + 0.05
        while time.time() < deadline:
            if self.ser.in_waiting:
                line = self.ser.readline().decode("utf-8", errors="replace").strip()
                if line.startswith("AZ") and "EL" in line:
                    parts = line.replace("AZ", "").replace("EL", " ").split()
                    if len(parts) >= 2:
                        try:
                            return float(parts[0]), float(parts[1])
                        except ValueError:
                            pass
        return None, None

    def close(self):
        try:
            self.ser.write(b"PARK\n")
            time.sleep(0.5)
            self.ser.close()
        except Exception:
            pass


def track_pass(rotator, tle, pass_info, lat=None, lon=None, elev=None):
    """Track a satellite pass in real-time."""
    name, l1, l2 = tle
    sat = ephem.readtle(name, l1, l2)
    obs = make_observer(lat, lon, elev)
    obs.horizon = "0"  # Track down to 0 degrees during a pass

    print()
    print(f"  ========================================")
    print(f"   TRACKING: {name}")
    print(f"  ========================================")
    print(f"  Max EL:    {pass_info['max_el']:.1f} deg")
    print(f"  Duration:  {pass_info['duration']:.0f}s")
    print(f"  Rise AZ:   {pass_info['rise_az']:.1f} deg")
    print(f"  Set AZ:    {pass_info['set_az']:.1f} deg")
    print()

    # Pre-position: slew to rise azimuth
    rise_az = pass_info["rise_az"]
    print(f"  [{time.strftime('%H:%M:%S')}] Slewing to AOS position: AZ {rise_az:.1f}")
    rotator.set_position(rise_az, 0)

    # Wait for pass to start (or start immediately if already in progress)
    now_utc = datetime.now(timezone.utc)
    rise_utc = pass_info["rise_time"].replace(tzinfo=timezone.utc)
    wait = (rise_utc - now_utc).total_seconds()
    if wait > 0:
        print(f"  [{time.strftime('%H:%M:%S')}] Waiting {wait:.0f}s for AOS...")
        # Wait but keep checking
        while wait > 0.5:
            time.sleep(min(wait, 1.0))
            now_utc = datetime.now(timezone.utc)
            wait = (rise_utc - now_utc).total_seconds()

    print(f"  [{time.strftime('%H:%M:%S')}] ** AOS — tracking started **")
    print()
    print(f"  {'':>8s}  {'--- Target ---':^16s}  {'--- Actual ---':^16s}  {'Error':^10s}")
    print(f"  {'':>8s}  {'AZ':>7s}  {'EL':>6s}  {'AZ':>7s}  {'EL':>6s}  {'AZ':>5s} {'EL':>4s}")
    print(f"  {'':>8s}  {'-------':>7s}  {'------':>6s}  {'-------':>7s}  {'------':>6s}  {'-----':>5s} {'----':>4s}")
    dt = 1.0 / UPDATE_HZ
    tick = 0
    act_az, act_el = None, None

    set_utc = pass_info["set_time"].replace(tzinfo=timezone.utc)

    try:
        while True:
            now_utc = datetime.now(timezone.utc)
            if now_utc >= set_utc:
                break

            # Compute current satellite position
            obs.date = ephem.Date(now_utc)
            sat.compute(obs)
            az = math.degrees(float(sat.az))
            el = math.degrees(float(sat.alt))

            # Send to rotator
            if el >= 0:
                rotator.set_position(az, el)

            # Read actual position every 3rd tick (~3 Hz)
            if tick % 3 == 0:
                rz, re = rotator.get_position()
                if rz is not None:
                    act_az, act_el = rz, re

            # Compute error
            err_az_s, err_el_s = "  -  ", " -  "
            if act_az is not None:
                err_az = az - act_az
                # Handle wrap-around
                if err_az > 180:
                    err_az -= 360
                elif err_az < -180:
                    err_az += 360
                err_el = el - act_el
                err_az_s = f"{err_az:+5.1f}"
                err_el_s = f"{err_el:+4.1f}"

            # Display
            remaining = (set_utc - now_utc).total_seconds()
            act_az_s = f"{act_az:7.1f}" if act_az is not None else "    ---"
            act_el_s = f"{act_el:6.1f}" if act_el is not None else "   ---"
            print(f"\r  T-{remaining:>4.0f}s  {az:7.1f}  {el:6.1f}  {act_az_s}  {act_el_s}  {err_az_s} {err_el_s}", end="", flush=True)

            tick += 1
            next_tick = time.time() + dt
            sleep_time = next_tick - time.time()
            if sleep_time > 0:
                time.sleep(sleep_time)

    except KeyboardInterrupt:
        print()
        print(f"  [{time.strftime('%H:%M:%S')}] Tracking interrupted")
        return False

    print()
    print(f"  [{time.strftime('%H:%M:%S')}] ** LOS — pass complete **")
    return True


def load_station_conf():
    """Load station location from ~/station.conf if it exists."""
    conf = os.path.expanduser("~/station.conf")
    try:
        if os.path.exists(conf):
            with open(conf, "r") as f:
                cfg = json.load(f)
            return cfg.get("lat"), cfg.get("lon"), cfg.get("elev")
    except Exception:
        pass
    return None, None, None


def auto_detect_serial():
    """Try to find the Pico serial port."""
    import glob
    # macOS: Pico shows up as /dev/cu.usbmodem*
    for p in sorted(glob.glob("/dev/cu.usbmodem*")):
        return p
    # Linux: Pico shows up as /dev/ttyACM*
    for p in sorted(glob.glob("/dev/ttyACM*")):
        return p
    return None


def main():
    parser = argparse.ArgumentParser(
        description="Real satellite tracker — direct USB serial or via rotctld")
    parser.add_argument("--serial", default=None, metavar="PORT",
                        help="Direct USB serial port to Pico (e.g. /dev/cu.usbmodem1401). "
                             "Auto-detects if set to 'auto'.")
    parser.add_argument("--host", default=ROTCTLD_HOST, help="rotctld host (default: localhost)")
    parser.add_argument("--port", type=int, default=ROTCTLD_PORT, help="rotctld port (default: 4533)")
    parser.add_argument("--sat", default=None, help="Satellite name to track (partial match)")
    parser.add_argument("--list", action="store_true", help="List upcoming passes and exit")
    parser.add_argument("--continuous", action="store_true", help="Track passes continuously")
    parser.add_argument("--groups", default=None, metavar="G1,G2,...",
                        help="Comma-separated TLE groups to fetch (default: stations,weather,amateur,noaa). "
                             "Use --groups all for everything, or --groups-list to see options.")
    parser.add_argument("--groups-list", action="store_true", help="List available TLE groups and exit")
    parser.add_argument("--lat", type=float, default=float(STATION_LAT),
                        help="Station latitude in degrees N (default: 51.1)")
    parser.add_argument("--lon", type=float, default=float(STATION_LON),
                        help="Station longitude in degrees E (default: 4.97)")
    parser.add_argument("--elev", type=float, default=STATION_ELEV,
                        help="Station elevation in meters ASL (default: 25)")
    args = parser.parse_args()

    if args.groups_list:
        print()
        print(f"  Available TLE groups (use with --groups):")
        print(f"  {'Key':<12s}  Description")
        print(f"  {'---':<12s}  -----------")
        for key, desc in TLE_GROUPS.items():
            default = " (default)" if key in DEFAULT_GROUPS else ""
            print(f"  {key:<12s}  {desc}{default}")
        print()
        print(f"  Examples:")
        print(f"    --groups stations,cubesat,science")
        print(f"    --groups active              # ~8000 sats, slow but complete")
        print(f"    --groups all                  # every group listed above")
        print()
        sys.exit(0)

    # Determine connection mode
    serial_port = args.serial
    if serial_port == "auto":
        serial_port = auto_detect_serial()
        if serial_port is None:
            print("  ERROR: No Pico serial port found. Specify with --serial /dev/...")
            sys.exit(1)

    # Resolve station location: CLI > ~/station.conf > hardcoded defaults
    conf_lat, conf_lon, conf_elev = load_station_conf()
    if args.lat == float(STATION_LAT) and conf_lat is not None:
        args.lat = conf_lat
    if args.lon == float(STATION_LON) and conf_lon is not None:
        args.lon = conf_lon
    if args.elev == STATION_ELEV and conf_elev is not None:
        args.elev = conf_elev

    print()
    print("  ========================================")
    print("   SatNOGS Ground Station — Sat Tracker")
    print("  ========================================")
    print()
    print(f"  Station:   {args.lat}N, {args.lon}E, {args.elev:.0f}m")
    if serial_port:
        print(f"  Rotator:   USB serial → {serial_port}")
    else:
        print(f"  Rotator:   rotctld → {args.host}:{args.port}")
    print(f"  Min elev:  {MIN_ELEV} deg")
    print()

    # Parse TLE groups
    groups = None
    if args.groups:
        if args.groups.lower() == "all":
            groups = list(TLE_GROUPS.keys())
        else:
            groups = [g.strip() for g in args.groups.split(",")]
            for g in groups:
                if g not in TLE_GROUPS:
                    print(f"  Warning: unknown group '{g}' (use --groups-list to see options)")

    # Fetch TLEs
    print("  Fetching TLE data...")
    tles = fetch_tles(groups)
    if not tles:
        print("  ERROR: No TLE data available!")
        sys.exit(1)
    print(f"  Total: {len(tles)} satellites loaded")
    print()

    # Filter by name if specified
    if args.sat:
        search = args.sat.upper()
        filtered = [(n, l1, l2) for n, l1, l2 in tles if search in n.upper()]
        if not filtered:
            print(f"  No satellite matching '{args.sat}' found!")
            print(f"  Available satellites containing similar names:")
            for n, _, _ in tles:
                if any(w in n.upper() for w in search.split()):
                    print(f"    - {n}")
            sys.exit(1)
        tles_to_search = filtered
        print(f"  Searching passes for {len(filtered)} matching satellite(s)")
    else:
        tles_to_search = tles

    # Find upcoming passes
    print(f"  Computing passes for next {PREDICT_HOURS}h...")
    passes = find_passes(tles_to_search, hours=PREDICT_HOURS,
                         lat=args.lat, lon=args.lon, elev=args.elev)

    if not passes:
        print("  No passes found in the prediction window.")
        print("  Try increasing PREDICT_HOURS or lowering MIN_ELEV.")
        sys.exit(0)

    # List mode
    if args.list:
        print()
        print(f"  {'#':>3s}  {'Satellite':<24s}  {'Rise (UTC)':>12s}  {'Max EL':>7s}  {'Duration':>8s}  {'AZ range'}")
        print(f"  {'---':>3s}  {'------------------------':<24s}  {'----------':>12s}  {'------':>7s}  {'--------':>8s}  {'--------'}")
        for i, p in enumerate(passes):
            rise_str = p["rise_time"].strftime("%H:%M:%S")
            dur_str = f"{p['duration']:.0f}s"
            az_range = f"{p['rise_az']:.0f}->{p['set_az']:.0f}"
            print(f"  {i+1:>3d}  {p['name']:<24s}  {rise_str:>12s}  {p['max_el']:>6.1f}°  {dur_str:>8s}  {az_range}")
        print()
        sys.exit(0)

    # Connect to rotator
    if serial_port:
        print(f"  Opening serial port {serial_port}...")
        try:
            rotator = SerialBackend(serial_port)
            cur_az, cur_el = rotator.get_position()
            if cur_az is not None:
                print(f"  Connected! Current position: AZ={cur_az} EL={cur_el}")
            else:
                print(f"  Connected! (position readback pending)")
        except Exception as e:
            print(f"  ERROR: Cannot open serial port: {e}")
            sys.exit(1)
    else:
        print(f"  Connecting to rotctld at {args.host}:{args.port}...")
        try:
            rotator = RotctlBackend(args.host, args.port)
            cur_az, cur_el = rotator.get_position()
            print(f"  Connected! Current position: AZ={cur_az} EL={cur_el}")
        except Exception as e:
            print(f"  ERROR: Cannot connect to rotctld: {e}")
            sys.exit(1)

    # Track passes
    try:
        pass_idx = 0
        while pass_idx < len(passes):
            p = passes[pass_idx]
            now_utc = datetime.now(timezone.utc)
            rise_utc = p["rise_time"].replace(tzinfo=timezone.utc)
            wait = (rise_utc - now_utc).total_seconds()

            print()
            print(f"  Next: {p['name']} — max EL {p['max_el']:.1f}°, {p['duration']:.0f}s")
            if wait > 60:
                print(f"  Pass starts in {wait/60:.1f} min ({p['rise_time'].strftime('%H:%M:%S')} UTC)")

            track_pass(rotator, p["tle"], p,
                       lat=args.lat, lon=args.lon, elev=args.elev)

            pass_idx += 1
            if pass_idx < len(passes) and not args.continuous:
                print()
                print(f"  Park and wait for next pass? (Ctrl+C to quit)")
                rotator.park()
                time.sleep(3)

        print()
        print("  All predicted passes complete!")

    except KeyboardInterrupt:
        print()
        print(f"  [{time.strftime('%H:%M:%S')}] Stopped — parking rotator")
        try:
            rotator.park()
        except Exception:
            pass

    finally:
        rotator.close()
        print("  Done.")


if __name__ == "__main__":
    main()
