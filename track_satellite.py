#!/usr/bin/env python3
"""
Real satellite tracker — computes live AZ/EL from TLE data and commands
the rotator via rotctld (hamlib) over the network.

Usage:
    python3 track_satellite.py                    # auto-pick next visible pass
    python3 track_satellite.py --sat "ISS"        # track ISS specifically
    python3 track_satellite.py --list             # show upcoming passes
    python3 track_satellite.py --host 10.72.3.105 # rotctld on remote Pi

Requires: pip install ephem
"""

import ephem
import socket
import time
import math
import sys
import argparse
import urllib.request
import json
from datetime import datetime, timezone

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
TLE_URLS = [
    ("https://celestrak.org/NORAD/elements/gp.php?GROUP=stations&FORMAT=TLE", "Space Stations"),
    ("https://celestrak.org/NORAD/elements/gp.php?GROUP=weather&FORMAT=TLE", "Weather"),
    ("https://celestrak.org/NORAD/elements/gp.php?GROUP=amateur&FORMAT=TLE", "Amateur"),
    ("https://celestrak.org/NORAD/elements/gp.php?GROUP=noaa&FORMAT=TLE", "NOAA"),
]


def fetch_tles():
    """Fetch TLE data from CelesTrak. Returns list of (name, line1, line2)."""
    tles = []
    for url, group in TLE_URLS:
        try:
            req = urllib.request.Request(url, headers={"User-Agent": "SatNOGS-Tracker/1.0"})
            resp = urllib.request.urlopen(req, timeout=10)
            lines = resp.read().decode().strip().split("\n")
            lines = [l.strip() for l in lines if l.strip()]
            for i in range(0, len(lines) - 2, 3):
                name = lines[i].strip()
                l1 = lines[i+1].strip()
                l2 = lines[i+2].strip()
                if l1.startswith("1 ") and l2.startswith("2 "):
                    tles.append((name, l1, l2))
            print(f"  Fetched {len(lines)//3} sats from {group}")
        except Exception as e:
            print(f"  Warning: failed to fetch {group}: {e}")
    return tles


def make_observer():
    """Create a PyEphem observer for our station."""
    obs = ephem.Observer()
    obs.lat = STATION_LAT
    obs.lon = STATION_LON
    obs.elevation = STATION_ELEV
    obs.horizon = str(MIN_ELEV)
    return obs


def find_passes(tles, hours=12, max_results=20):
    """Find upcoming satellite passes sorted by start time."""
    obs = make_observer()
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


def connect_rotctld(host, port):
    """Connect to rotctld and return socket."""
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.settimeout(5)
    s.connect((host, port))
    return s


def rotctld_cmd(sock, cmd):
    """Send command to rotctld and return response."""
    sock.sendall((cmd + "\n").encode())
    time.sleep(0.05)
    try:
        return sock.recv(1024).decode().strip()
    except socket.timeout:
        return ""


def set_position(sock, az, el):
    """Command rotator to move to AZ/EL."""
    # hamlib expects 0-360 for AZ
    az = az % 360
    el = max(0, min(90, el))
    rotctld_cmd(sock, f"P {az:.1f} {el:.1f}")


def get_position(sock):
    """Query current position from rotctld."""
    resp = rotctld_cmd(sock, "p")
    lines = resp.split("\n")
    if len(lines) >= 2:
        try:
            return float(lines[0]), float(lines[1])
        except ValueError:
            pass
    return None, None


def track_pass(sock, tle, pass_info):
    """Track a satellite pass in real-time."""
    name, l1, l2 = tle
    sat = ephem.readtle(name, l1, l2)
    obs = make_observer()
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
    set_position(sock, rise_az, 0)

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
    dt = 1.0 / UPDATE_HZ
    tick = 0
    last_pos_str = ""

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
                set_position(sock, az, el)

            # Read position every 5th tick
            if tick % 5 == 0:
                cur_az, cur_el = get_position(sock)
                if cur_az is not None:
                    last_pos_str = f"AZ{cur_az:>7.1f} EL{cur_el:>5.1f}"

            # Display
            bar_el = int(max(0, el) / 90 * 20)
            bar = "|" + "#" * bar_el + " " * (20 - bar_el) + "|"
            remaining = (set_utc - now_utc).total_seconds()
            phase = "AOS" if remaining > pass_info["duration"] * 0.85 else \
                    ("LOS" if remaining < pass_info["duration"] * 0.15 else "TRK")
            print(f"\r  {phase}  AZ{az:>7.1f}  EL{el:>5.1f}  {bar}  {last_pos_str:<22s}  T-{remaining:>5.0f}s", end="", flush=True)

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


def immediate_track(sock, tle):
    """Track a satellite that's currently above the horizon (or do a demo pass)."""
    name, l1, l2 = tle
    sat = ephem.readtle(name, l1, l2)
    obs = make_observer()
    obs.horizon = "0"

    obs.date = ephem.now()
    sat.compute(obs)
    az = math.degrees(float(sat.az))
    el = math.degrees(float(sat.alt))

    if el > 0:
        print(f"  {name} is above horizon! AZ={az:.1f} EL={el:.1f}")
        print(f"  Tracking live...")
        # Find when it sets
        obs.horizon = str(MIN_ELEV)
        try:
            info = obs.next_pass(sat)
            if info[4]:
                pass_info = {
                    "rise_time": datetime.now(timezone.utc),
                    "set_time": ephem.Date(info[4]).datetime(),
                    "max_el": el,
                    "duration": (info[4] - ephem.now()) * 24 * 3600,
                    "rise_az": az,
                    "set_az": math.degrees(float(info[5])) if info[5] else az,
                }
                return track_pass(sock, tle, pass_info)
        except Exception:
            pass
    return False


def main():
    parser = argparse.ArgumentParser(description="Real satellite tracker via rotctld")
    parser.add_argument("--host", default=ROTCTLD_HOST, help="rotctld host")
    parser.add_argument("--port", type=int, default=ROTCTLD_PORT, help="rotctld port")
    parser.add_argument("--sat", default=None, help="Satellite name to track (partial match)")
    parser.add_argument("--list", action="store_true", help="List upcoming passes and exit")
    parser.add_argument("--continuous", action="store_true", help="Track passes continuously")
    args = parser.parse_args()

    print()
    print("  ========================================")
    print("   SatNOGS Ground Station — Sat Tracker")
    print("  ========================================")
    print()
    print(f"  Station:   {STATION_LAT}N, {STATION_LON}E, {STATION_ELEV}m")
    print(f"  rotctld:   {args.host}:{args.port}")
    print(f"  Min elev:  {MIN_ELEV} deg")
    print()

    # Fetch TLEs
    print("  Fetching TLE data...")
    tles = fetch_tles()
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
    passes = find_passes(tles_to_search, hours=PREDICT_HOURS)

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

    # Connect to rotctld
    print(f"  Connecting to rotctld at {args.host}:{args.port}...")
    try:
        sock = connect_rotctld(args.host, args.port)
        cur_az, cur_el = get_position(sock)
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

            track_pass(sock, p["tle"], p)

            pass_idx += 1
            if pass_idx < len(passes) and not args.continuous:
                print()
                print(f"  Park and wait for next pass? (Ctrl+C to quit)")
                set_position(sock, 0, 0)
                time.sleep(3)

        print()
        print("  All predicted passes complete!")

    except KeyboardInterrupt:
        print()
        print(f"  [{time.strftime('%H:%M:%S')}] Stopped — parking rotator")
        try:
            set_position(sock, 0, 0)
        except Exception:
            pass

    finally:
        try:
            sock.close()
        except Exception:
            pass
        print("  Done.")


if __name__ == "__main__":
    main()
