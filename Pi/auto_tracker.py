#!/usr/bin/env python3
"""
Auto Tracker — cycle through all receivable satellites near their peak.

Instead of tracking one satellite for its entire pass, this scheduler
builds a timeline of all upcoming passes and visits each satellite near
its maximum elevation point (best link margin, shortest range, strongest
signal). After a configurable dwell time it slews to the next target.

Two modes:
  normal  — dwell ~60% of pass duration centred on peak, full RF capture
  test    — 30s dwell at peak only, fast cycling to test as many sats as
            possible in one session

Usage:
    python3 auto_tracker.py                     # normal mode
    python3 auto_tracker.py --mode test         # fast test mode
    python3 auto_tracker.py --mode test --list  # preview schedule
    python3 auto_tracker.py --hours 4           # look 4 hours ahead
    python3 auto_tracker.py --min-el 15         # skip low passes
"""

import os
import sys
import math
import json
import time
import signal
import socket
import argparse
import datetime
import tempfile
from pathlib import Path
from typing import Optional, List, Tuple

try:
    import ephem
except ImportError:
    print("ERROR: PyEphem required.  pip install ephem")
    sys.exit(1)

sys.path.insert(0, str(Path(__file__).parent))
from rf_hat import (
    CC1200Link, RADIO_UHF,
    STATE_RX, STATE_IDLE,
    doppler_shift,
)
from rf_backend import RfBackend, RfMetrics
from buzzer import Buzzer
from sat_library import SatLibrary, SatProfile, ModulationConfig, pick_best_uhf_transmitter

# Re-use heavy helpers from station.py
from station import (
    RotctlClient, RfHatManager, create_rf_backend,
    load_station_conf, fetch_tles, norad_id_from_tle,
    write_dashboard_status, clear_dashboard_status, record_pass_history,
    lookup_with_satnogs_fallback, log, make_observer,
    STATION_LAT, STATION_LON, STATION_ELEV,
    ROTCTLD_HOST, ROTCTLD_PORT,
    RF_HAT_PORT, UHF_FREQ_MHZ, UHF_PROFILE,
    VHF_FREQ_MHZ, VHF_PROFILE,
    STATUS_FILE, PREDICT_HOURS, MIN_ELEV,
)

# ---------------------------------------------------------------------------
# Globals
# ---------------------------------------------------------------------------
running = True

def _on_signal(sig, frame):
    global running
    running = False

signal.signal(signal.SIGINT, _on_signal)
signal.signal(signal.SIGTERM, _on_signal)


# ---------------------------------------------------------------------------
# Pass scheduler — build an optimal visit timeline
# ---------------------------------------------------------------------------
def find_all_passes(tles, hours=4, min_el=10.0, lat=None, lon=None, elev=None):
    """Find ALL upcoming passes for all TLE entries.  Returns a list of dicts
    sorted by max-elevation time, each with full orbital data."""
    obs = ephem.Observer()
    obs.lat = str(lat or STATION_LAT)
    obs.lon = str(lon or STATION_LON)
    obs.elevation = float(elev or STATION_ELEV)
    obs.horizon = "0"   # compute from real horizon so we get full trajectory
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
                # Skip passes already ended
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

    # Sort by max-elevation time (when we want to be pointing at each sat)
    passes.sort(key=lambda p: p["max_time"])
    return passes


def build_schedule(passes, mode="normal", slew_time=8.0):
    """Build a non-overlapping visit schedule from candidate passes.

    Each visit is a time window [start, end] centred on the satellite's
    peak elevation.  We greedily assign visits in max-el-time order,
    skipping any that would overlap with an already-scheduled visit
    (including slew time).

    Args:
        passes: sorted list of pass dicts from find_all_passes()
        mode: "normal" (60% of pass) or "test" (30s window)
        slew_time: seconds to allow for rotator slew between targets

    Returns list of visit dicts with start/end times and parent pass data.
    """
    schedule = []

    for p in passes:
        max_t = p["max_time"]
        dur = p["duration"]

        if mode == "test":
            # Test mode: 30s window centred on peak
            half = 15.0
        else:
            # Normal mode: 60% of pass centred on peak
            half = dur * 0.30

        visit_start = max_t - datetime.timedelta(seconds=half)
        visit_end = max_t + datetime.timedelta(seconds=half)

        # Clamp to actual pass bounds
        if visit_start < p["rise_time"]:
            visit_start = p["rise_time"]
        if visit_end > p["set_time"]:
            visit_end = p["set_time"]

        # Check overlap with previous visit (+ slew margin)
        if schedule:
            prev_end = schedule[-1]["end"]
            gap = (visit_start - prev_end).total_seconds()
            if gap < slew_time:
                # Overlap — skip this visit (greedy: keep earlier one)
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


# ---------------------------------------------------------------------------
# Visit execution — track one satellite for its scheduled window
# ---------------------------------------------------------------------------
def execute_visit(visit, rotctl, rf, args, sat_library, logfile=None,
                  buzzer=None, lat=None, lon=None, elev=None):
    """Track a single satellite for its scheduled visit window.

    Slews to the satellite's predicted AZ/EL, configures CC1200,
    captures packets until the visit window ends.
    """
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
    if rf:
        profile = lookup_with_satnogs_fallback(sat_library, norad_id, name)
        if profile:
            log(f"  Radio: {profile.freq_mhz:.3f} MHz — {profile.description}", logfile)
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

    # Wait for visit window to start
    wait = (visit_start - datetime.datetime.now(datetime.timezone.utc)).total_seconds()
    if wait > 0:
        log(f"  Waiting {wait:.0f}s for visit window...", logfile)
        _last_status_write = 0
        while wait > 0.5 and running:
            time.sleep(min(wait, 1.0))
            wait = (visit_start - datetime.datetime.now(datetime.timezone.utc)).total_seconds()
            # Keep slewing during wait
            obs.date = ephem.Date(datetime.datetime.now(datetime.timezone.utc))
            sat.compute(obs)
            az = math.degrees(float(sat.az))
            el = max(0, math.degrees(float(sat.alt)))
            try:
                rotctl.set_position(az, el)
            except Exception:
                pass
            # Keep status file fresh so dashboard shows slew state (every 2s)
            now_mono = time.time()
            if now_mono - _last_status_write >= 2.0:
                _last_status_write = now_mono
                write_dashboard_status({
                    "satellite": name,
                    "pass_progress": 0,
                    "streaming": False, "rssi": None, "packets": 0,
                    "freq_mhz": getattr(rf, 'uhf_base_freq_hz', 0) / 1e6 if rf and getattr(rf, 'uhf_base_freq_hz', 0) else 0,
                    "rf_backend": rf.name() if rf else "none",
                    "cmd_az": round(az, 2), "cmd_el": round(el, 2),
                    "max_el": round(visit["max_el"], 1),
                    "rise_az": round(visit["rise_az"], 1),
                    "set_az": round(visit["set_az"], 1),
                    "los_seconds": round(wait + dwell, 0),
                    "duration": round(visit["pass_duration"], 0),
                    "auto_mode": args.mode,
                })

    if not running:
        return 0

    log(f"  ** ON TARGET — listening **", logfile)
    if buzzer:
        buzzer.beep_aos()

    # Start RX
    if rf:
        rf.start_rx()
        if isinstance(rf, RfHatManager):
            rf.pkt_count = 0
            rf.total_bytes = 0
            rf.packets.clear()
            rf.rssi_min = None
            rf.rssi_max = None
            rf.rssi_sum = 0.0
            rf.rssi_count = 0

    c = 299_792_458.0
    last_doppler = 0.0
    last_metrics = 0.0
    tick = 0

    try:
        while running:
            now_utc = datetime.datetime.now(datetime.timezone.utc)
            if now_utc >= visit_end:
                break

            # Compute satellite position
            obs.date = ephem.Date(now_utc)
            sat.compute(obs)
            az = math.degrees(float(sat.az))
            el = math.degrees(float(sat.alt))

            # Track
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

            # Poll packets — ignore tiny frames (<4 bytes = noise false sync)
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

                # Dashboard status
                if isinstance(rf, RfHatManager):
                    status_data = rf.get_status_dict()
                else:
                    status_data = {"streaming": False, "rssi": None, "packets": 0,
                                   "freq_mhz": 0, "rf_backend": "none"}
                status_data["streaming"] = True  # we ARE actively listening
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
                # Doppler info
                _base_hz = getattr(rf, 'uhf_base_freq_hz', 0) or args.uhf_freq * 1e6
                _dop_hz = doppler_shift(_base_hz, float(sat.range_velocity)) - _base_hz
                status_data["doppler_hz"] = round(_dop_hz, 1)
                write_dashboard_status(status_data)

                # Console output
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
            rf.stop_rx()
            m = rf.get_metrics()
            pkts = m.packets
            log(f"  Visit done: {pkts} packets, {m.total_bytes} bytes", logfile)
        record_pass_history(name, visit["pass"], pkts,
                            m.total_bytes if rf else 0,
                            getattr(m, 'peak_rssi', None) if rf else None)

    return pkts


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(
        description="SatNOGS Auto Tracker — cycle through all receivable sats")

    parser.add_argument("--mode", default="normal", choices=["normal", "test"],
                        help="normal: 60%% of pass at peak.  test: 30s at peak (default: normal)")
    parser.add_argument("--hours", type=float, default=4,
                        help="Prediction window in hours (default: 4)")
    parser.add_argument("--min-el", type=float, default=10,
                        help="Minimum max-elevation to consider (default: 10°)")
    parser.add_argument("--list", action="store_true",
                        help="Print schedule and exit (don't track)")

    # Rotator
    parser.add_argument("--rot-host", default=ROTCTLD_HOST)
    parser.add_argument("--rot-port", type=int, default=ROTCTLD_PORT)

    # RF
    parser.add_argument("--rf-port", default=RF_HAT_PORT)
    parser.add_argument("--rf-mode", default="auto",
                        choices=["none", "cc1200", "rtlsdr", "auto"])
    parser.add_argument("--no-rf", action="store_true")
    parser.add_argument("--uhf-freq", type=float, default=UHF_FREQ_MHZ)
    parser.add_argument("--uhf-profile", default=UHF_PROFILE)
    parser.add_argument("--doppler", action="store_true", default=True)
    parser.add_argument("--no-doppler", dest="doppler", action="store_false")

    # RTL-SDR
    parser.add_argument("--rtlsdr-device", type=int, default=0)
    parser.add_argument("--rtlsdr-gain", default="auto")
    parser.add_argument("--rtlsdr-rate", type=int, default=240000)

    # VHF (needed for create_rf_backend signature)
    parser.add_argument("--vhf-freq", type=float, default=VHF_FREQ_MHZ)
    parser.add_argument("--vhf-profile", default=VHF_PROFILE)

    # Location
    parser.add_argument("--lat", type=float, default=None)
    parser.add_argument("--lon", type=float, default=None)
    parser.add_argument("--elev", type=float, default=None)

    # Output
    parser.add_argument("--log", default=None)

    args = parser.parse_args()
    if args.no_rf:
        args.rf_mode = "none"

    logfile = None
    if args.log:
        logfile = open(args.log, "a", encoding="utf-8")

    # Resolve location
    conf_lat, conf_lon, conf_elev = load_station_conf()
    sta_lat = args.lat or conf_lat or float(STATION_LAT)
    sta_lon = args.lon or conf_lon or float(STATION_LON)
    sta_elev = args.elev or conf_elev or STATION_ELEV

    log("==========================================")
    log(f" Auto Tracker — {args.mode.upper()} mode")
    log("==========================================")
    log(f"Station: {sta_lat}N, {sta_lon}E, {sta_elev:.0f}m")
    log(f"Window: {args.hours}h, min elevation: {args.min_el}°")
    if args.mode == "test":
        log("TEST MODE: 30s dwell per satellite — fast cycling")
    else:
        log("NORMAL MODE: 60% of pass duration at peak")

    # Satellite library
    sat_library = SatLibrary()
    log(f"Satellite library: {len(sat_library.satellites)} profiles")

    # Fetch TLEs
    log("Fetching TLEs...")
    tles = fetch_tles()
    if not tles:
        log("FATAL: No TLE data!")
        sys.exit(1)
    log(f"Loaded {len(tles)} satellites")

    # Find all passes
    log(f"Computing passes...")
    all_passes = find_all_passes(
        tles, hours=args.hours, min_el=args.min_el,
        lat=sta_lat, lon=sta_lon, elev=sta_elev)
    log(f"Found {len(all_passes)} passes above {args.min_el}°")

    if not all_passes:
        log("No passes found — nothing to do")
        sys.exit(0)

    # Build schedule
    schedule = build_schedule(all_passes, mode=args.mode)
    log(f"Scheduled {len(schedule)} visits ({args.mode} mode)")

    # Enrich with profile info for display
    for v in schedule:
        profile = lookup_with_satnogs_fallback(sat_library, v["norad_id"], v["name"])
        v["_profile"] = profile

    # Print schedule
    print()
    now_utc = datetime.datetime.now(datetime.timezone.utc)
    print(f"  {'#':>3s}  {'Satellite':<22s}  {'Peak (UTC)':>10s}  {'EL':>5s}  "
          f"{'Dwell':>5s}  {'Wait':>7s}  {'Freq':>8s}  {'Mode'}")
    print(f"  {'---':>3s}  {'----------------------':<22s}  {'----------':>10s}  {'-----':>5s}  "
          f"{'-----':>5s}  {'-------':>7s}  {'--------':>8s}  {'----'}")
    for i, v in enumerate(schedule):
        peak_str = v["max_time"].strftime("%H:%M:%S")
        wait_s = max(0, (v["start"] - now_utc).total_seconds())
        if wait_s > 3600:
            wait_str = f"{wait_s/3600:.1f}h"
        else:
            wait_str = f"{wait_s/60:.0f}m"
        prof = v.get("_profile")
        freq_str = f"{prof.freq_mhz:.1f}" if prof else "?"
        mode_str = ""
        if prof and prof.modulation:
            mode_str = f"{prof.modulation.symbol_rate_bps} {prof.modulation.format}"
        elif prof:
            mode_str = prof.description[:20]
        print(f"  {i+1:>3d}  {v['name']:<22s}  {peak_str:>10s}  "
              f"{v['max_el']:>4.0f}°  {v['duration']:>4.0f}s  {wait_str:>7s}  "
              f"{freq_str:>7s}M  {mode_str}")
    print()

    if args.list:
        sys.exit(0)

    # Connect rotctld
    log("Connecting to rotctld...")
    rotctl = RotctlClient(args.rot_host, args.rot_port)
    try:
        rotctl.connect()
        cur_az, cur_el = rotctl.get_position()
        log(f"Rotctld OK — AZ={cur_az} EL={cur_el}")
    except Exception as e:
        log(f"FATAL: Cannot connect to rotctld: {e}")
        sys.exit(1)

    # Open RF
    rf = create_rf_backend(args, logfile)

    # Buzzer
    cc1200_link = rf.link if isinstance(rf, RfHatManager) else None
    buzzer = Buzzer(cc1200_link)
    buzzer.beep_ready()

    # Execute schedule
    total_pkts = 0
    visited = 0

    try:
        for i, visit in enumerate(schedule):
            if not running:
                break

            now_utc = datetime.datetime.now(datetime.timezone.utc)

            # Skip visits whose window already passed
            if now_utc >= visit["end"]:
                log(f"Skipping {visit['name']} — window already passed")
                continue

            wait = (visit["start"] - now_utc).total_seconds()
            log(f"\n[{i+1}/{len(schedule)}] Next: {visit['name']} "
                f"(EL {visit['max_el']:.0f}°, {visit['duration']:.0f}s dwell) "
                f"in {max(0,wait):.0f}s")

            # Write dashboard status during idle/slew — use satellite name
            # so dashboard shows "tracking" state and displays the focus panel
            write_dashboard_status({
                "satellite": visit['name'],
                "pass_progress": 0,
                "streaming": False, "rssi": None, "packets": total_pkts,
                "freq_mhz": 0, "rf_backend": rf.name() if rf else "none",
                "auto_mode": args.mode,
                "schedule_pos": f"{i+1}/{len(schedule)}",
            })

            pkts = execute_visit(
                visit, rotctl, rf, args, sat_library,
                logfile=logfile, buzzer=buzzer,
                lat=sta_lat, lon=sta_lon, elev=sta_elev)

            total_pkts += pkts
            visited += 1
            log(f"  Score: {pkts} pkts (session total: {total_pkts})")

    finally:
        print()
        log("==========================================")
        log(f" Session complete: {visited} satellites visited, {total_pkts} packets")
        log("==========================================")
        clear_dashboard_status()
        rotctl.park()
        rotctl.close()
        if rf:
            rf.close()
        buzzer.cleanup()
        if logfile:
            logfile.close()


if __name__ == "__main__":
    main()
