#!/usr/bin/env python3
"""
SatNOGS ground station demo — thesis presentation mode + simulated satellite tracking.

Modes:
  1. Thesis demo (default): Individual axis movements, then combined, then fake passes.
  2. Passes only:           Just the simulated satellite passes (--passes-only).

Usage:
    python3 demo_tracking.py [serial_port]
    python3 demo_tracking.py --passes-only

Default port: auto-detect (macOS /dev/cu.usbmodem*, Linux /dev/ttyACM*)
Press Ctrl+C to stop and park.
"""

import serial
import time
import math
import sys
import os
import glob

# Fake satellite pass definitions: (name, az_start, az_end, el_max, duration_s)
# All AZ values kept within [0, 350] to stay well inside the ±360° firmware limits.
PASSES = [
    ("ISS (ZARYA)",          300,   80,  72, 28),
    ("NOAA 19",               20,  150,  45, 24),
    ("AetherSpace-1",        330,  100,  58, 26),
    ("METEOR-M2 3",          270,  350,  35, 20),
    ("FUNCUBE-1",             50,  170,  82, 30),
    ("SATNOGS-DEMO",         310,   60,  40, 22),
    ("CUTE-1.7+APD II",      100,  340,  55, 25),
    ("NOAA 18",              280,   50,  62, 27),
]

SLEW_PAUSE = 3    # seconds between passes (slewing to next AOS)
UPDATE_HZ = 10    # target + display update rate


def auto_detect_serial():
    """Try to find the Pico serial port."""
    for p in sorted(glob.glob("/dev/cu.usbmodem*")):
        return p
    for p in sorted(glob.glob("/dev/ttyACM*")):
        return p
    return None


def satellite_pass(t_norm, az_start, az_end, el_max):
    """
    Generate AZ/EL for a normalized time t_norm in [0, 1].
    EL follows a sin arc. AZ sweeps linearly but accelerates near zenith.
    """
    t_norm = max(0.0, min(1.0, t_norm))
    el = el_max * math.sin(math.pi * t_norm)

    warp = 0.3 * math.sin(math.pi * t_norm)
    az_frac = t_norm + warp * (t_norm - 0.5)
    az_frac = max(0.0, min(1.0, az_frac))
    az = az_start + (az_end - az_start) * az_frac

    return az, max(el, 0.5)


def send_target(port, az, el):
    """Fire-and-forget: send target, drain stale data."""
    port.reset_input_buffer()
    port.write(f"AZ{az:.1f} EL{el:.1f}\n".encode())


def get_pos(port):
    """Quick non-blocking position query."""
    port.reset_input_buffer()
    port.write(b"AZ\n")
    deadline = time.time() + 0.035
    while time.time() < deadline:
        if port.in_waiting:
            line = port.readline().decode("utf-8", errors="replace").strip()
            if line.startswith("AZ") and "EL" in line:
                return line
    return None


def wait_until_settled(port, target_az, target_el, timeout=30, deadband=1.5):
    """Wait until the rotator reaches the target (within deadband degrees)."""
    start = time.time()
    while time.time() - start < timeout:
        pos = get_pos(port)
        if pos:
            try:
                parts = pos.replace("AZ", "").replace("EL", " ").split()
                cur_az, cur_el = float(parts[0]), float(parts[1])
                az_err = abs(target_az - cur_az)
                el_err = abs(target_el - cur_el)
                if az_err < deadband and el_err < deadband:
                    return pos
            except (ValueError, IndexError):
                pass
        time.sleep(0.2)
    return get_pos(port)


def run_thesis_demo(port):
    """
    Thesis presentation demo:
      1) Elevation only:  0 -> 90 -> 0
      2) Azimuth only:    0 -> 170 -> 0
      3) Combined:        AZ 0->170 + EL 0->90 simultaneously
      4) Park
    """
    print()
    print("  ========================================")
    print("   Thesis Demo — Axis Demonstration")
    print("  ========================================")
    print()

    # --- Ensure we start at home ---
    print(f"  [{time.strftime('%H:%M:%S')}] Parking to home (AZ 0, EL 0)...")
    port.reset_input_buffer()
    port.write(b"PARK\n")
    time.sleep(1)
    wait_until_settled(port, 0, 0, timeout=20)
    time.sleep(1)

    # --- 1) Elevation only: 0 -> 90 -> 0 ---
    print()
    print(f"  [{time.strftime('%H:%M:%S')}] Demo 1: ELEVATION 0 -> 90 degrees")
    send_target(port, 0, 90)
    t_start = time.time()
    while time.time() - t_start < 15:
        pos = get_pos(port)
        actual = pos if pos else "---"
        elapsed = time.time() - t_start
        print(f"\r  {elapsed:>6.1f}s  target AZ  0.0 EL 90.0  actual {actual:<22s}", end="", flush=True)
        if pos:
            try:
                parts = pos.replace("AZ", "").replace("EL", " ").split()
                if float(parts[1]) > 88.0:
                    break
            except (ValueError, IndexError):
                pass
        time.sleep(0.3)
    print()
    time.sleep(1)

    print(f"  [{time.strftime('%H:%M:%S')}] Returning EL to 0...")
    send_target(port, 0, 0)
    wait_until_settled(port, 0, 0, timeout=15)
    time.sleep(1)

    # --- 2) Azimuth only: 0 -> 170 -> 0 ---
    # Using 170 instead of 180 to avoid the equidistance ambiguity:
    # at exactly 180 the firmware sees 0 and 360 as equally close, and
    # any small overshoot makes it continue forward to 360 instead of
    # unwinding back to 0.
    print()
    print(f"  [{time.strftime('%H:%M:%S')}] Demo 2: AZIMUTH 0 -> 170 degrees")
    send_target(port, 170, 0)
    t_start = time.time()
    while time.time() - t_start < 15:
        pos = get_pos(port)
        actual = pos if pos else "---"
        elapsed = time.time() - t_start
        print(f"\r  {elapsed:>6.1f}s  target AZ170.0 EL  0.0  actual {actual:<22s}", end="", flush=True)
        if pos:
            try:
                parts = pos.replace("AZ", "").replace("EL", " ").split()
                if float(parts[0]) > 168.0:
                    break
            except (ValueError, IndexError):
                pass
        time.sleep(0.3)
    print()
    time.sleep(1)

    print(f"  [{time.strftime('%H:%M:%S')}] Returning AZ to 0...")
    send_target(port, 0, 0)
    wait_until_settled(port, 0, 0, timeout=15)
    time.sleep(1)

    # --- 3) Combined: AZ 0->170, EL 0->90 simultaneously ---
    print()
    print(f"  [{time.strftime('%H:%M:%S')}] Demo 3: COMBINED — AZ 0->170, EL 0->90 (simultaneous)")
    send_target(port, 170, 90)
    t_start = time.time()
    while time.time() - t_start < 15:
        pos = get_pos(port)
        actual = pos if pos else "---"
        elapsed = time.time() - t_start
        print(f"\r  {elapsed:>6.1f}s  target AZ170.0 EL 90.0  actual {actual:<22s}", end="", flush=True)
        if pos:
            try:
                parts = pos.replace("AZ", "").replace("EL", " ").split()
                if float(parts[0]) > 168.0 and float(parts[1]) > 88.0:
                    break
            except (ValueError, IndexError):
                pass
        time.sleep(0.3)
    print()
    time.sleep(1)

    # --- Park ---
    print()
    print(f"  [{time.strftime('%H:%M:%S')}] Demo complete — parking rotator")
    port.reset_input_buffer()
    port.write(b"PARK\n")
    wait_until_settled(port, 0, 0, timeout=25)
    final = get_pos(port)
    print(f"  [{time.strftime('%H:%M:%S')}] Parked: {final}")
    time.sleep(2)
    print()
    print("  ----------------------------------------")
    print("   Axis demo complete!")
    print("  ----------------------------------------")


def run_fake_passes(port, port_name):
    """Run simulated satellite passes (the original demo)."""
    print()
    print("  ========================================")
    print("   SatNOGS Ground Station — Simulated Tracking")
    print("  ========================================")
    print()
    print(f"  Serial:  {port_name}")
    print(f"  Station: SatNOGS-RP2040-v0.2")
    print()

    pass_idx = 0
    while True:
        p = PASSES[pass_idx % len(PASSES)]
        name, az_start, az_end, el_max, duration = p

        print(f"  [{time.strftime('%H:%M:%S')}] Next pass: {name}")
        print(f"             AZ {az_start:+.0f} -> {az_end:+.0f}, EL max {el_max}, {duration}s")

        # Slew to AOS position
        aos_az, aos_el = satellite_pass(0.0, az_start, az_end, el_max)
        print(f"  [{time.strftime('%H:%M:%S')}] Slewing to AOS: AZ{aos_az:.1f} EL{aos_el:.1f}")
        send_target(port, aos_az, aos_el)
        time.sleep(SLEW_PAUSE)

        # Track the pass
        print(f"  [{time.strftime('%H:%M:%S')}] ** TRACKING {name} **")
        t_start = time.time()
        dt = 1.0 / UPDATE_HZ
        last_pos = ""
        i = 0

        while True:
            elapsed = time.time() - t_start
            t_norm = elapsed / duration
            if t_norm >= 1.0:
                break

            az, el = satellite_pass(t_norm, az_start, az_end, el_max)
            send_target(port, az, el)

            if i % 5 == 0:
                pos = get_pos(port)
                if pos:
                    last_pos = pos

            bar_el = int(el / 90 * 20)
            bar = "|" + "#" * bar_el + " " * (20 - bar_el) + "|"
            phase = "AOS" if t_norm < 0.15 else ("LOS" if t_norm > 0.85 else "TRK")
            print(f"\r  {phase}  AZ{az:>7.1f}  EL{el:>5.1f}  {bar}  {last_pos:<20s}", end="", flush=True)

            i += 1
            next_tick = t_start + i * dt
            remaining = next_tick - time.time()
            if remaining > 0:
                time.sleep(remaining)

        print()
        print(f"  [{time.strftime('%H:%M:%S')}] Pass complete — parking and unwinding")
        port.reset_input_buffer()
        port.write(b"PARK\n")
        wait_until_settled(port, 0, 0, timeout=25)
        final = get_pos(port)
        print(f"  [{time.strftime('%H:%M:%S')}] Parked: {final}")
        time.sleep(1)

        pass_idx += 1
        print()


def main():
    passes_only = "--passes-only" in sys.argv
    args = [a for a in sys.argv[1:] if a != "--passes-only"]

    port_name = args[0] if args else None
    if port_name is None or port_name == "auto":
        port_name = auto_detect_serial()
    if port_name is None:
        print("No serial port found. Pass it as argument: python3 demo_tracking.py /dev/ttyACM0")
        sys.exit(1)

    port = serial.Serial(port_name, 115200, timeout=0.05)
    time.sleep(2)
    while port.in_waiting:
        port.readline()

    try:
        if not passes_only:
            run_thesis_demo(port)
            print()
            print("  Continuing to simulated satellite passes...")
            print("  (Press Ctrl+C to stop)")
            time.sleep(3)

        run_fake_passes(port, port_name)

    except KeyboardInterrupt:
        print()
        print()
        print(f"  [{time.strftime('%H:%M:%S')}] Demo stopped — parking rotator")
        port.reset_input_buffer()
        port.write(b"PARK\n")
        time.sleep(1)
        port.close()
        print("  Done.")


if __name__ == "__main__":
    main()
