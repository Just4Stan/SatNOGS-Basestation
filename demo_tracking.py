#!/usr/bin/env python3
"""
SatNOGS ground station demo — simulates realistic satellite tracking.
The rotator sweeps through several fake passes with realistic AZ/EL profiles.

Usage:
    python3 demo_tracking.py [serial_port]

Default port: /dev/cu.usbmodem1401 (macOS) or /dev/ttyACM0 (Linux/Pi)
Press Ctrl+C to stop and park.
"""

import serial
import time
import math
import sys
import os

# Fake satellite pass definitions: (name, az_start, az_end, el_max, duration_s)
PASSES = [
    ("ISS (ZARYA)",         -60,   80,  72, 28),
    ("NOAA 19",              20,  150,  45, 24),
    ("AetherSpace-1",      -30,  100,  58, 26),
    ("METEOR-M2 3",        -90,  -10,  35, 20),
    ("FUNCUBE-1",            50,  170,  82, 30),
    ("SATNOGS-DEMO",       -50,   60,  40, 22),
    ("CUTE-1.7+APD II",    100,  -20,  55, 25),
    ("NOAA 18",            -80,   50,  62, 27),
]

SLEW_PAUSE = 3    # seconds between passes (slewing to next AOS)
UPDATE_HZ = 10    # target + display update rate


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


def main():
    port_name = sys.argv[1] if len(sys.argv) > 1 else None
    if port_name is None:
        for p in ["/dev/cu.usbmodem1401", "/dev/ttyACM0", "/dev/ttyUSB0"]:
            if os.path.exists(p):
                port_name = p
                break
    if port_name is None:
        print("No serial port found. Pass it as argument: python3 demo_tracking.py /dev/ttyACM0")
        sys.exit(1)

    port = serial.Serial(port_name, 115200, timeout=0.05)
    time.sleep(2)
    while port.in_waiting:
        port.readline()

    def send_target(az, el):
        """Fire-and-forget: send target, drain stale data."""
        port.reset_input_buffer()
        port.write(f"AZ{az:.1f} EL{el:.1f}\n".encode())

    def get_pos():
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

    print()
    print("  ========================================")
    print("   SatNOGS Ground Station — Live Tracking")
    print("  ========================================")
    print()
    print(f"  Serial:  {port_name}")
    print(f"  Station: SatNOGS-RP2040-v0.2")
    print()

    pass_idx = 0
    try:
        while True:
            p = PASSES[pass_idx % len(PASSES)]
            name, az_start, az_end, el_max, duration = p

            print(f"  [{time.strftime('%H:%M:%S')}] Next pass: {name}")
            print(f"             AZ {az_start:+.0f} -> {az_end:+.0f}, EL max {el_max}, {duration}s")

            # Slew to AOS position
            aos_az, aos_el = satellite_pass(0.0, az_start, az_end, el_max)
            print(f"  [{time.strftime('%H:%M:%S')}] Slewing to AOS: AZ{aos_az:.1f} EL{aos_el:.1f}")
            send_target(aos_az, aos_el)
            time.sleep(SLEW_PAUSE)

            # Track the pass with small, frequent target updates (10 Hz).
            # The PID handles small deltas smoothly — no shocks.
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

                # Send target every tick (fire-and-forget, ~1ms)
                send_target(az, el)

                # Read position every 5th tick (~2Hz) to avoid serial contention
                if i % 5 == 0:
                    pos = get_pos()
                    if pos:
                        last_pos = pos

                bar_el = int(el / 90 * 20)
                bar = "|" + "#" * bar_el + " " * (20 - bar_el) + "|"
                phase = "AOS" if t_norm < 0.15 else ("LOS" if t_norm > 0.85 else "TRK")
                print(f"\r  {phase}  AZ{az:>7.1f}  EL{el:>5.1f}  {bar}  {last_pos:<20s}", end="", flush=True)

                i += 1

                # Absolute timing: sleep until the next tick
                next_tick = t_start + i * dt
                remaining = next_tick - time.time()
                if remaining > 0:
                    time.sleep(remaining)

            print()
            print(f"  [{time.strftime('%H:%M:%S')}] Pass complete — parking")

            # Let rotator reach LOS, then park
            los_az, los_el = satellite_pass(1.0, az_start, az_end, el_max)
            send_target(los_az, los_el)
            time.sleep(1)
            send_target(0, 0)
            time.sleep(SLEW_PAUSE)

            pass_idx += 1
            print()

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
