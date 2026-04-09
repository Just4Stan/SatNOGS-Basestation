#!/usr/bin/env python3
"""
Torque test script for SatNOGS rotator AZ axis.

Attach a 50cm rod to the azimuth axis. Hang known weights at the end.
Torque = weight_kg * 9.81 * 0.5 (Nm)

Tests:
  1. Stall-find:  ramp duty from 15% to 95% in 5% steps, detect when motor
                  overcomes the load (encoder starts moving).
  2. Speed sweep: run at fixed duty levels, measure angular speed (deg/s)
                  with and without load.
  3. Hold test:   PID holds position while you push on the rod — logs
                  max error before it gives up.

Usage:
    python3 torque_test.py                          # interactive menu
    python3 torque_test.py --stall --weight 0.5     # stall test, 0.5 kg
    python3 torque_test.py --speed                  # speed sweep (no load)
    python3 torque_test.py --hold                   # PID hold test

Serial port: auto-detect or pass as last argument.
Results saved to torque_results_<timestamp>.csv
"""

import serial
import time
import math
import sys
import os
import glob
import csv
from datetime import datetime

ARM_LENGTH_M = 0.50  # rod length in meters

# Duty levels for sweep tests
DUTY_STEPS = [0.15, 0.20, 0.25, 0.30, 0.35, 0.40, 0.50, 0.60, 0.70, 0.80, 0.90, 0.95]

# AZ ticks per degree (from config.h)
AZ_TICKS_PER_DEG = 244.6


def auto_detect_serial():
    for p in sorted(glob.glob("/dev/cu.usbmodem*")):
        return p
    for p in sorted(glob.glob("/dev/ttyACM*")):
        return p
    return None


def send_cmd(port, cmd):
    """Send command, return response lines (up to 100ms)."""
    port.reset_input_buffer()
    port.write(f"{cmd}\n".encode())
    time.sleep(0.05)
    lines = []
    deadline = time.time() + 0.1
    while time.time() < deadline:
        if port.in_waiting:
            line = port.readline().decode("utf-8", errors="replace").strip()
            if line:
                lines.append(line)
        else:
            time.sleep(0.005)
    return lines


def get_ticks(port):
    """Read raw encoder ticks. Returns (az_ticks, el_ticks) or None."""
    lines = send_cmd(port, "TICKS")
    for line in lines:
        if "AZ_TICKS=" in line:
            try:
                parts = line.split()
                az = int(parts[0].split("=")[1])
                el = int(parts[1].split("=")[1])
                return az, el
            except (ValueError, IndexError):
                pass
    return None


def get_pos(port):
    """Read position in degrees. Returns (az_deg, el_deg) or None."""
    lines = send_cmd(port, "AZ")
    for line in lines:
        if line.startswith("AZ") and "EL" in line:
            try:
                parts = line.replace("AZ", "").replace("EL", " ").split()
                return float(parts[0]), float(parts[1])
            except (ValueError, IndexError):
                pass
    return None


def stop_motor(port):
    send_cmd(port, "STOP")


def set_duty_az(port, duty):
    """Set raw AZ duty (-1.0 to 1.0). Bypasses PID."""
    send_cmd(port, f"DUTY_AZ{duty:.3f}")


def zero_encoders(port):
    send_cmd(port, "ZERO")


def torque_from_weight(weight_kg):
    """Calculate torque at the axis from weight on rod end."""
    return weight_kg * 9.81 * ARM_LENGTH_M


def measure_speed(port, duty, duration=3.0, settle=0.5):
    """
    Run motor at given duty for duration seconds.
    Returns average speed in deg/s, or 0 if stalled.
    """
    # Zero and settle
    zero_encoders(port)
    time.sleep(0.1)

    set_duty_az(port, duty)
    time.sleep(settle)  # let motor accelerate

    # Measure
    t0 = get_ticks(port)
    if t0 is None:
        stop_motor(port)
        return 0.0

    time.sleep(duration)

    t1 = get_ticks(port)
    stop_motor(port)

    if t1 is None:
        return 0.0

    delta_ticks = abs(t1[0] - t0[0])
    delta_deg = delta_ticks / AZ_TICKS_PER_DEG
    speed = delta_deg / duration
    return speed


def stall_test(port, weight_kg, csv_writer):
    """
    Ramp duty from 15% to 95%. At each step, check if motor moves.
    The lowest duty that produces motion is the stall threshold.
    """
    torque = torque_from_weight(weight_kg)
    print(f"\n  Stall-find test")
    print(f"  Weight: {weight_kg:.2f} kg")
    print(f"  Arm length: {ARM_LENGTH_M:.2f} m")
    print(f"  Load torque: {torque:.3f} Nm")
    print(f"  {'Duty':>6s}  {'Speed (°/s)':>11s}  {'Status':>10s}")
    print(f"  {'─'*6}  {'─'*11}  {'─'*10}")

    stall_duty = None

    for duty in DUTY_STEPS:
        # Reset position
        zero_encoders(port)
        time.sleep(0.2)

        # Apply duty for 2 seconds
        set_duty_az(port, duty)
        time.sleep(0.3)

        t0 = get_ticks(port)
        time.sleep(2.0)
        t1 = get_ticks(port)
        stop_motor(port)

        if t0 is None or t1 is None:
            print(f"  {duty*100:5.0f}%  {'ERR':>11s}  {'read fail':>10s}")
            continue

        delta_ticks = abs(t1[0] - t0[0])
        delta_deg = delta_ticks / AZ_TICKS_PER_DEG
        speed = delta_deg / 2.0

        # Moving threshold: > 0.5 deg/s
        moving = speed > 0.5
        status = "MOVING" if moving else "STALLED"
        print(f"  {duty*100:5.0f}%  {speed:>10.1f}°  {status:>10s}")

        csv_writer.writerow({
            "test": "stall_find",
            "weight_kg": weight_kg,
            "torque_Nm": f"{torque:.3f}",
            "duty_pct": f"{duty*100:.0f}",
            "speed_dps": f"{speed:.2f}",
            "status": status,
        })

        if moving and stall_duty is None:
            stall_duty = duty

        time.sleep(0.5)

    if stall_duty is not None:
        print(f"\n  → Motor overcomes {torque:.3f} Nm at {stall_duty*100:.0f}% duty")
    else:
        print(f"\n  → Motor cannot overcome {torque:.3f} Nm at any duty (max 95%)")

    stop_motor(port)
    return stall_duty


def speed_sweep(port, weight_kg, csv_writer):
    """Run at each duty level, measure speed. weight_kg=0 for no-load."""
    torque = torque_from_weight(weight_kg) if weight_kg > 0 else 0.0
    label = f"{weight_kg:.2f} kg ({torque:.3f} Nm)" if weight_kg > 0 else "no load"
    print(f"\n  Speed sweep — {label}")
    print(f"  {'Duty':>6s}  {'Speed (°/s)':>11s}  {'360° time':>9s}")
    print(f"  {'─'*6}  {'─'*11}  {'─'*9}")

    for duty in DUTY_STEPS:
        speed = measure_speed(port, duty, duration=3.0)
        time_360 = f"{360.0/speed:.1f}s" if speed > 0.1 else "inf"

        print(f"  {duty*100:5.0f}%  {speed:>10.1f}°  {time_360:>9s}")

        csv_writer.writerow({
            "test": "speed_sweep",
            "weight_kg": weight_kg,
            "torque_Nm": f"{torque:.3f}",
            "duty_pct": f"{duty*100:.0f}",
            "speed_dps": f"{speed:.2f}",
            "status": "MOVING" if speed > 0.5 else "STALLED",
        })

        time.sleep(0.5)

    stop_motor(port)


def hold_test(port, csv_writer):
    """
    PID holds AZ at 0°. Push on the rod and observe max position error.
    Logs position at 10 Hz for 30 seconds.
    """
    print("\n  PID Hold test")
    print("  Rotator will hold AZ=0°. Push on the rod to test holding torque.")
    print("  Recording for 30 seconds. Press Ctrl+C to stop early.")
    print()

    # Command PID to hold at 0
    send_cmd(port, "AZ0 EL0")
    time.sleep(1)

    max_error = 0.0
    t_start = time.time()
    try:
        while time.time() - t_start < 30:
            pos = get_pos(port)
            elapsed = time.time() - t_start
            if pos:
                az, el = pos
                error = abs(az)
                max_error = max(max_error, error)
                print(f"\r  t={elapsed:5.1f}s  AZ={az:>7.2f}°  error={error:>6.2f}°  max={max_error:>6.2f}°", end="", flush=True)

                csv_writer.writerow({
                    "test": "hold",
                    "weight_kg": "",
                    "torque_Nm": "",
                    "duty_pct": "",
                    "speed_dps": "",
                    "status": f"t={elapsed:.1f} az={az:.2f} err={error:.2f}",
                })
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass

    print(f"\n\n  Max position error: {max_error:.2f}°")
    stop_motor(port)


def main():
    # Parse args
    weight_kg = 0.0
    do_stall = False
    do_speed = False
    do_hold = False
    port_name = None

    args = sys.argv[1:]
    i = 0
    while i < len(args):
        if args[i] == "--stall":
            do_stall = True
        elif args[i] == "--speed":
            do_speed = True
        elif args[i] == "--hold":
            do_hold = True
        elif args[i] == "--weight" and i + 1 < len(args):
            i += 1
            weight_kg = float(args[i])
        elif not args[i].startswith("-"):
            port_name = args[i]
        i += 1

    interactive = not (do_stall or do_speed or do_hold)

    if port_name is None:
        port_name = auto_detect_serial()
    if port_name is None:
        print("No serial port found. Pass as argument: python3 torque_test.py /dev/cu.usbmodemXXXX")
        sys.exit(1)

    # CSV output
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_path = f"torque_results_{timestamp}.csv"
    csv_file = open(csv_path, "w", newline="")
    csv_writer = csv.DictWriter(csv_file, fieldnames=[
        "test", "weight_kg", "torque_Nm", "duty_pct", "speed_dps", "status"
    ])
    csv_writer.writeheader()

    print(f"\n  SatNOGS Torque Test")
    print(f"  Serial: {port_name}")
    print(f"  Rod length: {ARM_LENGTH_M*100:.0f} cm")
    print(f"  Results: {csv_path}")

    port = serial.Serial(port_name, 115200, timeout=0.05)
    time.sleep(2)
    while port.in_waiting:
        port.readline()

    # Verify connection
    ver = send_cmd(port, "VE")
    if ver:
        print(f"  Firmware: {ver[0]}")
    else:
        print("  WARNING: no firmware response")

    try:
        if interactive:
            while True:
                print("\n  ─────────────────────────────")
                print("  1. Stall-find test (needs weight on rod)")
                print("  2. Speed sweep (no load)")
                print("  3. Speed sweep (with load)")
                print("  4. PID hold test (push on rod)")
                print("  5. Single duty test")
                print("  q. Quit")
                print()
                choice = input("  Select: ").strip()

                if choice == "1":
                    w = input("  Weight on rod end (kg): ").strip()
                    stall_test(port, float(w), csv_writer)
                elif choice == "2":
                    speed_sweep(port, 0, csv_writer)
                elif choice == "3":
                    w = input("  Weight on rod end (kg): ").strip()
                    speed_sweep(port, float(w), csv_writer)
                elif choice == "4":
                    hold_test(port, csv_writer)
                elif choice == "5":
                    d = input("  Duty (0-100%): ").strip()
                    duty = float(d) / 100.0
                    dur = input("  Duration (s) [3]: ").strip()
                    dur = float(dur) if dur else 3.0
                    print(f"\n  Running AZ at {duty*100:.0f}% for {dur:.0f}s...")
                    speed = measure_speed(port, duty, duration=dur)
                    print(f"  Speed: {speed:.1f} °/s")
                    if speed > 0.1:
                        print(f"  360° time: {360.0/speed:.1f}s")
                    csv_writer.writerow({
                        "test": "single",
                        "weight_kg": "",
                        "torque_Nm": "",
                        "duty_pct": f"{duty*100:.0f}",
                        "speed_dps": f"{speed:.2f}",
                        "status": "MOVING" if speed > 0.5 else "STALLED",
                    })
                elif choice in ("q", "Q"):
                    break
        else:
            if do_stall:
                stall_test(port, weight_kg, csv_writer)
            if do_speed:
                speed_sweep(port, weight_kg, csv_writer)
            if do_hold:
                hold_test(port, csv_writer)

    except KeyboardInterrupt:
        print("\n\n  Interrupted")

    finally:
        stop_motor(port)
        port.close()
        csv_file.close()
        print(f"\n  Results saved to {csv_path}")
        print("  Done.")


if __name__ == "__main__":
    main()
