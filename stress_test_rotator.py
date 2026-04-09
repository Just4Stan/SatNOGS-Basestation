#!/usr/bin/env python3
"""SatNOGS rotator stress test via rotctld TCP (EasyComm II / hamlib).

Runs 5 tests sequentially, reports PASS/FAIL for each.
Uses hamlib rotctld protocol: 'P az el' to set, 'p' to get position.

NOTE: The firmware reports AZ in its internal coordinate system (can be
negative, range [-360, +360]). We normalize to [0, 360) for comparison.
"""

import socket
import time
import sys

HOST = "127.0.0.1"
PORT = 4533
TIMEOUT = 2.0

def rotctld_cmd(sock, cmd):
    """Send command, return response string. Raises on timeout."""
    sock.sendall((cmd + "\n").encode())
    time.sleep(0.1)
    data = sock.recv(1024).decode().strip()
    return data

def get_pos(sock):
    """Get current AZ, EL as floats (raw from rotctld)."""
    resp = rotctld_cmd(sock, "p")
    lines = resp.split("\n")
    az = float(lines[0])
    el = float(lines[1])
    return az, el

def norm_az(az):
    """Normalize AZ to [0, 360)."""
    return az % 360.0

def az_error(actual, target):
    """Shortest angular distance between two AZ values."""
    a = norm_az(actual)
    t = norm_az(target)
    diff = abs(a - t)
    return min(diff, 360.0 - diff)

def set_pos(sock, az, el):
    """Command rotator to AZ, EL."""
    resp = rotctld_cmd(sock, f"P {az:.1f} {el:.1f}")
    return resp

def connect():
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(TIMEOUT)
    sock.connect((HOST, PORT))
    return sock

def park_and_wait(sock, wait=10):
    """Park rotator and wait for it to settle."""
    set_pos(sock, 0, 0)
    time.sleep(wait)

def wait_for_settle(sock, target_az, target_el, timeout=15, tolerance=2.0):
    """Wait until rotator is within tolerance of target, or timeout.
    Returns (settled, final_az, final_el, elapsed)."""
    t0 = time.time()
    while time.time() - t0 < timeout:
        az, el = get_pos(sock)
        az_err = az_error(az, target_az)
        el_err = abs(el - target_el)
        if az_err < tolerance and el_err < tolerance:
            return True, az, el, time.time() - t0
        time.sleep(0.3)
    az, el = get_pos(sock)
    return False, az, el, time.time() - t0


# -- Test 1: Rapid position commands (10 Hz for 10s) ----------------------
def test1(sock):
    print("\n" + "="*60)
    print("TEST 1: Rapid position commands (10 Hz for 10s)")
    print("="*60)
    park_and_wait(sock, 12)

    errors = []
    timeouts = 0
    t_start = time.time()
    n_cmds = 100  # 10 Hz * 10s

    for i in range(n_cmds):
        target_az = (i / (n_cmds - 1)) * 180.0  # 0 -> 180 linearly
        target_el = 0.0
        try:
            set_pos(sock, target_az, target_el)
            az, el = get_pos(sock)
            err = az_error(az, target_az)
            errors.append(err)
        except socket.timeout:
            timeouts += 1
            errors.append(float("nan"))

        # Pace at 10 Hz
        elapsed = time.time() - t_start
        expected = (i + 1) * 0.1
        if elapsed < expected:
            time.sleep(expected - elapsed)

    duration = time.time() - t_start
    valid = [e for e in errors if e == e]
    max_err = max(valid)
    avg_err = sum(valid) / len(valid)

    # Now wait for final settle and check
    time.sleep(5)
    final_az, final_el = get_pos(sock)
    final_err = az_error(final_az, 180.0)

    print(f"  Commands sent: {n_cmds} in {duration:.1f}s")
    print(f"  Timeouts: {timeouts}")
    print(f"  Max instantaneous AZ error: {max_err:.1f} deg (expected: motor lags behind)")
    print(f"  Avg instantaneous AZ error: {avg_err:.1f} deg")
    print(f"  Final position: AZ={final_az:.1f} (norm {norm_az(final_az):.1f}) EL={final_el:.1f}")
    print(f"  Final error to AZ 180: {final_err:.2f} deg")

    # Pass criteria: no timeouts + final position near 180
    passed = timeouts == 0 and final_err < 5.0
    status = "PASS" if passed else "FAIL"
    print(f"  Result: {status}")
    if timeouts > 0:
        print(f"  ANOMALY: {timeouts} timeouts during rapid commands")
    if final_err >= 5.0:
        print(f"  ANOMALY: Final AZ error {final_err:.1f} deg (>5 deg)")
    return passed


# -- Test 2: AZ wrap-around test ------------------------------------------
def test2(sock):
    print("\n" + "="*60)
    print("TEST 2: AZ wrap-around (shortest path)")
    print("="*60)
    park_and_wait(sock, 12)

    targets = [10, 350, 170, 190, 5, 355]
    results = []
    anomalies = []
    max_err = 0

    for i, target_az in enumerate(targets):
        # Read position before commanding
        az_before_raw, _ = get_pos(sock)
        az_before = norm_az(az_before_raw)

        set_pos(sock, target_az, 0)

        # Wait for settle (up to 15s for large moves)
        settled, az_after_raw, _, elapsed = wait_for_settle(sock, target_az, 0, timeout=15, tolerance=2.0)
        az_after = norm_az(az_after_raw)

        err = az_error(az_after_raw, target_az)
        max_err = max(max_err, err)

        # Check shortest path: the raw position change tells us the actual path taken
        raw_delta = abs(az_after_raw - az_before_raw)
        expected_short = min(abs(target_az - az_before) % 360, 360 - abs(target_az - az_before) % 360)

        results.append({
            "target": target_az,
            "before_raw": az_before_raw,
            "after_raw": az_after_raw,
            "before_norm": az_before,
            "after_norm": az_after,
            "err": err,
            "raw_delta": raw_delta,
            "settled": settled,
            "time": elapsed,
        })

        settle_str = f"settled in {elapsed:.1f}s" if settled else f"TIMEOUT after {elapsed:.1f}s"
        print(f"  Step {i+1}: target={target_az}  raw: {az_before_raw:.1f}->{az_after_raw:.1f}  "
              f"norm: {az_before:.1f}->{az_after:.1f}  err={err:.1f}  ({settle_str})")

        if not settled:
            anomalies.append(f"Step {i+1}: did not settle within 15s")
        if err > 5:
            anomalies.append(f"Step {i+1}: error {err:.1f} > 5 deg")

    passed = max_err < 5 and all(r["settled"] for r in results)
    status = "PASS" if passed else "FAIL"
    print(f"  Max error: {max_err:.2f} deg")
    print(f"  Result: {status}")
    for a in anomalies:
        print(f"  ANOMALY: {a}")
    return passed


# -- Test 3: EL tracking test ---------------------------------------------
def test3(sock):
    print("\n" + "="*60)
    print("TEST 3: EL tracking (0-45-90-45-0)")
    print("="*60)
    park_and_wait(sock, 12)

    targets = [0, 45, 90, 45, 0]
    max_err = 0
    anomalies = []

    for i, target_el in enumerate(targets):
        set_pos(sock, 0, target_el)
        settled, _, el, elapsed = wait_for_settle(sock, 0, target_el, timeout=10, tolerance=1.0)
        err = abs(el - target_el)
        max_err = max(max_err, err)

        status_char = "OK" if err < 1.0 else "MISS"
        settle_str = f"{elapsed:.1f}s" if settled else f"TIMEOUT {elapsed:.1f}s"
        print(f"  Step {i+1}: EL target={target_el}  actual={el:.2f}  err={err:.2f}  [{status_char}] ({settle_str})")

        if err >= 1.0:
            anomalies.append(f"Step {i+1}: EL err {err:.2f} > 1.0 deg")

    passed = max_err < 1.0
    status = "PASS" if passed else "FAIL"
    print(f"  Max EL error: {max_err:.2f} deg")
    print(f"  Result: {status}")
    for a in anomalies:
        print(f"  ANOMALY: {a}")
    return passed


# -- Test 4: Simultaneous AZ+EL -------------------------------------------
def test4(sock):
    print("\n" + "="*60)
    print("TEST 4: Simultaneous AZ+EL")
    print("="*60)
    park_and_wait(sock, 12)

    moves = [
        (270, 60, 15),
        (90, 15, 15),
        (180, 45, 10),
    ]
    max_az_err = 0
    max_el_err = 0
    anomalies = []

    for i, (t_az, t_el, timeout) in enumerate(moves):
        set_pos(sock, t_az, t_el)
        settled, az_raw, el, elapsed = wait_for_settle(sock, t_az, t_el, timeout=timeout, tolerance=2.0)
        a_err = az_error(az_raw, t_az)
        e_err = abs(el - t_el)
        max_az_err = max(max_az_err, a_err)
        max_el_err = max(max_el_err, e_err)

        ok = a_err < 2.0 and e_err < 2.0
        settle_str = f"{elapsed:.1f}s" if settled else f"TIMEOUT {elapsed:.1f}s"
        print(f"  Move {i+1}: P {t_az} {t_el} -> AZ={az_raw:.1f} (norm {norm_az(az_raw):.1f}) EL={el:.1f}  "
              f"(az_err={a_err:.2f}, el_err={e_err:.2f}) [{'OK' if ok else 'MISS'}] ({settle_str})")
        if not ok:
            anomalies.append(f"Move {i+1}: errors exceed 2 deg (az={a_err:.2f}, el={e_err:.2f})")

    passed = max_az_err < 2.0 and max_el_err < 2.0
    status = "PASS" if passed else "FAIL"
    print(f"  Max AZ error: {max_az_err:.2f}, Max EL error: {max_el_err:.2f}")
    print(f"  Result: {status}")
    for a in anomalies:
        print(f"  ANOMALY: {a}")
    return passed


# -- Test 5: Rapid back-and-forth (tracking jitter) -----------------------
def test5(sock):
    print("\n" + "="*60)
    print("TEST 5: Rapid back-and-forth (tracking jitter)")
    print("="*60)
    # First move to starting area and wait
    set_pos(sock, 100, 20)
    settled, _, _, _ = wait_for_settle(sock, 100, 20, timeout=20, tolerance=2.0)
    if not settled:
        print("  WARNING: Could not reach starting position")

    sequence = [
        (100, 20, 3),
        (110, 20, 3),
        (100, 20, 3),
        (110, 20, 3),
    ]
    max_err = 0
    anomalies = []
    hiccups = 0

    for i, (t_az, t_el, wait) in enumerate(sequence):
        set_pos(sock, t_az, t_el)
        time.sleep(wait)
        az_raw, el = get_pos(sock)

        a_err = az_error(az_raw, t_az)
        e_err = abs(el - t_el)
        err = max(a_err, e_err)
        max_err = max(max_err, err)

        ok = err < 3.0  # 3 deg tolerance for jitter test
        print(f"  Step {i+1}: P {t_az} {t_el} -> AZ={az_raw:.1f} (norm {norm_az(az_raw):.1f}) EL={el:.1f}  "
              f"(az_err={a_err:.2f}, el_err={e_err:.2f}) [{'OK' if ok else 'HICCUP'}]")
        if not ok:
            hiccups += 1
            anomalies.append(f"Step {i+1}: err {err:.2f} deg")

    passed = hiccups == 0
    status = "PASS" if passed else "FAIL"
    print(f"  Max error: {max_err:.2f} deg, Hiccups: {hiccups}")
    print(f"  Result: {status}")
    for a in anomalies:
        print(f"  ANOMALY: {a}")
    return passed


# -- Main ------------------------------------------------------------------
def main():
    print("SatNOGS Rotator Stress Test v2")
    print(f"Connecting to rotctld at {HOST}:{PORT}")

    try:
        sock = connect()
    except Exception as e:
        print(f"FATAL: Cannot connect to rotctld: {e}")
        sys.exit(1)

    print("Connected. Starting tests...\n")

    # Initial position check
    az, el = get_pos(sock)
    print(f"Current position: AZ={az:.1f} (norm {norm_az(az):.1f}) EL={el:.1f}")

    results = {}
    try:
        results["Test 1: Rapid commands"]     = test1(sock)
        results["Test 2: AZ wrap-around"]     = test2(sock)
        results["Test 3: EL tracking"]        = test3(sock)
        results["Test 4: Simultaneous AZ+EL"] = test4(sock)
        results["Test 5: Back-and-forth"]     = test5(sock)
    except Exception as e:
        print(f"\nERROR during test: {e}")
        import traceback
        traceback.print_exc()
    finally:
        # Park after tests
        print("\nParking rotator...")
        try:
            set_pos(sock, 0, 0)
        except:
            pass
        sock.close()

    # Summary
    print("\n" + "="*60)
    print("SUMMARY")
    print("="*60)
    for name, passed in results.items():
        print(f"  {name}: {'PASS' if passed else 'FAIL'}")

    total = len(results)
    passed_count = sum(1 for v in results.values() if v)
    print(f"\n  {passed_count}/{total} tests passed")

    if passed_count < total:
        sys.exit(1)

if __name__ == "__main__":
    main()
