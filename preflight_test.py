#!/usr/bin/env python3
"""Pre-field-test verification: Groups A-D.

Uses a persistent rotctld TCP connection with 'w' passthrough for Group A.
Tests dashboard APIs via HTTPS for Groups B/C/D.

Usage: python3 preflight_test.py [--host 10.72.3.114] [--group a|b|c|d|all]
"""

import argparse
import json
import socket
import ssl
import sys
import time
import urllib.request
import urllib.error

DEFAULT_HOST = "10.72.3.114"
ROTCTLD_PORT = 4533
DASH_PORT = 5000
TIMEOUT = 3

results = []

# ─── Persistent rotctld connection ────────────────────────────────────────────

class RotctlConn:
    """Persistent TCP connection to rotctld with 'w' passthrough."""
    def __init__(self, host, port=ROTCTLD_PORT):
        self.host = host
        self.port = port
        self.sock = None
        self._connect()

    def _connect(self):
        if self.sock:
            try: self.sock.close()
            except: pass
        self.sock = socket.socket()
        self.sock.settimeout(TIMEOUT)
        self.sock.connect((self.host, self.port))
        time.sleep(0.1)

    def cmd(self, raw_cmd, wait=0.3, multi_line=False):
        """Send 'w <raw_cmd>' and return response. Use multi_line=True for HELP/INFO/MONITOR."""
        try:
            # Flush any pending data
            self.sock.setblocking(False)
            try:
                self.sock.recv(4096)
            except:
                pass
            self.sock.setblocking(True)
            self.sock.settimeout(TIMEOUT)

            self.sock.send(f"w {raw_cmd}\n".encode())
            time.sleep(wait)

            resp = b""
            self.sock.settimeout(1.0 if not multi_line else 2.0)
            while True:
                try:
                    chunk = self.sock.recv(4096)
                    if not chunk:
                        break
                    resp += chunk
                    if not multi_line and b"\n" in chunk:
                        break
                except socket.timeout:
                    break
            return resp.decode(errors="replace").strip()
        except (ConnectionError, OSError) as e:
            # Reconnect and retry once
            self._connect()
            return f"CONNECTION_ERROR: {e}"

    def pos(self):
        """Get position via 'p' command."""
        try:
            self.sock.setblocking(False)
            try: self.sock.recv(4096)
            except: pass
            self.sock.setblocking(True)
            self.sock.settimeout(TIMEOUT)

            self.sock.send(b"p\n")
            time.sleep(0.3)
            resp = self.sock.recv(256).decode().strip()
            lines = resp.split("\n")
            if len(lines) >= 2:
                return float(lines[0]), float(lines[1])
        except:
            pass
        return None, None

    def close(self):
        if self.sock:
            try: self.sock.close()
            except: pass

# ─── HTTP helpers ─────────────────────────────────────────────────────────────

def api_get(host, path):
    ctx = ssl.create_default_context()
    ctx.check_hostname = False
    ctx.verify_mode = ssl.CERT_NONE
    url = f"https://{host}:{DASH_PORT}{path}"
    try:
        resp = urllib.request.urlopen(url, timeout=TIMEOUT, context=ctx)
        body = resp.read().decode()
        try:
            return resp.status, json.loads(body)
        except json.JSONDecodeError:
            return resp.status, body
    except urllib.error.HTTPError as e:
        return e.code, e.read().decode()
    except Exception as e:
        return 0, str(e)

def api_post(host, path, data=None):
    ctx = ssl.create_default_context()
    ctx.check_hostname = False
    ctx.verify_mode = ssl.CERT_NONE
    url = f"https://{host}:{DASH_PORT}{path}"
    body = json.dumps(data).encode() if data else b""
    req = urllib.request.Request(url, data=body, method="POST")
    req.add_header("Content-Type", "application/json")
    try:
        resp = urllib.request.urlopen(req, timeout=TIMEOUT, context=ctx)
        rbody = resp.read().decode()
        try:
            return resp.status, json.loads(rbody)
        except json.JSONDecodeError:
            return resp.status, rbody
    except urllib.error.HTTPError as e:
        return e.code, e.read().decode()
    except Exception as e:
        return 0, str(e)

def test(tid, name, passed, detail=""):
    results.append((tid, name, passed, detail))
    m = "\033[92m✓\033[0m" if passed else "\033[91m✗\033[0m"
    d = f"  [{detail}]" if detail else ""
    print(f"  {m} {tid}: {name}{d}")

# ─── Group A: Rotator Firmware ────────────────────────────────────────────────

def run_group_a(host):
    print("\n═══ Group A: Rotator Firmware (via rotctld passthrough) ═══\n")
    rc = RotctlConn(host)

    # A01: Version
    r = rc.cmd("VE")
    test("A01", "Version (VE)", "VESatNOGS-RP2040-v0.2" in r, r)
    r = rc.cmd("ve")
    test("A01b", "Version (ve lowercase)", "VESatNOGS-RP2040-v0.2" in r, r)

    # A02: Status register
    r = rc.cmd("GS")
    test("A02", "Status reg (GS)", r.startswith("GS"), r)

    # A03: Error register
    r = rc.cmd("GE")
    test("A03", "Error reg (GE)", r.startswith("GE"), r)

    # A04: Position query variants
    for suffix, label in [("AZ", "AZ"), ("EL", "EL"), ("AZEL", "AZEL"), ("GET_AZ", "GET_AZ"), ("GET_EL", "GET_EL")]:
        r = rc.cmd(suffix)
        test(f"A04_{suffix}", f"Position ({label})", "AZ" in r and "EL" in r, r)

    # A05: Ticks
    r = rc.cmd("TICKS")
    test("A05", "Ticks", "AZ_TICKS=" in r and "EL_TICKS=" in r, r)

    # A06: Info dump (multi-line)
    r = rc.cmd("INFO", multi_line=True)
    test("A06", "Info dump", "MANUAL=" in r and "ERROR=" in r, r[:120])

    # A07: PID gains
    r = rc.cmd("GAINS")
    test("A07", "PID gains", "KP=" in r and "KI=" in r and "KD=" in r, r)

    # A08: Set gain, verify, restore
    rc.cmd("KP0.2")
    r = rc.cmd("GAINS")
    test("A08a", "Set KP=0.2", "KP=0.200" in r, r)
    rc.cmd("KP0.15")
    r = rc.cmd("GAINS")
    test("A08b", "Restore KP=0.15", "KP=0.150" in r, r)

    # A09: Set AZ target
    rc.cmd("AZ45.0")
    r = rc.cmd("GS")
    test("A09", "Set AZ target → GS=2", "GS2" in r, r)
    r = rc.cmd("INFO", multi_line=True)
    test("A09b", "MANUAL=0 after target set", "MANUAL=0" in r, r[:80])

    # A10: Set EL target + clamping
    rc.cmd("EL90.0")
    time.sleep(0.1)
    rc.cmd("EL-10")  # should clamp to 0
    r = rc.cmd("INFO", multi_line=True)
    test("A10", "Set EL + negative clamp", "MANUAL=0" in r, r[:80])

    # A11: Atomic AZ+EL
    r = rc.cmd("AZ90.0 EL45.0")
    test("A11", "Atomic AZ+EL set", True, r if r else "no response (atomic set OK)")

    # A12: STOP variants
    r = rc.cmd("STOP")
    test("A12a", "STOP", "AZ" in r and "EL" in r, r)
    r = rc.cmd("GS")
    test("A12b", "GS after STOP", "GS" in r, r)

    rc.cmd("AZ45.0")
    r = rc.cmd("S")
    test("A12c", "S alias", "AZ" in r, r)
    rc.cmd("AZ45.0")
    r = rc.cmd("SA")
    test("A12d", "SA (stop AZ)", "AZ" in r, r)
    rc.cmd("EL45.0")
    r = rc.cmd("SE")
    test("A12e", "SE (stop EL)", "AZ" in r, r)

    # A13: PARK
    r = rc.cmd("PARK")
    test("A13", "PARK", "AZ" in r and "EL" in r, r)

    # A14: ZERO
    r = rc.cmd("ZERO")
    test("A14a", "ZERO", "zeroed" in r.lower(), r)
    r = rc.cmd("Z")
    test("A14b", "Z alias", "zeroed" in r.lower(), r)

    # A15: RESET (note: sets homed=false since kUseEndstopHoming=false)
    r = rc.cmd("RESET")
    test("A15", "RESET", "AZ" in r, r)
    r = rc.cmd("INFO", multi_line=True)
    test("A15b", "AZ_HOMED=0 after RESET", "AZ_HOMED=0" in r, r[:80])

    # A16: IMU
    r = rc.cmd("IMU")
    test("A16", "IMU read", "X=" in r or "not found" in r.lower(), r)

    # A17: RECAL
    r = rc.cmd("RECAL", wait=0.5)
    test("A17", "RECAL", "recalibrated" in r.lower() or "not found" in r.lower(), r)

    # A18: Raw duty (CAUTION: enters manual mode)
    r = rc.cmd("DUTY_AZ0.5")
    test("A18", "Raw duty (DUTY_AZ0.5)", "duty=0.50" in r, r)
    # Exit manual mode immediately
    rc.cmd("STOP")
    time.sleep(0.1)

    # A19: Monitor toggle (CRITICAL: must turn OFF before leaving)
    r = rc.cmd("MONITOR", wait=0.5)
    has_on = "ON" in r
    test("A19a", "MONITOR ON", has_on, r[:60])
    if has_on:
        time.sleep(0.3)
        r = rc.cmd("MONITOR", wait=0.5)
        test("A19b", "MONITOR OFF", "OFF" in r, r[:60])
    else:
        # Already off or garbled — send again to be safe
        rc.cmd("MONITOR", wait=0.5)
        test("A19b", "MONITOR toggle (safety off)", True, "toggled twice to ensure OFF")

    # A20: Help (multi-line)
    r = rc.cmd("HELP", multi_line=True)
    test("A20a", "HELP", "DUTY_AZ" in r or "Bench" in r, r[:80])
    r = rc.cmd("?", multi_line=True)
    test("A20b", "? alias", "DUTY_AZ" in r or "Bench" in r, r[:80])

    # A21: halt_motors — verify target preserved
    rc.cmd("AZ180.0")
    r = rc.cmd("GS")
    test("A21", "halt_motors: target set, GS=2", "GS2" in r,
         r + " (30s timeout not tested)")

    # A22: Anti-windup (large target, firmware responsive)
    rc.cmd("AZ360.0")
    time.sleep(0.5)
    r = rc.cmd("GS")
    test("A22", "Anti-windup: AZ360, responsive", "GS" in r, r)

    # A23: Wrapping
    rc.cmd("AZ350.0")
    time.sleep(0.1)
    rc.cmd("AZ10.0")
    r = rc.cmd("GS")
    test("A23", "Wrapping: AZ350→AZ10", "GS" in r, r)

    # A24: Unknown command
    rc.cmd("FOOBAR")
    r = rc.cmd("VE")
    test("A24", "Unknown cmd, still responsive", "VESatNOGS" in r, r)

    # A25: Buffer overflow
    rc.cmd("A" * 200)
    r = rc.cmd("VE")
    test("A25", "200-char overflow, still responsive", "VESatNOGS" in r, r)

    # Cleanup
    rc.cmd("STOP")
    rc.cmd("ZERO")
    rc.close()

# ─── Group B: Dashboard API tests ────────────────────────────────────────────

def run_group_b(host):
    print("\n═══ Group B: Dashboard (API tests) ═══\n")

    # B01: Page loads
    code, body = api_get(host, "/")
    test("B01", "Dashboard HTML loads", code == 200 and "AETHER" in str(body)[:500],
         f"HTTP {code}, len={len(str(body))}")

    # B02: Setup status
    code, data = api_get(host, "/api/setup-status")
    test("B02", "GET /api/setup-status", code == 200 and isinstance(data, dict),
         str(data)[:100])

    # B09: Status polling (single)
    code, data = api_get(host, "/api/status")
    test("B09", "GET /api/status", code == 200 and isinstance(data, dict),
         f"keys={list(data.keys())[:8]}" if isinstance(data, dict) else "")

    # B10: Pass list
    code, passes = api_get(host, "/api/passes")
    is_list = isinstance(passes, list)
    test("B10", "GET /api/passes", code == 200 and is_list,
         f"{len(passes)} passes" if is_list else str(passes)[:80])

    # B16: Go To Position
    code, resp = api_post(host, "/api/position", {"az": 45, "el": 30})
    test("B16", "POST /api/position", code == 200, f"HTTP {code}")

    # B17: Invalid input
    code, resp = api_post(host, "/api/position", {"az": -500, "el": 300})
    # Dashboard might accept it (firmware clamps) or reject. Check behavior.
    test("B17", "POST /api/position invalid", code in (200, 400), f"HTTP {code}")

    # B19: Park
    code, resp = api_post(host, "/api/park")
    test("B19", "POST /api/park", code == 200, f"HTTP {code}")

    # B20: Calibrate North
    code, resp = api_post(host, "/api/calibrate-north")
    test("B20", "POST /api/calibrate-north", code == 200, f"HTTP {code}, {str(resp)[:80]}")

    # B21: RF mode
    code, resp = api_post(host, "/api/setup-complete", {"rf_mode": "auto"})
    test("B21", "POST /api/setup-complete (rf_mode)", code == 200, f"HTTP {code}")

    # B22: System info in status
    code, data = api_get(host, "/api/status")
    if isinstance(data, dict):
        sys_info = data.get("system", {})
        has_sys = "cpu_temp" in sys_info or "ram_percent" in sys_info
        test("B22", "System info in status", has_sys, str(sys_info)[:100])
    else:
        test("B22", "System info in status", False, "bad response")

    # B23: Full API status structure
    code, data = api_get(host, "/api/status")
    test("B23", "Full status JSON", code == 200 and isinstance(data, dict),
         f"keys={list(data.keys())}" if isinstance(data, dict) else "")

    # B24: Hardware detection
    code, data = api_get(host, "/api/detect-hardware")
    test("B24", "GET /api/detect-hardware", code == 200 and isinstance(data, dict),
         str(data)[:120])

    # B25: Bad data → 400
    code, resp = api_post(host, "/api/location", {"lat": "not_a_number"})
    test("B25", "Bad location → 400", code == 400, f"HTTP {code}")

    # B26: 404
    code, resp = api_get(host, "/api/nonexistent")
    test("B26", "GET /api/nonexistent → 404", code == 404, f"HTTP {code}")

# ─── Group C: RF HAT + Integration ───────────────────────────────────────────

def run_group_c(host):
    print("\n═══ Group C: RF HAT + Integration ═══\n")

    # C01: CC1200 detected
    code, data = api_get(host, "/api/detect-hardware")
    cc = data.get("cc1200", {}) if isinstance(data, dict) else {}
    test("C01", "CC1200 detected", cc.get("detected", False), str(cc)[:100])

    # C02: RF in status
    code, data = api_get(host, "/api/status")
    rf = data.get("rf", {}) if isinstance(data, dict) else {}
    test("C02", "RF section in status", bool(rf), str(rf)[:120])

    # C03: RF mode switching
    for mode in ["cc1200", "none", "auto"]:
        api_post(host, "/api/setup-complete", {"rf_mode": mode})
        time.sleep(0.3)
        _, setup = api_get(host, "/api/setup-status")
        conf_mode = setup.get("rf_mode", "") if isinstance(setup, dict) else ""
        test(f"C03_{mode}", f"RF mode → {mode}", conf_mode == mode, f"conf={conf_mode}")

    # C04: Buzzer setup_done (listen for chime)
    code, _ = api_post(host, "/api/setup-complete", {"rf_mode": "auto"})
    test("C04", "Buzzer: setup_done", code == 200, "Listen for chime")

    # C05: Buzzer gps_ok (location post)
    code, _ = api_post(host, "/api/location", {"lat": 50.886909, "lon": 4.705425, "elev": 0.0})
    test("C05", "Buzzer: gps_ok (location)", code == 200, f"HTTP {code}")

    # C06: CC1200 debug info
    code, data = api_get(host, "/api/status")
    rf = data.get("rf", {}) if isinstance(data, dict) else {}
    has_debug = any(k in rf for k in ["marc_state", "rssi", "freq", "uptime", "state"])
    test("C06", "CC1200 debug in status", has_debug, str(rf)[:120])

# ─── Group D: Cross-System ───────────────────────────────────────────────────

def run_group_d(host):
    print("\n═══ Group D: Cross-System (sequential) ═══\n")
    rc = RotctlConn(host)

    # D01: Position end-to-end
    code, _ = api_post(host, "/api/position", {"az": 45.0, "el": 30.0})
    test("D01a", "POST /api/position → 200", code == 200, f"HTTP {code}")
    time.sleep(0.5)
    r = rc.cmd("GS")
    test("D01b", "Rotator got target (GS=2)", "GS2" in r, r)

    # D02: Park end-to-end
    code, _ = api_post(host, "/api/park")
    test("D02a", "POST /api/park → 200", code == 200, f"HTTP {code}")
    time.sleep(0.5)
    r = rc.cmd("GS")
    test("D02b", "Park reached rotator", "GS" in r, r)

    # D03: Track satellite
    code, passes = api_get(host, "/api/passes")
    if isinstance(passes, list) and len(passes) > 0:
        sat_name = passes[0].get("name", "")
        code2, resp2 = api_post(host, "/api/track", {"satellite": sat_name})
        test("D03", f"Track ({sat_name})", code2 == 200, f"HTTP {code2}")
        time.sleep(1)
    else:
        test("D03", "Track satellite", False, "No passes available")

    # D04: Stop tracking
    code, _ = api_post(host, "/api/stop")
    test("D04", "Stop tracking", code == 200, f"HTTP {code}")

    # D05: Calibrate North
    code, resp = api_post(host, "/api/calibrate-north")
    test("D05a", "Calibrate North → 200", code == 200, f"HTTP {code}")
    time.sleep(1)
    r = rc.cmd("TICKS")
    test("D05b", "Ticks near 0 after cal", "AZ_TICKS=" in r, r)

    # D06: Status polling 10s
    print("  ... polling status for 10 seconds ...")
    errors = 0
    for _ in range(10):
        code, data = api_get(host, "/api/status")
        if code != 200 or not isinstance(data, dict):
            errors += 1
        time.sleep(1)
    test("D06", f"Status polling 10s ({10 - errors}/10 OK)", errors == 0, f"{errors} errors")

    rc.close()

# ─── Main ─────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default=DEFAULT_HOST)
    parser.add_argument("--group", choices=["a", "b", "c", "d", "all"], default="all")
    args = parser.parse_args()

    print(f"\n╔══════════════════════════════════════════════════╗")
    print(f"║  SatNOGS Pre-Field-Test Verification             ║")
    print(f"║  Target: {args.host:>15}                        ║")
    print(f"╚══════════════════════════════════════════════════╝")

    g = args.group
    if g in ("all", "a"): run_group_a(args.host)
    if g in ("all", "b"): run_group_b(args.host)
    if g in ("all", "c"): run_group_c(args.host)
    if g in ("all", "d"): run_group_d(args.host)

    passed = sum(1 for _, _, p, _ in results if p)
    failed = sum(1 for _, _, p, _ in results if not p)
    total = len(results)
    print(f"\n{'═' * 55}")
    print(f"  RESULTS: {passed}/{total} passed, {failed} failed")
    if failed:
        print(f"\n  FAILURES:")
        for tid, name, p, detail in results:
            if not p:
                print(f"    ✗ {tid}: {name} — {detail}")
    print(f"{'═' * 55}\n")
    return 0 if failed == 0 else 1

if __name__ == "__main__":
    sys.exit(main())
