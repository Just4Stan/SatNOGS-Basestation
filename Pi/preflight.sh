#!/bin/bash
# ==========================================================================
# SatNOGS Basestation — Pre-Flight Test Script
#
# Run this on the Pi via SSH before going outside for a field test.
# Validates the entire boot-to-tracking pipeline.
#
# Usage:
#   ssh pi@<pi-ip> "cd ~/SatNOGS-Basestation/Pi && bash preflight.sh"
#   OR
#   sshpass -p satnogs123 ssh pi@<pi-ip> "cd ~/SatNOGS-Basestation/Pi && bash preflight.sh"
# ==========================================================================

PASS=0
FAIL=0
WARN=0

pass() { echo "  ✓ $1"; ((PASS++)); }
fail() { echo "  ✗ $1"; ((FAIL++)); }
warn() { echo "  ⚠ $1"; ((WARN++)); }

echo "==========================================="
echo " SatNOGS Pre-Flight Checklist"
echo "==========================================="
echo ""

# ------------------------------------------------------------------
# 1. System basics
# ------------------------------------------------------------------
echo "[1/8] System..."

if [ -f /etc/os-release ]; then
    . /etc/os-release
    pass "OS: $PRETTY_NAME"
else
    fail "Cannot read /etc/os-release"
fi

MEM=$(free -m | awk '/Mem:/{print $2}')
if [ "$MEM" -gt 400 ]; then
    pass "RAM: ${MEM}MB total"
else
    warn "RAM: ${MEM}MB (low — may have issues)"
fi

CPU_TEMP=$(cat /sys/class/thermal/thermal_zone0/temp 2>/dev/null)
if [ -n "$CPU_TEMP" ]; then
    TEMP_C=$((CPU_TEMP / 1000))
    if [ "$TEMP_C" -lt 80 ]; then
        pass "CPU temp: ${TEMP_C}°C"
    else
        warn "CPU temp: ${TEMP_C}°C (HOT)"
    fi
fi

# ------------------------------------------------------------------
# 2. WiFi
# ------------------------------------------------------------------
echo ""
echo "[2/8] WiFi..."

if ip addr show wlan0 | grep -q "inet "; then
    IP=$(ip addr show wlan0 | grep "inet " | awk '{print $2}' | cut -d/ -f1)
    pass "wlan0 connected: $IP"
else
    fail "wlan0 has no IP address"
fi

if rfkill list wifi | grep -q "Soft blocked: no"; then
    pass "WiFi not blocked (rfkill OK)"
else
    fail "WiFi is rfkill blocked!"
fi

# ------------------------------------------------------------------
# 3. Systemd services
# ------------------------------------------------------------------
echo ""
echo "[3/8] Services..."

for svc in rfkill-unblock rotctld dashboard station; do
    if systemctl is-enabled "${svc}.service" &>/dev/null; then
        if systemctl is-active "${svc}.service" &>/dev/null; then
            pass "${svc}.service: active"
        else
            if [ "$svc" = "rotctld" ]; then
                warn "${svc}.service: enabled but not active (USB not plugged in?)"
            elif [ "$svc" = "station" ]; then
                warn "${svc}.service: enabled but not active (waiting for setup?)"
            else
                fail "${svc}.service: enabled but not active"
            fi
        fi
    else
        fail "${svc}.service: not enabled"
    fi
done

# wifi-provision is oneshot, check separately
if systemctl is-enabled wifi-provision.service &>/dev/null; then
    pass "wifi-provision.service: enabled"
else
    warn "wifi-provision.service: not enabled (field WiFi setup won't work)"
fi

# ------------------------------------------------------------------
# 4. Serial ports / hardware
# ------------------------------------------------------------------
echo ""
echo "[4/8] Hardware..."

# Rotator USB
if ls /dev/ttyACM* &>/dev/null; then
    PORT=$(ls /dev/ttyACM* | head -1)
    pass "Rotator USB serial: $PORT"
else
    fail "No /dev/ttyACM* — rotator MotorPCB not connected"
fi

# RF HAT UART
if [ -e /dev/serial0 ]; then
    pass "UART serial0: present (RF HAT / CC1200)"
else
    warn "No /dev/serial0 — CC1200 HAT UART not available"
fi

# RTL-SDR
if lsusb 2>/dev/null | grep -qi "RTL2838\|0bda:2838"; then
    pass "RTL-SDR: detected on USB"
else
    warn "No RTL-SDR dongle detected"
fi

# ------------------------------------------------------------------
# 5. Python dependencies
# ------------------------------------------------------------------
echo ""
echo "[5/8] Python..."

for pkg in serial ephem; do
    if python3 -c "import $pkg" 2>/dev/null; then
        pass "python3 $pkg: OK"
    else
        fail "python3 $pkg: NOT INSTALLED"
    fi
done

# dnsmasq
if command -v dnsmasq &>/dev/null; then
    pass "dnsmasq: installed"
else
    warn "dnsmasq: not installed (captive portal won't auto-open)"
fi

# ------------------------------------------------------------------
# 6. Dashboard connectivity
# ------------------------------------------------------------------
echo ""
echo "[6/8] Dashboard..."

if curl -sk https://localhost:5000/api/status -o /tmp/dash_status.json 2>/dev/null; then
    pass "Dashboard HTTPS:5000 responding"

    # Check status content
    if python3 -c "
import json
with open('/tmp/dash_status.json') as f:
    s = json.load(f)
print('setup_complete:', s.get('setup_complete', 'MISSING'))
print('rotator_connected:', s.get('rotator',{}).get('connected', 'MISSING'))
print('location_set:', s.get('location',{}).get('set', 'MISSING'))
" 2>/dev/null; then
        pass "Status JSON: valid"
    else
        fail "Status JSON: parse error"
    fi
else
    fail "Dashboard not responding on https://localhost:5000"
fi

# Test setup-status endpoint
if curl -sk https://localhost:5000/api/setup-status -o /tmp/setup_status.json 2>/dev/null; then
    SETUP=$(python3 -c "import json; print(json.load(open('/tmp/setup_status.json')).get('setup_complete',False))" 2>/dev/null)
    if [ "$SETUP" = "True" ]; then
        pass "Setup wizard: complete"
    else
        warn "Setup wizard: not complete (will show wizard on phone)"
    fi
fi

# Test hardware detection endpoint
if curl -sk https://localhost:5000/api/detect-hardware -o /tmp/hw_detect.json 2>/dev/null; then
    python3 -c "
import json
hw = json.load(open('/tmp/hw_detect.json'))
for k,v in hw.items():
    det = v.get('detected', False)
    print(f'  {k}: {\"detected\" if det else \"not found\"} ({v.get(\"port\") or v.get(\"device\") or \"-\"})')
" 2>/dev/null
    pass "Hardware detection endpoint: OK"
else
    fail "Hardware detection endpoint: not responding"
fi

# ------------------------------------------------------------------
# 7. Rotctld
# ------------------------------------------------------------------
echo ""
echo "[7/8] Rotctld..."

if command -v rotctld &>/dev/null; then
    pass "rotctld binary: found"
else
    fail "rotctld binary: NOT FOUND (install hamlib-utils)"
fi

# Try querying position
ROTCTL_RESP=$(echo "p" | nc -w2 localhost 4533 2>/dev/null)
if [ -n "$ROTCTL_RESP" ]; then
    AZ=$(echo "$ROTCTL_RESP" | head -1)
    EL=$(echo "$ROTCTL_RESP" | tail -1)
    pass "Rotctld responding: AZ=${AZ} EL=${EL}"
else
    warn "Rotctld not responding on localhost:4533"
fi

# ------------------------------------------------------------------
# 8. Station config
# ------------------------------------------------------------------
echo ""
echo "[8/8] Configuration..."

CONF="$HOME/station.conf"
if [ -f "$CONF" ]; then
    pass "station.conf exists"
    python3 -c "
import json
with open('$CONF') as f:
    c = json.load(f)
lat = c.get('lat')
lon = c.get('lon')
rf = c.get('rf_mode', 'not set')
setup = c.get('setup_complete', False)
token = 'configured' if c.get('satnogs_token') else 'not set'
print(f'  Location: {lat}N, {lon}E' if lat else '  Location: NOT SET')
print(f'  RF mode: {rf}')
print(f'  Setup complete: {setup}')
print(f'  SatNOGS token: {token}')
" 2>/dev/null
else
    warn "station.conf: does not exist yet (created by setup wizard)"
fi

# SSL cert
if [ -d "$HOME/.station_ssl" ]; then
    pass "SSL cert directory exists"
else
    warn "No SSL cert dir — will be auto-generated on first dashboard start"
fi

# ------------------------------------------------------------------
# Summary
# ------------------------------------------------------------------
echo ""
echo "==========================================="
echo " Results: $PASS passed, $FAIL failed, $WARN warnings"
echo "==========================================="

if [ "$FAIL" -gt 0 ]; then
    echo ""
    echo "FIX FAILURES BEFORE FIELD TEST!"
    echo "Run: sudo bash ~/SatNOGS-Basestation/Pi/services/setup-pi.sh"
    exit 1
fi

if [ "$WARN" -gt 0 ]; then
    echo ""
    echo "Warnings are non-critical but may affect functionality."
fi

echo ""
echo "Pre-flight checklist complete. Ready for field test."
echo ""
echo "Field test procedure:"
echo "  1. Unplug Pi, take outside with rotator + tripod"
echo "  2. Mount rotator, connect cables, power on"
echo "  3. Wait ~60s for boot"
echo "  4. Connect phone to 'SatNOGS-Setup' WiFi (or known network)"
echo "  5. Open dashboard in browser, follow setup wizard"
echo "  6. Track a satellite!"
