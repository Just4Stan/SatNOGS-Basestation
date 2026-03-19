#!/bin/bash
# ==========================================================================
# SatNOGS Basestation — Full Pi 3A+ Setup Script
#
# Run this ONCE on a fresh Trixie (Debian 13) arm64 install.
# Handles everything: WiFi rfkill fix, UART, rotctld, Python deps, services.
#
# Usage:
#   # Copy repo to Pi first, then:
#   cd ~/SatNOGS-Basestation/Pi/services
#   sudo bash setup-pi.sh
#
# After running: reboot, then open https://<pi-ip>:5000 on your phone.
# ==========================================================================
set -e

REPO_DIR="$(cd "$(dirname "$0")/../.." && pwd)"
PI_DIR="$REPO_DIR/Pi"
SERVICES_DIR="$PI_DIR/services"

echo "==========================================="
echo " SatNOGS Basestation — Pi Setup"
echo "==========================================="
echo "Repo: $REPO_DIR"
echo ""

# ------------------------------------------------------------------
# 1. WiFi rfkill fix (Trixie blocks WiFi without regulatory domain)
# ------------------------------------------------------------------
echo "[1/9] WiFi rfkill fix..."

# Kill systemd-rfkill permanently — it re-blocks WiFi on NM restart
systemctl mask systemd-rfkill.service systemd-rfkill.socket 2>/dev/null || true
systemctl stop systemd-rfkill.service systemd-rfkill.socket 2>/dev/null || true

# Tell NetworkManager to not touch rfkill
mkdir -p /etc/NetworkManager/conf.d
cat > /etc/NetworkManager/conf.d/no-rfkill.conf << 'NM_EOF'
[main]
rc-manager=unmanaged

[device-wifi]
wifi.scan-rand-mac-address=no

[ifupdown]
managed=true
NM_EOF

# Set regulatory domain in boot config
BOOT_CONFIG="/boot/firmware/config.txt"
if [ -f "$BOOT_CONFIG" ]; then
    if ! grep -q "cfg80211.ieee80211_regdom" "$BOOT_CONFIG"; then
        echo "" >> "$BOOT_CONFIG"
        echo "# WiFi regulatory domain (Belgium)" >> "$BOOT_CONFIG"
        echo "dtparam=cfg80211.ieee80211_regdom=BE" >> "$BOOT_CONFIG"
        echo "  Added regulatory domain to config.txt"
    else
        echo "  Regulatory domain already in config.txt"
    fi
fi

# Create rfkill-unblock service (runs before networking)
cat > /etc/systemd/system/rfkill-unblock.service << 'RFKILL_EOF'
[Unit]
Description=Unblock WiFi rfkill at boot
DefaultDependencies=no
Before=network-pre.target
Wants=network-pre.target

[Service]
Type=oneshot
ExecStart=/usr/sbin/rfkill unblock all
ExecStart=/usr/sbin/iw reg set BE
ExecStart=/usr/sbin/ip link set wlan0 up
RemainAfterExit=yes

[Install]
WantedBy=sysinit.target
RFKILL_EOF

systemctl daemon-reload
systemctl enable rfkill-unblock.service
echo "  rfkill-unblock.service installed, systemd-rfkill masked"

# Unblock now too
rfkill unblock all 2>/dev/null || true
iw reg set BE 2>/dev/null || true
ip link set wlan0 up 2>/dev/null || true

# ------------------------------------------------------------------
# 2. WiFi network configuration
# ------------------------------------------------------------------
echo "[2/9] WiFi network..."

# Add WiFi network via NetworkManager if nmcli is available
if command -v nmcli &>/dev/null; then
    # Check if connection already exists
    if ! nmcli connection show "maakleerplek" &>/dev/null; then
        nmcli connection add type wifi con-name "maakleerplek" \
            ssid "maakleerplek - cowork" \
            wifi-sec.key-mgmt wpa-psk \
            wifi-sec.psk "3EmmertjesWater" \
            connection.autoconnect yes 2>/dev/null && \
            echo "  Added maakleerplek WiFi connection" || \
            echo "  Failed to add WiFi (add manually)"
    else
        echo "  maakleerplek WiFi already configured"
    fi
else
    # Fallback: wpa_supplicant config
    WPA_CONF="/etc/wpa_supplicant/wpa_supplicant.conf"
    if [ ! -f "$WPA_CONF" ] || ! grep -q "maakleerplek" "$WPA_CONF"; then
        cat >> "$WPA_CONF" << 'WPA_EOF'

network={
    ssid="maakleerplek - cowork"
    psk="3EmmertjesWater"
    key_mgmt=WPA-PSK
}
WPA_EOF
        echo "  Added WiFi to wpa_supplicant.conf"
    else
        echo "  WiFi already in wpa_supplicant.conf"
    fi
fi

# ------------------------------------------------------------------
# 3. UART setup (for RF HAT communication)
# ------------------------------------------------------------------
echo "[3/9] UART setup..."

if [ -f "$BOOT_CONFIG" ]; then
    # Enable UART
    if ! grep -q "^enable_uart=1" "$BOOT_CONFIG"; then
        echo "enable_uart=1" >> "$BOOT_CONFIG"
        echo "  Enabled UART"
    fi
    # Disable Bluetooth (frees /dev/serial0 for RF HAT)
    if ! grep -q "dtoverlay=disable-bt" "$BOOT_CONFIG"; then
        echo "dtoverlay=disable-bt" >> "$BOOT_CONFIG"
        echo "  Disabled Bluetooth overlay"
    fi
fi

# Disable serial console (so /dev/serial0 is free for RF HAT)
if [ -f /boot/firmware/cmdline.txt ]; then
    sed -i 's/ console=serial0,[0-9]*//g' /boot/firmware/cmdline.txt
    sed -i 's/ console=ttyAMA0,[0-9]*//g' /boot/firmware/cmdline.txt
    echo "  Serial console disabled"
fi
systemctl disable serial-getty@ttyAMA0.service 2>/dev/null || true
systemctl stop serial-getty@ttyAMA0.service 2>/dev/null || true

# ------------------------------------------------------------------
# 4. Install system packages
# ------------------------------------------------------------------
echo "[4/9] System packages..."

apt-get update -qq
apt-get install -y -qq python3-pip hamlib-utils git 2>/dev/null
echo "  hamlib-utils (rotctld), python3-pip installed"

# ------------------------------------------------------------------
# 5. Install Python dependencies
# ------------------------------------------------------------------
echo "[5/9] Python dependencies..."

pip install --break-system-packages pyserial ephem 2>/dev/null || \
pip install pyserial ephem
echo "  pyserial, ephem installed"

# ------------------------------------------------------------------
# 6. Install rotctld service
# ------------------------------------------------------------------
echo "[6/9] rotctld service..."

cat > /etc/systemd/system/rotctld.service << 'ROTCTLD_EOF'
[Unit]
Description=Hamlib rotctld for SatNOGS Rotator (EasyComm)
After=dev-ttyACM0.device
BindsTo=dev-ttyACM0.device

[Service]
Type=simple
ExecStart=/usr/bin/rotctld -m 204 -r /dev/ttyACM0 -s 115200 -T 0.0.0.0
Restart=always
RestartSec=5
User=pi

[Install]
WantedBy=multi-user.target
ROTCTLD_EOF

systemctl daemon-reload
systemctl enable rotctld.service
echo "  rotctld.service installed (auto-starts when USB plugged in)"

# ------------------------------------------------------------------
# 7. Install station services
# ------------------------------------------------------------------
echo "[7/9] WiFi provisioning service..."

cp "$SERVICES_DIR/wifi-provision.service" /etc/systemd/system/
# Install dnsmasq for captive portal DNS redirect
apt-get install -y -qq dnsmasq 2>/dev/null
# Disable dnsmasq by default — wifi_provision.py manages it on demand
systemctl disable dnsmasq.service 2>/dev/null || true
systemctl stop dnsmasq.service 2>/dev/null || true

systemctl daemon-reload
systemctl enable wifi-provision.service
echo "  wifi-provision.service installed and enabled"

# ------------------------------------------------------------------
# 8. Install station services
# ------------------------------------------------------------------
echo "[8/9] Station services..."

cp "$SERVICES_DIR/dashboard.service" /etc/systemd/system/
cp "$SERVICES_DIR/station.service"   /etc/systemd/system/

systemctl daemon-reload
systemctl enable dashboard.service
systemctl enable station.service
echo "  dashboard.service + station.service installed and enabled"

# ------------------------------------------------------------------
# 9. SSH key persistence + user setup
# ------------------------------------------------------------------
echo "[9/9] SSH setup..."

# Ensure SSH is enabled
systemctl enable ssh 2>/dev/null || systemctl enable sshd 2>/dev/null || true
systemctl start ssh 2>/dev/null || systemctl start sshd 2>/dev/null || true

# Add pi user to dialout group (for serial port access)
usermod -aG dialout pi 2>/dev/null || true

# Allow pi user passwordless sudo for station management commands
# dashboard.py: systemctl stop/start rotctld, shutdown
# wifi_provision.py (if run manually): nmcli, dnsmasq, killall
cat > /etc/sudoers.d/satnogs << 'SUDOERS_EOF'
pi ALL=(ALL) NOPASSWD: /usr/bin/systemctl stop rotctld.service
pi ALL=(ALL) NOPASSWD: /usr/bin/systemctl start rotctld.service
pi ALL=(ALL) NOPASSWD: /usr/bin/systemctl restart rotctld.service
pi ALL=(ALL) NOPASSWD: /sbin/shutdown
pi ALL=(ALL) NOPASSWD: /usr/sbin/shutdown
pi ALL=(ALL) NOPASSWD: /usr/bin/nmcli
pi ALL=(ALL) NOPASSWD: /usr/sbin/dnsmasq
pi ALL=(ALL) NOPASSWD: /usr/bin/killall dnsmasq
SUDOERS_EOF
chmod 440 /etc/sudoers.d/satnogs
echo "  SSH enabled, pi user in dialout group, sudoers configured"

# ------------------------------------------------------------------
# Done
# ------------------------------------------------------------------
echo ""
echo "==========================================="
echo " Setup complete! Reboot to apply all changes."
echo "==========================================="
echo ""
echo "After reboot:"
echo "  1. Plug in MotorPCB USB → rotctld starts automatically"
echo "  2. If no known WiFi: connect phone to 'SatNOGS-Setup' AP"
echo "  3. Enter your hotspot/WiFi credentials in the captive portal"
echo "  4. Open https://<pi-ip>:5000 on your phone"
echo "  5. Follow the setup wizard (GPS, North, antenna, receiver)"
echo ""
echo "Quick checks after reboot:"
echo "  systemctl status rfkill-unblock    # WiFi fix"
echo "  systemctl status wifi-provision    # WiFi provisioning"
echo "  systemctl status rotctld           # rotator (needs USB)"
echo "  systemctl status dashboard         # web dashboard"
echo "  systemctl status station           # pass tracker"
echo "  ip addr show wlan0                 # WiFi IP"
echo ""
echo "Run: sudo reboot"
