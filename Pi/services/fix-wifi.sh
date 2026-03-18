#!/bin/bash
# Fix Trixie WiFi rfkill hell — run once, never again.
# Usage: sudo bash fix-wifi.sh
set -e

echo "Fixing WiFi rfkill permanently..."

# 1. Kill systemd-rfkill so nothing can soft-block ever again
systemctl mask systemd-rfkill.service systemd-rfkill.socket 2>/dev/null
systemctl stop systemd-rfkill.service systemd-rfkill.socket 2>/dev/null || true

# 2. Tell NetworkManager to not touch rfkill
mkdir -p /etc/NetworkManager/conf.d
cat > /etc/NetworkManager/conf.d/no-rfkill.conf << 'EOF'
[main]
rc-manager=unmanaged

[device-wifi]
wifi.scan-rand-mac-address=no

[ifupdown]
managed=true
EOF

# 3. Set regulatory domain
iw reg set BE 2>/dev/null || true
if [ -f /boot/firmware/config.txt ]; then
    grep -q "cfg80211.ieee80211_regdom" /boot/firmware/config.txt || \
        echo "dtparam=cfg80211.ieee80211_regdom=BE" >> /boot/firmware/config.txt
fi

# 4. rfkill-unblock service that runs before anything else
cat > /etc/systemd/system/rfkill-unblock.service << 'UNIT'
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
UNIT

systemctl daemon-reload
systemctl enable rfkill-unblock.service

# 5. Unblock and bring up right now
rfkill unblock all
ip link set wlan0 up
systemctl restart NetworkManager

echo "Done. WiFi should be up. Checking in 5s..."
sleep 5
ip addr show wlan0 | grep -E "state|inet "
echo ""
echo "If you see an IP above, you're good. This persists across reboots."
