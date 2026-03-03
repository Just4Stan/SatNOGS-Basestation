#!/bin/bash
# Install SatNOGS ground station systemd services.
# Run on the Pi: bash install.sh
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

echo "Installing SatNOGS ground station services..."

# Install Python dependencies
pip install pyserial ephem 2>/dev/null || pip install --break-system-packages pyserial ephem

# Copy service files
sudo cp "$SCRIPT_DIR/dashboard.service" /etc/systemd/system/
sudo cp "$SCRIPT_DIR/station.service"   /etc/systemd/system/

# Reload and enable
sudo systemctl daemon-reload
sudo systemctl enable dashboard.service
sudo systemctl enable station.service

# Start dashboard immediately (station waits for GPS location)
sudo systemctl start dashboard.service

echo ""
echo "Services installed:"
echo "  dashboard.service — starts on boot (HTTPS on port 5000)"
echo "  station.service   — starts on boot (waits for GPS via dashboard)"
echo ""
echo "Commands:"
echo "  sudo systemctl status dashboard    # check dashboard"
echo "  sudo systemctl status station      # check tracker"
echo "  sudo journalctl -u station -f      # live tracker logs"
echo ""
echo "Next: open https://<pi-ip>:5000 on your phone and set GPS location."
