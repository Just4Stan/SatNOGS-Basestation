# SatNOGS Ground Station — Quick Start Guide

## What's in the box
- Rotator unit (AZ/EL motors + 3D-printed chassis)
- Raspberry Pi 3A+ (pre-configured, mounted on motor PCB or separate)
- 24V power supply + barrel jack cable
- USB cable (Pi to MotorPCB)
- Antenna (attach later — NOT during setup)

## Setup (5 minutes)

### 1. Assemble
1. Screw rotator onto tripod (standard 1/4"-20 thread)
2. Connect **24V power** to the motor board
3. Connect **USB cable** from MotorPCB to the Pi
4. Connect **Pi power** (micro-USB, use a quality cable)
5. **DO NOT attach the antenna yet** — wait for the app to tell you

### 2. Connect your phone
1. Wait ~60 seconds for the Pi to boot
2. On your phone, look for WiFi network **"SatNOGS-Setup"**
3. Connect to it (no password)
4. A setup page should open automatically
5. Enter your **phone hotspot** or **home WiFi** name and password
6. The Pi will connect to your WiFi and redirect you to the dashboard

> **Tip:** If the setup page doesn't open automatically, go to **http://192.168.4.1** in your browser.

> **Already on the same WiFi?** If the Pi is on a known network, skip this step. Open **https://\<pi-ip\>:5000** directly (accept the certificate warning).

### 3. Follow the setup wizard
The dashboard guides you through 5 steps:

| Step | What to do |
|------|-----------|
| **GPS** | Tap "Use My GPS" to share your phone's location |
| **Point North** | Use your phone compass or a real compass. Rotate the station until it faces North. Tap Confirm. |
| **Attach Antenna** | The mount is now level. Attach your antenna horizontally. Tap Done. |
| **Choose Receiver** | Pick CC1200 HAT, RTL-SDR, or None |
| **Ready!** | You're set — go to the tracker |

### 4. Track satellites
- The **Passes** tab shows upcoming satellites
- Tap **TRACK** on any satellite to start tracking
- Watch live AZ/EL position and signal data in the **Status** tab
- Enable **Auto-Track** to let the station pick passes automatically

### 5. Pack up
1. Tap **Park** (returns antenna to North/level)
2. Tap **Shutdown** (safely powers off the Pi)
3. Wait 10 seconds, then unplug power
4. Remove antenna, fold tripod

## Troubleshooting

| Problem | Solution |
|---------|----------|
| "SatNOGS-Setup" WiFi doesn't appear | Wait 60s. If still missing, check Pi power (solid red LED = power OK) |
| Dashboard won't load | Accept the browser certificate warning (it's a self-signed cert, this is normal) |
| "No rotator connected" | Check USB cable from Pi to MotorPCB. Try a different cable. |
| Antenna not moving | Check 24V power supply is connected and switched on |
| No satellites in pass list | Wait 30s — the station is downloading orbital data. Needs internet. |
| Phone compass seems wrong | Move phone away from the motors (magnets interfere). Or use a real compass. |

## LED status (on rotator board)
- **Green** = idle, ready
- **Blue** = tracking a satellite
- **Cyan** = on target (pointing at satellite)
- **Red** = fault (check USB + power)
