# Ground Station User Guide

How to set up, operate, and verify the SatNOGS ground station in the field.
Written for the AetherSpace team — assumes you have the hardware assembled but
haven't operated the station before.

## What You Need

| Item | Notes |
|------|-------|
| Ground station (rotator + RF HAT + Pi, mounted on tripod) | Antenna attached |
| 24V power supply or battery pack | Powers the motors via the motor PCB |
| USB-C power for the Pi | 5V 2.5A minimum (quality cable!) |
| Phone with mobile data | WiFi hotspot for Pi + browser for dashboard |
| Compass app on phone | For north alignment at setup |

Optional but helpful:
- Laptop (for SSH debugging if something goes wrong)
- USB-UART adapter (for debugging RF HAT directly from laptop)
- Extension cable / power strip if using mains power

## Field Setup (5 steps, no laptop needed)

### Step 1: Place the tripod and point north

Find a spot with a clear view of the sky. Avoid buildings, trees, and metal
structures — they block signal and reflect RF. A rooftop or open field is ideal.

**Point the antenna at true north** — this is critical. The rotator assumes
AZ = 0° is true north at power-on.

1. Open a compass app on your phone (iPhone: built-in Compass, Android: Digital Compass)
2. Hold your phone flat, away from the rotator motors
3. Physically rotate the entire tripod until the antenna boom points north
4. Tighten the tripod head so it doesn't drift

Within ±5° is fine — don't stress about sub-degree precision.

### Step 2: Create WiFi hotspot and power on

On your phone, enable **WiFi Hotspot** (iPhone: Settings → Personal Hotspot;
Android: Settings → Hotspot). The Pi auto-connects to a remembered hotspot.

> **First-time setup only:** The Pi needs to be pre-configured to connect to
> your hotspot: `sudo nmcli dev wifi connect "Your Hotspot" password "your_password"`
> The Pi remembers WiFi networks — after the first time it auto-connects.

Power-on sequence:
1. Connect 24V to the motor PCB
2. Connect USB-C to the Pi (boots in ~30 seconds)

**What happens automatically at boot:**
- Pi connects to your phone hotspot
- `rotctld` starts (rotator control)
- `dashboard` starts (web interface on port 5000)
- `station` starts in daemon mode (waits for GPS location)
- Rotator Pico calibrates elevation from IMU
- Triple beep = station ready

### Step 3: Set location from your phone

Open your phone browser and go to:

```
https://<pi-ip>:5000
```

> The Pi's IP is shown in your phone's hotspot "Connected Devices" list.
> First visit: your browser will show a certificate warning — tap **Advanced →
> Proceed** (this is normal, the Pi uses a self-signed certificate for HTTPS).

The dashboard will show **"Location not set"** at the top. Tap
**"Set Location from GPS"** — your phone's GPS sends coordinates to the Pi.

Once set, the station auto-starts tracking the next satellite pass.

### Step 4: Watch it track

The dashboard shows live data:
- **Rotator**: current AZ/EL position
- **Current Pass**: satellite name + progress bar
- **RF**: RSSI, packet count (when RF HAT is connected)
- **System**: CPU temperature, RAM, uptime

The antenna physically tracks satellites across the sky. You can verify by
comparing with **Look4Sat** (Android) or **ISS Detector** (iOS).

### Step 5: Pack up

When done, tap **"Park Antenna"** on the dashboard — the antenna returns to
north/horizontal. Then tap **"Shutdown Pi"** — the Pi parks and powers off safely.

Disconnect power and pack up.

## First-Time Pi Setup

If this is a fresh Pi or the services haven't been installed yet:

```bash
# SSH into the Pi (one-time setup)
ssh pi@<pi-ip>
# Password: satnogs123

# Clone the repo (or copy the Pi/ directory)
git clone <repo-url> ~/SatNOGS-Basestation

# Install services (dashboard + station auto-start on boot)
bash ~/SatNOGS-Basestation/Pi/services/install.sh

# Configure WiFi hotspot
sudo nmcli dev wifi connect "Your Hotspot" password "your_password"

# Reboot — everything starts automatically from now on
sudo reboot
```

After this, no SSH is needed for normal field operation.

## Advanced: Manual Operation (SSH)

If you need to run things manually or debug:

```bash
# Check service status
sudo systemctl status dashboard
sudo systemctl status station
sudo journalctl -u station -f   # live tracker logs

# Manual rotator test
rotctl -m 2 -r localhost:4533 p           # query position
rotctl -m 2 -r localhost:4533 P 45 30     # move to AZ 45, EL 30
rotctl -m 2 -r localhost:4533 P 0 0       # park

# Manual RF HAT test
cd ~/SatNOGS-Basestation/Pi
python3 -c "
from rf_hat import CC1200Link
link = CC1200Link()
link.open('/dev/serial0')
import time; time.sleep(0.1)
print('PING:', link.ping())
info = link.get_info()
if info: print(f'Radios: {info.count}, Part: 0x{info.part:02X}')
link.close()
"
```

## Web Dashboard (Phone/Laptop)

The station includes a web dashboard — your primary control interface in the
field. No SSH needed for normal operation.

### Accessing the dashboard

The dashboard starts automatically on boot (as a systemd service). Open
**https://\<pi-ip\>:5000** on your phone or laptop (same WiFi/hotspot network).

> First visit: your browser shows a certificate warning because the Pi uses
> a self-signed HTTPS certificate. Tap **Advanced → Proceed** — this is
> safe on your private network. HTTPS is required for the phone GPS feature.

### What the dashboard shows

- **Location banner**: shows "Location not set" with GPS button if no location configured
- **Rotator**: current AZ/EL, connected/disconnected
- **RF Transceiver**: RSSI, frequency, packet count, streaming status
- **Current Pass**: satellite name + progress bar (only visible during a pass)
- **Location**: current lat/lon/elev with "Update GPS" button
- **System**: CPU temperature, RAM usage, uptime
- **Controls**: Park Antenna + Shutdown Pi buttons

The page auto-refreshes every second — no manual refresh needed.

### Dashboard controls

| Button | What it does |
|--------|-------------|
| **Set Location from GPS** | Uses your phone's GPS to set the station's lat/lon/elev. Saved to `~/station.conf` — the tracker picks it up automatically. |
| **Update GPS** | Re-reads your phone's GPS (e.g., if you moved the station). |
| **Park Antenna** | Sends the rotator to AZ 0, EL 0 (north, horizontal). |
| **Shutdown Pi** | Parks the antenna, then safely shuts down the Pi. |

### LED Indicators

Both Picos have WS2812B NeoPixel LEDs that show system state at a glance:

**Rotator Pico (GP16, 1 LED):**
| Color | Meaning |
|-------|---------|
| White pulse | Booting / initializing |
| Yellow blink | Homing in progress |
| Green | Idle — ready for commands |
| Blue | Tracking — moving to target |
| Cyan | On target — within deadband |
| Red | Fault — motor error |

**RF HAT Pico (GP5, 4 LEDs):**
| Color | Meaning |
|-------|---------|
| White | Initializing radios |
| Green | Ready — both CC1200s OK |
| Blue | RX streaming active |
| Red | Init fault — CC1200 failed |

All 4 RF HAT LEDs show the same colour (used as a status strip).

### Buzzer Sounds

A passive buzzer on the RF HAT Pico (GP4) gives audio cues:
- **Triple beep** at startup: station ready
- **Rising two-tone** (low→high): satellite AOS (acquisition of signal)
- **Falling two-tone** (high→low): satellite LOS (loss of signal)
- **Short high beep**: packet received
- **Low tone**: error

You can hear these from several meters away — useful when you're watching the
antenna move and want to know when a pass starts/ends.

## Tracking a Satellite

### Option A: Full automated pass (recommended)

```bash
cd ~/SatNOGS-Basestation/Pi

# See what's coming up
python3 station.py --list

# Track a specific satellite with UHF capture + Doppler correction
python3 station.py --sat "ISS" --doppler --log passes/iss.log

# Or just auto-pick the next good pass
python3 station.py --doppler --log passes/auto.log
```

**What you'll see on screen:**

```
[14:20:01.234] ========== PASS: ISS (ZARYA) ==========
[14:20:01.235]   Max EL: 72.3 deg
[14:20:01.235]   Duration: 412s
[14:20:01.236] Slewing to AOS: AZ 215.3 EL 0.0
[14:20:15.100] Waiting 123s for AOS...
[14:22:18.500] ** AOS — pass started **
[14:22:18.502] RF HAT: UHF RX streaming enabled
  AZ  220.3  EL  5.2 |##                  | pkts=0     T-  400s
  AZ  235.1  EL 22.7 |#####               | pkts=0     T-  350s
[14:24:30.100] RX [1] 24 bytes: 55 55 7A 0E ... (first packet!)
  AZ  290.4  EL 72.1 |################    | pkts=3     T-  200s
[14:28:30.500] ** LOS — pass complete **
[14:28:30.501] Pass summary: 7 packets, 168 bytes
```

### Option B: Rotator-only tracking (no RF)

If you just want to test the rotator mechanics:

```bash
# station.py with --no-rf flag
python3 station.py --sat "ISS" --no-rf

# Or use the standalone tracker (no RF HAT needed)
python3 ~/SatNOGS-Basestation/track_satellite.py --sat "ISS"
```

Watch the antenna physically follow the satellite across the sky.

### Option C: RF-only capture (no rotator)

If the rotator isn't working but you want to test the CC1200:

```bash
# Point the antenna manually at the sky, then:
python3 capture.py --freq 433.0 --log test_capture.log --duration 300
```

## How to Know It's Working

### Visual checks

| What to look at | What "working" looks like |
|-----------------|--------------------------|
| Antenna movement | Smoothly tracks from horizon to horizon during a pass |
| Antenna at AOS | Pre-positions to the rise azimuth before the satellite arrives |
| Antenna at LOS | Returns to park position (0°, 0°) after the pass |
| Console output | Shows increasing packet count and RSSI readings |
| RSSI during pass | Rises as satellite approaches, peaks at max elevation, falls at LOS |

### Using your phone to verify

Install **Look4Sat** (Android, free) or **ISS Detector** (iOS/Android):

1. Open the app and select the same satellite you're tracking
2. Hold your phone up to the sky — it shows where the satellite is with AR overlay
3. Compare the satellite's position on your phone with where the antenna is pointing
4. They should match within a few degrees

This is the easiest "is my rotator actually tracking correctly?" test.

### Understanding the metrics

During a pass, `station.py` logs periodic metrics:

```
METRICS: MARC=0x0D RXBYTES=0 RSSI=-95.2 dBm | pkts=0 bytes=0 T-380s
```

| Metric | What it means | Good value |
|--------|---------------|------------|
| MARC | CC1200 state machine | 0x0D = RX active (good) |
| RXBYTES | Bytes in RX FIFO | 0 when idle, >0 when receiving |
| RSSI | Signal strength | > -100 dBm means something's there |
| pkts | Received packets | Any packets = success! |

**Typical RSSI values:**
- -120 dBm: noise floor (no signal)
- -100 dBm: weak signal, might decode
- -80 dBm: good signal
- -60 dBm: strong signal (satellite overhead)

### First satellite to try

For your first test, track **ISS (ZARYA)** — it's the brightest and strongest
satellite. Even with an omnidirectional antenna you can often detect it.

For AetherSpace specifically: use whatever frequency and modulation parameters
the AetherSpace team provides. The default 433 MHz / 2-GFSK config matches
the expected downlink.

## Transmitting (VHF Uplink)

**Important:** Only transmit if you have proper amateur radio licensing and
the AetherSpace team has authorized uplink commands.

```bash
cd ~/SatNOGS-Basestation/Pi

# Send a hex command packet
python3 transmit.py "DE 4F 4E 34 53 41 54"

# Send an ASCII string
python3 transmit.py --ascii "Hello AetherSpace"

# Send a binary command file
python3 transmit.py --file commands/activate.bin

# Transmit with station.py during a pass (sends after LOS)
python3 station.py --sat "AETHERSPACE" --tx commands/beacon_request.bin
```

## Connecting to SatNOGS Network

Once the station is verified working, you can register it on the global
SatNOGS network:

1. Create an account at [network.satnogs.org](https://network.satnogs.org)
2. Register your station (latitude, longitude, elevation, antenna type)
3. Get your API token and station ID
4. Configure `satnogs-client` on the Pi (see Pi/README.md step 5)
5. Your station appears on the global map and can receive scheduled observations

The SatNOGS web dashboard shows:
- Your station's success rate
- Waterfall plots from each observation
- Decoded telemetry data
- Upcoming scheduled passes

## Shutting Down

**From your phone (recommended):**
Open the dashboard and tap **Shutdown Pi**. This parks the antenna first,
then safely powers off.

**From SSH:**
```bash
rotctl -m 2 -r localhost:4533 P 0 0    # park
sudo shutdown -h now                    # power off
```

Always park before shutdown to avoid leaving the antenna in an awkward
position that might stress cables.

## Troubleshooting in the Field

### "I can't SSH to the Pi"

1. Check your phone hotspot is active and the Pi is listed as connected
2. The Pi might have a different IP — check your phone's hotspot settings
   for connected devices
3. Try `ssh pi@raspberrypi.local` (mDNS, works on most networks)
4. Last resort: connect a monitor and keyboard directly

### "The antenna isn't moving"

1. Check 24V power to the motor PCB (measure with multimeter if available)
2. `sudo systemctl status rotctld` — is it running?
3. `rotctl -m 2 -r localhost:4533 p` — does it respond?
4. Try a manual command: `rotctl -m 2 -r localhost:4533 P 90 45`
5. If no response: check USB cable from Pi to rotator Pico (`ls /dev/ttyACM*`)

### "RF HAT doesn't respond"

1. Check UART wiring (Pi GPIO14 → Pico GP1, Pi GPIO15 → Pico GP0, common GND)
2. Is `/dev/serial0` present? (`ls -la /dev/serial0`)
3. Is the RF HAT Pico powered? (LED should be on)
4. Run ping test: `python3 -c "from rf_hat import *; ..."`

### "No packets received during a pass"

This is normal for most passes! Not every satellite transmits continuously,
and your antenna may not have enough gain. Things to check:
- RSSI: if it stays at -120 dBm, you're not receiving anything (check antenna connection, frequency)
- MARCSTATE: should be 0x0D during RX. If it's 0x01 (IDLE), the radio isn't in RX mode
- Frequency: verify the CC1200 frequency readback matches your target
- Profile: make sure the SmartRF profile was loaded successfully
- Try a **known strong satellite** first (ISS, NOAA weather sats) before AetherSpace

### "The antenna is pointing in the wrong direction"

1. Was the station pointed at true north when powered on?
2. Check with your phone compass + Look4Sat: where is the satellite vs where is the antenna?
3. If there's a consistent offset, you can compensate:
   - Power off, physically re-align to north, power on again
   - Or note the offset and add it mentally (a firmware AZ offset command could be added)

## Tips for Successful Passes

1. **Start with high-elevation passes** (max EL > 40°). Low passes are weak and short.
2. **ISS is the best first target** — very strong, frequent passes, easy to verify visually at dusk/dawn.
3. **Don't worry about Doppler for the first test.** The CC1200's RX filter is wide enough for most cases. Add `--doppler` once basic tracking works.
4. **Log everything.** Use `--log` to save pass data. You'll want this for the thesis.
5. **Check the weather.** Rain doesn't affect UHF much, but wet connectors and wind loading on the antenna do.
6. **Battery life:** The Pi + motors draw about 5-10W average during tracking. A 12V/24V field battery with a DC-DC converter can run the station for hours.
