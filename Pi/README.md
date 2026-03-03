# Raspberry Pi Setup — SatNOGS Ground Station

Complete setup guide for the Pi 3 Model A+ that runs the ground station.
The Pi talks to two RP2040 Picos simultaneously:

It should be compatible with any Raspberry Pi from the Pi3+ and beyond so Pi4, Pi5 and Pi Zero as of writing. This is untested


```
                  ┌───────────────────────────────────────────────────┐
                  │  Raspberry Pi 3 Model A+                          │
                  │  Bookworm arm64, IP 10.72.3.105                   │
                  │                                                   │
                  │  ┌────────────┐  ┌────────────┐  ┌────────────┐   │
                  │  │ rotctld    │  │ station.py │  │ satnogs-   │   │
                  │  │ (hamlib)   │  │ capture.py │  │ client     │   │
                  │  │ port 4533  │  │ transmit.py│  │ (optional) │   │
                  │  └─────┬──────┘  └─────┬──────┘  └─────┬──────┘   │
                  │        │               │               │          │
                  │   USB serial      Pi UART          TCP 4533       │
                  │  /dev/ttyACM0    /dev/serial0    → rotctld        │
                  └────────┼───────────────┼───────────────┘          │
                           │               │                          │
                  ┌────────┴────────┐ ┌────┴──────────────────┐       │
                  │  Rotator Pico   │ │  RF HAT Pico          │       │
                  │  EasyComm proto │ │  COBS binary proto    │       │
                  │  (USB CDC)      │ │  (UART 115200)        │       │
                  │                 │ │                       │       │
                  │  AZ/EL motors   │ │  CC1200 UHF + VHF     │       │
                  └─────────────────┘ └───────────────────────┘       │
```

## Current Pi State

| Item | Value |
|------|-------|
| Model | Pi 3 Model A+ (512 MB RAM) |
| OS | Raspberry Pi OS Bookworm arm64 |
| IP | 10.72.3.105 |
| User | pi / satnogs123 |
| WiFi | "maakleerplek - cowork" (NetworkManager/nmcli) |
| USB | Rotator Pico → /dev/ttyACM0 |
| UART | RF HAT Pico → /dev/serial0 (GPIO14 TXD, GPIO15 RXD) |

## What's Already Installed

- `libhamlib-utils` (provides `rotctld`, `rotctl`)
- `rotctld.service` — systemd service, model 204, listening on 0.0.0.0:4533
- SatNOGS client framework (Docker + Ansible) — needs station registration to go live
- Python 3 with `pip`

## RF Data Flow — UHF Receive + VHF Transmit

The ground station uses **Mode V/U**, the standard amateur satellite convention:
- **UHF (432 MHz) = downlink / receive** — AetherSpace CubeSat transmits at 433 MHz
- **VHF (145 MHz) = uplink / transmit** — commands sent to CubeSat at 145.9 MHz

SatNOGS itself is **receive-only** (no uplink). The VHF TX path is custom, outside SatNOGS.

### Data flow during a satellite pass

```
    CelesTrak TLE ──→ PyEphem orbit prediction
                            │
                            ├──→ AZ/EL ──→ rotctld ──→ Rotator Pico ──→ motors
                            │
                            └──→ range rate ──→ Doppler shift
                                                  │
                         ┌────────────────────────┤
                         │                        │
                  UHF RX freq adjust       VHF TX freq adjust
                  (433 MHz ± Δf)           (145 MHz ± Δf)
                         │                        │
                         └────── RF HAT Pico ─────┘
                                    │
                              /dev/serial0
                                    │
                         ┌──────────┴──────────┐
                         │  Packet log file    │
                         │  Console output     │
                         │  SiDS submission    │
                         └─────────────────────┘
```

### Pi-side Python scripts

| Script | Purpose | Radio | Interface |
|--------|---------|-------|-----------|
| `rf_hat.py` | Core protocol library (import, don't run) | Both | UART |
| `capture.py` | Standalone UHF packet capture | UHF (433 MHz) | UART |
| `transmit.py` | Standalone VHF packet transmitter | VHF (145 MHz) | UART |
| `station.py` | Full pass controller (rotator + RX + TX + Doppler) | Both | UART + TCP |
| `dashboard.py` | Web dashboard — live station status page | — | TCP (rotctld) + JSON file |
| `buzzer.py` | Buzzer driver (AOS/LOS/packet beeps via RF HAT Pico) | — | UART (via rf_hat.py) |

### `rf_hat.py` — Protocol library

Shared library imported by all other scripts. Handles:
- **COBS framing** + CRC-16/CCITT-FALSE (matches firmware protocol)
- **CC1200Link** class with threaded RX loop and event queue
- All protocol commands: PING, GET_INFO, SELECT_RADIO, SET_STATE, SET_STREAMING, GET_METRICS
- Register access: read/write normal and extended CC1200 registers
- TX: write to FIFO, transmit, flush
- **Frequency helpers**: `freq_to_regs()`, `regs_to_freq()`, `doppler_shift()`
- **SmartRF profile loading**: parse TI SmartRF Studio exports, apply via register writes

### `capture.py` — UHF Packet Capture

Standalone receiver. Opens UART, selects UHF radio, loads SmartRF profile, enables
RX streaming, and logs all received packets.

```bash
python3 capture.py                                  # defaults (433 MHz)
python3 capture.py --freq 435.0                     # specific frequency
python3 capture.py --log packets.log                # log to file
python3 capture.py --port /dev/ttyUSB0              # alternate serial port
python3 capture.py --duration 300                   # capture for 5 minutes
```

### `transmit.py` — VHF Packet Transmitter

Sends packets on the VHF CC1200 (145 MHz uplink).

```bash
python3 transmit.py "48 65 6C 6C 6F"               # hex payload
python3 transmit.py --ascii "Hello CubeSat"         # ASCII string
python3 transmit.py --file cmd.bin                  # binary file
python3 transmit.py --freq 145.9 "DE ON4SAT"        # custom freq (MHz)
python3 transmit.py --repeat 5 --interval 2 "AA BB" # repeat 5x, 2s apart
python3 transmit.py --power 0x3F "test"             # set PA power level
```

### `dashboard.py` — Web Dashboard & Field Control Center

Self-contained HTTPS dashboard using only the Python standard library (no Flask).
Opens `https://<pi-ip>:5000` in any browser — designed for phone-first field use.

```bash
python3 dashboard.py                    # default: HTTPS on port 5000
python3 dashboard.py --port 8080        # custom port
python3 dashboard.py --no-ssl           # HTTP only (GPS won't work from phone)
```

**Features:**
- **Phone GPS**: tap "Set Location" → browser Geolocation API sends lat/lon/elev to Pi
- **Park Antenna**: sends park command to rotctld
- **Shutdown Pi**: parks antenna, then safely shuts down
- **Live status**: rotator AZ/EL, RF RSSI/packets, pass progress, system health

**HTTPS:** The dashboard auto-generates a self-signed certificate (stored in
`~/.station_ssl/`). This is required because the browser Geolocation API only
works over HTTPS. First visit: tap "Advanced → Proceed" in the browser warning.

**Endpoints:**
| Method | Path | Description |
|--------|------|-------------|
| GET | `/` | HTML dashboard page |
| GET | `/api/status` | JSON status blob (polled every 1s by JS) |
| POST | `/api/location` | Save `{lat, lon, elev}` to `~/station.conf` |
| POST | `/api/park` | Send park command to rotctld |
| POST | `/api/shutdown` | Park + `sudo shutdown -h now` |

**How it works:** Polls rotctld (TCP 4533) for AZ/EL and reads
`~/.station_status.json` (written by `station.py` during a pass) for RF metrics.

### `buzzer.py` — Audio Feedback

Sends beep commands to the RF HAT Pico over UART. The passive buzzer is on **Pico GP2**
(driven via switching transistor + hardware PWM). The Pico plays tones locally —
the Pi just sends a pattern ID.

| Event | Sound | Frequency |
|-------|-------|-----------|
| Station ready | Triple beep | 1000 Hz |
| AOS (satellite rise) | Rising two-tone | 800 → 1200 Hz |
| LOS (satellite set) | Falling two-tone | 1200 → 800 Hz |
| Packet received | Short high beep | 1500 Hz |
| Error | Low tone 0.5s | 400 Hz |

Hardware: passive piezo buzzer on Pico GP4 with NPN/MOSFET switching transistor.
Requires an open CC1200Link (passed to constructor). Falls back gracefully when
no link is available (beeps are silently skipped, e.g., `--no-rf` mode).

### `station.py` — Full Station Controller

The main script for a real satellite pass. Orchestrates all three subsystems:

1. **Rotator tracking** — sends AZ/EL to rotctld at 10 Hz
2. **UHF packet capture** — CC1200 RX streaming at 433 MHz
3. **Doppler correction** — updates CC1200 FREQ registers in real-time (2 Hz)
4. **VHF TX** (optional) — transmit a command packet after the pass

```bash
# Manual operation:
python3 station.py                                  # auto-pick next pass
python3 station.py --sat "ISS"                      # track specific satellite
python3 station.py --list                           # list upcoming passes
python3 station.py --sat "ISS" --no-rf              # rotator only
python3 station.py --sat "ISS" --doppler            # enable Doppler correction
python3 station.py --sat "ISS" --tx cmd.bin         # TX on VHF after pass
python3 station.py --log pass.log                   # log everything to file

# Daemon mode (auto-start, no SSH needed):
python3 station.py --daemon --no-rf --doppler       # wait for GPS, loop passes
```

**Daemon mode** (`--daemon`): designed for systemd auto-start. Waits for
`~/station.conf` (set from phone GPS via dashboard), then continuously:
fetches TLEs → finds passes → tracks them → sleeps → repeats. No SSH needed.

**Station location** is resolved in priority order:
1. `--lat`/`--lon`/`--elev` CLI flags
2. `~/station.conf` (JSON file, set via dashboard GPS or manually)
3. Hardcoded defaults (KU Leuven Campus Geel)

### SmartRF register profiles

| File | Band | Frequency | Status |
|------|------|-----------|--------|
| `configs/smartrf_uhf_435.txt` | UHF (410-480 MHz) | 435 MHz | Verified from SmartRF Studio |
| `configs/smartrf_vhf_145.txt` | VHF (136-160 MHz) | 145.9 MHz | Adapted from UHF — needs VNA tuning |
| `configs/cc1200_regmap_profile.json` | — | — | Register name → address mapping |

The VHF config has only the frequency registers changed (FS_CFG, FREQ2/1/0). The FS synthesizer
registers may need re-tuning via SmartRF Studio for optimal VHF performance.

### Frequency register math

The CC1200 frequency is set by four registers:

```
FS_CFG[3:0]  = FSD_BANDSELECT  →  selects LO divider
FREQ[23:0]   = frequency word   →  sets actual frequency

f_RF = FREQ × f_XOSC / (LO_div × 2^16)
FREQ = round(f_RF × LO_div × 2^16 / f_XOSC)
```

| Band | FSD_BANDSELECT | LO divider | FS_CFG | Frequency range |
|------|---------------|------------|--------|-----------------|
| 820-960 MHz | 0x02 | 4 | 0x12 | — |
| 410-480 MHz | 0x04 | 8 | 0x14 | UHF (433 MHz) |
| 273-320 MHz | 0x06 | 12 | 0x16 | — |
| 164-192 MHz | 0x0A | 20 | 0x1A | — |
| 136-160 MHz | 0x0B | 24 | 0x1B | VHF (145 MHz) |

Crystal frequency: 40 MHz (RF HAT hardware).

**UHF 433 MHz**: FS_CFG=0x14, FREQ=0x560000 → f = 5,636,096 × 40M / (8 × 65536) = 430.0 MHz
(profile default; `set_frequency()` overrides to exact target)

**VHF 145.9 MHz**: FS_CFG=0x1B, FREQ=0x578A3D → f = 5,737,021 × 40M / (24 × 65536) = 145.9 MHz

### Doppler correction

At 433 MHz, a LEO satellite at ~7.5 km/s causes up to ±10 kHz Doppler shift.
At 145 MHz, the shift is about ±3.5 kHz. The CC1200's channel filter is typically
25-100 kHz wide, so Doppler correction is optional for wide filters but important
for narrow filters or coherent demodulation.

`station.py --doppler` computes the shift from PyEphem's `range_velocity` and updates
the CC1200 FREQ registers at 2 Hz during the pass.

## Step-by-Step Setup

### 1. Fresh OS Install (if starting from scratch)

Flash **Raspberry Pi OS Bookworm arm64 Lite** (no desktop needed) using Raspberry Pi Imager.
In the imager settings, set hostname, enable SSH, and configure WiFi.

```bash
# After first boot, SSH in:
ssh pi@10.72.3.105

# Update
sudo apt update && sudo apt upgrade -y
```

### 2. Enable UART for RF HAT

The Pi's hardware UART (GPIO14/15) is used by the RF HAT Pico. On Bookworm, it needs to be
enabled and the serial console disabled (so it doesn't interfere):

```bash
# Add to /boot/firmware/config.txt:
sudo tee -a /boot/firmware/config.txt > /dev/null << 'EOF'

# Enable UART for RF HAT Pico communication
enable_uart=1
dtoverlay=disable-bt
EOF

# Disable serial console (frees /dev/serial0 for our use)
sudo systemctl disable serial-getty@ttyAMA0.service
sudo systemctl stop serial-getty@ttyAMA0.service

# Remove console=serial0 from cmdline.txt if present
sudo sed -i 's/console=serial0,[0-9]* //g' /boot/firmware/cmdline.txt

sudo reboot
```

After reboot, `/dev/serial0` should exist and point to `/dev/ttyAMA0` (the PL011 UART).

**Verify:**
```bash
ls -la /dev/serial0
# Should show: /dev/serial0 -> ttyAMA0

# Quick test (if RF HAT Pico is connected):
pip install pyserial
python3 -c "
import serial
s = serial.Serial('/dev/serial0', 115200, timeout=1)
print('UART open:', s.name)
s.close()
"
```

### 3. Install Rotator Control (hamlib)

```bash
sudo apt install -y libhamlib-utils

# Create systemd service
sudo tee /etc/systemd/system/rotctld.service > /dev/null << 'EOF'
[Unit]
Description=Hamlib rotctld for SatNOGS rotator
After=network.target
StartLimitIntervalSec=60
StartLimitBurst=5

[Service]
Type=simple
ExecStart=/usr/bin/rotctld -m 204 -r /dev/ttyACM0 -s 115200 -T 0.0.0.0
Restart=always
RestartSec=5
User=pi

[Install]
WantedBy=multi-user.target
EOF

sudo systemctl daemon-reload
sudo systemctl enable --now rotctld

# Verify
rotctl -m 2 -r localhost:4533 p    # should return AZ/EL position
```

**Note:** `-m 204` = SatNOGS rotator (EasyComm3). `-T 0.0.0.0` makes it accessible from
any machine on the network (useful for testing from your laptop).

### 4. Install Pi-side Python Tools

```bash
# Install dependencies
pip install pyserial ephem

# Clone the repo on the Pi (or copy just the Pi/ directory):
git clone <repo-url> ~/SatNOGS-Basestation
cd ~/SatNOGS-Basestation/Pi

# Test RF HAT connection
python3 -c "
from rf_hat import CC1200Link
link = CC1200Link()
link.open('/dev/serial0')
import time; time.sleep(0.1)
print('PING:', link.ping())
link.close()
"

# Test UHF capture (Ctrl+C to stop)
python3 capture.py --duration 10

# Test VHF transmit
python3 transmit.py --ascii "test"

# Full station pass
python3 station.py --list              # see upcoming passes
python3 station.py --sat "ISS"         # track ISS with RX
python3 station.py --sat "ISS" --doppler --log iss_pass.log
```

### 5. Install SatNOGS Client (Optional)

The SatNOGS client handles observation scheduling, rotator control, and data submission.
It's Docker-based and needs ~1 GB RAM — tight on the 512 MB Pi 3 A+.

```bash
# Official Ansible installer
curl -sfL https://satno.gs/install | sh -s --

# Run setup wizard
sudo satnogs-setup
```

Configuration values:
```
SATNOGS_API_TOKEN=<from network.satnogs.org after station registration>
SATNOGS_STATION_ID=<your station ID>
SATNOGS_STATION_LAT=51.1
SATNOGS_STATION_LON=4.97
SATNOGS_STATION_ELEV=25
SATNOGS_ROT_MODEL=ROT_MODEL_NETROTCTL
SATNOGS_ROT_PORT=localhost:4533
```

**If RAM is too tight:** Run satnogs-client on a Pi 3B+/4, or skip Docker and run
`rotctld` + `station.py` manually.

## How Everything Runs Together

### Scenario A: Field Deployment (recommended — no SSH needed)

Install services once, then everything auto-starts on boot:

```bash
# One-time setup (via SSH):
bash ~/SatNOGS-Basestation/Pi/services/install.sh
```

After reboot, the field workflow is:
1. Place station, point north, power on
2. Open `https://<pi-ip>:5000` on your phone
3. Tap "Set Location from GPS"
4. Station auto-tracks passes, dashboard shows live status
5. Tap "Park" or "Shutdown" when done

Services: `rotctld` + `dashboard` + `station` (daemon mode) all start on boot.

### Scenario A2: Manual Pass (specific satellite, SSH)

```bash
# rotctld + dashboard are already running as services

# Track a specific satellite:
python3 station.py --sat "ISS" --doppler --log passes/iss.log
```

**What happens:**
1. `station.py` fetches TLEs from CelesTrak
2. Finds the next pass for the requested satellite
3. Connects to `rotctld` (TCP 4533) for rotator control
4. Opens RF HAT via UART, loads UHF SmartRF profile, tunes to 433 MHz
5. Pre-positions rotator to AOS azimuth
6. At AOS: starts RX streaming + begins tracking at 10 Hz
7. During pass: updates AZ/EL, polls for RX packets, optionally corrects Doppler at 2 Hz
8. At LOS: stops RX, parks rotator, logs summary
9. Optionally transmits a VHF command packet after the pass

### Scenario B: Separate Tracking + Capture

Run rotator tracking and RF capture in separate terminals:

```bash
# Terminal 1: rotctld is already running as a service
sudo systemctl status rotctld

# Terminal 2: satellite tracking (sends AZ/EL to rotctld)
python3 ~/SatNOGS-Basestation/track_satellite.py --sat "ISS"

# Terminal 3: UHF packet capture (independent of rotator)
python3 ~/SatNOGS-Basestation/Pi/capture.py --log packets.log
```

### Scenario C: SatNOGS Automated + station.py RF

Run SatNOGS for scheduling and rotator control. Run RF capture as a systemd service:

```bash
sudo tee /etc/systemd/system/rf-capture.service > /dev/null << 'EOF'
[Unit]
Description=RF HAT CC1200 UHF packet capture
After=network.target

[Service]
Type=simple
WorkingDirectory=/home/pi/SatNOGS-Basestation/Pi
ExecStart=/usr/bin/python3 /home/pi/SatNOGS-Basestation/Pi/capture.py --log /home/pi/packets.log
Restart=always
RestartSec=5
User=pi

[Install]
WantedBy=multi-user.target
EOF

sudo systemctl daemon-reload
sudo systemctl enable --now rf-capture
```

## Connection Details

### UART — RF HAT Pico

| Pi Side | RF HAT Pico Side | Wire |
|---------|-------------------|------|
| GPIO14 (TXD) | GP1 (UART0 RX) | Pi TX → Pico RX |
| GPIO15 (RXD) | GP0 (UART0 TX) | Pico TX → Pi RX |
| GND | GND | Common ground |

Settings: 115200 baud, 8N1, no flow control.

On Bookworm with `enable_uart=1` and `dtoverlay=disable-bt`, the UART appears as:
- `/dev/serial0` → symlink to `/dev/ttyAMA0` (PL011 hardware UART)

The protocol is **binary** (not text like the rotator). See
[RF HAT Protocol](../Firmware/rp2040-rf-hat/README.md#binary-protocol-cobs-over-uart) for details.

### USB Serial — Rotator Pico

| Interface | Device | Protocol |
|-----------|--------|----------|
| USB serial | /dev/ttyACM0 | Text-based EasyComm (115200 baud) |

If the Pico disconnects/reconnects, the device name might change (`ttyACM1`, etc.).
To make it persistent, create a udev rule:

```bash
# Find the Pico's serial number
udevadm info /dev/ttyACM0 | grep SERIAL

# Create udev rule for a fixed symlink
sudo tee /etc/udev/rules.d/99-satnogs-rotator.rules > /dev/null << 'EOF'
SUBSYSTEM=="tty", ATTRS{idVendor}=="2e8a", ATTRS{idProduct}=="000a", SYMLINK+="satnogs-rotator"
EOF

sudo udevadm control --reload-rules
sudo udevadm trigger
# Now /dev/satnogs-rotator always points to the Rotator Pico
```

Then update the rotctld service to use `/dev/satnogs-rotator` instead of `/dev/ttyACM0`.

## Quick Test Commands

### Test rotator (from any machine on the network)

```bash
# Query position
echo "p" | nc 10.72.3.105 4533

# Move to AZ 180, EL 45
echo "P 180.0 45.0" | nc 10.72.3.105 4533

# Park (return to 0, 0)
echo "P 0.0 0.0" | nc 10.72.3.105 4533

# Using rotctl directly on the Pi
rotctl -m 2 -r localhost:4533 p           # query
rotctl -m 2 -r localhost:4533 P 90.0 30.0 # move
```

### Test RF HAT (on the Pi)

```bash
cd ~/SatNOGS-Basestation/Pi

# Ping test — verifies UART + COBS + CRC are working
python3 -c "
from rf_hat import CC1200Link
link = CC1200Link()
link.open('/dev/serial0')
import time; time.sleep(0.1)
print('PING:', link.ping())
info = link.get_info()
if info: print(f'Part: 0x{info.part:02X}, Ver: 0x{info.ver:02X}')
link.close()
"

# Quick UHF capture (10 seconds)
python3 capture.py --duration 10

# Quick VHF TX test
python3 transmit.py --ascii "test"

# Full pass with ISS
python3 station.py --sat "ISS" --doppler
```

## Pi Directory Structure

```
Pi/
├── README.md               ← this file (setup + technical reference)
├── user_guide.md           ← field operation guide for AetherSpace team
├── requirements.txt        ← pip dependencies (pyserial, ephem)
├── rf_hat.py               ← CC1200 protocol library (shared)
├── capture.py              ← UHF packet capture script
├── transmit.py             ← VHF packet transmit script
├── station.py              ← full station pass controller (+ daemon mode)
├── dashboard.py            ← HTTPS web dashboard + field control center
├── buzzer.py               ← buzzer driver (sends patterns to RF HAT Pico GP4)
├── services/
│   ├── install.sh              ← one-command service installer
│   ├── dashboard.service       ← systemd: dashboard on boot
│   └── station.service         ← systemd: auto-tracker on boot (daemon mode)
└── configs/
    ├── cc1200_regmap_profile.json  ← register name → address map
    ├── smartrf_uhf_435.txt         ← UHF SmartRF profile (verified)
    └── smartrf_vhf_145.txt         ← VHF SmartRF profile (adapted, needs tuning)
```

## What Still Needs to Be Done

### Already Working
- [x] Rotator Pico firmware (EasyComm, PID, encoders, IMU)
- [x] RF HAT Pico firmware (dual CC1200, COBS protocol, profile loading)
- [x] rotctld systemd service on Pi
- [x] Satellite tracking script (`track_satellite.py`, direct USB serial + rotctld)
- [x] RF HAT Python protocol library (`rf_hat.py`)
- [x] UHF packet capture script (`capture.py`)
- [x] VHF packet transmit script (`transmit.py`)
- [x] Full station controller with Doppler (`station.py` + daemon mode)
- [x] SmartRF profiles for UHF and VHF
- [x] Web dashboard (`dashboard.py`) — HTTPS, phone GPS, park, shutdown
- [x] Buzzer feedback (`buzzer.py`) — AOS/LOS/packet audio cues
- [x] NeoPixel status LEDs on both Picos (rotator + RF HAT)
- [x] Systemd services for auto-start (dashboard + station daemon)
- [x] Phone GPS → `~/station.conf` → automatic station location
- [x] Station location config (`~/station.conf`, CLI flags, or hardcoded defaults)

### TODO
- [ ] **Enable UART on the Pi** — add `enable_uart=1` and `dtoverlay=disable-bt` to config.txt
- [ ] **Test RF HAT over Pi UART** — ping, register read, UHF RX test
- [ ] **VHF SmartRF profile tuning** — run SmartRF Studio for 145 MHz, or verify on VNA
- [ ] **SatNOGS station registration** — register at network.satnogs.org, get API token
- [ ] **SiDS packet submission** — submit decoded packets to SatNOGS DB
- [ ] **SatNOGS pre/post observation hooks** — auto-start/stop RF capture per scheduled pass
- [ ] **udev rules** — persistent device names for both Picos
- [ ] **RTL-SDR** (optional) — for standard SatNOGS waterfall alongside CC1200 capture

## Troubleshooting

### Rotator Pico not appearing as /dev/ttyACM0
- Check USB cable (data cable, not charge-only)
- Try `ls /dev/ttyACM*` — might be ttyACM1 if something else claimed ACM0
- Pi micro-USB undervoltage can brownout USB — use a quality cable and power supply

### UART not working (/dev/serial0 missing)
- Verify `enable_uart=1` is in `/boot/firmware/config.txt`
- Verify `dtoverlay=disable-bt` is in `/boot/firmware/config.txt`
- Verify serial console is disabled: `sudo systemctl status serial-getty@ttyAMA0.service`
- Check `dmesg | grep -i uart` for kernel messages
- On Bookworm, the config file is at `/boot/firmware/config.txt` (not `/boot/config.txt`)

### RF HAT PING fails
- Check UART wiring: Pi GPIO14 (TX) → Pico GP1 (RX), Pi GPIO15 (RX) → Pico GP0 (TX)
- Ensure common GND between Pi and RF HAT
- Verify RF HAT Pico is powered and running (LED should blink)
- Try `python3 -c "import serial; s=serial.Serial('/dev/serial0', 115200); print(s.name)"` to verify port opens

### WiFi drops on Pi 3 A+
- Bookworm uses NetworkManager (not wpa_supplicant)
- Reconfigure: `sudo nmcli dev wifi connect "SSID" password "PASS"`
- Headless WiFi config at first boot is unreliable on Bookworm — configure via ethernet first

### SatNOGS Docker out of memory
- Pi 3 A+ has only 512 MB RAM — Docker + SatNOGS is tight
- Add swap: `sudo dphys-swapfile swapoff && sudo sed -i 's/CONF_SWAPSIZE=.*/CONF_SWAPSIZE=1024/' /etc/dphys-swapfile && sudo dphys-swapfile setup && sudo dphys-swapfile swapon`
- Or use a Pi 3B+ / Pi 4 with more RAM
- Or skip SatNOGS Docker entirely and run rotctld + station.py manually

### rotctld fails to start
- Check if Pico is connected: `ls /dev/ttyACM0`
- Check service status: `sudo systemctl status rotctld`
- Test manually: `rotctld -m 204 -r /dev/ttyACM0 -s 115200 -vvv` (verbose mode)

### No packets received during a pass
- Check RSSI in metrics output — if below -120 dBm, antenna/matching may be wrong
- Verify frequency: `station.py` logs the actual CC1200 frequency readback
- Check MARCSTATE: should be 0x0D (RX) during capture, not 0x01 (IDLE)
- Ensure CC1200 part number reads 0x20 (not 0x00 which means SPI failure)
- Try without Doppler first (`--no-doppler`) to rule out frequency update issues
