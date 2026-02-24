# SatNOGS Basestation

RP2040-based AZ/EL rotator controller for a SatNOGS-compatible ground station. Custom KiCad motor control PCB, PID closed-loop control, and EasyComm/hamlib protocol over USB CDC.

<p align="center">
  <img src="Images/rotatorimg1.jpg" width="400" alt="Rotator assembly"/>
  <img src="Images/rotatorimg2.jpg" width="400" alt="Rotator assembly"/>
</p>

<p align="center">
  <b>3D-printable rotator</b> · <b>1/4"-20 tripod mount</b> · <b>SatNOGS compatible</b>
</p>

---

## Features

- Full PID motor control with encoder feedback (100 Hz loop)
- EasyComm serial protocol — plug-and-play with hamlib (`rotctl -m 204`)
- ADXL345 IMU for automatic elevation calibration at startup
- Runtime PID tuning, live telemetry, and raw motor control via serial
- Endstop homing, soft limits, driver fault detection

## Repository structure

```
Firmware/                RP2040 firmware (PlatformIO, Arduino-Pico core)
MotorPCB/                KiCad 9 schematic + PCB (13 hierarchical sub-sheets)
Hardware/                Datasheets: TB6642FG motor driver, FAPG36-555-EN motor
demo_tracking.py         Simulated satellite pass demo (direct serial)
track_satellite.py       Live satellite tracker via rotctld (requires ephem)
```

## Mechanical design

The rotator is designed in Onshape and fully 3D-printable. Mounts to any standard **1/4"-20 camera tripod**.

[View CAD on Onshape](https://cad.onshape.com/documents/a73149deb7dec0be4e4b4c14/w/e82e548d7d93085f89291bd5/e/c3e50364326e22d368cc64e2?renderMode=0&uiState=699de3ddf7c036f801c48281)

## Hardware

| Component | Details |
|-----------|---------|
| **MCU** | RP2040 (Raspberry Pi Pico) |
| **Motor driver** | TB6642FG/FTG full-bridge (50V, PWM) |
| **Motors** | FAPG36-555-EN, 24V, 16 RPM, 516:1 gearbox |
| **Encoders** | 2-channel Hall (FT-555), 64 edges/rev quadrature |
| **IMU** | ADXL345 accelerometer (SPI) |
| **Antenna** | LPRS YAGI-434A, 434 MHz, 10 dBi |

### Axis gearing

| Axis | External ratio | Total ratio | Ticks/degree |
|------|---------------|-------------|-------------|
| Azimuth | 15:40 (2.667:1) | 1376:1 | 244.6 |
| Elevation | 40:55 (1.375:1) | 709.5:1 | 126.1 |

AZ travel: **+/-360 deg** (no slip ring, rewinds between passes).

### GPIO pin map

| Function | GPIO | Function | GPIO |
|----------|------|----------|------|
| AZ IN1 | GP15 | EL IN1 | GP27 |
| AZ IN2 | GP14 | EL IN2 | GP28 |
| AZ PWM | GP22 | EL PWM | GP21 |
| AZ ALERT | GP19 | EL ALERT | GP20 |
| AZ ENC A | GP11 | EL ENC A | GP13 |
| AZ ENC B | GP10 | EL ENC B | GP12 |
| AZ END | GP8 | EL END | GP9 |

## Firmware

### Quick start

```sh
pip install platformio
cd Firmware/rp2040-satnogs-rotator
pio run                     # build
```

Flash: hold **BOOTSEL**, plug USB, copy `firmware.uf2` to the **RPI-RP2** drive.

Connect: `pio device monitor` or any serial terminal at 115200 baud.

### Serial protocol (EasyComm / hamlib compatible)

| Command | Response | Description |
|---------|----------|-------------|
| `VE` | `VESatNOGS-RP2040-v0.2` | Firmware version |
| `AZ` or `EL` | `AZ45.0 EL30.0` | Query position |
| `AZ45.0 EL30.0` | *(moves)* | Set target |
| `GS` | `GS1` / `GS2` | Status: 1=idle, 2=moving |
| `GE` | `GE1` / `GE5` / `GE9` | Error: 1=ok, 4=unhomed, 8=fault |
| `RESET` | `AZ0.0 EL0.0` | Home axes |
| `PARK` | position | Park to (0,0) |
| `STOP` | position | Stop all, snap target to current |

### Tuning & debug commands

| Command | Description |
|---------|-------------|
| `DUTY_AZ0.3` / `DUTY_EL-0.5` | Raw motor duty (-1..1, bypasses PID) |
| `TICKS` | Raw encoder counts |
| `INFO` | Full status dump |
| `MONITOR` | Toggle live telemetry at 10 Hz |
| `GAINS` | Show PID gains |
| `KP0.3` / `KI0.05` / `KD0.1` | Set PID gains at runtime |
| `IMU` | Read accelerometer |
| `RECAL` | Re-calibrate EL from IMU |
| `HELP` | List all commands |

### Control loop

- **100 Hz** PID with 0.05 deg deadband
- Default gains: Kp=0.15, Ki=0.03, Kd=0.02 (runtime tunable)
- D-term low-pass filtered, duty output smoothed for jerk-free tracking
- Anti-windup, endstop safety gates, driver fault detection
- 20 kHz PWM, max duty 95% (35 deg/s slew, full 360° in ~10s)

### Slew speed vs duty

| Duty | Speed (deg/s) | 360° time |
|------|--------------|-----------|
| 35%  | 11.2         | 32s       |
| 55%  | 18.9         | 19s       |
| 75%  | 26.9         | 13s       |
| 95%  | 35.0         | 10s       |

## SatNOGS integration

Tested end-to-end on **Raspberry Pi 3 Model A+** (Bookworm arm64) with the Pico connected via USB.

### Raspberry Pi setup

```sh
# Install hamlib
sudo apt install libhamlib-utils

# Create systemd service for rotctld
sudo tee /etc/systemd/system/rotctld.service > /dev/null << 'EOF'
[Unit]
Description=Hamlib rotctld for SatNOGS rotator
After=network.target

[Service]
Type=simple
ExecStart=/usr/bin/rotctld -m 204 -r /dev/ttyACM0 -s 115200 -T 0.0.0.0
Restart=always
RestartSec=5
User=pi

[Install]
WantedBy=multi-user.target
EOF

sudo systemctl enable --now rotctld
```

### Verify from any machine on the network

```sh
# Query position
echo "p" | nc <pi-ip> 4533

# Move to AZ 90, EL 30
echo "P 90.0 30.0" | nc <pi-ip> 4533
```

### Real satellite tracking

```sh
# On the Pi (requires: pip install ephem pyserial)
python3 track_satellite.py --list              # upcoming passes
python3 track_satellite.py --sat "ISS"         # track specific satellite
python3 track_satellite.py                     # auto-pick next pass
```

### SatNOGS client (optional)

```sh
curl -sfL https://satno.gs/install | sh -s --    # Ansible-based install
sudo satnogs-setup                                # configure station
```

Configure `satnogs-client`:
- `SATNOGS_ROT_MODEL=ROT_MODEL_NETROTCTL`
- `SATNOGS_ROT_PORT=localhost:4533`

> **Note**: The SatNOGS Docker install requires ~1 GB RAM. A Pi 3 A+ (512 MB) can run it but is tight. Pi 3B+ or Pi 4 recommended for the full SatNOGS stack.

## Design philosophy

- **Economical**: minimal BOM, proving satellite tracking without expensive commercial hardware
- **Reproducible**: 3D-printable rotator on a standard camera tripod
- **433 MHz downlink** for AetherSpace CubeSat; lightweight uplink possible later
