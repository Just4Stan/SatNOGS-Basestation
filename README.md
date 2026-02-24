# SatNOGS Basestation

RP2040-based AZ/EL rotator controller for a SatNOGS-compatible ground station, built around a custom KiCad motor control PCB. Speaks the SatNOGS/EasyComm serial protocol over USB CDC for direct hamlib integration.

## Repository structure

```
Firmware/                RP2040 firmware (PlatformIO, Arduino-Pico core)
MotorPCB/                KiCad 9 schematic + PCB (13 hierarchical sub-sheets)
Hardware/                Datasheets: TB6642FG motor driver, FAPG36-555-EN motor, encoder docs
Images/                  Rotator photos and plots
demo_tracking.py         Simulated satellite pass demo (direct serial)
track_satellite.py       Live satellite tracker via rotctld (requires ephem)
```

## Hardware

### Motor driver (PCB)

- **TB6642FG/FTG** full-bridge DC motor driver (50V max, PWM capable)
- Datasheet: `Hardware/C2150576.pdf`

### Motors

- **FAPG36-555-EN** micro DC planetary gear motor with Hall encoder
- Variant: **24V**, **6mm shaft**, **16 RPM**
- Gearbox ratio: **516:1** 
- Spec sheet: `Hardware/motor_specs_24V_6mm_16rpm.jpg`

### Encoder (FT-555 2-channel Hall)

Two square-wave channels (A/B) in quadrature, 3.3V output level.

Documentation: `Hardware/encoder_documentation.jpg`

### Axis gearing + travel

| Axis | External gearing | Total ratio (516:1 gearbox confirmed) |
|------|-----------------|--------------------------------------|
| Azimuth | 15:40 (2.667:1) | 1376:1 |
| Elevation | 40:55 (1.375:1) | 709.5:1 |

Azimuth is limited, ~**+/-360 deg** around center. Rewind between passes.

### GPIO pin map (RP2040 Pico, from `MotorPCB/motorPCB.net`)

| Function | GPIO | Function | GPIO |
|----------|------|----------|------|
| AZ EN/IN1 | GP15 | EL EN/IN1 | GP27 |
| AZ PH/IN2 | GP14 | EL PH/IN2 | GP28 |
| AZ PWM | GP22 | EL PWM | GP21 |
| AZ ALERT | GP19 | EL ALERT | GP20 |
| AZ ENC A | GP11 | EL ENC A | GP13 |
| AZ ENC B | GP10 | EL ENC B | GP12 |
| AZ END | GP8 | EL END | GP9 |
| Pico LED | GP25 | | |

## Firmware

### Build (PlatformIO)

```sh
pip install platformio          # once
cd Firmware/rp2040-satnogs-rotator
pio run
```

The UF2 is copied to the repo root after each build: `firmware.uf2`

### Flash

1. Hold **BOOTSEL** on the Pico, plug USB in (mounts as **RPI-RP2** drive).
2. Copy `firmware.uf2` onto that drive.
3. Pico reboots, onboard LED starts blinking at 1 Hz.

### Serial monitor

```sh
pio device monitor
```

Or any serial terminal at 115200 baud on the Pico's USB port.

### Serial protocol (EasyComm / SatNOGS compatible)

Response format follows the SatNOGS rotator firmware convention: command prefix echoed in response (e.g. `VE` responds with `VE...`, `GS` with `GS...`). This is required for hamlib model 204 compatibility.

| Command | Response | Description |
|---------|----------|-------------|
| `VE` | `VESatNOGS-RP2040-v0.2` | Firmware version (hamlib strips first 2 chars) |
| `AZ` or `EL` | `AZ45.0 EL30.0` | Query current position |
| `AZ45.0 EL30.0` | *(moves)* | Set target position |
| `GS` | `GS1` or `GS2` | Status: 1=idle, 2=moving |
| `GE` | `GE1`, `GE5`, `GE9` | Error: 1=ok, 4=not homed, 8=fault |
| `RESET` | `AZ0.0 EL0.0` | Start homing sequence |
| `PARK` | `AZ45.0 EL30.0` | Park to (0,0), replies with current pos |
| `S` or `STOP` | `AZ45.0 EL30.0` | Stop all, snap target to current pos |
| `SA` / `SE` | `AZ45.0 EL30.0` | Stop AZ / EL only, replies with pos |

### Bench test / tuning commands

| Command | Response | Description |
|---------|----------|-------------|
| `DUTY_AZ0.3` | `AZ duty=0.30` | Raw AZ motor at 30% (bypasses PID) |
| `DUTY_EL-0.5` | `EL duty=-0.50` | Raw EL motor at 50% reverse |
| `TICKS` | `AZ_TICKS=1234 EL_TICKS=567` | Raw encoder tick counts |
| `INFO` | *(multi-line)* | Full status dump (position, ticks, endstops, alerts) |
| `ZERO` | `Encoders zeroed` | Reset encoder counters to 0 |
| `MONITOR` | `MONITOR ON/OFF` | Toggle live tick stream at 10 Hz |
| `GAINS` | `KP=0.150 KI=0.030 KD=0.020` | Show current PID gains |
| `KP0.3` | `KP=0.300 KI=... KD=...` | Set Kp (also `KI<f>`, `KD<f>`) |
| `IMU` | `X=... Y=... Z=... g EL=... deg` | Read ADXL345 accelerometer |
| `RECAL` | `EL recalibrated from IMU: ...` | Re-calibrate EL from IMU |
| `HELP` | *(command listing)* | Show all available commands |

Use `STOP` to halt motors after `DUTY_*` commands. Sending any `AZ<value>` or `EL<value>` exits manual mode back to the PID controller.

### Control loop

- 100 Hz tick rate, full PID controller with 0.05 deg deadband
- Default gains: Kp=0.15, Ki=0.03, Kd=0.02 (tunable at runtime via `KP`/`KI`/`KD` commands)
- D-term low-pass filtered (alpha=0.15) to prevent derivative spikes
- Duty output smoothing (alpha=0.05) for jerk-free motion
- Anti-windup clamping on integral term (5 deg-seconds max)
- No minimum duty floor — the I-term handles stiction naturally (avoids limit-cycle oscillation)
- Max duty capped at 35% (adjustable in `config.h`)
- 20 kHz PWM switching frequency
- Endstop safety gates prevent driving into a pressed switch
- STOP command snaps target to current position and clears PID state (no resume drift)

### Calibration

The firmware needs a few hardware-specific values before it can point accurately. Edit `Firmware/rp2040-satnogs-rotator/src/config.h`.

**1. Confirm gearing chain**

Gearbox ratio confirmed: **516:1** (88,064 AZ ticks = 360° with 40/15 external gear).

**2. Encoder PPR** (confirmed)

Hall encoder: 16 PPR per channel, 64 edges/rev (quadrature × 4).

**3. Ticks per degree**

```
AZ: 244.6 ticks/deg  (64 × 516 × 40/15 / 360)
EL: 126.1 ticks/deg  (64 × 516 × 55/40 / 360)
```

These values are set in `config.h`.

**4. Verify direction + polarity**

- If AZ/EL move backwards: flip `kAzInvert` / `kElInvert`
- If endstops read wrong: adjust `kEndstopActiveLow`

**5. Protocol smoke test (Pi side)**

```sh
rotctl -m 204 -r /dev/ttyACM0
```

## SatNOGS integration

On the Raspberry Pi running SatNOGS:

```sh
rotctld -m 204 -r /dev/ttyACM0
```

(USB CDC ignores baud rate — no `-s` flag needed.)

Configure `satnogs-client`:
- `SATNOGS_ROT_MODEL=ROT_MODEL_NETROTCTL`
- `SATNOGS_ROT_PORT=localhost:4533`

## Mechanical design

The rotator assembly is designed in Onshape and is fully 3D-printable. It mounts to any standard **1/4"-20 tripod** (camera tripod thread), keeping the total cost low — no custom machining or welded frames required.

<p align="center">
  <img src="Images/rotatorimg1.jpg" width="400" alt="Rotator assembly front view"/>
  <img src="Images/rotatorimg2.jpg" width="400" alt="Rotator assembly side view"/>
</p>

[Onshape CAD document](https://cad.onshape.com/documents/a73149deb7dec0be4e4b4c14/w/e82e548d7d93085f89291bd5/e/c3e50364326e22d368cc64e2?renderMode=0&uiState=699de3ddf7c036f801c48281)

## Design philosophy

- **Economical ground station**: minimal BOM AZ/EL rotator proving satellite tracking without expensive commercial hardware
- **Standard 1/4"-20 tripod mount**: use any camera tripod as a base — no custom mounts needed
- **433 MHz downlink only** for now; lightweight uplink may come later

