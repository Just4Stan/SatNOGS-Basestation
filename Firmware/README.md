# Firmware — SatNOGS Ground Station Software System

Two RP2040 Pico microcontrollers, controlled by a single Raspberry Pi 3 Model A+,
form the ground station electronics:

```
                          ┌─────────────────────────────────────────────────────┐
                          │  Raspberry Pi 3 Model A+                            │
                          │  (Bookworm arm64, 10.72.3.105)                      │
                          │                                                     │
                          │  ┌──────────────┐   ┌────────────────────────────┐  │
                          │  │ rotctld      │   │ Python packet capture      │  │
                          │  │ model 204    │   │ (COBS/UART protocol)       │  │
                          │  │ port 4533    │   │                            │  │
                          │  └──────┬───────┘   └────────────┬───────────────┘  │
                          │         │                        │                  │
                          │    USB serial              UART (GPIO14/15)         │
                          │    /dev/ttyACM0            115200 baud              │
                          └─────────┼────────────────────────┼──────────────────┘
                                    │                        │
                          ┌─────────┴─────────┐    ┌─────────┴─────────────────┐
                          │  Rotator Pico     │    │  RF HAT Pico              │
                          │  (Motor Control)  │    │  (CC1200 Controller)      │
                          │                   │    │                           │
                          │  USB CDC serial   │    │  UART0 (GP0/GP1)          │
                          │  EasyComm protocol│    │  COBS binary protocol     │
                          │                   │    │                           │
                          │  ┌──────┐ ┌──────┐│    │  ┌───────┐  ┌───────┐     │
                          │  │AZ mot│ │EL mot││    │  │CC1200 │  │CC1200 │     │
                          │  │TB6642│ │TB6642││    │  │UHF    │  │VHF    │     │
                          │  │IN1/2 │ │IN1/2 ││    │  │432 MHz│  │144 MHz│     │
                          │  │+PWM  │ │+PWM  ││    │  │SPI1   │  │SPI0   │     │
                          │  └──────┘ └──────┘│    │  └───────┘  └───────┘     │
                          │  ┌──────┐ ┌──────┐│    │                           │
                          │  │AZ enc│ │EL enc││    │  ┌───────┐  ┌───────┐     │
                          │  │quad  │ │quad  ││    │  │UHF    │  │VHF    │     │
                          │  │IRQ   │ │IRQ   ││    │  │SMA    │  │SMA    │     │
                          │  └──────┘ └──────┘│    │  │antenna│  │antenna│     │
                          │  ┌──────┐ ┌──────┐│    │  └───────┘  └───────┘     │
                          │  │ADXL  │ │end-  ││    │                           │
                          │  │345   │ │stops ││    │                           │
                          │  │SPI0  │ │GPIO  ││    │                           │
                          │  └──────┘ └──────┘│    │                           │
                          └───────────────────┘    └───────────────────────────┘
```

## How the System Works

### 1. Rotator (antenna pointing)

The **Rotator Pico** sits on the Motor Control PCB (`MotorPCB/`) and drives two FAPG36-555-EN
gear motors (24V, 516:1 internal gearbox) through TB6642FG H-bridge drivers. Quadrature encoders
on each axis provide position feedback. An ADXL345 accelerometer on SPI0 gives absolute elevation
at boot from the gravity vector.

The Pico speaks the **SatNOGS EasyComm** protocol over **USB CDC serial** at 115200 baud.
On the Pi, `rotctld` (hamlib model 204) connects to `/dev/ttyACM0` and exposes a TCP socket
(port 4533). `satnogs-client` (or any hamlib client) sends pointing commands via `rotctl`.

This path is fully stock SatNOGS — no custom Pi software is needed for rotator control.

**Key specs:**
- AZ: 244.6 encoder ticks/degree (64 CPR x 516:1 x 40/15 external)
- EL: 126.1 encoder ticks/degree (64 CPR x 516:1 x 55/40 external)
- PID control at 100 Hz (Kp=0.15, Ki=0.03, Kd=0.02)
- Slew speed: ~35 deg/s at 95% duty
- Soft limits: AZ +/-360 deg, EL 0-180 deg

### 2. RF Front-End (packet transceiver)

The **RF HAT Pico** sits on the RF HAT PCB (`RF_HAT/`) — a Pi 3A+ HAT carrying two TI CC1200
sub-GHz transceivers. The Pico controls both CC1200s over SPI and communicates with the Pi
over **UART** (GP0/GP1, 115200 baud, wired to Pi GPIO14/15).

This is the same architecture used by the [M17 CC1200 HAT reference design](RF_HAT/reference_designs/M17_CC1200_HAT/).
The binary protocol uses **COBS framing** with **CRC-16/CCITT-FALSE** integrity checks.

The CC1200 is a hardware packet modem — it handles modulation/demodulation, sync word detection,
and CRC checking internally. It does **not** produce IQ samples like an SDR.

**Key specs:**
- UHF: 432 MHz CC1200 on SPI1 (GP10-13)
- VHF: 144 MHz CC1200 on SPI0 (GP16-19)
- Default config: 2-GFSK, ~2.4 kbps, ~5 kHz deviation, +14 dBm TX
- Dual-radio select via protocol (MSG_SELECT_RADIO)
- SmartRF Studio register profiles loaded at runtime

### 3. Pi Software Stack

The Raspberry Pi runs:

| Component | Purpose | Interface |
|-----------|---------|-----------|
| `rotctld` | hamlib rotator daemon | USB serial → Rotator Pico |
| `satnogs-client` | SatNOGS scheduler + pass execution | TCP → rotctld (port 4533) |
| Packet capture script | CC1200 control + frame logging | UART → RF HAT Pico |

**Data flow during a satellite pass:**

1. `satnogs-client` schedules a pass from the SatNOGS network
2. `satnogs-client` sends AZ/EL pointing commands via `rotctld` → Rotator Pico tracks the satellite
3. A Python script (or systemd service) on the Pi:
   - Selects the correct radio (UHF or VHF) via `MSG_SELECT_RADIO`
   - Loads the appropriate SmartRF register profile via `PROFILE_BEGIN` / `PROFILE_CHUNK` / `PROFILE_APPLY`
   - Enables RX streaming via `MSG_SET_STREAMING`
   - Reads incoming packets from `EVT_RX_DATA` events
   - Optionally applies Doppler correction by recalculating CC1200 frequency registers mid-pass
4. Decoded frames are submitted to [SatNOGS DB](https://db.satnogs.org/) via the SiDS API

### 4. SatNOGS Integration Notes

SatNOGS is SDR-centric (GNU Radio + SoapySDR flowgraphs). The CC1200 cannot plug into that
pipeline directly because it produces decoded packets, not raw IQ. Integration options:

1. **Recommended (current plan):** Run `satnogs-client` for scheduling and rotator control.
   Capture CC1200 packets independently via a Python script on the Pi. Submit decoded frames
   to SatNOGS DB via the SiDS API.

2. **Pre/post scripts:** Use satnogs-client observation hooks to start/stop CC1200 packet capture
   around scheduled passes automatically.

3. **Doppler correction:** Write a minimal `rigctld` shim that translates hamlib `set_freq` commands
   to CC1200 frequency register writes (FREQ2/FREQ1/FREQ0 in extended register space).

4. **Hybrid:** Add an RTL-SDR for standard SatNOGS waterfall/audio/IQ artifacts. Use the CC1200
   as the primary packet decoder (hardware demod is more reliable for narrow-band FSK).

## Project Structure

```
Firmware/
├── README.md                          ← this file
├── rp2040-satnogs-rotator/            ← Rotator Pico firmware (PlatformIO)
│   ├── platformio.ini
│   ├── copy_uf2.py
│   ├── README.md                      ← detailed rotator firmware docs
│   └── src/
│       ├── main.cpp                   Arduino entry point + main loop
│       ├── pins.h                     GPIO map (from MotorPCB netlist)
│       ├── config.h                   Calibration + PID + limits
│       ├── motor_pwm.cpp/h            TB6642FG PWM driver
│       ├── quadrature.cpp/h           Gray-code encoder decoder (IRQ)
│       ├── rotator.cpp/h              Closed-loop PID + homing + safety
│       ├── satnogs_protocol.cpp/h     EasyComm command parser
│       └── adxl345.cpp/h              ADXL345 accelerometer (SPI)
│
├── rp2040-rf-hat/                     ← RF HAT Pico firmware (PlatformIO)
│   ├── platformio.ini
│   ├── copy_uf2.py
│   ├── README.md                      ← detailed RF HAT firmware docs
│   ├── lib/cc1200_driver/             CC1200 SPI driver + HAL
│   │   ├── cc1200.c/h                 Register/FIFO/strobe access
│   │   ├── cc1200_hal.c/h             RP2040 SPI + GPIO HAL
│   │   └── library.json
│   ├── src/
│   │   ├── main.cpp                   Arduino entry point + dual CC1200 init
│   │   ├── config.h                   Pin map + radio constants
│   │   ├── proto.c/h                  Binary protocol handler (COBS/UART)
│   │   ├── cobs.c/h                   COBS framing codec
│   │   └── crc16.c/h                  CRC-16/CCITT-FALSE
│   └── tools/                         PC/Pi test tools (Python)
│       ├── cc1200_gui.py              GUI for interactive CC1200 control
│       ├── cc1200_diag.py             CLI diagnostic tool
│       ├── generate_regmap.py         Register map generator
│       ├── smartrf_config.txt         SmartRF Studio 435 MHz config
│       ├── cc1200_regmap_all.json     Full register name→address map
│       └── cc1200_regmap_profile.json Profile-applicable registers
```

## Build & Flash

Both projects use [PlatformIO](https://platformio.org/) with the Arduino-Pico (earlephilhower) core.
Raw Pico SDK APIs (hardware_pwm, hardware_spi, hardware_gpio, etc.) work alongside Arduino.

```bash
# Rotator Pico
cd Firmware/rp2040-satnogs-rotator
pio run                    # build → firmware.uf2 (copied to repo root)
pio run -t upload          # flash via BOOTSEL
pio device monitor         # USB serial console (115200 baud)

# RF HAT Pico
cd Firmware/rp2040-rf-hat
pio run                    # build → rf_hat_firmware.uf2 (copied to repo root)
pio run -t upload          # flash via BOOTSEL
pio device monitor         # USB debug serial (115200 baud)
```

**Flashing:** Hold BOOTSEL on the Pico, plug USB, release. The Pico appears as a USB drive
(`/Volumes/RPI-RP2`) — drag the `.uf2` file onto it. Alternatively, `pio run -t upload` uses
the 1200bps serial reset trick to enter bootloader automatically.

## Hardware

| Component | PCB | Specs |
|-----------|-----|-------|
| AZ/EL motors | MotorPCB | FAPG36-555-EN, 24V, 516:1, 16 RPM |
| Motor drivers | MotorPCB | TB6642FG, IN1/IN2 + PWM mode |
| Quadrature encoders | MotorPCB | Built into motors, 64 CPR (16 PPR x 4x decode) |
| ADXL345 accelerometer | MotorPCB | SPI0, EL calibration from gravity |
| Endstop switches | MotorPCB | 1 per axis, active-low with pull-up |
| CC1200 UHF | RF HAT | 432 MHz, SPI1, 40 MHz crystal |
| CC1200 VHF | RF HAT | 144 MHz, SPI0, 40 MHz crystal |
| CC1200 power | RF HAT | 3.3V from Pico's onboard RT6150 regulator |

## Origin

The rotator firmware was developed from scratch for the Motor Control PCB.
The RF HAT firmware is based on [CC1200RXFrontend](https://github.com/RobbeLehaen/CC1200RXFrontend)
by Robbe Lehaen, adapted for dual-radio UART operation on the RF HAT hardware.
