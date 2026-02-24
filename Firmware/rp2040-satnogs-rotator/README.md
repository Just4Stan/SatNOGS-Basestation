# rp2040-satnogs-rotator

RP2040 Pico firmware for the SatNOGS rotator controller. See the [main README](../../README.md) for the full project overview.

## Source files

| File | Purpose |
|------|---------|
| `src/main.cpp` | Entry point, LED heartbeat, serial I/O loop |
| `src/pins.h` | GPIO pin definitions (matches PCB netlist) |
| `src/config.h` | Calibration values (ticks/deg, PID gains, duty limits) |
| `src/motor_pwm.cpp` | TB6642FG PWM driver (IN1/IN2 + PWM mode) |
| `src/quadrature.cpp` | Gray-code quadrature decoder (IRQ-driven) |
| `src/rotator.cpp` | Closed-loop PID controller, homing, manual mode |
| `src/satnogs_protocol.cpp` | EasyComm command parser + bench test commands |
| `src/adxl345.cpp` | ADXL345 accelerometer driver (SPI, optional IMU) |

## Quick start

```sh
pio run              # build
pio run -t upload    # flash (Pico must be in BOOTSEL mode)
pio device monitor   # serial console
```

## How the firmware works

### Boot sequence

1. USB CDC serial initialized at 115200 baud
2. Motor PWM channels configured (20 kHz, IN1/IN2 direction mode)
3. Quadrature encoders initialized with GPIO interrupts (both edges, both channels)
4. ADXL345 IMU probed on SPI0 (optional — works without it)
5. If `kUseEndstopHoming = true`: homing sequence runs (searches for endstops)
6. Otherwise: starts in manual mode (motors off, encoders counting)
7. Main loop runs: 100 Hz control tick + serial command parsing + 1 Hz LED heartbeat

### Control modes

The firmware has three control modes:

- **Manual mode** (default at boot): Motors controlled directly via `DUTY_AZ`/`DUTY_EL` commands. PID is bypassed. Useful for bench testing and debugging.
- **PID mode**: Activated by sending any `AZ<value>` or `EL<value>` command. The PID controller drives both axes toward their target positions at 100 Hz.
- **Homing mode**: Activated by `RESET`. Searches for endstop switches in both directions per axis.

### PID controller details

- Full PID with anti-windup and filtered derivative
- Default gains: Kp=0.15, Ki=0.03, Kd=0.02 (tunable at runtime via `KP`/`KI`/`KD` serial commands)
- 0.05 deg deadband: within this range, P and I output are zero but D-term acts as an active brake
- No minimum duty floor — the I-term handles stiction naturally (a forced minimum causes limit-cycle oscillation)
- D-term uses exponential low-pass filter (alpha=0.15) to prevent derivative spikes from encoder noise
- Duty output smoothing (alpha=0.05) for jerk-free continuous tracking
- Max duty capped at 35% (`kMaxDuty` in config.h)
- `STOP` command snaps target to current position and clears all PID state

### EasyComm protocol

The serial protocol is compatible with hamlib model 204 (`ROT_MODEL_EASYCOMM3`). Responses echo the command prefix (e.g. `VE` responds `VESatNOGS-RP2040-v0.2\n`, `GS` responds `GS1\n`). This is required by hamlib which strips the first 2 characters.

Commands: `VE`, `GS`, `GE`, `AZ`, `EL`, `AZ<f> EL<f>`, `SA`, `SE`, `STOP`, `RESET`, `PARK`.

All control commands (`STOP`, `RESET`, `PARK`, `SA`, `SE`) reply with the current position (`AZ%.1f EL%.1f\n`).

### Encoder handling

Quadrature decoding runs in GPIO edge interrupts (both edges on both channels = 4x decoding). Uses a Gray-code state transition table for direction detection. The `volatile int32_t ticks_` counter is updated atomically in the ISR.

Encoder pins are swapped in firmware (A<->B) vs. PCB netlist labels to get the correct count direction without inverting in software.

### Safety features

- **Endstop gates**: PID won't drive further into a pressed endstop (direction-aware check)
- **Driver fault monitoring**: TB6642FG ALERT pins checked every tick; faults stop all motors (when `kIgnoreDriverFaults = false`)
- **Soft limits**: AZ clamped to +/-360 deg, EL to 0-180 deg
- **Homing timeout**: 20 seconds total, 10 seconds per direction search phase

## Configuration

In `config.h`:

- `kUseEndstopHoming = false` — set `true` once endstop switches are wired
- `kIgnoreDriverFaults = true` — set `false` for production (24V motor supply connected)
