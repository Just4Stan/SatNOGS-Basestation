# rp2040-satnogs-rotator

RP2040 Pico firmware for the SatNOGS rotator controller. Part of the [SatNOGS Basestation](../../README.md) project — see also [`MotorPCB/`](../../MotorPCB/) for the KiCad PCB this firmware runs on.

See the [Firmware README](../README.md) for how this fits into the full ground station system.

## Quick Start

```bash
pio run              # build → firmware.uf2 (copied to repo root)
pio run -t upload    # flash (Pico must be in BOOTSEL mode)
pio device monitor   # USB serial console (115200 baud)
```

## Source Files

| File | Lines | Purpose |
|------|-------|---------|
| `main.cpp` | ~195 | Arduino entry point, IMU-based EL homing, encoder IRQ setup, main loop (100 Hz tick + serial parser + heartbeat + NeoPixel) |
| `pins.h` | 42 | GPIO pin map — extracted from `MotorPCB/motorPCB.net` KiCad netlist |
| `config.h` | 62 | All calibration constants: encoder ticks/deg, PID gains, duty limits, soft limits, homing params |
| `motor_pwm.cpp/h` | 81/30 | TB6642FG H-bridge driver: direction via IN1/IN2 GPIOs, speed via hardware PWM (20 kHz) |
| `quadrature.cpp/h` | 55/24 | Quadrature encoder decoder: ISR-driven, 4x decoding via Gray-code state transition table |
| `rotator.cpp/h` | ~430/122 | Closed-loop PID controller, homing state machine, AZ shortest-path wrapping, cable-wrap recentering, runaway detection, endstop + fault safety gates |
| `satnogs_protocol.cpp/h` | 250/~30 | EasyComm command parser + bench test commands (DUTY, TICKS, MONITOR, GAINS, IMU, etc.) |
| `adxl345.cpp/h` | ~120/~40 | ADXL345 accelerometer driver (SPI0): reads gravity vector, computes elevation angle |
| `status_led.cpp/h` | ~80/~20 | NeoPixel RGB status LED on GP16: idle (green), tracking (blue), on-target (cyan), fault (red) |

## How the Codebase Works

### Entry Point (`main.cpp`)

`setup()` runs once at boot:

1. **USB CDC serial** initialized (115200 baud) — this is the command interface to the Pi
2. **NeoPixel status LED** initialized on GP16
3. **Motor PWM** — two `MotorPwm` instances created for AZ and EL, configured at 20 kHz
4. **Quadrature encoders** — two `Quadrature` instances, with GPIO edge interrupts enabled on all 4 encoder pins via a shared ISR callback
5. **Rotator** — the `Rotator` object takes ownership of motors and encoders
6. **ADXL345 IMU** — probed on SPI0
7. **AZ calibration** — encoder zeroed (assumes antenna points north at power-on)
8. **EL homing** — if IMU found, the motor physically drives EL to horizontal (0°) using IMU feedback, then zeros the encoder. Timeout after 15s falls back to IMU-based static calibration. If no IMU, EL assumed 0° (antenna must be placed horizontal before power-on).
9. **Protocol handler** — `SatnogsProtocol` wraps the Rotator and optional IMU

The main loop runs inside `setup()` (not in Arduino's `loop()`), timed by `get_absolute_time()`:

- **100 Hz**: `rotator.tick()` — reads encoders, runs PID, drives motors
- **1 Hz**: LED heartbeat toggle + NeoPixel state update (idle/tracking/on-target/fault)
- **20+ Hz**: NeoPixel animation update (blink patterns)
- **10 Hz**: optional monitor stream (if `MONITOR` command is active)
- **Always**: serial character accumulation → line-based command dispatch

### Motor Driver (`motor_pwm.cpp`)

Drives the TB6642FG in IN1/IN2 direction mode:

- **Forward**: IN1=HIGH, IN2=LOW, PWM sets speed
- **Reverse**: IN1=LOW, IN2=HIGH, PWM sets speed
- **Stop**: IN1=LOW, IN2=LOW, PWM=0

The PWM frequency is computed from the system clock (125 MHz default):
```
f_pwm = f_sys / (clkdiv × (wrap + 1))
```
At 20 kHz with 125 MHz system clock: wrap = 6249, giving 16-bit duty resolution.

The `set(float duty)` method takes a signed value (-1.0 to +1.0) — sign controls direction,
magnitude controls duty cycle.

### Quadrature Decoder (`quadrature.cpp`)

Uses 4x decoding: both rising and falling edges on both A and B channels trigger the ISR.
The ISR reads the current AB state, combines it with the previous state to form a 4-bit index
into a lookup table that returns -1, 0, or +1:

```
State transition table (last_state << 2 | new_state):
AB: 00→01 = -1,  00→10 = +1
    01→00 = +1,  01→11 = -1
    10→00 = -1,  10→11 = +1
    11→01 = +1,  11→10 = -1
```

The `volatile int32_t ticks_` counter is updated atomically in the ISR. `set_ticks()` allows
calibration routines to preset the counter (e.g., from IMU elevation reading).

A shared GPIO IRQ callback (`gpio_irq_callback` in main.cpp) dispatches to the correct
`Quadrature` instance by checking which GPIO triggered the interrupt.

### PID Controller (`rotator.cpp`)

The `Rotator::tick()` method runs the control loop at 100 Hz:

1. **Measure**: read encoder ticks, convert to degrees via `ticks_per_deg` and invert flag
2. **Fault check**: read TB6642FG ALERT pins (active-high = fault)
3. **Runaway detection**: if actual position exceeds soft limits by more than 50°, emergency stop all motors (safety gate against positive-feedback failures)
4. **Mode dispatch**: manual mode skips PID; homing mode runs the homing state machine
5. **PID compute** (per axis):
   - Compute error = target - measured
   - If |error| < deadband (0.05 deg): output zero, clear PID state
   - Otherwise: compute P + I + D terms
   - D-term uses exponential low-pass filter (alpha=0.15) to suppress encoder noise spikes
   - I-term has anti-windup clamp (max 5 deg-seconds)
   - Output clamped to [-kMaxDuty, +kMaxDuty] (95%)
   - No minimum duty floor — the I-term naturally overcomes stiction
6. **Motor inversion**: the `kAzInvert`/`kElInvert` flags flip both the position reading (in `ticks_to_deg()`) and the motor output (in `update_control()`). Both must be inverted together — inverting only the position creates positive feedback (runaway)
7. **Duty smoothing**: exponential filter (alpha=0.05) on the PID output prevents start/stop twitching during continuous satellite tracking
8. **Endstop safety**: if an endstop is pressed and the error points into it, stop that motor
9. **AZ recentering**: after the rotator settles near ±360°, the encoder is shifted by one full turn to bring AZ back near 0°. This prevents cable-wrap accumulation across multiple passes

### AZ Shortest-Path Wrapping

When a new AZ target is set, `choose_wrapped_target()` picks the equivalent angle (target, target+360, or target-360) that is closest to the current position while staying within the soft limits (±360°). This means:

- Tracking north-crossing passes (e.g. AZ 350° → AZ 10°) takes the short 20° path, not the long 340° path
- PARK from AZ -310° goes to -360° (equivalent to 0°, 50° away) instead of the long way to 0° (310°)
- The logic is in the firmware, so it works identically whether commands come from the Python tracker, rotctld on the Pi, or any other hamlib client

### Homing (`rotator.cpp`)

When `kUseEndstopHoming = true`, the `RESET` command triggers homing:

1. Both axes run simultaneously, each with their own state machine
2. **Phase 0**: drive in `kHomeFirstDir` direction at `kHomeDuty` (25%) for up to `kHomePhaseTimeoutMs` (10s)
3. **Phase 1**: if endstop not found, reverse direction for another 10s
4. When endstop triggers (GPIO reads active): stop motor, zero encoder, mark axis homed
5. Global timeout of 20s aborts homing if both axes aren't found

Currently disabled (`kUseEndstopHoming = false`) — the IMU provides absolute EL reference and
AZ is assumed north at power-on.

### EasyComm Protocol (`satnogs_protocol.cpp`)

Compatible with hamlib model 204 (`ROT_MODEL_EASYCOMM3`). Response format echoes the command
prefix because hamlib strips the first 2 characters.

**SatNOGS commands (hamlib-compatible):**

| Command | Response | Action |
|---------|----------|--------|
| `AZ` or `EL` | `AZ<f> EL<f>\n` | Query current position |
| `AZ<f>` | (none) | Set AZ target, enter PID mode |
| `EL<f>` | (none) | Set EL target, enter PID mode |
| `AZ<f> EL<f>` | (none) | Set both targets atomically |
| `VE` | `VESatNOGS-RP2040-v0.2\n` | Version query |
| `GS` | `GS<n>\n` | Status: 1=idle, 2=moving |
| `GE` | `GE<n>\n` | Error: 1=OK, 4=not homed, 8=driver fault |
| `SA` | `AZ<f> EL<f>\n` | Stop azimuth |
| `SE` | `AZ<f> EL<f>\n` | Stop elevation |
| `STOP` or `S` | `AZ<f> EL<f>\n` | Stop all motors |
| `RESET` or `R` | `AZ<f> EL<f>\n` | Start homing sequence |
| `PARK` | `AZ<f> EL<f>\n` | Go to park position (AZ 0, EL 0) |

**Bench test commands (bypass PID):**

| Command | Response | Action |
|---------|----------|--------|
| `DUTY_AZ<f>` | `AZ duty=<f>\n` | Set raw AZ motor duty (-1..1) |
| `DUTY_EL<f>` | `EL duty=<f>\n` | Set raw EL motor duty (-1..1) |
| `TICKS` | `AZ_TICKS=<n> EL_TICKS=<n>\n` | Print encoder tick counts |
| `ZERO` or `Z` | `Encoders zeroed\n` | Reset encoder counters |
| `MONITOR` or `MON` | `MONITOR ON/OFF\n` | Toggle 10 Hz status stream |
| `INFO` | Multi-line dump | Full status (position, ticks, endstops, alerts, mode) |
| `GAINS` | `KP=<f> KI=<f> KD=<f>\n` | Show current PID gains |
| `KP<f>` / `KI<f>` / `KD<f>` | `KP=<f> KI=<f> KD=<f>\n` | Set PID gain at runtime |
| `IMU` | `X=<f> Y=<f> Z=<f> g EL=<f> deg\n` | Read accelerometer |
| `RECAL` | `EL recalibrated from IMU: <f> deg\n` | Reset EL reference from IMU |
| `HELP` or `?` | Command list | Print help |

### IMU-Based EL Homing (`main.cpp` + `adxl345.cpp`)

At boot, the ADXL345 accelerometer on SPI0 provides an absolute elevation reference from the gravity vector. The firmware uses this to physically home the EL axis:

1. Read IMU to get current elevation angle (averaged over 10 samples)
2. Drive EL motor toward 0° (horizontal) at 25% duty
3. Continuously read IMU (averaged over 3 samples) to check progress
4. When IMU reads within ±1.5° of horizontal: stop motor, zero encoder
5. Timeout after 15 seconds — if homing fails, calibrate encoder from IMU's last reading as fallback
6. If IMU not detected: EL assumed 0° (antenna must be placed horizontal before power-on)

The `RECAL` command re-reads the IMU and recalibrates EL at any time during operation.

**IMU mounting**: the ADXL345 is mounted on the Motor PCB with X-axis pointing up at EL=0° (horizontal). Elevation is computed as `atan2(z, x)` from the gravity vector.

## GPIO Pin Map

Extracted from `MotorPCB/motorPCB.net` (KiCad netlist, U5 = Pico module):

| Function | GPIO | Pico Pin | Notes |
|----------|------|----------|-------|
| AZ IN1 | GP15 | 20 | TB6642FG direction |
| AZ IN2 | GP14 | 19 | TB6642FG direction |
| AZ PWM | GP22 | 29 | Hardware PWM (20 kHz) |
| AZ ALERT | GP19 | 25 | Fault detect (active-high) |
| AZ Encoder A | GP11 | 15 | Swapped A↔B for correct direction |
| AZ Encoder B | GP10 | 14 | Swapped A↔B for correct direction |
| AZ Endstop | GP8 | 11 | Active-low with pull-up |
| EL IN1 | GP28 | 34 | Swapped IN1↔IN2 for correct direction |
| EL IN2 | GP27 | 32 | Swapped IN1↔IN2 for correct direction |
| EL PWM | GP21 | 27 | Hardware PWM (20 kHz) |
| EL ALERT | GP20 | 26 | Fault detect (active-high) |
| EL Encoder A | GP12 | 16 | |
| EL Encoder B | GP13 | 17 | |
| EL Endstop | GP9 | 12 | Active-low with pull-up |
| IMU MISO | GP4 | 6 | SPI0 (ADXL345) |
| IMU CS | GP5 | 7 | SPI0 |
| IMU SCK | GP6 | 9 | SPI0 |
| IMU MOSI | GP7 | 10 | SPI0 |
| LED | GP25 | — | Pico onboard |

## Configuration (`config.h`)

| Parameter | Default | Description |
|-----------|---------|-------------|
| `kUseEndstopHoming` | `false` | Enable endstop homing on RESET |
| `kIgnoreDriverFaults` | `true` | Ignore TB6642FG ALERT (set false with 24V supply) |
| `kAzTicksPerDegree` | 244.6 | Encoder ticks per degree of AZ output rotation |
| `kElTicksPerDegree` | 126.1 | Encoder ticks per degree of EL output rotation |
| `kAzInvert` | `false` | Invert AZ encoder direction |
| `kElInvert` | `true` | Invert EL encoder direction |
| `kKp` / `kKi` / `kKd` | 0.15 / 0.03 / 0.02 | PID gains (tunable at runtime) |
| `kDeadbandDeg` | 0.05 | Position deadband in degrees |
| `kMaxDuty` | 0.95 | Max PWM duty (95% = ~35 deg/s) |
| `kPwmHz` | 20000 | PWM frequency |
| `kDtSeconds` | 0.01 | Control loop period (100 Hz) |
| `kDFilterAlpha` | 0.15 | D-term low-pass filter coefficient |
| `kDutyFilterAlpha` | 0.05 | Duty output smoothing coefficient |
| `kAzMinDeg` / `kAzMaxDeg` | -360 / +360 | AZ soft limits |
| `kElMinDeg` / `kElMaxDeg` | 0 / 180 | EL soft limits |
| `kParkAzDeg` / `kParkElDeg` | 0 / 0 | Park position |

## Speed Test Results (24V)

| Duty | Speed (deg/s) | 360 deg time |
|------|---------------|--------------|
| 35% | 11.2 | 32s |
| 55% | 18.9 | 19s |
| 75% | 26.9 | 13s |
| 95% | 35.0 | 10s |

Speed scales linearly with duty up to 95%. At 100%, TB6642FG overcurrent protection can trigger on motor stall.

## Integration with Pi

On the Pi, the rotator appears as `/dev/ttyACM0`. Run:

```bash
# Start rotator daemon
rotctld -m 204 -r /dev/ttyACM0 -s 115200 -T 0.0.0.0 &

# Test from command line
rotctl -m 2 -r localhost:4533 P 180.0 45.0   # point to AZ 180, EL 45
rotctl -m 2 -r localhost:4533 p               # query position
```

SatNOGS client connects to `rotctld` via netrotctl — no custom software needed.
