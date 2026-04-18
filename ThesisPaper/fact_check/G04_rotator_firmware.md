# G4 — Rotator Firmware

## Claim 1 — PID gains, deadband, sample period
**Location:** ch05_firmware_software.tex:L11-15
**Quote:** "$K_p = 0.15$, $K_i = 0.03$, $K_d = 0.02$ ... Deadband: 0.05° ... $\Delta t = 0.01$ s"
**Verdict:** OK
**Computed/Evidence:** config.h:L38-43 — `kKp=0.15f`, `kKi=0.03f`, `kKd=0.02f`, `kDeadbandDeg=0.05f`, `kDtSeconds=0.01f`.

## Claim 2 — Anti-windup description
**Location:** ch05_firmware_software.tex:L20
**Quote:** "Integral accumulation is skipped when $|u| \geq 0.95$ and the error sign would grow the integral further."
**Verdict:** PARTIAL
**Computed/Evidence:** rotator.cpp:L312-318 — saturation test is `|duty| >= kMaxDuty` (kMaxDuty=0.95, so numerically identical). Skip condition uses `error_would_increase = (error>0 && integral>0) || (error<0 && integral<0)` — i.e. error and integral have the same sign, which grows the integral further. Integral is clamped to ±5°·s after accumulation (kIntegralMaxDeg). Description is accurate; thesis phrases "error sign would grow the integral" slightly loosely (it's the error-vs-integral sign match) but effectively correct.
**Note:** The thesis writes `$|u|$` — the code tests the provisional PID output `duty` against `kMaxDuty`, consistent with "output saturation" interpretation.

## Claim 3 — Derivative LPF α=0.15
**Location:** ch05_firmware_software.tex:L21
**Quote:** "First-order LPF on the derivative term, $\alpha = 0.15$"
**Verdict:** OK
**Computed/Evidence:** config.h:L44 — `kDFilterAlpha = 0.15f`. rotator.cpp:L302 applies `d_filtered += alpha * (raw_deriv - d_filtered)`.

## Claim 4 — Duty LPF α=0.05 → 0.15
**Location:** ch05_firmware_software.tex:L22
**Quote:** "First-order LPF on the PWM duty, $\alpha = 0.05$ initially; raised to $\alpha = 0.15$ during field testing"
**Verdict:** PARTIAL
**Computed/Evidence:** config.h:L45 — current `kDutyFilterAlpha = 0.15f`. Final value matches. "Initially 0.05" is a historical claim not verifiable from current source (no git-history check done), but consistent with the CLAUDE.md project note "duty filter alpha=0.05" which pre-dated the tuning.

## Claim 5 — Max duty 0.95
**Location:** ch05_firmware_software.tex:L23
**Quote:** "Maximum duty: 0.95, to avoid TB6642FG overcurrent on stall."
**Verdict:** OK
**Computed/Evidence:** config.h:L49 — `kMaxDuty = 0.95f`. rotator.cpp:L324 clamps output to ±kMaxDuty.

## Claim 6 — kElInvert inverts both encoder reading AND PID output
**Location:** ch05_firmware_software.tex:L25-26
**Quote:** "kElInvert = true in pins.h inverts both the encoder reading and the PID output sign."
**Verdict:** OK
**Computed/Evidence:** `kElInvert` is defined in **config.h:L21** (not pins.h — minor location error). Inversion of encoder reading: rotator.cpp:L236 via `ticks_to_deg(... , kElInvert)`. Inversion of PID output: rotator.cpp:L274 `if (config::kElInvert) el_raw = -el_raw;`. Both inversions confirmed.
**Note:** Thesis says "in pins.h" — the flag is actually in config.h. Cosmetic, but incorrect file reference.

## Claim 7 — 64 CPR / 244.6 AZ / 126.1 EL ticks per degree
**Location:** ch05_firmware_software.tex:L59
**Quote:** "4× decode produces the 64 CPR / 244.6 AZ ticks/deg / 126.1 EL ticks/deg resolution"
**Verdict:** OK
**Computed/Evidence:** config.h:L16-17 — `kAzTicksPerDegree=244.6f`, `kElTicksPerDegree=126.1f`, comments state "64 ticks/rev × 516:1 × 40/15" and "× 55/40". 64 × 516 × (40/15) / 360 = 244.62. 64 × 516 × (55/40) / 360 = 126.13. Both match.

## Claim 8 — Inter-edge period 117 µs, ISR latency <10 µs
**Location:** ch05_firmware_software.tex:L61
**Quote:** "inter-edge period on the fastest axis is $1/(35 \cdot 244.6) = 117$ µs; the measured ISR latency is <10 µs per edge."
**Verdict:** PARTIAL
**Computed/Evidence:** Math: 1/(35 × 244.6) = 1/8561 s = 116.8 µs, rounds to 117 µs. OK. ISR latency <10 µs is a measurement claim with no trace in the source — unverified from code; quadrature.cpp:L49-54 `on_edge()` is short (one gpio_get pair, table lookup) and plausibly <10 µs, but no measurement hook exists.
**Note:** Decoding is 4× edge-per-cycle Gray-code (quadrature.cpp:L16-21), consistent with "4× decode".

## Claim 9 — EL homing uses ADXL345 gravity vector
**Location:** ch05_firmware_software.tex:L66
**Quote:** "EL: ADXL345 gravity vector → drive to horizontal → zero encoder."
**Verdict:** OK
**Computed/Evidence:** main.cpp:L77-112 runs a closed loop reading `imu.elevation_deg_averaged()` and driving EL motor until |el| ≤ 1.5°, then `rotator.calibrate_el(0.0f)`. adxl345.cpp:L75-83 computes `atan2(z, x)` as the elevation angle from the gravity vector. RECAL command (satnogs_protocol.cpp:L204-213) re-runs calibration from IMU.

## Claim 10 — ADXL345 SPI0 wiring
**Location:** ch05_firmware_software.tex:L73
**Quote:** "SPI0 wiring to the MotorPCB (MISO GP4, CSn GP5, SCK GP6, MOSI GP7)"
**Verdict:** OK
**Computed/Evidence:** pins.h:L33-36 — `kSpiMiso=4`, `kSpiCs=5`, `kSpiSck=6`, `kSpiMosi=7`. adxl345.cpp uses `spi0`.

## Claim 11 — Runaway e-stop at soft-limit + 50°
**Location:** ch05_firmware_software.tex:L81
**Quote:** "e-stop if position exceeds soft limits by 50°."
**Verdict:** OK
**Computed/Evidence:** rotator.cpp:L250 — `constexpr float kRunawayMarginDeg = 50.0f;` Applied to both AZ and EL soft limits at L251-262.

## Claim 12 — 8 s hardware watchdog
**Location:** ch05_firmware_software.tex:L83
**Quote:** "8 s hardware watchdog, including during the EL homing loop."
**Verdict:** OK
**Computed/Evidence:** main.cpp:L136 `watchdog_enable(8000, true)`. `watchdog_update()` called inside the EL homing loop (L103) and inside the main loop (L139).

## Claim 13 — EasyComm command set
**Location:** ch05_firmware_software.tex:L89-90
**Quote:** "Standard: AZ/EL set/get, PARK, STOP, VE, GS/GE. Custom: RESET, RECAL, MONITOR, TICKS, ZERO, GAINS, IMU."
**Verdict:** OK
**Computed/Evidence:** All commands found in satnogs_protocol.cpp: AZ/EL set + get (L61-82), PARK (L89), STOP/S/SA/SE (L94-108), VE (L109), GS (L113), GE (L118), RESET/R (L84), RECAL (L204), MONITOR/MON (L159), TICKS (L136), ZERO/Z (L165), GAINS + KP/KI/KD (L173-191), IMU (L193). Extras not listed in thesis: INFO, HELP/?, DUTY_AZ/DUTY_EL (bench test). Missing from thesis list but present: GET_AZ/GET_EL aliases.
**Note:** Thesis omits DUTY_AZ/DUTY_EL (bench), INFO, HELP — acceptable since these are bench-only/debug and not EasyComm-standard.

## Claim 14 — "~250 lines of C++ in satnogs_protocol.cpp"
**Location:** ch05_firmware_software.tex:L92
**Quote:** "The full EasyComm + extensions parser is implemented in satnogs_protocol.cpp (~250 lines of C++)."
**Verdict:** OK
**Computed/Evidence:** `wc -l` = 250 lines exactly.

## Claim 15 — Appendix A Motor Pico pin table
**Location:** appendix_a_pinout.tex:L14-28
**Verdict:** OK
**Computed/Evidence:** All rows cross-checked against pins.h:
- GP2 NeoPixel = kNeoPixel (L44) OK
- GP4/5/6/7 SPI0 MISO/CS/SCK/MOSI = kSpiMiso/kSpiCs/kSpiSck/kSpiMosi (L33-36) OK
- GP10 Encoder AZ-B = kAzEncB (L24) OK; GP11 Encoder AZ-A = kAzEncA (L23) OK; A/B swap note matches pins.h comment.
- GP12 Encoder EL-A = kElEncA (L25) OK; GP13 Encoder EL-B = kElEncB (L26) OK.
- GP14 AZ IN2 = kAzIn2 (L12) OK; GP16 AZ IN1 (bodge) = kAzIn1 (L11) OK.
- GP21 EL PWM = kElPwm (L19) OK; GP22 AZ PWM = kAzPwm (L13) OK.
- GP27 EL IN2 = kElIn2 (L18) OK; GP28 EL IN1 = kElIn1 (L17) OK.
**Note:** pins.h comment at L17-18 notes the EL IN1/IN2 firmware swap ("was IN1, swapped for correct UP direction"). Appendix A reports the as-wired-in-firmware assignment, which is the correct way to present it. Missing from appendix: GP8/GP9 endstops (kAzEndstop=8, kElEndstop=9), GP19 kAzAlert, GP20 kElAlert, GP25 onboard LED. Non-critical omissions.

## Claim 16 — ch03 rotator-pinout table
**Location:** ch03_system_design.tex:L196-208
**Quote:** "Motor IN1/IN2/PWM: AZ=GP16/GP14/GP22, EL=GP28/GP27/GP21. Encoder A/B: AZ=GP11/GP10, EL=GP12/GP13."
**Verdict:** OK
**Computed/Evidence:** All six pin pairs match pins.h (see Claim 15 cross-references). Encoder AZ shown as A/B = GP11/GP10 (swapped from "natural" numerical order) — matches firmware intent where `kAzEncA=11, kAzEncB=10` produces positive count direction.
