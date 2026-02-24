# Hardware references

Datasheets and reference material for the rotator hardware.

## Motor driver (PCB)

- `Hardware/C2150576.pdf`: Toshiba **TB6642FG/FTG** full-bridge DC motor driver datasheet (50 V max, PWM capable).

## Motors

- `Hardware/motor_specs_24V_6mm_16rpm.jpg`: Motor spec sheet for the selected variant:
  - Model family: **FAPG36-555-EN** (micro DC planetary gear motor with encoder)
  - Variant: **24 V**, **6 mm shaft**, **16 RPM**
  - Gearbox ratio: **516:1** (confirmed by calibration)
  - Encoder wiring (per spec sheet): Red = Motor+, Black = Motor-, Green = Hall VCC, Blue = Hall GND, White = Hall OUT1, Yellow = Hall OUT2

## Encoder documentation (FT-555)

The motor includes a **2-channel incremental magnetic encoder** (Hall A/B) that outputs two square-wave channels in quadrature.

Reference: `Hardware/encoder_documentation.jpg`

Electrical level:
- Hall encoder VCC / output level is **3.3 V**.

Terminal definition (front of PCB, left to right):
1. MOTOR+ (motor positive)
2. MOTOR- (motor negative)
3. HALL SENSOR GND (encoder negative)
4. HALL SENSOR VCC (encoder positive)
5. HALL SENSOR A Vout (A-phase output)
6. HALL SENSOR B Vout (B-phase output)

Wire color mapping (from spec sheet):
- Red = MOTOR+
- Black = MOTOR-
- Blue = HALL GND
- Green = HALL VCC
- White = HALL A (OUT1)
- Yellow = HALL B (OUT2)

## Axis gearing + travel

External gearing (after the motor's internal 516:1 gearbox):
- **Azimuth**: 15:40 reduction (ratio = **40/15 = 2.667:1**)
- **Elevation**: 40:55 reduction (ratio = **55/40 = 1.375:1**)

Total reduction (encoder shaft to axis output):
- **Azimuth total**: `516 * (40/15) = 1376:1`
- **Elevation total**: `516 * (55/40) = 709.5:1`

Azimuth travel:
- Not continuous (no slip ring), approximately **+/-360 deg around center**.
- The firmware rewinds between satellite passes so cable wrap does not accumulate.

## Antenna

- `Hardware/4020944.pdf`: **LPRS YAGI-434A** — 434 MHz 7-element Yagi antenna datasheet
  - Frequency range: **390-480 MHz**
  - Gain: **10 dBi** at 450 MHz
  - Polarization: **vertical**
  - Impedance: 50 ohm, SMA Male connector
  - VSWR: <=1.5:1
  - F/B ratio: >15 dB
  - Dimensions: **1000 x 400 x 70 mm**
  - Weight: **575 g**
  - Max input power: 100 W

## Torque budget and wind load

**Motor torque at axis output** (24V, 516:1 gearbox + external gearing):

| Axis | External ratio | Rated torque | Max torque |
|------|---------------|-------------|-----------|
| AZ | 40/15 = 2.667:1 | 13.8 Nm | 39.2 Nm |
| EL | 55/40 = 1.375:1 | 7.1 Nm | 20.2 Nm |

**Antenna gravity load** (YAGI-434A, 575 g, CG at ~0.5 m from EL pivot):
- EL gravity torque (horizontal): **2.82 Nm** — EL motor has **3x safety margin** at rated load

**Wind load estimate** (Yagi broadside projected area ~318 cm2, Cd=1.2):

| Wind speed | AZ torque | % of rated |
|------------|----------|-----------|
| 40 km/h | 1.4 Nm | 10% |
| 80 km/h | 5.8 Nm | 42% |
| 120 km/h | 13.0 Nm | 94% |

The 575 g Yagi is well within the capacity of these motors. AZ handles wind up to ~**120 km/h**, EL up to ~**90 km/h** before reaching rated torque.
