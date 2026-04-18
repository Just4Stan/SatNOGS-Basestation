# G3 — Rotator Hardware + Gearing

## Claim 1 — AZ external gear ratio
**Location:** ch03_system_design.tex:L119
**Quote:** "AZ 15:40 External ratio 40/15 = 2.667:1"
**Verdict:** OK
**Computed/Evidence:** 40/15 = 2.6667 → rounds to 2.667. Matches config.h comment "AZ 40/15". Driver 15 teeth, driven 40 teeth, reduction 40/15.

## Claim 2 — EL external gear ratio
**Location:** ch03_system_design.tex:L120
**Quote:** "EL 40:55 External ratio 55/40 = 1.375:1"
**Verdict:** OK
**Computed/Evidence:** 55/40 = 1.3750 exactly. Matches config.h comment "EL 55/40". Driver 40 teeth, driven 55 teeth, reduction 55/40.

## Claim 3 — AZ ticks/deg derivation
**Location:** ch03_system_design.tex:L130
**Quote:** "AZ ticks/deg = 64 × 516 × (40/15) / 360 = 88064/360 = 244.6"
**Verdict:** OK
**Computed/Evidence:** 64 × 516 × 2.6667 = 88064.00; 88064/360 = 244.6222 → 244.6. Matches config.h kAzTicksPerDegree = 244.6f.

## Claim 4 — EL ticks/deg derivation
**Location:** ch03_system_design.tex:L131
**Quote:** "EL ticks/deg = 64 × 516 × (55/40) / 360 = 45408/360 = 126.1"
**Verdict:** OK
**Computed/Evidence:** 64 × 516 × 1.375 = 45408.00; 45408/360 = 126.1333 → 126.1. Matches config.h kElTicksPerDegree = 126.1f.

## Claim 5 — FAPG36-555-EN datasheet specs (516:1, ~12 RPM, 4 Nm, 6 mm D-shaft)
**Location:** ch03_system_design.tex:L153
**Quote:** "24~V brushed DC motor with a 516:1 internal planetary gearbox. Manufacturer spec: approximately 12~RPM no-load at 24~V, nominal output torque 4~Nm, 6~mm D-shaft output."
**Verdict:** WRONG
**Computed/Evidence:** FAPG36-555-EN datasheet (MOTOR_RF_HAT/datasheets/FAPG36-555-EN_motor_specs.jpg) lists gear ratios 1/4, 1/5, 1/14, 1/19, 1/27, 1/51, 1/71, 1/100, 1/139, 1/189, 1/369, 1/721. 516:1 is NOT a listed ratio. At 24 V the 1/369 variant gives 16.3 RPM no-load / 52.6 kgf·cm (5.16 Nm); 1/721 gives 8.3 RPM / 102.74 kgf·cm (10.07 Nm). "12 RPM" and "4 Nm" do not match any single ratio line; closest 24 V no-load speed to 12 RPM is 16.3 RPM (1/369). 6 mm D-shaft is unverifiable from the datasheet image (dimension field shows Ø6 on dim sketch, plausible).
**Note:** Memory says "16 RPM, 516:1" — datasheet has no 516:1 row. Either the purchased variant is custom/undocumented, or the ratio is mis-stated. Reconcile with the vendor or empirically verify.

## Claim 6 — Output torque budget
**Location:** ch03_system_design.tex:L160-162
**Quote:** "AZ: 4 Nm × 2.667 = 10.7 Nm; EL: 4 Nm × 1.375 = 5.5 Nm"
**Verdict:** PARTIAL
**Computed/Evidence:** Arithmetic correct: 4 × 2.6667 = 10.667 → 10.7 Nm; 4 × 1.375 = 5.5 Nm exactly. However, the 4 Nm input figure is not datasheet-supported for 516:1 (see Claim 5), so the absolute values are uncertain.
**Note:** Arithmetic OK; input torque value propagates the Claim 5 issue.

## Claim 7 — 4/3 mm module non-standard in ISO 53
**Location:** ch03_system_design.tex:L108
**Quote:** "4/3 mm module is non-standard in the ISO 53 preferred series"
**Verdict:** OK
**Computed/Evidence:** ISO 53 series I (preferred): 1, 1.25, 1.5, 2, 2.5, 3, 4, 5, 6, 8, 10. Series II: 1.125, 1.375, 1.75, 2.25, 2.75, 3.5, 4.5, 5.5, 7, 9. 4/3 = 1.333 is in neither series. Claim correct.

## Claim 8 — ASA vs PLA Tg
**Location:** ch03_system_design.tex:L57
**Quote:** "ASA T_g ≈ 100°C, Vicat ≈ 105°C vs PLA T_g ≈ 60°C"
**Verdict:** OK
**Computed/Evidence:** ASA typical Tg 100-110 °C, Vicat softening 95-105 °C (grade-dependent). PLA Tg 55-65 °C. All three figures within standard literature ranges.

## Claim 9 — PID / control-loop parameters
**Location:** appendix_c_pid.tex:L14-28
**Quote:** Kp=0.15, Ki=0.03, Kd=0.02, Deadband=0.05°, Loop=100 Hz, I-clamp=±5°·s, D-filter α=0.15, Duty-filter α=0.15, Max duty=95%, PWM=20 kHz, AZ invert=false, EL invert=true, Runaway=50°
**Verdict:** OK
**Computed/Evidence:** All values match config.h: kKp=0.15f, kKi=0.03f, kKd=0.02f, kDeadbandDeg=0.05f, kDtSeconds=0.01f (100 Hz), kIntegralMaxDeg=5.0f, kDFilterAlpha=0.15f, kDutyFilterAlpha=0.15f, kMaxDuty=0.95f, kPwmHz=20000, kAzInvert=false, kElInvert=true. Runaway=50° matches rotator.cpp:250 (kRunawayMarginDeg = 50.0f). Appendix note "Duty-filter α initially 0.05, raised to 0.15 during field testing" is plausible and traceable.

## Claim 10 — Encoder PPR/CPR (12 PPR datasheet vs 16 PPR empirical)
**Location:** ch03_system_design.tex:L153
**Quote:** "datasheet specifies 12 PPR (48 CPR in 4× decode); the purchased units empirically measure as 16 PPR (64 CPR in 4× decode)"
**Verdict:** PARTIAL
**Computed/Evidence:** Firmware uses 64 ticks/rev (see config.h comments on kAzTicksPerDegree/kElTicksPerDegree: "64 ticks/rev × 516:1 gearbox ..."). So 64 CPR empirical = 16 PPR is consistent with firmware. FT-555 encoder datasheet (MOTOR_RF_HAT/datasheets/FT-555_encoder_pinout.jpg) does NOT state PPR — only pinout. FAPG36-555-EN datasheet image gives encoder terminal wiring but no PPR count. "Datasheet specifies 12 PPR / 48 CPR" is not evidenced by either on-disk datasheet.
**Note:** Empirical 64 CPR is verified by firmware; datasheet 12 PPR / 48 CPR claim is unverifiable from the datasheets in this repo.

## Claim 11 — Stepper holding current 0.8-1.5 A per phase
**Location:** ch03_system_design.tex:L171
**Quote:** "a stepper requires continuous holding current (typically 0.8–1.5 A per phase at rated current per the Pololu stepper catalogue)"
**Verdict:** OK
**Computed/Evidence:** Typical NEMA 17 bipolar steppers are spec'd 0.4-2.0 A/phase; common "high-torque" NEMA 17 units (e.g. Pololu #2267, #1200-series) fall in the 1.0-1.7 A/phase band. The 0.8-1.5 A/phase range is representative of the Pololu catalogue mid-range and is a defensible generic figure.
