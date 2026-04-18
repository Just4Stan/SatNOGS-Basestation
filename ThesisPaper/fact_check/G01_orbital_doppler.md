# G1 — Orbital Mechanics + Doppler

## Claim 1 — LEO 400 km visibility window
**Location:** ch01_introduction.tex:L17
**Quote:** "A LEO satellite at 400 km altitude is visible from a given ground station for 5-10 minutes per 92-minute orbit."
**Verdict:** OK
**Computed/Evidence:** Theoretical zenith-pass t_max = 10.16 min; practical passes above ~10° elevation are 5-8 min; orbital period T = 92.41 min (R_E=6371 km). 5-10 min window and "92-minute orbit" both consistent.

## Claim 2 — Total angular rate at zenith (ch01)
**Location:** ch01_introduction.tex:L19-22
**Quote:** "$\dot{\theta}_{\max} = v/h = 7.67/400 = 0.01918$ rad/s = 1.10 deg/s; v = sqrt(mu/(R_E+h)) = 7.67 km/s"
**Verdict:** OK
**Computed/Evidence:** v = sqrt(3.986e14/6.771e6) = 7672.59 m/s; v/h = 7672.59/400000 = 0.019181 rad/s = 1.0990 deg/s. Rounded values match.

## Claim 3 — Instantaneous AZ rate exceeds 10 deg/s near zenith
**Location:** ch01_introduction.tex:L22 (also ch02:L37)
**Quote:** "The instantaneous azimuth rate exceeds 10 deg/s near zenith because a small displacement of the sub-satellite point maps to a large arc in azimuth."
**Verdict:** OK
**Computed/Evidence:** Physics-defensible. AZ = atan2 of horizontal components; when the sub-satellite ground track passes within a few km of station, AZ sweeps ~180° in seconds. For a pass with closest-approach elevation >85°, AZ rate diverges as 1/cos(el_peak). Total angular rate bound v/h = 1.1 deg/s holds, but AZ component alone is unbounded at true zenith.

## Claim 4 — Orbital velocity at 400 km (ch02)
**Location:** ch02_background.tex:L9-14
**Quote:** "v = sqrt(3.986e14 / 6.771e6) = 7.669 km/s" using R_E = 6371 km
**Verdict:** PARTIAL
**Computed/Evidence:** Correct value is 7.6726 km/s → 7.673 km/s at 3 decimals. Thesis says 7.669; off by 4 m/s (rounds down instead of up). Chapter 1 uses 7.67 which is fine.
**Note:** Third decimal slightly wrong; either write 7.673 km/s or round to 7.67 km/s.

## Claim 5 — Orbital period at 400 km
**Location:** ch02_background.tex:L14-18
**Quote:** "T = 2π√((6.771e6)³/3.986e14) = 5554 s = 92.6 min"
**Verdict:** WRONG
**Computed/Evidence:** With stated values (R_E=6371 km, h=400 km, mu=3.986e14), T = 5544.86 s = 92.41 min. The 5554 s figure corresponds to R_E ≈ 6378 km (equatorial). Ch01 L17 says "92-minute orbit" which is fine.
**Note:** Replace with T = 5545 s = 92.4 min, or switch to R_E = 6378 km consistently.

## Claim 6 — Maximum slant range at horizon
**Location:** ch02_background.tex:L20-24
**Quote:** "d_max = sqrt(6771² − 6371²) = 2293 km"
**Verdict:** OK
**Computed/Evidence:** sqrt(6771² − 6371²) = 2292.77 km → 2293 km. Correct.

## Claim 7 — Maximum pass duration at zenith
**Location:** ch02_background.tex:L26-30
**Quote:** "t_max = T · α/π ≈ 10.1 min at h = 400 km zenith pass, α = arccos(R_E/(R_E+h))"
**Verdict:** OK
**Computed/Evidence:** α = arccos(6371/6771) = 19.793° = 0.3456 rad; α/π = 0.10996; t_max = 5544.86 × 0.10996 = 609.7 s = 10.16 min → 10.1 min. Correct (uses self-consistent T=5545, not the stated 5554).

## Claim 8 — Max Doppler shift at 433 MHz
**Location:** ch02_background.tex:L81-89
**Quote:** "v_r,max ≈ v = 7669 m/s. Δf_max = 7669/3e8 × 433e6 = 11.07 kHz (± 11 kHz)"
**Verdict:** OK
**Computed/Evidence:** 7669/3e8 × 433e6 = 11068.7 Hz = 11.07 kHz. Arithmetic matches stated numbers. (True v = 7672.6 m/s gives 11.074 kHz, rounds same.)

## Claim 9 — Max Doppler shift at 145.9 MHz
**Location:** ch02_background.tex:L91-94
**Quote:** "Δf_max = 7669/3e8 × 145.9e6 = 3.73 kHz (± 3.7 kHz)"
**Verdict:** OK
**Computed/Evidence:** 7669/3e8 × 145.9e6 = 3729.4 Hz = 3.73 kHz. Correct.

## Claim 10 — Worst-case Doppler rate at 432 MHz near TCA
**Location:** ch02_background.tex:L96
**Quote:** "worst-case rate of approximately 200 Hz/s at 432 MHz near TCA"
**Verdict:** OK
**Computed/Evidence:** Max Doppler rate ≈ (f0/c)·(v²/h) for zenith pass. At h=400 km, f0=432 MHz: 432e6/3e8 × 7672.6²/400e3 = 211.9 Hz/s. "Approximately 200 Hz/s" is the correct order. (At 600 km drops to 137 Hz/s.)

## Claim 11 — Fraction of orbit per pass
**Location:** ch02_background.tex:L119-120
**Quote:** "t_max/T ≈ 10.1/92.6 = 10.9% at 400 km"
**Verdict:** OK
**Computed/Evidence:** 10.1/92.6 = 10.907%. True α/π = 10.996%. Arithmetic as stated is correct; the slight discrepancy tracks the 5554 vs 5545 period issue in Claim 5 but rounds to 10.9% either way.
