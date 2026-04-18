# G2 — Link Budget

## Claim 1a — Slant range 30°/600 km
**Location:** ch02_background.tex:L47
**Quote:** "30\degree\ peak elevation, giving a slant range of 1075~km"
**Verdict:** OK
**Computed/Evidence:** sqrt(6371²·0.25 + 600² + 2·6371·600) − 6371·0.5 = 4260.6 − 3185.5 = 1075.1 km

## Claim 1b — Slant range zenith/400 km
**Location:** ch02_background.tex:L47
**Quote:** "zenith pass at 400~km altitude (best case, 400~km slant)"
**Verdict:** OK
**Computed/Evidence:** sqrt(6371² + 400² + 2·6371·400) − 6371 = 6771.0 − 6371 = 400.0 km

## Claim 1c — Slant range 10°/600 km
**Location:** ch02_background.tex:L47
**Quote:** "horizon pass at 10\degree\ elevation / 600~km altitude (worst case, 1932~km slant)"
**Verdict:** OK
**Computed/Evidence:** sqrt(6371²·0.03015 + 600² + 2·6371·600) − 6371·0.17365 = 3038.0 − 1106.3 = 1931.6 km ≈ 1932

## Claim 2a — FSPL at 432 MHz, 1075 km
**Location:** ch02_background.tex:L58
**Quote:** "FSPL at 432 MHz ... $-$145.8~dB"
**Verdict:** OK
**Computed/Evidence:** 20log10(1075) + 20log10(432) + 32.45 = 60.63 + 52.71 + 32.45 = 145.79 dB

## Claim 2b — FSPL at 432 MHz, 400 km
**Location:** ch02_background.tex:L58
**Quote:** "FSPL ... $-$137.2~dB" (zenith column)
**Verdict:** OK
**Computed/Evidence:** 20log10(400) + 85.16 = 52.04 + 85.16 = 137.20 dB

## Claim 2c — FSPL at 432 MHz, 1932 km
**Location:** ch02_background.tex:L58
**Quote:** "FSPL ... $-$150.9~dB" (10°/600 km column)
**Verdict:** OK
**Computed/Evidence:** 20log10(1932) + 85.16 = 65.72 + 85.16 = 150.88 dB

## Claim 3a — RX power 30°/600 km
**Location:** ch02_background.tex:L65
**Quote:** "Received power at CC1200 ... $-$125.7~dBm" (30°/600 column)
**Verdict:** OK
**Computed/Evidence:** 14 + 2 − 145.8 − 0.5 − 3.0 + 9.0 − 0.9 − 0.5 = −125.7 dBm

## Claim 3b — RX power zenith/400 km
**Location:** ch02_background.tex:L65
**Quote:** "Received power ... $-$116.7~dBm" (zenith column)
**Verdict:** OK
**Computed/Evidence:** 14 + 2 − 137.2 − 0.3 − 3.0 + 9.0 − 0.9 − 0.3 = −116.7 dBm (atm and pointing reduced to 0.3 dB in zenith column per table)

## Claim 3c — RX power 10°/600 km
**Location:** ch02_background.tex:L65
**Quote:** "Received power ... $-$131.3~dBm" (10°/600 column)
**Verdict:** OK
**Computed/Evidence:** 14 + 2 − 150.9 − 1.0 − 3.0 + 9.0 − 0.9 − 0.5 = −131.3 dBm

## Claim 4 — Theoretical margins vs −120 dBm
**Location:** ch02_background.tex:L67
**Quote:** "Theoretical margin ... $-$5.7~dB ... +3.3~dB ... $-$11.3~dB"
**Verdict:** OK
**Computed/Evidence:** −125.7 − (−120) = −5.7; −116.7 − (−120) = +3.3; −131.3 − (−120) = −11.3

## Claim 5 — Measured margins vs −105 dBm
**Location:** ch02_background.tex:L69
**Quote:** "Measured margin ... $-$20.7~dB ... $-$11.7~dB ... $-$26.3~dB"
**Verdict:** OK
**Computed/Evidence:** −125.7+105 = −20.7; −116.7+105 = −11.7; −131.3+105 = −26.3

## Claim 6 — Polarisation mismatch −3 dB circular-to-linear
**Location:** ch02_background.tex:L60
**Quote:** "Polarisation mismatch (circ $\leftrightarrow$ lin) $-$3.0~dB"
**Verdict:** OK
**Computed/Evidence:** Canonical textbook value: circular-to-linear average loss = 10log10(0.5) = −3.01 dB (linear antenna captures one of two orthogonal components).

## Claim 7 — SPF5189Z system NF ≈ 0.7 dB via Friis
**Location:** ch06_results.tex:L273
**Quote:** "SPF5189Z-class LNA mast-mounted ... reduces the system noise figure to approximately 0.7~dB via the Friis cascade"
**Verdict:** OK
**Computed/Evidence:** Friis with F_LNA=1.122 (0.5 dB), G_LNA=100 (20 dB), F_cable=1.230 (0.9 dB), F_CC1200=5.012 (7 dB): F_sys = 1.122 + 0.23/100 + 4.012/81.3 = 1.174 → NF = 10log10(1.174) = 0.70 dB. SPF5189Z datasheet: NF ≈ 0.6 dB at 900 MHz, ~0.7–0.8 dB at 432 MHz; gain ~19–22 dB at UHF. Assumed 0.5 dB NF is optimistic but within typical mast-mounted module published specs.
**Note:** Input NF of 0.5 dB is at the optimistic end for 432 MHz — using datasheet 0.6–0.7 dB would give system NF ≈ 0.8–0.9 dB. Within rounding of the claim.

## Claim 8 — Thermal baseline −127 dBm at 10 kHz BW
**Location:** ch06_results.tex:L272, ch02_background.tex:L96
**Quote:** "15--20~dB above the $-$127~dBm thermal baseline" / "10~kHz RX filter bandwidth"
**Verdict:** PARTIAL
**Computed/Evidence:** Pure thermal at 10 kHz BW, 290 K = −174 + 40 = −134 dBm. The −127 dBm figure equals −134 + 7 dB CC1200 NF, i.e. it is the noise-referenced-to-input INCLUDING receiver NF, not thermal alone. The 50 kHz BW interpretation in the prompt is inconsistent with the SmartRF config (CHAN_BW=0x8E → ~10 kHz) and with the draft_link_budget.md derivation (−174 + 7 + 40 = −127 dBm).
**Note:** Label "thermal baseline" is imprecise; it is actually thermal+NF floor. Consistent with 10 kHz BW once NF is folded in. Recommend wording "effective noise floor with 7 dB CC1200 NF" or "kTB + NF".
