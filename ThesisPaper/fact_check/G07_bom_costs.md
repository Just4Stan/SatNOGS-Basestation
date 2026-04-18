# G7 — BOM + Costs

## Claim 1 — Placed components & unique LCSC items
**Location:** ThesisPaper/chapters/appendix_d_smartrf.tex:12
**Quote:** "123 placed components (excluding test pads and mounting holes), 48 unique LCSC line items"
**Verdict:** PARTIAL
**Computed/Evidence:** Summing Quantity column of `MOTOR_RF_HAT/production/RF_HATV1_bom.csv` over all 58 non-header data rows: total qty = 131. Excluding the 6 test pads (TP1–TP6): **125 placed components**, not 123. Unique LCSC part numbers (non-empty LCSC #): **48** — matches. Note: the BOM has no "mounting hole" rows to exclude; only the 6 TPs exclude. Two of the J-connectors (J1, J2 single-pin headers) and J4/J5 (SMA) have blank LCSC fields but are real placed parts.
**Note:** Off by 2 (likely counts J1/J2 single-pin headers as non-components, or double-counts elsewhere).

## Claim 2 — Capacitors 19 values, 71 qty, €7.41
**Location:** appendix_d_smartrf.tex:20
**Quote:** "Capacitors — 19 values (0603/0805/1206), CC1200 matching + bypass & 71 & 7.41"
**Verdict:** PARTIAL
**Computed/Evidence:** From BOM: **19 unique capacitor values** OK, **71 qty** OK. €7.41 price not verifiable from CSV (no price column present); taken on faith.
**Note:** Value/qty match; price unverifiable in-repo.

## Claim 3 — Inductors 11 values, 16 qty, €1.12
**Location:** appendix_d_smartrf.tex:21
**Quote:** "Inductors — 11 values (0603 RF + 3030 power), CC1200 matching & 16 & 1.12"
**Verdict:** WRONG
**Computed/Evidence:** BOM has **11 unique inductor values** OK, but total qty = **17** (L1×1 + L10×1 + L11/L7×2 + L12/L17×2 + L13×1 + L14×1 + L15/L8×2 + L16×1 + L2/L3/L5×3 + L4×1 + L6/L9×2 = 17). If the 2× 15 nH (L15/L8) are pulled out as "non-LCSC" per Table D.2, 15 remain on LCSC — still not 16. But L15/L8 ARE in the LCSC BOM with part C23651, contradicting the non-LCSC line item in Table D.2.
**Note:** Double-counted: L15/L8 appear both in "Inductors" line (qty 16) and separately as "15 nH 2 × 0.50" in Table D.2, or quantities were miscounted. Either way, 16 is incorrect.

## Claim 4 — Resistors 7 values, 14 qty, €0.07
**Location:** appendix_d_smartrf.tex:22
**Quote:** "Resistors — 7 values (0603) & 14 & 0.07"
**Verdict:** OK
**Computed/Evidence:** BOM resistor rows: R1 (100k), R10 (18R), R11–R14 (100R ×4), R2 (13.7k), R3/R4/R5/R7 (10k ×4), R6/R8 (56k ×2), R9 (22R) → **7 unique values, 14 qty**. Price unverifiable in-repo.

## Claim 5 — Component LCSC part numbers
**Location:** appendix_d_smartrf.tex:23–33
**Quote:** CC1200RHBR (C122881), A4950ELJTR-T (C82404), LMR51420YFDDCR (C7296200), WS2812B-2020 (C965555), 40 MHz crystal (C5380316), BSS138 (C112239), 1N4148WS (C2128), XT30PW-M (C431092), 2×20 Pi header (C50980), JST XH 6-pin (C157919)
**Verdict:** OK
**Computed/Evidence:** All LCSC part numbers match `RF_HATV1_bom.csv` exactly (U3/U5=C122881, U6/U7=C82404, U2=C7296200, D2/D4/D5/D6=C965555, Y1/Y2=C5380316, Q1=C112239, D3=C2128, U1=C431092, J3=C50980, J6/J7=C157919).
**Note:** JST XH qty 2 matches (J6 + J7, both C157919).

## Claim 6 — LCSC subtotal €28.44
**Location:** appendix_d_smartrf.tex:35
**Quote:** "LCSC subtotal ... **28.44**"
**Verdict:** UNVERIFIABLE (internal inconsistency)
**Computed/Evidence:** Sum of the 14 line-item prices in Table D.1 = 7.41 + 1.12 + 0.07 + 4.43 + 1.10 + 0.84 + 0.28 + 0.12 + 0.01 + 0.01 + 0.14 + 0.24 + 0.14 + 0.33 = **€16.24**. Claimed subtotal is €28.44 — gap of €12.20. Caption note "incl. MOQ rounding for passives" implies the subtotal reflects LCSC reel MOQs that the line prices do not; unit pricing not present in BOM CSV so it cannot be independently reconstructed.
**Note:** Significant gap between line-item sum and subtotal. Either the MOQ uplift (€12.20 on ~€4 of parts) is real but unmotivated, or the subtotal is wrong. Needs the original LCSC cart export to close.

## Claim 7 — Non-LCSC subtotal €6.00 (SMA 2× 1.50 + 15 nH 2× 0.50 + Pico 4.00)
**Location:** appendix_d_smartrf.tex:50–54
**Quote:** "SMA edge-mount connectors 2 1.50 / 15 nH 0603 inductor 2 0.50 / Raspberry Pi Pico 1 4.00 / Non-LCSC subtotal 6.00"
**Verdict:** PARTIAL
**Computed/Evidence:** 1.50 + 0.50 + 4.00 = **6.00** — arithmetic OK. However:
  (a) 15 nH (L15/L8, C23651) IS in the LCSC BOM — conflicts with "not on LCSC" framing; only the SMAs (J4/J5, no LCSC #) and RPi Pico (U4 = C7203002, which IS an LCSC number) are separate.
  (b) RPi Pico U4 has LCSC number C7203002 in BOM, so calling it "Non-LCSC" is incorrect unless they mean "sourced outside LCSC" for the thesis build.
**Note:** Arithmetic fine; part categorisation is inconsistent with BOM CSV.

## Claim 8 — 4-layer JLCPCB bare PCB €6.00/board from 5-pc order
**Location:** appendix_d_smartrf.tex:69
**Quote:** "Bare PCB (4-layer, JLCPCB, per board from 5-piece order) 6.00"
**Verdict:** UNVERIFIABLE
**Computed/Evidence:** No JLCPCB quote file in `MOTOR_RF_HAT/production/`. 4-layer JLCPCB 5-pc prototype order is in the €25–40 range (≈€5–8/board) per public pricing, so €6/board is plausible.
**Note:** Plausible but no invoice/quote in repo.

## Claim 9 — Off-board subtotal €59.50
**Location:** appendix_d_smartrf.tex:81–92
**Quote:** "FAPG36 2 16.00 / Pi3A+ 25.00 / ASA 400g 10.00 / balls 2.00 / screws 3.00 / PVC 2.00 / ADXL345 1.50 / Off-board subtotal 59.50"
**Verdict:** OK (with caveat)
**Computed/Evidence:** 16 + 25 + 10 + 2 + 3 + 2 + 1.50 = **59.50** OK. Caveat: the FAPG36 row reads "Qty 2, € 16.00" which here means line total €16 (≈€8/motor). Draft/memory says "FAPG36 €16 each" (so line €32) — there's a unit-vs-total ambiguity in Appendix D. Table arithmetic is self-consistent only if 16.00 is the LINE total.
**Note:** Caption should clarify "16.00 is line total for 2 motors". If motors are actually €16 each, off-board subtotal should be €75.50, not €59.50.

## Claim 10 — Grand total ~€100
**Location:** appendix_d_smartrf.tex:112
**Quote:** "Grand total per station core ~€100"
**Verdict:** OK
**Computed/Evidence:** 28.44 + 6.00 + 6.00 + 59.50 = **€99.94 ≈ €100** OK. (Assumes Claim 6 is accepted and Claim 9's unit-vs-total reading is as-stated.)

## Claim 11 — v1 total ~€145
**Location:** ch03_system_design.tex:272 (table body L267-272)
**Quote:** "MotorPCB ~30 + RF HAT ~55 + Pi 25 + Mechanical ~35 = v1 total ~€145"
**Verdict:** OK
**Computed/Evidence:** 30 + 55 + 25 + 35 = **145** OK. Component line items are order-of-magnitude estimates; no CSV backing in repo for v1 board totals (no MotorPCB/production or RF_HAT/production BOM CSVs checked in this pass).

## Claim 12 — v2 PCB subtotal ~€40
**Location:** ch03_system_design.tex:290-294
**Quote:** "Components (LCSC) 28.44 + SMA+15nH 2.00 + Pico 4.00 + Bare PCB 6.00 = PCB subtotal ~40"
**Verdict:** OK
**Computed/Evidence:** 28.44 + 2.00 + 4.00 + 6.00 = **€40.44 ≈ €40** OK. Internally consistent with Appendix D Table D.2 (non-LCSC line items sum 6.00 = 2.00 SMA+ind + 4.00 Pico).

## Claim 13 — FAPG36 motors 2× 16.00 matches Appendix D
**Location:** ch03_system_design.tex:297 vs appendix_d_smartrf.tex:84
**Quote:** ch03: "FAPG36-555-EN motors (2×) & 16.00"; App D: "FAPG36-555-EN motor (24 V, 516:1) AliExpress 2 16.00"
**Verdict:** OK (with same unit/total ambiguity)
**Computed/Evidence:** Both sources list "2 × 16.00" for the motor row. ch03 mechanical subtotal 16+10+5+2+1.5 = **34.5 ≈ 35** OK. App D off-board subtotal 59.50 requires 16.00 as line total. Consistent, but both share the ambiguity flagged in Claim 9.
**Note:** Memory file says motors are ≈€16 each at AliExpress. If that's the real unit price, both tables understate by €16.

## Claim 14 — ~€40 PCB vs ~€100 station consistent
**Location:** ch03_system_design.tex:255 + :314 vs appendix_d_smartrf.tex:112
**Quote:** "v2 reaches ~€40 for the combined controller/RF PCB, ~€75 for the rotator hardware excluding the Pi, and ~€100 for the station core"
**Verdict:** OK
**Computed/Evidence:** v2 PCB 40.44 + Mechanical 34.5 = **€74.94 ≈ €75** rotator-excl-Pi OK. +€25 Pi = **€99.94 ≈ €100** station core OK. Matches Appendix D grand total. All three figures (€40 / €75 / €100) are internally consistent and cross-reference cleanly.

## Claim 15 — ch06 cost-comparison table vs ch02 Table 2.2 and ch03 Table 3.3
**Location:** ch06_results.tex:299-306 vs ch02_background.tex:138-142 vs ch03_system_design.tex:307
**Quote:** ch06 "Yaesu 760–900 / 1200+; SPID 1200–1300 / 1800+; SatNOGS v3 300–500 / 600+; This work ~75 / ~100"
**Verdict:** OK
**Computed/Evidence:**
  - ch02 Table 2.2 (rotator-comparison): Yaesu 760–900, SPID 1200–1300, SatNOGS v3 300–500, This work "~75 rotator / ~40 PCB alone". Rotator columns match ch06.
  - ch06 adds a "Total incl. Pi" column with 1200+/1800+/600+/~100 not present in ch02 Table 2.2.
  - ch03 Table 3.3 (v2-cost) gives v2 station-core total €100 — matches ch06's €100.
  - ch02 "SuperAntennaz ~600" row is in ch02 but NOT in ch06 table (omitted).
**Note:** SuperAntennaz is missing from ch06 comparison. Yaesu/SPID/SatNOGS v3 "total" column (1200+/1800+/600+) is only in ch06, not ch02 — no citations for those station-total figures in ch06. Those headline "total" numbers are not backed in the repo.

## Extra — Inductor categorisation inconsistency (cross-claim)
**Location:** appendix_d_smartrf.tex:21 vs :51
**Verdict:** WRONG
**Computed/Evidence:** Appendix D Table D.1 claims 16 LCSC-sourced inductors; Table D.2 lists "15 nH 0603 inductor, 2, 0.50" as non-LCSC. BOM CSV shows 15 nH (L15/L8) with LCSC C23651 — it IS an LCSC part. Either the Table D.2 15 nH row is spurious (double-counted), or Table D.1's "16" is wrong.
**Note:** Reconcile: BOM has 17 inductors total, all with LCSC numbers. Table D.1 should read "Inductors 11 values, 17 qty"; Table D.2 should drop the 15 nH line (or split into "not on LCSC due to stock issue" with justification).

## Extra — Raspberry Pi Pico categorisation
**Location:** appendix_d_smartrf.tex:52
**Verdict:** PARTIAL
**Computed/Evidence:** Pico (U4) has LCSC number C7203002 in BOM CSV. Table D.2 treats it as non-LCSC.
**Note:** Justification may be "hand-soldered from a pre-assembled module not ordered in LCSC BOM build" but this is not stated; reader cannot reconcile Table D.1 vs D.2 vs BOM CSV without external knowledge.
