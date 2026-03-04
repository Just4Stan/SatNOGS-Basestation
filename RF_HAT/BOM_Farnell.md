# RF_HAT — Farnell Order List

Dual CC1200 Transceiver HAT (UHF 432 MHz + VHF 144 MHz).
97 components total, 90 from Farnell Belgium (40 line items).

## Order Table (Farnell Belgium)

Copy this table into the order form at [be.farnell.com](https://be.farnell.com).

| Farnell Code | Description | Pkg | Qty |
|-------------|-------------|-----|-----|
| 2345430 | CC1200RHBR transceiver (QFN-32) | QFN-32 | 2 |
| 3643332 | Raspberry Pi Pico | Module | 1 |
| 1712847 | 40 MHz crystal (Epson TSX-3225) | 3225 | 2 |
| 2495363 | 47nF X7R 25V cap | 0603 | 22 |
| 721980 | 15pF C0G 50V cap | 0603 | 8 |
| 722080 | 100pF C0G 50V cap | 0603 | 5 |
| 1759022 | 10nF X7R 25V cap | 0603 | 4 |
| 1828900 | 220nF X7R 25V cap | 0603 | 2 |
| 2904766 | 5.1pF C0G 50V cap | 0603 | 3 |
| 1572627 | 56pF C0G 50V cap | 0603 | 1 |
| 2280868 | 39pF C0G 50V cap | 0603 | 1 |
| 3759543 | 6.0pF C0G 50V cap (sub for 6.2pF) | 0603 | 1 |
| 1813415 | 2.2pF C0G 50V cap | 0603 | 1 |
| 721864 | 1.5pF C0G 50V cap | 0603 | 1 |
| 2672137 | 1.5nF C0G 50V cap (UHF PLL) | 0603 | 1 |
| 1813422 | 1.8nF C0G 50V cap (VHF PLL) | 0603 | 1 |
| 2496882 | 1nF C0G 50V cap (UHF DC block) | 0603 | 1 |
| 1650835 | 100nF X7R 25V cap (BOOT) | 0603 | 1 |
| 3013463 | 4.7uF 50V X5R cap (buck input) | 0805 | 1 |
| 4292940 | 22uF 25V X5R cap (buck output) | 0805 | 1 |
| 9238603 | 10k resistor 1% (CS/RST pull-ups) | 0603 | 4 |
| 9238697 | 56k resistor 1% (RBIAS) | 0603 | 2 |
| 9238271 | 18R resistor 1% (UHF PA bias) | 0603 | 1 |
| 9238280 | 22R resistor 1% (VHF PA bias) | 0603 | 1 |
| 9238727 | 100k resistor 1% (FB divider top) | 0603 | 1 |
| 2138429 | 13.7k resistor 1% (FB divider bot) | 0603 | 1 |
| 2211954 | Schottky diode 30V (Bourns CD0603) | 0603 | 1 |
| 1515375 | 15nH RF inductor (Murata LQG18HN) | 0603 | 2 |
| 2470386 | 43nH RF inductor (Murata LQW18AN) | 0603 | 1 |
| 1515382 | 22nH RF inductor (Murata LQG18HN) | 0603 | 2 |
| 3354667 | 56nH RF inductor (Murata LQG18HH) | 0603 | 2 |
| 1515383 | 27nH RF inductor (Murata LQG18HN) | 0603 | 2 |
| 9528032 | 100nH RF inductor (Murata LQW18AN) | 0603 | 3 |
| 1515390 | 47nH RF inductor (Murata LQG18HN) | 0603 | 1 |
| 3471564 | 270nH RF inductor (Murata LQW18AN) | 0603 | 1 |
| 3678407 | 120nH RF inductor (Murata LQW18AN) | 0603 | 1 |
| 1515396 | 82nH RF inductor (Murata LQG18HN) | 0603 | 1 |
| 2112467 | SMA edge-mount 50 ohm (Amphenol 132289) | Through-hole | 2 |
| 9838309 | 2x20 pin header 2.54mm (3M) | Through-hole | 1 |

**40 line items, 90 components**

## Not on Farnell (already sourced / source separately)

| Ref | Value | Source | Notes |
|-----|-------|--------|-------|
| U5 | LMR51420YFDDCR | Mouser / LCSC (C7296200) | Buck converter, already have |
| U4 | XT30PW-M | TME / AliExpress | XT30 power connector, ahttps://claude.ai/referral/w5YuHm_zBlready have |
| L18 | XRTC303020D4R7MBCA | LCSC (C39846837) | 4.7uH power inductor 3030, already have |
| H1-H4 | MountingHole_Pad | — | PCB feature, no component |https://claude.ai/referral/w5YuHm_zBg

## Schematic Fix Required

**C24 (22uF buck output cap)**: footprint in schematic is currently `C_0603` which is physically impossible for 22uF. Change to `C_0805_2012Metric` before ordering. The Farnell part (4292940) is 0805.

## Notes

- 6.2pF cap (C4): exact value not on Farnell, 6.0pF substituted — tune with VNA
- VHF matching network: adapted from CC1120 169 MHz reference, needs VNA tuning
- CC1200 packaging: RHBT = cut tape, RHBR = reel — same IC, either works for 2 pcs
- All Farnell codes verified against schematic footprints (0603/0805/QFN-32/3225/SMA)
