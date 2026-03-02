# RF_HAT — LCSC/JLCPCB Assembly BOM

Dual CC1200 Transceiver HAT (UHF 432 MHz + VHF 144 MHz).
97 components total, 91 from LCSC (36 unique line items), 6 not on LCSC.

## JLCPCB Assembly Order Table

| LCSC Code | Description | Pkg | Qty | Basic/Ext |
|-----------|-------------|-----|-----|-----------|
| C122881 | CC1200RHBR transceiver (QFN-32) | QFN-32 | 2 | Extended |
| C5380316 | 40 MHz crystal 12pF 10ppm | 3225 | 2 | Extended |
| C1622 | 47nF X7R 50V cap (Samsung) | 0603 | 22 | **Basic** |
| C1644 | 15pF C0G 50V cap (Samsung) | 0603 | 8 | **Basic** |
| C14858 | 100pF C0G 50V cap (Samsung) | 0603 | 5 | **Basic** |
| C57112 | 10nF X7R 50V cap (FH) | 0603 | 4 | **Basic** |
| C61816 | 5.1pF C0G 50V cap (FH) | 0603 | 3 | Extended |
| C21120 | 220nF X7R 25V cap (Samsung) | 0603 | 2 | **Basic** |
| C1651 | 2.2pF C0G 50V cap (FH) | 0603 | 1 | Extended |
| C445758 | 39pF C0G 50V cap (TDK) | 0603 | 1 | Extended |
| C48579262 | 6.0pF C0G 50V cap (sub for 6.2pF) | 0603 | 1 | Extended |
| C338104 | 56pF C0G 50V cap (TDK) | 0603 | 1 | Extended |
| C342910 | 1nF C0G 50V cap (TDK) | 0603 | 1 | Extended |
| C76600 | 1.5nF C0G 50V cap (TDK PLL) | 0603 | 1 | Extended |
| C343008 | 1.8nF C0G 100V cap (TDK PLL) | 0603 | 1 | Extended |
| C695130 | 1.5pF C0G 50V cap (TDK) | 0603 | 1 | Extended |
| C76939 | 100nF X7R 25V cap (BOOT) | 0603 | 1 | Extended |
| C98192 | 4.7uF 50V X5R cap (Samsung) | 0805 | 1 | Extended |
| C2762594 | 22uF X5R cap | 0603 | 1 | Extended |
| C25804 | 10k resistor 1% (UNI-ROYAL) | 0603 | 4 | **Basic** |
| C23206 | 56k resistor 1% (UNI-ROYAL) | 0603 | 2 | **Basic** |
| C23345 | 22R resistor 1% (UNI-ROYAL, VHF PA) | 0603 | 1 | **Basic** |
| C270364 | 100k resistor 1% (FB divider top) | 0603 | 1 | Extended |
| C245924 | 18R resistor 1% (YAGEO, UHF PA) | 0603 | 1 | Extended |
| C138175 | 13.7k resistor 1% (FB divider bot) | 0603 | 1 | Extended |
| C3759853 | Schottky diode 40V (Bourns CD0603) | 0603 | 1 | Extended |
| C7296200 | LMR51420YFDDCR buck converter | SOT-23-6 | 1 | Extended |
| C23651 | 15nH RF inductor (Sunlord SDCL1608) | 0603 | 2 | Extended |
| C29683 | 47nH RF inductor (Sunlord SDCL1608) | 0603 | 2 | Extended |
| C12831 | 56nH RF inductor (Sunlord SDCL1608) | 0603 | 2 | Extended |
| C98071 | 22nH RF inductor (Murata LQW18AN) | 0603 | 2 | Extended |
| C162550 | 27nH RF inductor (Murata LQG18HN) | 0603 | 2 | Extended |
| C98089 | 100nH RF inductor (Murata LQW18AN) | 0603 | 3 | Extended |
| C668477 | 270nH RF inductor (Murata LQW18AN) | 0603 | 1 | Extended |
| C307612 | 120nH RF inductor (Murata LQW18AN) | 0603 | 1 | Extended |
| C98085 | 82nH RF inductor (Murata LQW18AN) | 0603 | 1 | Extended |
| C39846837 | 4.7uH power inductor (3030) | 3030 | 1 | Extended |
| C50980 | 2x20 pin header 2.54mm | TH | 1 | Extended |
| C431092 | XT30PW-M power connector | TH | 1 | Extended |

**39 line items, 91 SMT/TH components**

## Not on LCSC (hand-solder)

| Ref | Value | Notes |
|-----|-------|-------|
| J1, J2 | SMA edge-mount 50 ohm | Amphenol 132289 (Farnell 2112467) |
| U3 | Raspberry Pi Pico | Buy from Farnell (3643332) or Pimoroni |
| H1-H4 | Mounting holes | PCB feature, no component |

## Basic vs Extended Breakdown

**8 Basic line items** (no extra fee, 48 components):
- 47nF, 15pF, 100pF, 10nF, 220nF capacitors
- 10k, 56k, 22R resistors

**31 Extended line items** (~$3 each = ~$93 extended fee):
- CC1200 IC, crystal, buck IC, Schottky diode (unavoidably Extended)
- RF matching caps: 2.2pF, 5.1pF, 6.0pF, 39pF, 56pF, 1nF, 1.5nF, 1.8nF, 1.5pF (specialty C0G values)
- RF inductors: all nH-range inductors are Extended on JLCPCB
- Buck passives: 4.7uF 50V, 22uF, 100nF, 18R, 13.7k, 100k
- Connectors: XT30, pin header

> **Why so many Extended?** The CC1200 RF matching network requires specific C0G capacitor values (2.2pF, 5.1pF, 39pF etc.) and RF inductors that are not in JLCPCB's Basic library. Basic only covers standard jellybean values (10nF, 47nF, 100nF, 10k, etc.). Every CC1200 design will have a similar Extended count.

## Substitutions

| Ref | Schematic | LCSC Part | Notes |
|-----|-----------|-----------|-------|
| L2 | 43nH | C29683 (47nH Sunlord) | Nearest standard value, VNA tune |
| C4 | 6.2pF | C48579262 (6.0pF) | Nearest available, VNA tune |
| L1,L5 | 15nH | C23651 (Sunlord SDCL1608) | Replaces Murata, 160K stock |
| L4,L7 | 56nH | C12831 (Sunlord SDCL1608) | Replaces Murata, cheaper |
| L11 | 47nH | C29683 (Sunlord SDCL1608) | Replaces Murata, 129K stock |
| C23 | 1.8nF 50V | C343008 (1.8nF 100V TDK) | Same value, better stock |

## Production File Checklist

1. Export BOM from KiCad: `File > Fabrication Outputs > BOM` (LCSC field will be included)
2. Export placement file: `File > Fabrication Outputs > Component Placement (.pos)`
3. Export Gerbers: `File > Fabrication Outputs > Gerbers`
4. Upload to JLCPCB: select 6-layer, 1.2mm, ENIG, impedance control (JLC06121H-3313)
5. Upload BOM + placement for assembly — JLCPCB matches by LCSC code
6. Exclude hand-solder components (J1, J2, U3, H1-H4) from placement
