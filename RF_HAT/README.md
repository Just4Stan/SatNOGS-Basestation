# RF_HAT — Dual CC1200 Transceiver HAT for Raspberry Pi

KiCad 9.0 project for a Raspberry Pi 3A+ HAT with two TI CC1200 transceivers (UHF 432 MHz + VHF 144 MHz), controlled by a Raspberry Pi Pico over SPI.

## Architecture

```
Pi 3A+ (40-pin header)
  └─ UART ─── Raspberry Pi Pico (SPI host + 3.3V supply)
                 ├─ SPI0 ─── CC1200 #2 (VHF 144 MHz) ─── SMA
                 └─ SPI1 ─── CC1200 #1 (UHF 432 MHz) ─── SMA
```

**Power**: 5V USB into Pico VSYS. Pico's onboard RT6150 regulator provides 3.3V for both CC1200s.

## Circuit Design (circuit-synth)

The schematic is defined as Python code using [circuit-synth](https://github.com/circuit-synth/circuit-synth), which generates valid KiCad project files with proper netlist connectivity.

```bash
pip install circuit-synth
python3 rf_hat_circuit.py
# Open RF_HAT_out/RF_HAT.kicad_pro in KiCad
```

**Why circuit-synth?** The CC1200 matching network has ~100 components across UHF and VHF variants. Defining them in Python gives:
- Clean, version-controlled netlist (no dangling labels)
- Reusable `_cc1200_passives()` function for both frequency variants
- Automatic reference numbering matching M17 schematic layout order
- One-command regeneration after any change

### File Structure

| File | Description |
|------|-------------|
| `rf_hat_circuit.py` | Circuit definition (source of truth) |
| `RF_HAT_out/` | Generated KiCad project (.kicad_pro, .kicad_sch, .net) |
| `reference_designs/M17_CC1200_HAT/` | M17 Project CC1200 HAT (verified 433 MHz, our primary UHF reference) |
| `reference_designs/upsat-comms-hardware-master/` | UPSat CubeSat CC1120 comms (145/435 MHz reference) |
| `datasheets/` | CC1200 datasheet, CC120x user guide, TI reference designs |
| `old_schematics/` | Previous hand-placed KiCad schematics + Altium import (reference only) |

## RF Matching Network — 3-Path Topology

Each CC1200 shares a single SMA antenna through three RF paths: TX matching, RX differential balun, and TRX coupling switch. The CC1200's internal TRX_SW pin handles TX/RX switching (shorted to GND in TX to protect LNA, high-Z in RX to pass signal).

### Topology Diagram (UHF designators shown)

```
PA BIAS PATH (DC feed to PA pin):
    AVDD_FILT ─── R_PA(18Ω) ──┬── L4(56nH) ─── PA pin 17
                     C5(56pF)──┘
                     across R_PA

TX MATCHING (PA → SMA):
    PA pin 17 ─── C2(39pF) ─┬─ L1(15nH) ─┬─ L2(43nH) ─┬─ L3(22nH) ─┬─ C10(1nF) ── SMA
                             |              |             |             |   DC block
                       C1(2.2pF)──┘    C3(5.1pF)    C4(6.2pF)
                       feedback         to TRX_SW    to GND

TRX COUPLING (TX/RX switch):
    TRX_SW pin 18 ──┬── C3(5.1pF) ── mn2 (L1/L2 junction)
                    └── L5(15nH)  ── balun center

LNA DIFFERENTIAL BALUN (RX input):
    LNA_P pin 19 ──┬── L7(56nH) ── LNA_N pin 20    (bridge inductor)
                   ├── L6(27nH) ── GND               (bias path)
                   └── C8(5.1pF) ── balun center      (coupling)

    LNA_N pin 20 ──┬── L7(56nH) ── LNA_P             (bridge)
                   ├── L8(27nH) ── balun center       (from TRX)
                   └── C9(5.1pF) ── GND               (shunt)
```

### Component Designator Mapping (M17 → Our Design)

RF matching components are created first in the code to get low reference numbers, matching the M17 CC1200_HAT schematic for easy visual layout in KiCad.

**Inductors (per CC1200):**

| Our Ref | M17 Ref | Function | UHF Value | VHF Value |
|---------|---------|----------|-----------|-----------|
| L1 | L1 | TX match #1 | 15nH | 22nH |
| L2 | L2 | TX match #2 | 43nH | 120nH |
| L3 | L3 | TX match #3 | 22nH | 82nH |
| L4 | L4 | PA RF choke | 56nH | 270nH |
| L5 | L9 | TRX coupling | 15nH | 47nH |
| L6 | L6 | LNA_P bias to GND | 27nH | 100nH |
| L7 | L7 | LNA bridge (P↔N) | 56nH | 180nH |
| L8 | L8 | LNA_N to balun | 27nH | 100nH |

**Capacitors (per CC1200):**

| Our Ref | M17 Ref | Function | UHF Value | VHF Value |
|---------|---------|----------|-----------|-----------|
| C1 | C1 | TX feedback (mn1↔mn2) | 2.2pF | 1.5pF |
| C2 | C2 | TX series (PA→mn1) | 39pF | 100pF |
| C3 | C3 | TRX coupling (TRX_SW→mn2) | 5.1pF | 15pF |
| C4 | C4 | TX shunt (mn3→GND) | 6.2pF | 15pF |
| C5 | C5 | PA bias bypass | 56pF | 100pF |
| C6 | C6 | PA AVDD decoupling | 10nF | 10nF |
| C7 | C7 | PA AVDD decoupling | 100pF | 100pF |
| C8 | C8 | LNA_P balun coupling | 5.1pF | 15pF |
| C9 | C9 | LNA_N shunt to GND | 5.1pF | 15pF |
| C10 | C27 | DC block before SMA | 1nF | 100pF |

**Resistors (per CC1200):**

| Function | UHF Value | VHF Value | Notes |
|----------|-----------|-----------|-------|
| SPI SCLK series | 100R | 100R | EMI reduction (M17 R4) |
| SPI MOSI series | 100R | 100R | EMI reduction (M17 R5) |
| SPI MISO series | 100R | 100R | EMI reduction (M17 R6) |
| CS pull-up | 10k | 10k | Prevents floating during boot (M17 R7) |
| RESET pull-up | 10k | 10k | Prevents floating during boot (M17 R9) |
| PA bias | 18R | 22R | DC bias feed (M17 R3) |
| RBIAS | 56k | 56k | CC120X standard (TIDR222 R14) |

## UHF 432 MHz Values

**Source**: M17 CC1200_HAT Rev B (SP5WWP/DB9MAT, verified +14dBm at 433.475 MHz). Cross-referenced with TI SWRR122 (CC1200EM 420-470 MHz Reference Design) and TIDU921 Figure 13.

All values are exact copies from the M17 verified working design.

## VHF 144 MHz Values

**Source**: Adapted from TI CC1120EM 169 MHz design (TIDR220 schematic, TIDR222 BOM). No official TI reference exists for 136-160 MHz with CC1200.

Inductors scaled up ~17% for 144 MHz (lower frequency requires larger inductance). **WARNING**: These are starting values only — VNA tuning is required during board bring-up.

## Common Elements (both bands)

| Component | Value | Source |
|-----------|-------|--------|
| Crystal | 40 MHz, NDK NX3225SA | SWRS123D Section 4.14 |
| Crystal load caps | 15 pF | SWRS123D (min 10 pF) |
| RBIAS | 56k | CC120X standard (TIDR222 R14) |
| DCPL caps | 100 nF x4 (pins 6, 21, 26, 29) | SWRS123D Figure 6-1 |
| Bypass caps | 47 nF x7 + 220 nF + 10 nF | TIDU921 BOM |
| Ferrite bead | 1k@100MHz | TIDR222 L1 (BLM15HG102SN1D) |
| SPI series resistors | 100R x3 | M17 CC1200_HAT (EMI reduction) |
| CS/RESET pull-ups | 10k x2 | M17 CC1200_HAT (float prevention) |
| PLL loop filter | 1.5nF (UHF) / 1.8nF (VHF) | M17 C16 / scaled for VHF |

## GPIO Assignments (Pico)

| Function | GPIO | Pin | Net |
|----------|------|-----|-----|
| UART TX | GP0 | 1 | UART_TX |
| UART RX | GP1 | 2 | UART_RX |
| UHF GPIO3 | GP6 | 9 | UHF_GPIO3 |
| UHF GPIO0 | GP7 | 10 | UHF_GPIO0 |
| UHF RESET | GP8 | 11 | UHF_RESET |
| UHF GPIO2 | GP9 | 12 | UHF_GPIO2 |
| UHF SCLK | GP10 | 14 | UHF_SCLK |
| UHF MOSI | GP11 | 15 | UHF_MOSI |
| UHF MISO | GP12 | 16 | UHF_MISO |
| UHF CSN | GP13 | 17 | UHF_CSN |
| VHF MISO | GP16 | 21 | VHF_MISO |
| VHF CSN | GP17 | 22 | VHF_CSN |
| VHF SCLK | GP18 | 24 | VHF_SCLK |
| VHF MOSI | GP19 | 25 | VHF_MOSI |
| VHF RESET | GP20 | 26 | VHF_RESET |
| VHF GPIO2 | GP21 | 27 | VHF_GPIO2 |
| VHF GPIO0 | GP22 | 29 | VHF_GPIO0 |
| VHF GPIO3 | GP26 | 31 | VHF_GPIO3 |

## Reference Documents

| Document | Title | Used For |
|----------|-------|----------|
| M17 CC1200_HAT | M17-Project/CC1200_HAT-hw (GitHub) | Primary UHF topology + values |
| SWRS123D | CC1200 Datasheet | Pin functions, crystal, DCPL caps |
| SWRR122 | CC1200EM 420-470 MHz Reference Design | UHF matching values cross-check |
| SWRU346 | CC120X User Guide (114 pages) | Register config, RF design guidance |
| TIDR220 | CC1120EM 169 MHz Schematic | VHF matching topology |
| TIDR222 | CC1120EM 169 MHz BOM | VHF component part numbers |
| TIDU921 | Multiband wM-Bus RF Subsystem | Bypass cap values, multi-band ref |
| TIDU512 | CC1120 169 MHz wM-Bus Design | Additional VHF reference |

## Modifying the Design

Edit `rf_hat_circuit.py` and re-run. Key functions:

- `_cc1200_passives()` — Full CC1200 subcircuit (3-path RF matching, power, crystal, decoupling)
- `cc1200_uhf_sheet()` — UHF 432 MHz CC1200 with SPI resistors + GPIO
- `cc1200_vhf_sheet()` — VHF 144 MHz CC1200 with SPI resistors + GPIO
- `pico_sheet()` — Pico with all GPIO assignments
- `pi_header_sheet()` — Pi 40-pin header with UART passthrough
- `rf_hat()` — Top-level circuit that wires everything together

After regeneration, open in KiCad for layout and visual review.
