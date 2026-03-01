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

**Why circuit-synth?** The CC1200 matching network has ~89 components across UHF and VHF variants. Defining them in Python gives:
- Clean, version-controlled netlist (no dangling labels)
- Reusable `_cc1200_passives()` function for both frequency variants
- Automatic reference numbering (C1..C50, L1..L15, etc.)
- One-command regeneration after any change

### File Structure

| File | Description |
|------|-------------|
| `rf_hat_circuit.py` | Circuit definition (source of truth) |
| `RF_HAT_out/` | Generated KiCad project (.kicad_pro, .kicad_sch, .net) |
| `datasheets/` | CC1200 datasheet, CC120x user guide, TI reference designs |
| `old_schematics/` | Previous hand-placed KiCad schematics + Altium import (reference only) |

## Key Components (89 total)

- **2x CC1200** (QFN-32): Sub-1GHz transceiver, 40 MHz crystal, 50 Ohm RF matching
- **Raspberry Pi Pico**: SPI host for both CC1200s, UART bridge to Pi, 3.3V supply
- **Raspberry_Pi_2_3 header**: 40-pin HAT connector (UART passthrough)
- Per-CC1200: ferrite bead, 4x DCPL caps, 9x bypass caps, RBIAS, PA bias, PLL filter, XOSC load caps
- 8x test points on SPI signals

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

## Matching Network Values & Sources

### UHF 432 MHz
**Source**: TI SWRR122 (CC1200EM 420-470 MHz Reference Design), verified against Altium import part numbers.

| Component | Values | Part Numbers (from Altium ref) |
|-----------|--------|-------------------------------|
| Inductors (8) | 56nH, 56nH, 27nH, 27nH, 15nH, 43nH, 22nH, 15nH | Murata LQG18HN series (0603) |
| Shunt caps (8) | 5.1pF, 56pF, 100pF, 39pF, 5.1pF, 6.2pF, 2.2pF, 5.1pF | AVX 06035A series (0603 C0G) |
| PA bias | 18R | |
| RBIAS | 56k | |
| PLL filter | 1.5nF | Murata GCM188R71H152KA37D |
| XOSC load | 15pF | |

Cross-referenced with TIDU921 Figure 13 (420-470 MHz band schematic, same topology).

### VHF 144 MHz
**Source**: Adapted from CC1120EM 169 MHz design (TIDR220 schematic, TIDR222 BOM). No official TI reference exists for 136-160 MHz.

| Component | Values | Notes |
|-----------|--------|-------|
| Inductors (7) | 270nH, 22nH, 120nH, 82nH, 180nH, 100nH, 100nH | Scaled up from 169 MHz values |
| Shunt caps (8) | 100pF, 18pF, 15pF, 1.5pF, 15pF, 15pF, 15pF, 100pF | |
| PA bias | 22R | |
| RBIAS | 56k | |
| PLL filter | 1.8nF | Slightly larger for lower frequency |
| XOSC load | 15pF | |

**WARNING**: VHF matching network needs VNA tuning during bring-up. The 169 MHz reference values are a starting point, not an exact match for 144 MHz.

### Common Elements (both bands)
| Component | Value | Source |
|-----------|-------|--------|
| Crystal | 40 MHz, NDK NX3225SA | SWRS123D Section 4.14 |
| Crystal load caps | 15 pF | SWRS123D (min 10 pF) |
| RBIAS | 56k | CC120X standard (TIDR222 R14) |
| DCPL caps | 100 nF x4 | SWRS123D Figure 6-1 |
| Bypass caps | 47 nF x7 + 220 nF + 10 nF | TIDU921 BOM |
| Ferrite bead | 1k@100MHz | TIDR222 L1 (BLM15HG102SN1D) |

### Reference Documents
| Document | Title |
|----------|-------|
| SWRS123D | CC1200 Datasheet (Figure 6-1, Section 4.14, Section 7.2) |
| SWRR122 | CC1200EM 420-470 MHz Reference Design (UHF source) |
| SWRU346 | CC120X User Guide (114 pages) |
| TIDR220 | CC1120EM 169 MHz Schematic (VHF reference) |
| TIDR222 | CC1120EM 169 MHz BOM (component part numbers) |
| TIDU921 | Multiband wM-Bus RF Subsystem (169/433/868 MHz schematics) |
| TIDU512 | CC1120 169 MHz wM-Bus Design |

## Modifying the Design

Edit `rf_hat_circuit.py` and re-run. Key functions:

- `_cc1200_passives()` — Full CC1200 subcircuit (power, SPI, crystal, matching network, SMA)
- `pico_sheet()` — Pico with all GPIO assignments
- `pi_header_sheet()` — Pi 40-pin header with UART passthrough
- `rf_hat()` — Top-level circuit that wires everything together

After regeneration, open in KiCad for layout and visual review.
