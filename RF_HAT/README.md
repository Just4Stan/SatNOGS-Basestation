# RF_HAT — Dual CC1200 Transceiver HAT for Raspberry Pi

KiCad 9.0 project for a Raspberry Pi 3A+ HAT with two TI CC1200 transceivers (UHF 432 MHz + VHF 144 MHz), controlled by a Raspberry Pi Pico over SPI. Part of the [SatNOGS Basestation](../README.md) project.

## Architecture

```
Pi 3A+ (40-pin header)
  └─ UART ─── Raspberry Pi Pico (SPI host + 3.3V supply)
                 ├─ SPI0 ─── CC1200 #2 (VHF 144 MHz) ─── SMA
                 └─ SPI1 ─── CC1200 #1 (UHF 432 MHz) ─── SMA
```

**Power**: External buck converter provides 5V through a Schottky diode to the Pico's VSYS pin (USB can power simultaneously via internal Schottky OR-ing). Pico's onboard RT6150 regulator provides 3.3V for both CC1200s.

## File Structure

| Path | Description |
|------|-------------|
| `RF_HAT_out/` | KiCad project (open `RF_HAT.kicad_pro` in KiCad 9) |
| `rf_hat_circuit.py` | Original circuit-synth source (reference only — board is now edited directly in KiCad) |
| `add_farnell_fields.py` | Script to add Farnell order codes to schematic symbols |
| `BOM_Farnell.md` | Bill of materials with Farnell part numbers |
| `datasheets/` | CC1200 datasheet, CC120x user guide, TI reference designs |
| `reference_designs/` | M17 CC1200_HAT (verified 433 MHz) + UPSat CC1120 (145/435 MHz) |
| `old_schematics/` | Previous hand-placed schematics + Altium import (reference only) |

## RF Matching Network — 3-Path Topology

Each CC1200 shares a single SMA antenna through three RF paths: TX matching, RX differential balun, and TRX coupling switch. The CC1200's internal TRX_SW pin handles TX/RX switching.

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

## Component Values

### Inductors (per CC1200)

| Ref | Function | UHF 432 MHz | VHF 144 MHz |
|-----|----------|-------------|-------------|
| L1 | TX match #1 | 15nH | 22nH |
| L2 | TX match #2 | 43nH | 120nH |
| L3 | TX match #3 | 22nH | 82nH |
| L4 | PA RF choke | 56nH | 270nH |
| L5 | TRX coupling | 15nH | 47nH |
| L6 | LNA_P bias to GND | 27nH | 100nH |
| L7 | LNA bridge (P-N) | 56nH | 180nH |
| L8 | LNA_N to balun | 27nH | 100nH |

### Capacitors (per CC1200)

| Ref | Function | UHF 432 MHz | VHF 144 MHz |
|-----|----------|-------------|-------------|
| C1 | TX feedback | 2.2pF | 1.5pF |
| C2 | TX series (PA-mn1) | 39pF | 100pF |
| C3 | TRX coupling | 5.1pF | 15pF |
| C4 | TX shunt | 6.2pF | 15pF |
| C5 | PA bias bypass | 56pF | 100pF |
| C6 | PA AVDD decoupling | 10nF | 10nF |
| C7 | PA AVDD decoupling | 100pF | 100pF |
| C8 | LNA_P balun coupling | 5.1pF | 15pF |
| C9 | LNA_N shunt | 5.1pF | 15pF |
| C10 | DC block before SMA | 1nF | 100pF |

### Common elements (both bands)

| Component | Value | Source |
|-----------|-------|--------|
| Crystal | 40 MHz, NDK NX3225SA | SWRS123D Section 4.14 |
| Crystal load caps | 15 pF | SWRS123D |
| RBIAS | 56k | CC120X standard |
| DCPL caps | 100 nF x4 | SWRS123D Figure 6-1 |
| Ferrite bead | 1k@100MHz | BLM15HG102SN1D |
| SPI series resistors | 100R x3 | EMI reduction |
| CS/RESET pull-ups | 10k x2 | Float prevention |

## Value Sources

**UHF 432 MHz**: M17 CC1200_HAT Rev B (SP5WWP/DB9MAT, verified +14dBm at 433.475 MHz). Cross-referenced with TI SWRR122 (CC1200EM 420-470 MHz reference design).

**VHF 144 MHz**: Adapted from TI CC1120EM 169 MHz design (TIDR220/TIDR222). No official TI reference exists for 136-160 MHz. Inductors scaled ~17% for 144 MHz. **VNA tuning required during board bring-up.**

## GPIO Assignments (Pico)

| Function | GPIO | Function | GPIO |
|----------|------|----------|------|
| UART TX | GP0 | VHF MISO | GP16 |
| UART RX | GP1 | VHF CSN | GP17 |
| UHF GPIO3 | GP6 | VHF SCLK | GP18 |
| UHF GPIO0 | GP7 | VHF MOSI | GP19 |
| UHF RESET | GP8 | VHF RESET | GP20 |
| UHF GPIO2 | GP9 | VHF GPIO2 | GP21 |
| UHF SCLK | GP10 | VHF GPIO0 | GP22 |
| UHF MOSI | GP11 | VHF GPIO3 | GP26 |
| UHF MISO | GP12 | | |
| UHF CSN | GP13 | | |

## PCB Fabrication (JLCPCB 6-Layer)

The board is configured for JLCPCB's **JLC06121H-3313** 6-layer stackup (1.2mm nominal):

| Layer | Type | Thickness | Material | Er |
|-------|------|-----------|----------|-----|
| F.Cu | Copper | 0.035mm (1oz) | — | — |
| Dielectric 1 | Prepreg 3313 | 0.0994mm | NP-155F | 4.1 |
| In1.Cu | Copper | 0.0152mm (0.5oz) | — | — |
| Dielectric 2 | Core | 0.35mm | NP-155F | 4.36 |
| In2.Cu | Copper | 0.0152mm (0.5oz) | — | — |
| Dielectric 3 | Prepreg 2116 | 0.1164mm | NP-155F | 4.16 |
| In3.Cu | Copper | 0.0152mm (0.5oz) | — | — |
| Dielectric 4 | Core | 0.35mm | NP-155F | 4.36 |
| In4.Cu | Copper | 0.0152mm (0.5oz) | — | — |
| Dielectric 5 | Prepreg 3313 | 0.0994mm | NP-155F | 4.1 |
| B.Cu | Copper | 0.035mm (1oz) | — | — |

### Impedance Control

RF traces to the SMA connectors use a `50Ohm` net class (0.15mm / 6 mil width), calculated for F.Cu microstrip over In1.Cu ground plane (h=0.0994mm, Er=4.1, 1oz copper). Verify with [JLCPCB's impedance calculator](https://jlcpcb.com/pcb-impedance-calculator) before ordering.

### Ordering Notes

- Select **6-layer**, **1.2mm** thickness, **ENIG** surface finish (free for 6-layer)
- Enable **impedance control** and specify stackup **JLC06121H-3313**
- Dielectric constraints are enabled in the KiCad project (`dielectric_constraints yes`)

## Reference Documents

| Document | Title | Used For |
|----------|-------|----------|
| M17 CC1200_HAT | M17-Project/CC1200_HAT-hw (GitHub) | Primary UHF topology + values |
| SWRS123D | CC1200 Datasheet | Pin functions, crystal, DCPL caps |
| SWRR122 | CC1200EM 420-470 MHz Reference Design | UHF matching cross-check |
| SWRU346 | CC120X User Guide (114 pages) | Register config, RF design |
| TIDR220 | CC1120EM 169 MHz Schematic | VHF matching topology |
| TIDR222 | CC1120EM 169 MHz BOM | VHF component part numbers |
| TIDU921 | Multiband wM-Bus RF Subsystem | Bypass cap values |
| TIDU512 | CC1120 169 MHz wM-Bus Design | Additional VHF reference |
