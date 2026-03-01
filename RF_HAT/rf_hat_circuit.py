#!/usr/bin/env python3
"""
RF HAT - Dual CC1200 Transceiver HAT for Raspberry Pi 3A+

Generates a KiCad 9 project with hierarchical sheets using circuit-synth.
Architecture:
  Pi 3A+ (40-pin header) --UART--> Pico --SPI--> CC1200 UHF (432 MHz)
                                        --SPI--> CC1200 VHF (144 MHz)

Power: 5V USB into Pico VSYS, Pico onboard RT6150 provides 3.3V for CC1200s.

Usage:
    pip install circuit-synth
    python3 rf_hat_circuit.py
    # Open RF_HAT_out/RF_HAT.kicad_pro in KiCad

The auto-placement is rough - open in KiCad and rearrange components visually.
The netlist connectivity is the important part; layout is manual.

Design Sources & Matching Network Verification:
================================================
Common elements (both bands):
  - CC1200 datasheet: SWRS123D (Figure 6-1 typical application circuit)
  - Crystal: 40 MHz, NDK NX3225SA (per SWRS123D Section 4.14, 38.4-40 MHz)
  - Crystal load caps: 15 pF (SWRS123D Section 4.14, min 10 pF)
  - RBIAS: 56k (standard CC120X value, confirmed in TIDR222 BOM R14=56k)
  - DCPL caps: 100 nF per pin (SWRS123D Figure 6-1)
  - Ferrite bead: 1k@100MHz (TIDR222 BOM L1=BLM15HG102SN1D)
  - EXT_XOSC tied to GND (SWRS123D: ground when using crystal on Q1/Q2)
  - Bypass caps: 47 nF x7 on VDD (TIDU921 BOM, Murata GRM155R71E473KA88D)

UHF 420-470 MHz matching network:
  - Source: SWRR122 (CC1200EM 420-470 MHz Reference Design)
  - Values extracted from Altium import (cc1200_altium_ref.kicad_sch) which
    contains SWRR122 part numbers:
    L: 56nH (LQG18HN56NJ00D), 56nH, 27nH (LQG18HN27NJ00D), 27nH,
       15nH (LQG18HN15NJ00D), 43nH (LQG18HN43NJ00), 22nH (LQG18HN22NJ00D),
       15nH
    C: 5.1pF (06035A5R1CAT2A), 56pF (06035A560JAT2A), 100pF (06035A101JAT2A),
       39pF (06035A390JAT2A), 5.1pF, 6.2pF (06035A6R2CAT2A),
       2.2pF (06035A2R2CAT2A), 5.1pF
    PLL: 1.5nF (GCM188R71H152KA37D)
  - Also cross-referenced with TIDU921 Figure 13 (420-470 MHz band schematic)

VHF 136-160 MHz matching network:
  - No official TI CC1200 reference design exists for 136-160 MHz band.
  - Closest TI reference: CC1120EM 169 MHz (TIDR220/TIDR222, 164-192 MHz).
  - Values scaled from 169 MHz reference for lower 144 MHz target:
    larger inductors reflect longer wavelength (e.g. 270nH vs 220nH).
  - VHF matching network WILL NEED VNA tuning during board bring-up.
  - PLL: 1.8nF (slightly larger than UHF 1.5nF for lower frequency)

WARNING: The matching networks are starting points from reference designs.
Final tuning with a VNA is essential for optimal TX/RX performance,
especially the VHF band which has no exact TI reference.
"""

from circuit_synth import Component, Net, circuit


# =============================================================================
# Helper: place one CC1200 with all passives
# =============================================================================
def _cc1200_passives(cc, prefix, vcc_3v3, gnd,
                     r_bias_val, r_pa_val,
                     l_values, c_match_values,
                     c_xosc_val, c_pll_val):
    """Wire CC1200 power, crystal, decoupling, matching network, and SMA."""

    # --- Power: VDD pins per SWRS123D pinout ---
    # Pins 1(VDD_GUARD), 5(DVDD), 12(DVDD), 13(AVDD_IF), 15(AVDD_RF),
    #      22(AVDD_SYNTH1), 25(AVDD_PFD_CHP), 27(AVDD_XOSC), 28(DCPL_SYNTH2)
    for pin in [1, 5, 12, 13, 15, 22, 25, 27, 28]:
        cc[pin] += vcc_3v3
    cc[33] += gnd       # GND exposed pad
    cc["EXT_XOSC"] += gnd  # Ground when using crystal (SWRS123D)

    # --- 40 MHz crystal ---
    xtal = Component(symbol="Device:Crystal_GND24", ref="Y",
                      value="40MHz",
                      footprint="Crystal:Crystal_SMD_3225-4Pin_3.2x2.5mm")
    xosc_q1 = Net(f"{prefix}_XOSC_Q1")
    xosc_q2 = Net(f"{prefix}_XOSC_Q2")
    cc["XOSC_Q1"] += xosc_q1
    cc["XOSC_Q2"] += xosc_q2
    xtal[1] += xosc_q1
    xtal[3] += xosc_q2
    xtal[2] += gnd
    xtal[4] += gnd

    # Crystal load caps
    for net in [xosc_q1, xosc_q2]:
        c = Component(symbol="Device:C", ref="C", value=c_xosc_val,
                       footprint="Capacitor_SMD:C_0603_1608Metric")
        c[1] += net
        c[2] += gnd

    # --- RBIAS: 56k per CC120X standard (TIDR222 BOM R14) ---
    r_bias = Component(symbol="Device:R", ref="R", value=r_bias_val,
                        footprint="Resistor_SMD:R_0603_1608Metric")
    rbias_net = Net(f"{prefix}_RBIAS")
    cc["RBIAS"] += rbias_net
    r_bias[1] += rbias_net
    r_bias[2] += gnd

    # --- DCPL caps: 100nF per SWRS123D Figure 6-1 ---
    for pin, name in [(6, "DCPL"), (21, "DCPL_VCO"),
                       (26, "DCPL_PFD"), (29, "DCPL_XOSC")]:
        cap = Component(symbol="Device:C", ref="C", value="100nF",
                         footprint="Capacitor_SMD:C_0603_1608Metric")
        net = Net(f"{prefix}_{name}")
        cc[pin] += net
        cap[1] += net
        cap[2] += gnd

    # --- PLL loop filter ---
    c_pll = Component(symbol="Device:C", ref="C", value=c_pll_val,
                       footprint="Capacitor_SMD:C_0603_1608Metric")
    lpf0 = Net(f"{prefix}_LPF0")
    lpf1 = Net(f"{prefix}_LPF1")
    cc["LPF0"] += lpf0
    cc["LPF1"] += lpf1
    c_pll[1] += lpf0
    c_pll[2] += lpf1

    # --- Bypass caps: 47nF per VDD pin (TIDU921 BOM) ---
    for val in ["220nF", "10nF", "47nF", "47nF",
                "47nF", "47nF", "47nF", "47nF", "47nF"]:
        c = Component(symbol="Device:C", ref="C", value=val,
                       footprint="Capacitor_SMD:C_0603_1608Metric")
        c[1] += vcc_3v3
        c[2] += gnd

    # --- Ferrite bead: BLM15HG102SN1D per TIDR222 BOM L1 ---
    fb = Component(symbol="Device:FerriteBead", ref="FB", value="1k@100MHz",
                    footprint="Inductor_SMD:L_0603_1608Metric")
    avdd_filt = Net(f"{prefix}_AVDD_FILT")
    fb[1] += vcc_3v3
    fb[2] += avdd_filt
    c_f = Component(symbol="Device:C", ref="C", value="10nF",
                     footprint="Capacitor_SMD:C_0603_1608Metric")
    c_f[1] += avdd_filt
    c_f[2] += gnd

    # --- RF matching network ---
    pa_net = Net(f"{prefix}_PA")
    cc["PA"] += pa_net

    r_pa = Component(symbol="Device:R", ref="R", value=r_pa_val,
                      footprint="Resistor_SMD:R_0603_1608Metric")
    mn1 = Net(f"{prefix}_MN1")
    r_pa[1] += pa_net
    r_pa[2] += mn1

    # Inductor chain
    prev = mn1
    for i, val in enumerate(l_values):
        l = Component(symbol="Device:L", ref="L", value=val,
                       footprint="Inductor_SMD:L_0603_1608Metric")
        nxt = Net(f"{prefix}_LN{i}")
        l[1] += prev
        l[2] += nxt
        prev = nxt
    ant_net = prev

    # Shunt caps at inductor nodes (clamped to available nodes)
    num_nodes = len(l_values)
    for i, val in enumerate(c_match_values):
        c = Component(symbol="Device:C", ref="C", value=val,
                       footprint="Capacitor_SMD:C_0603_1608Metric")
        idx = min(i, num_nodes - 1)
        c[1] += Net(f"{prefix}_LN{idx}")
        c[2] += gnd

    # LNA + T/R switch — both connect to antenna node
    cc["LNA_P"] += ant_net
    cc["LNA_N"] += gnd
    cc["TRX_SW"] += ant_net

    # SMA
    sma = Component(symbol="Connector:Conn_Coaxial", ref="J", value="SMA",
                     footprint="Connector_Coaxial:SMA_Amphenol_132289_EdgeMount")
    sma[1] += ant_net
    sma[2] += gnd


# =============================================================================
# Top-level: hierarchical circuit with 4 sub-sheets
# =============================================================================
@circuit(name="RF_HAT")
def rf_hat():
    """Dual CC1200 Transceiver HAT for Raspberry Pi 3A+"""

    # Power nets
    vcc_5v = Net("+5V")
    vcc_3v3 = Net("+3.3V")
    gnd = Net("GND")

    # UART
    uart_tx = Net("UART_TX")
    uart_rx = Net("UART_RX")

    # UHF SPI + control
    uhf_sclk = Net("UHF_SCLK")
    uhf_mosi = Net("UHF_MOSI")
    uhf_miso = Net("UHF_MISO")
    uhf_csn = Net("UHF_CSN")
    uhf_reset = Net("UHF_RESET")
    uhf_gpio0 = Net("UHF_GPIO0")
    uhf_gpio2 = Net("UHF_GPIO2")
    uhf_gpio3 = Net("UHF_GPIO3")

    # VHF SPI + control
    vhf_sclk = Net("VHF_SCLK")
    vhf_mosi = Net("VHF_MOSI")
    vhf_miso = Net("VHF_MISO")
    vhf_csn = Net("VHF_CSN")
    vhf_reset = Net("VHF_RESET")
    vhf_gpio0 = Net("VHF_GPIO0")
    vhf_gpio2 = Net("VHF_GPIO2")
    vhf_gpio3 = Net("VHF_GPIO3")

    # ===================== Sheet 1: CC1200 UHF 432 MHz =====================
    @circuit(name="CC1200_UHF_432MHz")
    def cc1200_uhf_sheet():
        """CC1200 UHF 432 MHz transceiver with matching network"""
        cc = Component(symbol="RF:CC1200", ref="U", value="CC1200RHBR",
                        footprint="Package_DFN_QFN:QFN-32-1EP_5x5mm_P0.5mm_EP3.45x3.45mm")
        # SPI + control
        cc["SCLK"] += uhf_sclk
        cc["SI"] += uhf_mosi
        cc["SO(GPIO1)"] += uhf_miso
        cc["~{CS}"] += uhf_csn
        cc["~{RESET}"] += uhf_reset
        cc["GPIO0"] += uhf_gpio0
        cc["GPIO2"] += uhf_gpio2
        cc["GPIO3"] += uhf_gpio3
        # Passives: SWRR122 ref design (CC1200EM 420-470 MHz), see docstring
        _cc1200_passives(cc, "UHF", vcc_3v3, gnd,
                          r_bias_val="56k", r_pa_val="18R",
                          l_values=["56nH", "56nH", "27nH", "27nH",
                                    "15nH", "43nH", "22nH", "15nH"],
                          c_match_values=["5.1pF", "56pF", "100pF", "39pF",
                                          "5.1pF", "6.2pF", "2.2pF", "5.1pF"],
                          c_xosc_val="15pF", c_pll_val="1.5nF")
        # Test points
        for net in [uhf_sclk, uhf_mosi, uhf_miso, uhf_csn]:
            tp = Component(symbol="Connector:TestPoint", ref="TP",
                            footprint="TestPoint:TestPoint_Pad_D1.0mm")
            tp[1] += net

    cc1200_uhf_sheet()

    # ===================== Sheet 2: CC1200 VHF 144 MHz =====================
    @circuit(name="CC1200_VHF_144MHz")
    def cc1200_vhf_sheet():
        """CC1200 VHF 144 MHz transceiver with matching network"""
        cc = Component(symbol="RF:CC1200", ref="U", value="CC1200RHBR",
                        footprint="Package_DFN_QFN:QFN-32-1EP_5x5mm_P0.5mm_EP3.45x3.45mm")
        cc["SCLK"] += vhf_sclk
        cc["SI"] += vhf_mosi
        cc["SO(GPIO1)"] += vhf_miso
        cc["~{CS}"] += vhf_csn
        cc["~{RESET}"] += vhf_reset
        cc["GPIO0"] += vhf_gpio0
        cc["GPIO2"] += vhf_gpio2
        cc["GPIO3"] += vhf_gpio3
        # Passives: adapted from CC1120EM 169 MHz (TIDR222), needs VNA tuning
        _cc1200_passives(cc, "VHF", vcc_3v3, gnd,
                          r_bias_val="56k", r_pa_val="22R",
                          l_values=["270nH", "22nH", "120nH", "82nH",
                                    "180nH", "100nH", "47nH"],
                          c_match_values=["100pF", "100pF", "15pF", "1.5pF",
                                          "15pF", "15pF", "15pF", "100pF"],
                          c_xosc_val="15pF", c_pll_val="1.8nF")
        for net in [vhf_sclk, vhf_mosi, vhf_miso, vhf_csn]:
            tp = Component(symbol="Connector:TestPoint", ref="TP",
                            footprint="TestPoint:TestPoint_Pad_D1.0mm")
            tp[1] += net

    cc1200_vhf_sheet()

    # ===================== Sheet 3: Pico SPI Host ==========================
    @circuit(name="Pico_SPI_Host")
    def pico_sheet():
        """Raspberry Pi Pico - SPI host for both CC1200s"""
        pico = Component(symbol="MCU_Module:RaspberryPi_Pico", ref="U",
                          value="RaspberryPi_Pico",
                          footprint="Module:RaspberryPi_Pico_Common_Unspecified")
        # Power
        pico[39] += vcc_5v     # VSYS
        pico[36] += vcc_3v3    # 3V3(OUT)
        pico["GND"] += gnd

        # UART
        pico[1] += uart_tx     # GP0
        pico[2] += uart_rx     # GP1

        # UHF SPI1
        pico[14] += uhf_sclk   # GP10
        pico[15] += uhf_mosi   # GP11
        pico[16] += uhf_miso   # GP12
        pico[17] += uhf_csn    # GP13

        # UHF control
        pico[11] += uhf_reset  # GP8
        pico[12] += uhf_gpio2  # GP9
        pico[10] += uhf_gpio0  # GP7
        pico[9] += uhf_gpio3   # GP6

        # VHF SPI0
        pico[21] += vhf_miso   # GP16
        pico[22] += vhf_csn    # GP17
        pico[24] += vhf_sclk   # GP18
        pico[25] += vhf_mosi   # GP19

        # VHF control
        pico[26] += vhf_reset  # GP20
        pico[27] += vhf_gpio2  # GP21
        pico[29] += vhf_gpio0  # GP22
        pico[31] += vhf_gpio3  # GP26_ADC0

        # Bypass caps
        for rail in [vcc_3v3, vcc_5v]:
            c = Component(symbol="Device:C", ref="C", value="100nF",
                           footprint="Capacitor_SMD:C_0603_1608Metric")
            c[1] += rail
            c[2] += gnd

    pico_sheet()

    # ===================== Sheet 4: Pi Header ==============================
    @circuit(name="Pi_Header")
    def pi_header_sheet():
        """Raspberry Pi 40-pin GPIO header - UART passthrough"""
        pi = Component(symbol="Connector:Raspberry_Pi_2_3", ref="J",
                        value="Raspberry_Pi_2_3",
                        footprint="Connector_PinHeader_2.54mm:PinHeader_2x20_P2.54mm_Vertical")
        pi[2] += vcc_5v
        pi[4] += vcc_5v
        pi["GND"] += gnd
        pi[8] += uart_tx     # GPIO14/TXD
        pi[10] += uart_rx    # GPIO15/RXD
        # Pi 3.3V (not primary supply, just connected)
        vcc_3v3_pi = Net("VCC_3V3_PI")
        pi[1] += vcc_3v3_pi
        pi[17] += vcc_3v3_pi

    pi_header_sheet()


# =============================================================================
# Generate
# =============================================================================
if __name__ == "__main__":
    print("Generating RF HAT KiCad project...")
    design = rf_hat()
    design.generate_kicad_project(project_name="RF_HAT_out")
    print("Done! Open RF_HAT_out/RF_HAT.kicad_pro in KiCad.")
    print("Note: Auto-placement is rough. Rearrange components in KiCad schematic editor.")
