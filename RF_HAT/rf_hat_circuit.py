#!/usr/bin/env python3
"""
RF HAT — Dual CC1200 Transceiver HAT for Raspberry Pi 3A+

Generates a KiCad 9 project with hierarchical sheets using circuit-synth.
Architecture:
  Pi 3A+ (40-pin header) --UART--> Pico --SPI--> CC1200 UHF (432 MHz)
                                        --SPI--> CC1200 VHF (144 MHz)

Power: 5V USB into Pico VSYS, Pico onboard RT6150 provides 3.3V for CC1200s.

Usage:
    pip install circuit-synth
    python3 rf_hat_circuit.py
    # Open RF_HAT_out/RF_HAT.kicad_pro in KiCad

RF Matching Network — 3-path topology
======================================
Based on M17 CC1200_HAT (SP5WWP/DB9MAT, Rev B, verified +14dBm at 433 MHz)
and TI reference designs SWRR122 (420-470 MHz) / TIDR220 (169 MHz).

Component naming follows M17 schematic designators for easy visual mapping.
See topology diagram in _cc1200_passives() docstring.

UHF 432 MHz — M17 CC1200_HAT / TI SWRR122 (exact match)
VHF 144 MHz — TI CC1120EM 169 MHz (TIDR220/222), scaled for 144 MHz
              WARNING: VHF values need VNA tuning during board bring-up.
"""

from circuit_synth import Component, Net, circuit


# =============================================================================
# Helper: CC1200 with 3-path RF topology
# =============================================================================
def _cc1200_passives(cc, prefix, vcc_3v3, gnd,
                     r_bias_val, c_xosc_val, c_pll_val,
                     # PA bias: AVDD → R_pa (C_bias across) → L_choke → PA
                     r_pa_val, l_choke_val, c_pa_bias_val,
                     # TX: PA → C_series → L1 → L2 → L3 → C_dc → SMA
                     c_series_val, l_tx_vals, c_feedback_val, c_shunt_val,
                     c_dc_val,
                     # TRX: TRX_SW → C_trx → mn2, TRX_SW → L_trx → balun
                     c_trx_val, l_trx_val,
                     # LNA differential balun
                     l_bridge_val, l_bias_p_val, l_n_val, c_p_val, c_n_val):
    """Wire CC1200 power, crystal, decoupling, and 3-path RF matching network.

    RF Matching Network Topology (M17 designators shown for UHF):
    =============================================================

    PA BIAS PATH (DC feed to PA pin):
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        AVDD_FILT ─── R3(18Ω) ──┬── L4(56nH) ─── PA pin 17
                      C5(56pF)──┘
                      across R3

    TX MATCHING (PA → SMA):
    ~~~~~~~~~~~~~~~~~~~~~~~
        PA pin 17 ─── C2(39pF) ─┬─ L1(15nH) ─┬─ L2(43nH) ─┬─ L3(22nH) ─┬─ C27(1nF) ── SMA
                                |              |             |             |   DC block
                          C1(2.2pF)──┘    C3(5.1pF)    C4(6.2pF)
                          feedback         to TRX_SW    to GND

    TRX COUPLING (TX/RX switch):
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        TRX_SW pin 18 ──┬── C3(5.1pF) ── mn2 (L1/L2 junction)
                        └── L9(15nH)  ── balun center

    LNA DIFFERENTIAL BALUN (RX input):
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        LNA_P pin 19 ──┬── L7(56nH) ── LNA_N pin 20
                       ├── L6(27nH) ── GND  (bias path)
                       └── C8(5.1pF) ── balun center
                                            ├── L8(27nH) ── LNA_N pin 20
                                            └── L9(15nH) ── TRX_SW pin 18

        LNA_N pin 20 ──┬── L7(56nH) ── LNA_P (bridge)
                       ├── L8(27nH) ── balun center
                       └── C9(5.1pF) ── GND  (shunt)

    References:
        UHF: M17 CC1200_HAT (github.com/M17-Project/CC1200_HAT-hw)
             TI SWRR122, CC120XEM-420-470-RD
        VHF: TI TIDR220/222, CC1120EM 169 MHz Reference Design
             Values scaled ~17% for 144 MHz (needs VNA tuning)
    """

    # --- Ferrite bead: +3.3V → FB → filtered rail ---
    fb = Component(symbol="Device:FerriteBead", ref="FB", value="1k@100MHz",
                    footprint="Inductor_SMD:L_0603_1608Metric")
    avdd_filt = Net(f"{prefix}_AVDD_FILT")
    fb[1] += vcc_3v3
    fb[2] += avdd_filt

    # =================================================================
    # RF MATCHING NETWORK — created first for low ref designators
    # Component order follows M17 schematic: L1-L4, L6-L9, C1-C9, C27
    # =================================================================

    pa_net = Net(f"{prefix}_PA")
    cc["PA"] += pa_net
    mn1 = Net(f"{prefix}_MN1")      # PA → C2 → [mn1] → L1
    mn2 = Net(f"{prefix}_MN2")      # L1 → [mn2] → L2 (also C3/TRX tap)
    mn3 = Net(f"{prefix}_MN3")      # L2 → [mn3] → L3 (also C4 shunt)
    ant_pre = Net(f"{prefix}_ANT_PRE")  # L3 → [ant_pre] → C27
    ant_net = Net(f"{prefix}_ANT")      # C27 → [ant] → SMA
    bias_node = Net(f"{prefix}_PA_BIAS")
    trx_net = Net(f"{prefix}_TRX")
    cc["TRX_SW"] += trx_net
    balun_center = Net(f"{prefix}_BALUN")
    lna_p = Net(f"{prefix}_LNA_P")
    lna_n = Net(f"{prefix}_LNA_N")
    cc["LNA_P"] += lna_p
    cc["LNA_N"] += lna_n

    # --- TX series inductors (M17: L1, L2, L3) ---
    l1 = Component(symbol="Device:L", ref="L", value=l_tx_vals[0],  # M17 L1
                     footprint="Inductor_SMD:L_0603_1608Metric")
    l1[1] += mn1
    l1[2] += mn2

    l2 = Component(symbol="Device:L", ref="L", value=l_tx_vals[1],  # M17 L2
                     footprint="Inductor_SMD:L_0603_1608Metric")
    l2[1] += mn2
    l2[2] += mn3

    l3 = Component(symbol="Device:L", ref="L", value=l_tx_vals[2],  # M17 L3
                     footprint="Inductor_SMD:L_0603_1608Metric")
    l3[1] += mn3
    l3[2] += ant_pre

    # --- PA bias choke (M17: L4) ---
    l_choke = Component(symbol="Device:L", ref="L", value=l_choke_val,  # M17 L4
                          footprint="Inductor_SMD:L_0603_1608Metric")
    l_choke[1] += bias_node
    l_choke[2] += pa_net

    # --- TRX coupling inductor (M17: L9 → our L5) ---
    l_trx = Component(symbol="Device:L", ref="L", value=l_trx_val,  # M17 L9
                        footprint="Inductor_SMD:L_0603_1608Metric")
    l_trx[1] += balun_center
    l_trx[2] += trx_net

    # --- LNA differential balun inductors (M17: L6, L7, L8) ---
    l_bp = Component(symbol="Device:L", ref="L", value=l_bias_p_val,  # M17 L6
                      footprint="Inductor_SMD:L_0603_1608Metric")
    l_bp[1] += lna_p
    l_bp[2] += gnd

    l_br = Component(symbol="Device:L", ref="L", value=l_bridge_val,  # M17 L7
                      footprint="Inductor_SMD:L_0603_1608Metric")
    l_br[1] += lna_p
    l_br[2] += lna_n

    l_ln = Component(symbol="Device:L", ref="L", value=l_n_val,       # M17 L8
                      footprint="Inductor_SMD:L_0603_1608Metric")
    l_ln[1] += lna_n
    l_ln[2] += balun_center

    # --- TX matching caps (M17: C1, C2, C3, C4) ---
    c_fb = Component(symbol="Device:C", ref="C", value=c_feedback_val,  # M17 C1
                      footprint="Capacitor_SMD:C_0603_1608Metric")
    c_fb[1] += mn1
    c_fb[2] += mn2

    c_ser = Component(symbol="Device:C", ref="C", value=c_series_val,   # M17 C2
                       footprint="Capacitor_SMD:C_0603_1608Metric")
    c_ser[1] += pa_net
    c_ser[2] += mn1

    c_trx = Component(symbol="Device:C", ref="C", value=c_trx_val,     # M17 C3
                        footprint="Capacitor_SMD:C_0603_1608Metric")
    c_trx[1] += trx_net
    c_trx[2] += mn2

    c_sh = Component(symbol="Device:C", ref="C", value=c_shunt_val,    # M17 C4
                      footprint="Capacitor_SMD:C_0603_1608Metric")
    c_sh[1] += mn3
    c_sh[2] += gnd

    # --- PA bias caps (M17: C5, C6, C7) ---
    c_bias = Component(symbol="Device:C", ref="C", value=c_pa_bias_val,  # M17 C5
                        footprint="Capacitor_SMD:C_0603_1608Metric")
    c_bias[1] += bias_node
    c_bias[2] += avdd_filt

    c_pa1 = Component(symbol="Device:C", ref="C", value="10nF",   # M17 C6
                        footprint="Capacitor_SMD:C_0603_1608Metric")
    c_pa1[1] += avdd_filt
    c_pa1[2] += gnd

    c_pa2 = Component(symbol="Device:C", ref="C", value="100pF",  # M17 C7
                        footprint="Capacitor_SMD:C_0603_1608Metric")
    c_pa2[1] += avdd_filt
    c_pa2[2] += gnd

    # --- LNA balun caps (M17: C8, C9) ---
    c_lp = Component(symbol="Device:C", ref="C", value=c_p_val,    # M17 C8
                      footprint="Capacitor_SMD:C_0603_1608Metric")
    c_lp[1] += balun_center
    c_lp[2] += lna_p

    c_ln = Component(symbol="Device:C", ref="C", value=c_n_val,    # M17 C9
                      footprint="Capacitor_SMD:C_0603_1608Metric")
    c_ln[1] += lna_n
    c_ln[2] += gnd

    # --- DC blocking cap before SMA (M17: C27) ---
    c_dc = Component(symbol="Device:C", ref="C", value=c_dc_val,   # M17 C27
                      footprint="Capacitor_SMD:C_0603_1608Metric")
    c_dc[1] += ant_pre
    c_dc[2] += ant_net

    # --- PA bias resistor (M17: R3) ---
    r_pa = Component(symbol="Device:R", ref="R", value=r_pa_val,   # M17 R3
                      footprint="Resistor_SMD:R_0603_1608Metric")
    r_pa[1] += avdd_filt
    r_pa[2] += bias_node

    # --- SMA connector ---
    sma = Component(symbol="Connector:Conn_Coaxial", ref="J", value="SMA",
                     footprint="Connector_Coaxial:SMA_Amphenol_132289_EdgeMount")
    sma[1] += ant_net
    sma[2] += gnd

    # =================================================================
    # POWER, CRYSTAL, DECOUPLING (higher ref designators)
    # =================================================================

    # Ferrite bead post-filter cap
    c_f = Component(symbol="Device:C", ref="C", value="10nF",
                     footprint="Capacitor_SMD:C_0603_1608Metric")
    c_f[1] += avdd_filt
    c_f[2] += gnd

    # --- Power: AVDD pins fed from filtered rail ---
    for pin in [1, 5, 12, 13, 15, 22, 25, 27, 28]:
        cc[pin] += avdd_filt
    cc[33] += gnd       # GND exposed pad
    cc["EXT_XOSC"] += gnd  # Ground when using crystal (SWRS123D)

    # --- 40 MHz crystal (NDK NX3225SA per SWRS123D Section 4.14) ---
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

    # Crystal load caps (15pF per SWRS123D)
    for net in [xosc_q1, xosc_q2]:
        c = Component(symbol="Device:C", ref="C", value=c_xosc_val,
                       footprint="Capacitor_SMD:C_0603_1608Metric")
        c[1] += net
        c[2] += gnd

    # --- RBIAS: 56k per CC120X standard (TIDR222 BOM R14) ---
    r_bias = Component(symbol="Device:R", ref="R", value=r_bias_val,  # M17 R14
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

    # --- PLL loop filter (M17: C16) ---
    c_pll = Component(symbol="Device:C", ref="C", value=c_pll_val,
                       footprint="Capacitor_SMD:C_0603_1608Metric")
    lpf0 = Net(f"{prefix}_LPF0")
    lpf1 = Net(f"{prefix}_LPF1")
    cc["LPF0"] += lpf0
    cc["LPF1"] += lpf1
    c_pll[1] += lpf0
    c_pll[2] += lpf1

    # --- Bypass caps on AVDD_FILT rail (TIDU921 BOM) ---
    for val in ["220nF", "10nF", "47nF", "47nF",
                "47nF", "47nF", "47nF", "47nF", "47nF"]:
        c = Component(symbol="Device:C", ref="C", value=val,
                       footprint="Capacitor_SMD:C_0603_1608Metric")
        c[1] += avdd_filt
        c[2] += gnd


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
        """CC1200 UHF 432 MHz — M17/SWRR122 matching network"""
        cc = Component(symbol="RF:CC1200", ref="U", value="CC1200RHBR",
                        footprint="Package_DFN_QFN:QFN-32-1EP_5x5mm_P0.5mm_EP3.45x3.45mm")

        # SPI with 100Ω series resistors (per M17: R4, R5, R6)
        for name, ext_net, cc_pin in [("SCLK", uhf_sclk, "SCLK"),
                                       ("MOSI", uhf_mosi, "SI"),
                                       ("MISO", uhf_miso, "SO(GPIO1)")]:
            r = Component(symbol="Device:R", ref="R", value="100R",
                           footprint="Resistor_SMD:R_0603_1608Metric")
            cc_side = Net(f"UHF_{name}_CC")
            r[1] += ext_net
            r[2] += cc_side
            cc[cc_pin] += cc_side

        # CS and RESET with 10k pull-ups (per M17: R7, R9)
        cc["~{CS}"] += uhf_csn
        cc["~{RESET}"] += uhf_reset
        for net in [uhf_csn, uhf_reset]:
            r = Component(symbol="Device:R", ref="R", value="10k",
                           footprint="Resistor_SMD:R_0603_1608Metric")
            r[1] += vcc_3v3
            r[2] += net

        # GPIO
        cc["GPIO0"] += uhf_gpio0
        cc["GPIO2"] += uhf_gpio2
        cc["GPIO3"] += uhf_gpio3

        # RF passives — UHF 432 MHz
        # Source: M17 CC1200_HAT Rev B (SP5WWP, verified +14dBm at 433.475 MHz)
        # Cross-ref: TI SWRR122 (CC1200EM 420-470 MHz Reference Design)
        _cc1200_passives(cc, "UHF", vcc_3v3, gnd,
                          r_bias_val="56k", c_xosc_val="15pF", c_pll_val="1.5nF",
                          # PA bias
                          r_pa_val="18R", l_choke_val="56nH", c_pa_bias_val="56pF",
                          # TX matching
                          c_series_val="39pF",
                          l_tx_vals=["15nH", "43nH", "22nH"],
                          c_feedback_val="2.2pF", c_shunt_val="6.2pF",
                          c_dc_val="1nF",
                          # TRX coupling
                          c_trx_val="5.1pF", l_trx_val="15nH",
                          # LNA differential balun
                          l_bridge_val="56nH", l_bias_p_val="27nH",
                          l_n_val="27nH", c_p_val="5.1pF", c_n_val="5.1pF")

        # Test points on SPI signals
        for net in [uhf_sclk, uhf_mosi, uhf_miso, uhf_csn]:
            tp = Component(symbol="Connector:TestPoint", ref="TP",
                            footprint="TestPoint:TestPoint_Pad_D1.0mm")
            tp[1] += net

    cc1200_uhf_sheet()

    # ===================== Sheet 2: CC1200 VHF 144 MHz =====================
    @circuit(name="CC1200_VHF_144MHz")
    def cc1200_vhf_sheet():
        """CC1200 VHF 144 MHz — TIDR220 adapted, needs VNA tuning"""
        cc = Component(symbol="RF:CC1200", ref="U", value="CC1200RHBR",
                        footprint="Package_DFN_QFN:QFN-32-1EP_5x5mm_P0.5mm_EP3.45x3.45mm")

        # SPI with 100Ω series resistors
        for name, ext_net, cc_pin in [("SCLK", vhf_sclk, "SCLK"),
                                       ("MOSI", vhf_mosi, "SI"),
                                       ("MISO", vhf_miso, "SO(GPIO1)")]:
            r = Component(symbol="Device:R", ref="R", value="100R",
                           footprint="Resistor_SMD:R_0603_1608Metric")
            cc_side = Net(f"VHF_{name}_CC")
            r[1] += ext_net
            r[2] += cc_side
            cc[cc_pin] += cc_side

        # CS and RESET with 10k pull-ups
        cc["~{CS}"] += vhf_csn
        cc["~{RESET}"] += vhf_reset
        for net in [vhf_csn, vhf_reset]:
            r = Component(symbol="Device:R", ref="R", value="10k",
                           footprint="Resistor_SMD:R_0603_1608Metric")
            r[1] += vcc_3v3
            r[2] += net

        # GPIO
        cc["GPIO0"] += vhf_gpio0
        cc["GPIO2"] += vhf_gpio2
        cc["GPIO3"] += vhf_gpio3

        # RF passives — VHF 144 MHz
        # Source: TI CC1120EM 169 MHz (TIDR220 schematic, TIDR222 BOM)
        # Inductors scaled up ~17% for 144 MHz (lower freq → larger L)
        # WARNING: Starting values only — VNA tuning required!
        _cc1200_passives(cc, "VHF", vcc_3v3, gnd,
                          r_bias_val="56k", c_xosc_val="15pF", c_pll_val="1.8nF",
                          # PA bias
                          r_pa_val="22R", l_choke_val="270nH", c_pa_bias_val="100pF",
                          # TX matching
                          c_series_val="100pF",
                          l_tx_vals=["22nH", "120nH", "82nH"],
                          c_feedback_val="1.5pF", c_shunt_val="15pF",
                          c_dc_val="100pF",
                          # TRX coupling
                          c_trx_val="15pF", l_trx_val="47nH",
                          # LNA differential balun
                          l_bridge_val="180nH", l_bias_p_val="100nH",
                          l_n_val="100nH", c_p_val="15pF", c_n_val="15pF")

        # Test points on SPI signals
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
        pi[8] += uart_rx     # GPIO14/TXD → Pico RX (crossover)
        pi[10] += uart_tx    # GPIO15/RXD ← Pico TX (crossover)
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
