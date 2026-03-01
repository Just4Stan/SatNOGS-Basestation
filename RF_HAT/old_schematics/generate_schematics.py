#!/usr/bin/env python3
"""
Generate KiCad 9.0 sub-schematic files for the RF HAT project.

Reads symbol definitions from KiCad's installed libraries and produces
valid .kicad_sch files for all 5 sub-sheets:
  - cc1200_uhf.kicad_sch  (432 MHz CC1200 transceiver)
  - cc1200_vhf.kicad_sch  (144 MHz CC1200 transceiver)
  - pico.kicad_sch         (Raspberry Pi Pico SPI host)
  - power.kicad_sch        (Power supply: 24V -> 5V -> 3.3V)
  - pi_header.kicad_sch    (Raspberry Pi 2x20 GPIO header)

Each file contains:
  - lib_symbols section with all symbol types used on that sheet
  - Symbol instances placed in a grid layout
  - global_label elements for cross-sheet nets
  - label elements for local nets within the sheet
  - Power symbols (+3.3V, GND, +5V, +24V) at appropriate pin positions
  - no_connect flags where needed
  - NO wire segments -- just labels at pin positions
"""

import os
import re
import uuid

# ---------------------------------------------------------------------------
# Paths
# ---------------------------------------------------------------------------
OUTPUT_DIR = os.path.dirname(os.path.abspath(__file__))
KICAD_SYM_DIR = "/Applications/KiCad/KiCad.app/Contents/SharedSupport/symbols"

# Root schematic UUID (from RF_HAT.kicad_sch)
ROOT_UUID = "2911cfb4-d970-4141-8275-8153133b791c"

# Sheet UUIDs (from the root schematic sheet references)
SHEET_UUIDS = {
    "cc1200_vhf": "b1000001-cc12-4a00-a001-000000000001",
    "cc1200_uhf": "b1000001-cc12-4a00-a001-000000000002",
    "pico":       "b1000001-cc12-4a00-a001-000000000003",
    "power":      "b1000001-cc12-4a00-a001-000000000004",
    "pi_header":  "b1000001-cc12-4a00-a001-000000000005",
}

# Project name for instances
PROJECT_NAME = "RF_HAT"

# ---------------------------------------------------------------------------
# UUID helper
# ---------------------------------------------------------------------------
def new_uuid():
    return str(uuid.uuid4())


# ---------------------------------------------------------------------------
# S-expression symbol extractor
# ---------------------------------------------------------------------------
def extract_symbol(lib_file, symbol_name):
    """Extract a top-level symbol definition from a KiCad .kicad_sym file.

    Returns the text block including the outer (symbol "Name" ...) parens.
    """
    path = os.path.join(KICAD_SYM_DIR, lib_file)
    with open(path, "r", encoding="utf-8") as f:
        text = f.read()

    # Find the start of (symbol "symbol_name"
    # Need to match at the first indentation level (one tab)
    escaped = re.escape(symbol_name)
    pattern = re.compile(r'^\t\(symbol "' + escaped + r'"', re.MULTILINE)
    m = pattern.search(text)
    if m is None:
        raise ValueError(f"Symbol '{symbol_name}' not found in {lib_file}")

    start = m.start()
    # Count parens to find end
    depth = 0
    i = start
    while i < len(text):
        if text[i] == '(':
            depth += 1
        elif text[i] == ')':
            depth -= 1
            if depth == 0:
                return text[start:i+1]
        i += 1
    raise ValueError(f"Could not find end of symbol '{symbol_name}' in {lib_file}")


def make_lib_symbol(lib_prefix, symbol_name, raw_text):
    """Wrap an extracted symbol definition so it has the lib_id as its name.

    In the lib_symbols section of a .kicad_sch, symbols are named like
    "Device:R" rather than just "R".  We also need to rewrite internal
    sub-symbol references (e.g. "R_0_1" -> "Device:R_0_1").
    """
    full_name = f"{lib_prefix}:{symbol_name}"
    # Replace the first occurrence of (symbol "Name" with (symbol "Lib:Name"
    old_tag = f'(symbol "{symbol_name}"'
    new_tag = f'(symbol "{full_name}"'
    result = raw_text.replace(old_tag, new_tag, 1)

    # Child sub-symbol names (e.g. "R_0_1", "R_1_1") must NOT get the
    # library prefix -- KiCad expects them bare.  Only the parent gets it.
    return result


# ---------------------------------------------------------------------------
# Collect all needed lib_symbol text blocks
# ---------------------------------------------------------------------------
def build_lib_symbols_cache():
    """Extract and cache all symbol definitions needed across all sheets."""
    symbols = {}

    device_syms = ["R", "C", "L", "Crystal_GND24", "FerriteBead"]
    for s in device_syms:
        raw = extract_symbol("Device.kicad_sym", s)
        symbols[f"Device:{s}"] = make_lib_symbol("Device", s, raw)

    # RF: CC1200
    raw = extract_symbol("RF.kicad_sym", "CC1200")
    symbols["RF:CC1200"] = make_lib_symbol("RF", "CC1200", raw)

    # Connector: Conn_Coaxial, TestPoint, Raspberry_Pi_2_3
    for s in ["Conn_Coaxial", "TestPoint"]:
        raw = extract_symbol("Connector.kicad_sym", s)
        symbols[f"Connector:{s}"] = make_lib_symbol("Connector", s, raw)

    raw = extract_symbol("Connector.kicad_sym", "Raspberry_Pi_2_3")
    symbols["Connector:Raspberry_Pi_2_3"] = make_lib_symbol("Connector", "Raspberry_Pi_2_3", raw)

    # MCU_Module: RaspberryPi_Pico
    raw = extract_symbol("MCU_Module.kicad_sym", "RaspberryPi_Pico")
    symbols["MCU_Module:RaspberryPi_Pico"] = make_lib_symbol("MCU_Module", "RaspberryPi_Pico", raw)

    # Power symbols
    for s in ["+3.3V", "GND", "+5V", "+24V", "PWR_FLAG"]:
        raw = extract_symbol("power.kicad_sym", s)
        symbols[f"power:{s}"] = make_lib_symbol("power", s, raw)

    # Conn_01x02_Pin for power input connector
    raw = extract_symbol("Connector.kicad_sym", "Conn_01x02_Pin")
    symbols["Connector:Conn_01x02_Pin"] = make_lib_symbol("Connector", "Conn_01x02_Pin", raw)

    return symbols


# ---------------------------------------------------------------------------
# KiCad schematic generation helpers
# ---------------------------------------------------------------------------
def sch_header(sheet_uuid, paper="A3"):
    return f"""(kicad_sch
\t(version 20250114)
\t(generator "eeschema")
\t(generator_version "9.0")
\t(uuid "{sheet_uuid}")
\t(paper "{paper}")
"""


def sch_lib_symbols(needed_keys, cache):
    lines = ["\t(lib_symbols"]
    for key in sorted(needed_keys):
        if key in cache:
            # Indent the symbol block by two tabs
            block = cache[key]
            # The extracted block starts with \t(symbol ...  -- add one more tab
            for line in block.splitlines():
                # Preserve relative indentation: source is at 1-tab base,
                # add 1 tab to shift into the (lib_symbols ...) block
                lines.append("\t" + line)
    lines.append("\t)")
    return "\n".join(lines)


def place_symbol(lib_id, x, y, rotation, ref_des, ref_value, footprint,
                 pin_uuids, sheet_path, unit=1, is_power=False, dnp=False,
                 extra_props=None):
    """Return the text for a placed symbol instance."""
    uid = new_uuid()
    ref_prefix = "#PWR" if is_power else ref_des.rstrip("0123456789")

    rot_str = f"{rotation}" if rotation else "0"

    # Reference position offset
    ref_x = x + 2
    ref_y = y - 2
    val_x = x + 2
    val_y = y + 2

    lines = []
    lines.append(f'\t(symbol')
    lines.append(f'\t\t(lib_id "{lib_id}")')
    lines.append(f'\t\t(at {x} {y} {rot_str})')
    lines.append(f'\t\t(unit {unit})')
    lines.append(f'\t\t(exclude_from_sim no)')
    if is_power:
        lines.append(f'\t\t(in_bom no)')
    else:
        lines.append(f'\t\t(in_bom yes)')
    lines.append(f'\t\t(on_board yes)')
    lines.append(f'\t\t(dnp {"yes" if dnp else "no"})')
    lines.append(f'\t\t(fields_autoplaced yes)')
    lines.append(f'\t\t(uuid "{uid}")')

    lines.append(f'\t\t(property "Reference" "{ref_des}"')
    lines.append(f'\t\t\t(at {ref_x} {ref_y} 0)')
    lines.append(f'\t\t\t(effects')
    lines.append(f'\t\t\t\t(font')
    lines.append(f'\t\t\t\t\t(size 1.27 1.27)')
    lines.append(f'\t\t\t\t)')
    if is_power:
        lines.append(f'\t\t\t\t(hide yes)')
    lines.append(f'\t\t\t)')
    lines.append(f'\t\t)')

    lines.append(f'\t\t(property "Value" "{ref_value}"')
    lines.append(f'\t\t\t(at {val_x} {val_y} 0)')
    lines.append(f'\t\t\t(effects')
    lines.append(f'\t\t\t\t(font')
    lines.append(f'\t\t\t\t\t(size 1.27 1.27)')
    lines.append(f'\t\t\t\t)')
    if is_power:
        pass  # show value for power symbols
    lines.append(f'\t\t\t)')
    lines.append(f'\t\t)')

    lines.append(f'\t\t(property "Footprint" "{footprint}"')
    lines.append(f'\t\t\t(at {x} {y} 0)')
    lines.append(f'\t\t\t(effects')
    lines.append(f'\t\t\t\t(font')
    lines.append(f'\t\t\t\t\t(size 1.27 1.27)')
    lines.append(f'\t\t\t\t)')
    lines.append(f'\t\t\t\t(hide yes)')
    lines.append(f'\t\t\t)')
    lines.append(f'\t\t)')

    lines.append(f'\t\t(property "Datasheet" "~"')
    lines.append(f'\t\t\t(at {x} {y} 0)')
    lines.append(f'\t\t\t(effects')
    lines.append(f'\t\t\t\t(font')
    lines.append(f'\t\t\t\t\t(size 1.27 1.27)')
    lines.append(f'\t\t\t\t)')
    lines.append(f'\t\t\t\t(hide yes)')
    lines.append(f'\t\t\t)')
    lines.append(f'\t\t)')

    if extra_props:
        for pname, pval in extra_props:
            lines.append(f'\t\t(property "{pname}" "{pval}"')
            lines.append(f'\t\t\t(at {x} {y} 0)')
            lines.append(f'\t\t\t(effects')
            lines.append(f'\t\t\t\t(font')
            lines.append(f'\t\t\t\t\t(size 1.27 1.27)')
            lines.append(f'\t\t\t\t)')
            lines.append(f'\t\t\t\t(hide yes)')
            lines.append(f'\t\t\t)')
            lines.append(f'\t\t)')

    # Pin UUIDs
    for pin_num in pin_uuids:
        lines.append(f'\t\t(pin "{pin_num}"')
        lines.append(f'\t\t\t(uuid "{new_uuid()}")')
        lines.append(f'\t\t)')

    # Instances
    lines.append(f'\t\t(instances')
    lines.append(f'\t\t\t(project "{PROJECT_NAME}"')
    lines.append(f'\t\t\t\t(path "{sheet_path}"')
    lines.append(f'\t\t\t\t\t(reference "{ref_des}")')
    lines.append(f'\t\t\t\t\t(unit {unit})')
    lines.append(f'\t\t\t\t)')
    lines.append(f'\t\t\t)')
    lines.append(f'\t\t)')
    lines.append(f'\t)')

    return "\n".join(lines)


def place_global_label(name, x, y, angle=0, shape="bidirectional"):
    """Return text for a global_label element."""
    uid = new_uuid()
    justify = "left" if angle == 0 else ("right" if angle == 180 else "left")
    return f"""\t(global_label "{name}"
\t\t(shape {shape})
\t\t(at {x} {y} {angle})
\t\t(effects
\t\t\t(font
\t\t\t\t(size 1.27 1.27)
\t\t\t)
\t\t\t(justify {justify})
\t\t)
\t\t(uuid "{uid}")
\t\t(property "Intersheets" ""
\t\t\t(at 0 0 0)
\t\t\t(effects
\t\t\t\t(font
\t\t\t\t\t(size 1.27 1.27)
\t\t\t\t)
\t\t\t\t(hide yes)
\t\t\t)
\t\t)
\t)"""


def place_label(name, x, y, angle=0):
    """Return text for a local label element."""
    uid = new_uuid()
    justify = "left" if angle == 0 else ("right" if angle == 180 else "left")
    return f"""\t(label "{name}"
\t\t(at {x} {y} {angle})
\t\t(effects
\t\t\t(font
\t\t\t\t(size 1.27 1.27)
\t\t\t)
\t\t\t(justify {justify})
\t\t)
\t\t(uuid "{uid}")
\t)"""


def place_no_connect(x, y):
    uid = new_uuid()
    return f'\t(no_connect (at {x} {y}) (uuid "{uid}"))'


def place_text(text, x, y, size=2.54):
    uid = new_uuid()
    escaped = text.replace('"', '\\"')
    return f"""\t(text "{escaped}"
\t\t(exclude_from_sim no)
\t\t(at {x} {y} 0)
\t\t(effects
\t\t\t(font
\t\t\t\t(size {size} {size})
\t\t\t)
\t\t\t(justify left)
\t\t)
\t\t(uuid "{uid}")
\t)"""


def sch_sheet_instances(sheet_uuid, page_num):
    return f"""\t(sheet_instances
\t\t(path "/{ROOT_UUID}/{sheet_uuid}"
\t\t\t(page "{page_num}")
\t\t)
\t)"""


def sch_footer():
    return ")"


def write_sch(filename, sheet_uuid, page_num, needed_libs, cache, body_elements):
    """Assemble and write a complete .kicad_sch file."""
    path = os.path.join(OUTPUT_DIR, filename)
    parts = []
    parts.append(sch_header(new_uuid()))   # each sub-sheet gets its own UUID
    parts.append(sch_lib_symbols(needed_libs, cache))
    parts.append("")  # blank line
    for elem in body_elements:
        parts.append(elem)
    parts.append(sch_sheet_instances(sheet_uuid, page_num))
    parts.append(sch_footer())
    content = "\n".join(parts) + "\n"
    with open(path, "w", encoding="utf-8") as f:
        f.write(content)
    print(f"  Written: {path}")


# ---------------------------------------------------------------------------
# Pin position helpers for known symbols
# (connection point = at position, which is where labels go)
# ---------------------------------------------------------------------------

# For 2-pin vertical components (R, C, L, FerriteBead):
#   pin "1" at (0, 3.81) relative to symbol origin
#   pin "2" at (0, -3.81)

# CC1200 pin positions (from KiCad library):
CC1200_PINS = {
    # pin_number: (rel_x, rel_y) -- connection point
    # Left side (pointing right, angle=0)
    "2":  (-17.78, 25.4),    # ~RESET
    "11": (-17.78, 20.32),   # ~CS
    "8":  (-17.78, 17.78),   # SCLK
    "7":  (-17.78, 15.24),   # SI (MOSI)
    "9":  (-17.78, 12.7),    # SO (MISO/GPIO1)
    "10": (-17.78, 7.62),    # GPIO0
    "4":  (-17.78, 5.08),    # GPIO2
    "3":  (-17.78, 2.54),    # GPIO3
    "30": (-17.78, -2.54),   # XOSC_Q1
    "31": (-17.78, -5.08),   # XOSC_Q2
    "32": (-17.78, -7.62),   # EXT_XOSC
    "14": (-17.78, -12.7),   # RBIAS
    "23": (-17.78, -17.78),  # LPF0
    "24": (-17.78, -20.32),  # LPF1
    "33": (-17.78, -25.4),   # GND (EP)
    # Right side (pointing left, angle=180)
    "5":  (17.78, 25.4),     # DVDD
    "12": (17.78, 22.86),    # DVDD
    "1":  (17.78, 20.32),    # VDD_GUARD
    "13": (17.78, 15.24),    # AVDD_IF
    "15": (17.78, 12.7),     # AVDD_RF
    "25": (17.78, 10.16),    # AVDD_PFD_CHP
    "28": (17.78, 7.62),     # AVDD_XOSC
    "22": (17.78, 5.08),     # AVDD_SYNTH1
    "27": (17.78, 2.54),     # AVDD_SYNTH2
    "6":  (17.78, -2.54),    # DCPL
    "21": (17.78, -5.08),    # DCPL_VCO
    "26": (17.78, -7.62),    # DCPL_PFD_CHP
    "29": (17.78, -10.16),   # DCPL_XOSC
    "19": (17.78, -15.24),   # LNA_P
    "20": (17.78, -17.78),   # LNA_N
    "18": (17.78, -20.32),   # TRX_SW
    "17": (17.78, -25.4),    # PA
}

# Pin 16 (NC) is not exposed in the KiCad symbol.

# RaspberryPi_Pico pin positions (from KiCad library):
PICO_PINS = {
    # Left side (angle=0)
    "30": (-22.86, 22.86),   # RUN
    "37": (-22.86, 20.32),   # 3V3_EN
    "1":  (-22.86, 15.24),   # GPIO0
    "2":  (-22.86, 12.7),    # GPIO1
    "4":  (-22.86, 10.16),   # GPIO2
    "5":  (-22.86, 7.62),    # GPIO3
    "6":  (-22.86, 5.08),    # GPIO4
    "7":  (-22.86, 2.54),    # GPIO5
    "9":  (-22.86, 0),       # GPIO6
    "10": (-22.86, -2.54),   # GPIO7
    "11": (-22.86, -5.08),   # GPIO8
    "12": (-22.86, -7.62),   # GPIO9
    "14": (-22.86, -10.16),  # GPIO10
    "15": (-22.86, -12.7),   # GPIO11
    "16": (-22.86, -15.24),  # GPIO12
    "17": (-22.86, -17.78),  # GPIO13
    "19": (-22.86, -20.32),  # GPIO14
    "20": (-22.86, -22.86),  # GPIO15
    # Top (angle=270)
    "39": (-5.08, 38.1),     # VSYS
    "40": (0, 38.1),         # VBUS
    "36": (5.08, 38.1),      # 3V3_OUT
    # Bottom (angle=90)  -- GND pins all at same point
    "3":  (0, -35.56),       # GND
    "8":  (0, -35.56),       # GND
    "13": (0, -35.56),       # GND
    "18": (0, -35.56),       # GND
    "23": (0, -35.56),       # GND
    "28": (0, -35.56),       # GND
    "38": (0, -35.56),       # GND
    "33": (0, -35.56),       # AGND (same point)
    # Right side (angle=180)
    "21": (22.86, 12.7),     # GPIO16
    "22": (22.86, 10.16),    # GPIO17
    "24": (22.86, 7.62),     # GPIO18
    "25": (22.86, 5.08),     # GPIO19
    "26": (22.86, 2.54),     # GPIO20
    "27": (22.86, 0),        # GPIO21
    "29": (22.86, -2.54),    # GPIO22
    "35": (22.86, -7.62),    # ADC_VREF
    "31": (22.86, -12.7),    # GPIO26_ADC0
    "32": (22.86, -15.24),   # GPIO27_ADC1
    "34": (22.86, -17.78),   # GPIO28_ADC2
}

# Raspberry_Pi_2_3 pin positions (from KiCad library):
PI_HEADER_PINS = {
    # Left side GPIOs (angle=0)
    "8":  (-20.32, 22.86),   # GPIO14/TXD
    "10": (-20.32, 20.32),   # GPIO15/RXD
    "36": (-20.32, 15.24),   # GPIO16
    "11": (-20.32, 12.7),    # GPIO17
    "12": (-20.32, 10.16),   # GPIO18/PWM0
    "35": (-20.32, 5.08),    # GPIO19/MISO1
    "38": (-20.32, 2.54),    # GPIO20/MOSI1
    "40": (-20.32, 0),       # GPIO21/SCLK1
    "15": (-20.32, -5.08),   # GPIO22
    "16": (-20.32, -7.62),   # GPIO23
    "18": (-20.32, -10.16),  # GPIO24
    "22": (-20.32, -12.7),   # GPIO25
    "37": (-20.32, -15.24),  # GPIO26
    "13": (-20.32, -17.78),  # GPIO27
    # Top power
    "2":  (-5.08, 33.02),    # 5V (primary)
    "4":  (-5.08, 33.02),    # 5V (secondary, hidden)
    "1":  (5.08, 33.02),     # 3V3 (primary)
    "17": (5.08, 33.02),     # 3V3 (secondary, hidden)
    # Bottom GND
    "6":  (0, -33.02),       # GND (primary)
    "9":  (0, -33.02),       # GND (hidden)
    "14": (0, -33.02),       # GND (hidden)
    "20": (0, -33.02),       # GND (hidden)
    "25": (0, -33.02),       # GND (hidden)
    "30": (0, -33.02),       # GND (hidden)
    "34": (0, -33.02),       # GND (hidden)
    "39": (0, -33.02),       # GND (hidden)
    # Right side GPIOs (angle=180)
    "27": (20.32, 22.86),    # ID_SD/GPIO0
    "28": (20.32, 20.32),    # ID_SC/GPIO1
    "3":  (20.32, 15.24),    # SDA/GPIO2
    "5":  (20.32, 12.7),     # SCL/GPIO3
    "7":  (20.32, 7.62),     # GCLK0/GPIO4
    "29": (20.32, 5.08),     # GCLK1/GPIO5
    "31": (20.32, 2.54),     # GCLK2/GPIO6
    "26": (20.32, -2.54),    # ~CE1~/GPIO7
    "24": (20.32, -5.08),    # ~CE0~/GPIO8
    "21": (20.32, -7.62),    # MISO0/GPIO9
    "19": (20.32, -10.16),   # MOSI0/GPIO10
    "23": (20.32, -12.7),    # SCLK0/GPIO11
    "32": (20.32, -17.78),   # PWM0/GPIO12
    "33": (20.32, -20.32),   # PWM1/GPIO13
}


# ---------------------------------------------------------------------------
# Convenience: place a 2-pin vertical component with labels
# ---------------------------------------------------------------------------
def vert_component(lib_id, ref, value, footprint, x, y, pin1_label, pin2_label,
                   sheet_path, is_pin1_global=False, is_pin2_global=False,
                   pin1_is_power=None, pin2_is_power=None):
    """Place a 2-pin vertical component (R, C, L, FerriteBead) and its labels.

    Returns list of text elements.
    pin1 is at top (0, 3.81) -> absolute (x, y+3.81)
    pin2 is at bottom (0, -3.81) -> absolute (x, y-3.81)

    pin1_is_power/pin2_is_power: if set to a tuple (lib_id, ref_des, value),
    place a power symbol instead of a label.
    """
    elems = []
    # Place the component
    elems.append(place_symbol(lib_id, x, y, 0, ref, value, footprint,
                              ["1", "2"], sheet_path))

    p1_x, p1_y = x, y - 3.81  # pin 1 at top when orientation is default (270deg from top)
    p2_x, p2_y = x, y + 3.81  # pin 2 at bottom

    # Wait -- let me re-check. For vertical R/C/L in KiCad:
    # pin 1: (at 0 3.81 270) means connection at (0, 3.81), angle 270 means pointing down
    # pin 2: (at 0 -3.81 90) means connection at (0, -3.81), angle 90 means pointing up
    # So pin 1 is at TOP (y + 3.81 in schematic coords? No... KiCad Y is inverted)
    # Actually in KiCad schematics, Y increases downward on screen but the coordinates
    # use standard math coords in the file format.
    # pin 1 at (0, 3.81) = 3.81 units UP from symbol center
    # But schematic Y-axis: positive Y = downward on screen
    # Actually KiCad 9 schematic coords: Y increases downward.
    # So (0, 3.81) is BELOW the center, and (0, -3.81) is ABOVE.
    # Wait no. Let me re-check from the real file:
    # In encoders.kicad_sch, C at (175.26, 34.29):
    # pin 1 connection at (175.26, 34.29 + (-3.81)) = NOT... actually the pin
    # at (0, 3.81 with angle 270) for pin 1. In KiCad screen coords Y-down,
    # 3.81 = below center. So pin 1 is at bottom? Actually angle 270 means
    # the pin points downward (270 degrees from positive X axis).

    # Let me just use the convention: for a vertical R/C/L placed at (x, y):
    #   pin 1 at (x, y-3.81)  (above in screen coords = smaller Y)
    #   pin 2 at (x, y+3.81)  (below in screen coords = larger Y)
    # Wait: KiCad schematic Y axis: larger Y = further down the page.
    # Pin (at 0 3.81 270): 3.81 in Y means 3.81 below center, angle 270 = pointing down.
    # So pin 1's connection point is at y_center + 3.81 (below center).
    # But "below" in the schematic means larger Y value.
    # Hmm, but visually resistors have pin 1 at top. Let me re-read:
    # angle 270 = pin extends downward from connection point into the body.
    # Connection point is the wire endpoint. For a vertical resistor:
    # pin 1 (at 0 3.81 270): connection at (0, 3.81), pin goes DOWN into body.
    # So pin 1's wire connection is at the BOTTOM of the component (large Y = down on screen).
    # pin 2 (at 0 -3.81 90): connection at (0, -3.81), pin goes UP into body.
    # So pin 2's wire connection is at the TOP of the component (small Y = up on screen).

    # This is confusing. In KiCad 9 schematics, Y increases DOWNWARD.
    # So pin 1 at (0, 3.81) = pin 1 at bottom, pin 2 at (0, -3.81) = pin 2 at top.

    # For our purposes, let's just use:
    p1_x, p1_y = x, y + 3.81   # pin 1 connection (bottom of vertical part)
    p2_x, p2_y = x, y - 3.81   # pin 2 connection (top of vertical part)

    # Place labels/power at pin positions
    if pin1_is_power:
        pw_lib, pw_ref, pw_val = pin1_is_power
        elems.append(place_symbol(pw_lib, p1_x, p1_y, 0, pw_ref, pw_val, "",
                                  ["1"], sheet_path, is_power=True))
    elif pin1_label:
        if is_pin1_global:
            elems.append(place_global_label(pin1_label, p1_x, p1_y, 0))
        else:
            elems.append(place_label(pin1_label, p1_x, p1_y, 0))

    if pin2_is_power:
        pw_lib, pw_ref, pw_val = pin2_is_power
        elems.append(place_symbol(pw_lib, p2_x, p2_y, 0, pw_ref, pw_val, "",
                                  ["1"], sheet_path, is_power=True))
    elif pin2_label:
        if is_pin2_global:
            elems.append(place_global_label(pin2_label, p2_x, p2_y, 0))
        else:
            elems.append(place_label(pin2_label, p2_x, p2_y, 0))

    return elems


# ---------------------------------------------------------------------------
# Power symbol counter
# ---------------------------------------------------------------------------
_pwr_counter = [0]

def next_pwr_ref():
    _pwr_counter[0] += 1
    return f"#PWR{_pwr_counter[0]:04d}"


def pwr_33(x, y, sheet_path):
    return place_symbol("power:+3.3V", x, y, 0, next_pwr_ref(), "+3.3V", "",
                        ["1"], sheet_path, is_power=True)


def pwr_gnd(x, y, sheet_path):
    return place_symbol("power:GND", x, y, 0, next_pwr_ref(), "GND", "",
                        ["1"], sheet_path, is_power=True)


def pwr_5v(x, y, sheet_path):
    return place_symbol("power:+5V", x, y, 0, next_pwr_ref(), "+5V", "",
                        ["1"], sheet_path, is_power=True)


def pwr_24v(x, y, sheet_path):
    return place_symbol("power:+24V", x, y, 0, next_pwr_ref(), "+24V", "",
                        ["1"], sheet_path, is_power=True)


def pwr_flag(x, y, sheet_path):
    return place_symbol("power:PWR_FLAG", x, y, 0, next_pwr_ref(), "PWR_FLAG", "",
                        ["1"], sheet_path, is_power=True)


# ---------------------------------------------------------------------------
# Sheet generators
# ---------------------------------------------------------------------------

def generate_cc1200_sheet(prefix, freq_label, sheet_uuid, page_num, cache,
                          # Component designators
                          u_ref, y_ref, j_ref, r_refs, l_refs, fb_ref,
                          c_refs, tp_refs,
                          # Component values
                          r_values, l_values, c_values,
                          # Net label prefixes (UHF_ or VHF_)
                          net_prefix):
    """Generate a CC1200 transceiver sub-sheet (UHF or VHF)."""
    sheet_path = f"/{ROOT_UUID}/{sheet_uuid}"
    elems = []

    # Title
    elems.append(place_text(f"CC1200 {freq_label} Transceiver", 25.4, 25.4, 3.81))

    # ---- CC1200 placement ----
    cc_x, cc_y = 127, 101.6   # center of CC1200
    elems.append(place_symbol(
        "RF:CC1200", cc_x, cc_y, 0, u_ref, "CC1200RHBR",
        "Package_DFN_QFN:QFN-32-1EP_5x5mm_P0.5mm_EP3.45x3.45mm",
        [str(i) for i in range(1, 34)],
        sheet_path
    ))

    # ---- Power connections on CC1200 ----
    # VDD pins: 1, 5, 12, 13, 15, 22, 25, 27, 28 -> +3.3V
    vdd_pins = ["1", "5", "12", "13", "15", "22", "25", "27", "28"]
    for p in vdd_pins:
        px = cc_x + CC1200_PINS[p][0]
        py = cc_y + CC1200_PINS[p][1]
        elems.append(pwr_33(px, py, sheet_path))

    # GND: pin 33 (EP) and pin 32 (EXT_XOSC)
    for p in ["33", "32"]:
        px = cc_x + CC1200_PINS[p][0]
        py = cc_y + CC1200_PINS[p][1]
        elems.append(pwr_gnd(px, py, sheet_path))

    # ---- SPI / control global labels ----
    spi_map = {
        "7":  f"{net_prefix}MOSI",
        "8":  f"{net_prefix}SCLK",
        "9":  f"{net_prefix}MISO",
        "11": f"{net_prefix}CSN",
        "2":  f"{net_prefix}RESET",
        "10": f"{net_prefix}GPIO0",
        "4":  f"{net_prefix}GPIO2",
        "3":  f"{net_prefix}GPIO3",
    }
    for pin, net in spi_map.items():
        px = cc_x + CC1200_PINS[pin][0]
        py = cc_y + CC1200_PINS[pin][1]
        elems.append(place_global_label(net, px, py, 180))

    # ---- Local net labels on CC1200 pins ----
    local_map = {
        "30": f"{prefix}_XOSC_Q1",
        "31": f"{prefix}_XOSC_Q2",
        "14": f"{prefix}_RBIAS",
        "23": f"{prefix}_LPF0",
        "24": f"{prefix}_LPF1",
        "6":  f"{prefix}_DCPL",
        "21": f"{prefix}_DCPL_VCO",
        "26": f"{prefix}_DCPL_PFD",
        "29": f"{prefix}_DCPL_XOSC",
        "19": f"{prefix}_LNA_P",
        "20": f"{prefix}_LNA_N",
        "18": f"{prefix}_TRX_SW",
        "17": f"{prefix}_PA",
    }
    for pin, net in local_map.items():
        px = cc_x + CC1200_PINS[pin][0]
        py = cc_y + CC1200_PINS[pin][1]
        # Right-side pins need label on the right (angle=0)
        # Left-side pins need label on the left (angle=180)
        if CC1200_PINS[pin][0] > 0:
            elems.append(place_label(net, px, py, 0))
        else:
            elems.append(place_label(net, px, py, 180))

    # ---- Crystal Y ----
    # Crystal_GND24 pins: 1 at (-3.81,0), 2 at (0,5.08), 3 at (3.81,0), 4 at (0,-5.08)
    # Place crystal to the left of CC1200, near XOSC pins
    cry_x, cry_y = 86.36, 96.52
    elems.append(place_symbol(
        "Device:Crystal_GND24", cry_x, cry_y, 0, y_ref, "40MHz",
        "Crystal:Crystal_SMD_3225-4Pin_3.2x2.5mm",
        ["1", "2", "3", "4"], sheet_path
    ))
    # Crystal pin 1 at (-3.81, 0) -> XOSC_Q1
    elems.append(place_label(f"{prefix}_XOSC_Q1", cry_x - 3.81, cry_y, 180))
    # Crystal pin 3 at (3.81, 0) -> XOSC_Q2
    elems.append(place_label(f"{prefix}_XOSC_Q2", cry_x + 3.81, cry_y, 0))
    # Crystal pins 2,4 -> GND
    elems.append(pwr_gnd(cry_x, cry_y + 5.08, sheet_path))
    elems.append(pwr_gnd(cry_x, cry_y - 5.08, sheet_path))

    # ---- XOSC load caps (C23, C24 equivalents) ----
    xosc_c1_idx = len(c_refs) - 4  # second-to-last pair
    xosc_c2_idx = len(c_refs) - 3
    cap_xosc1_x, cap_xosc1_y = 78.74, 93.98
    elems.append(place_symbol(
        "Device:C", cap_xosc1_x, cap_xosc1_y, 0, c_refs[xosc_c1_idx], c_values[xosc_c1_idx],
        "Capacitor_SMD:C_0805_2012Metric", ["1", "2"], sheet_path
    ))
    elems.append(place_label(f"{prefix}_XOSC_Q1", cap_xosc1_x, cap_xosc1_y + 3.81, 0))
    elems.append(pwr_gnd(cap_xosc1_x, cap_xosc1_y - 3.81, sheet_path))

    cap_xosc2_x, cap_xosc2_y = 93.98, 93.98
    elems.append(place_symbol(
        "Device:C", cap_xosc2_x, cap_xosc2_y, 0, c_refs[xosc_c2_idx], c_values[xosc_c2_idx],
        "Capacitor_SMD:C_0805_2012Metric", ["1", "2"], sheet_path
    ))
    elems.append(place_label(f"{prefix}_XOSC_Q2", cap_xosc2_x, cap_xosc2_y + 3.81, 0))
    elems.append(pwr_gnd(cap_xosc2_x, cap_xosc2_y - 3.81, sheet_path))

    # ---- SMA connector ----
    # Conn_Coaxial: pin 1 at (-5.08, 0), pin 2 at (0, -5.08)
    sma_x, sma_y = 215.9, 127
    elems.append(place_symbol(
        "Connector:Conn_Coaxial", sma_x, sma_y, 0, j_ref, "SMA",
        "Connector_Coaxial:SMA_Amphenol_132289_EdgeMount",
        ["1", "2"], sheet_path
    ))
    elems.append(place_label(f"{prefix}_ANT", sma_x - 5.08, sma_y, 180))
    elems.append(pwr_gnd(sma_x, sma_y + 5.08, sheet_path))

    # ---- RBIAS resistor ----
    rb_x, rb_y = 86.36, 114.3
    elems.append(place_symbol(
        "Device:R", rb_x, rb_y, 0, r_refs[0], r_values[0],
        "Resistor_SMD:R_0402_1005Metric", ["1", "2"], sheet_path
    ))
    elems.append(place_label(f"{prefix}_RBIAS", rb_x, rb_y + 3.81, 0))
    elems.append(pwr_gnd(rb_x, rb_y - 3.81, sheet_path))

    # ---- PA bias / TRX resistor ----
    r2_x, r2_y = 190.5, 127
    elems.append(place_symbol(
        "Device:R", r2_x, r2_y, 0, r_refs[1], r_values[1],
        "Resistor_SMD:R_0805_2012Metric", ["1", "2"], sheet_path
    ))
    elems.append(place_label(f"{prefix}_PA", r2_x, r2_y + 3.81, 0))
    elems.append(place_label(f"{prefix}_MN1", r2_x, r2_y - 3.81, 0))

    # ---- Inductors (matching network) ----
    # Place inductors in a row to the right of CC1200
    l_base_x = 165.1
    l_base_y = 76.2
    for i, (l_ref, l_val) in enumerate(zip(l_refs, l_values)):
        lx = l_base_x + (i % 4) * 12.7
        ly = l_base_y + (i // 4) * 15.24
        elems.append(place_symbol(
            "Device:L", lx, ly, 0, l_ref, l_val,
            "Inductor_SMD:L_0603_1608Metric", ["1", "2"], sheet_path
        ))
        # Local labels for matching network
        elems.append(place_label(f"{prefix}_LN{i}A", lx, ly + 3.81, 0))
        elems.append(place_label(f"{prefix}_LN{i}B", lx, ly - 3.81, 0))

    # ---- Ferrite bead ----
    fb_x, fb_y = 165.1, 55.88
    elems.append(place_symbol(
        "Device:FerriteBead", fb_x, fb_y, 0, fb_ref, "1k",
        "Inductor_SMD:L_0402_1005Metric", ["1", "2"], sheet_path
    ))
    elems.append(pwr_33(fb_x, fb_y + 3.81, sheet_path))
    elems.append(place_label(f"{prefix}_AVDD_FILT", fb_x, fb_y - 3.81, 0))

    # ---- Capacitors (decoupling & matching) ----
    # Place capacitors in a grid below the CC1200
    c_base_x = 152.4
    c_base_y = 139.7
    # Skip the last 3 that were already placed (XOSC caps + last bypass)
    main_caps = min(len(c_refs) - 3, 20)  # limit grid size
    for i in range(main_caps):
        cx = c_base_x + (i % 6) * 12.7
        cy = c_base_y + (i // 6) * 15.24
        elems.append(place_symbol(
            "Device:C", cx, cy, 0, c_refs[i], c_values[i],
            "Capacitor_SMD:C_0603_1608Metric" if "pF" in c_values[i] or "nF" in c_values[i].lower() else "Capacitor_SMD:C_0805_2012Metric",
            ["1", "2"], sheet_path
        ))
        # First several caps are decoupling (to GND)
        if i < 4:
            # Matching network caps - local labels
            elems.append(place_label(f"{prefix}_CN{i}A", cx, cy + 3.81, 0))
            elems.append(pwr_gnd(cx, cy - 3.81, sheet_path))
        else:
            # Decoupling/bypass caps
            elems.append(pwr_33(cx, cy + 3.81, sheet_path))
            elems.append(pwr_gnd(cx, cy - 3.81, sheet_path))

    # ---- Remaining bypass cap ----
    last_c_idx = len(c_refs) - 1
    lc_x, lc_y = 165.1, 60.96 + 15.24
    elems.append(place_symbol(
        "Device:C", lc_x, lc_y, 0, c_refs[last_c_idx], c_values[last_c_idx],
        "Capacitor_SMD:C_0603_1608Metric", ["1", "2"], sheet_path
    ))
    elems.append(pwr_33(lc_x, lc_y + 3.81, sheet_path))
    elems.append(pwr_gnd(lc_x, lc_y - 3.81, sheet_path))

    # ---- PLL filter cap (C9/C34) between LPF0 and LPF1 ----
    pll_x, pll_y = 86.36, 121.92 + 7.62
    pll_c_idx = 8 if len(c_refs) > 10 else len(c_refs) - 5
    # just use a dedicated placement
    elems.append(place_symbol(
        "Device:C", pll_x, pll_y, 0, c_refs[min(pll_c_idx, len(c_refs)-1)],
        c_values[min(pll_c_idx, len(c_values)-1)],
        "Capacitor_SMD:C_0603_1608Metric", ["1", "2"], sheet_path
    ))
    elems.append(place_label(f"{prefix}_LPF0", pll_x, pll_y + 3.81, 0))
    elems.append(place_label(f"{prefix}_LPF1", pll_x, pll_y - 3.81, 0))

    # ---- Test points ----
    tp_base_x = 55.88
    tp_base_y = 55.88
    tp_nets = [f"{net_prefix}MOSI", f"{net_prefix}MISO", f"{net_prefix}SCLK", f"{net_prefix}CSN"]
    for i, (tp_ref, tp_net) in enumerate(zip(tp_refs, tp_nets)):
        tpx = tp_base_x + i * 10.16
        tpy = tp_base_y
        elems.append(place_symbol(
            "Connector:TestPoint", tpx, tpy, 0, tp_ref, tp_ref,
            "TestPoint:TestPoint_Pad_1.0x1.0mm", ["1"], sheet_path
        ))
        elems.append(place_global_label(tp_net, tpx, tpy, 0))

    # ---- Decoupling local labels for DCPL pins ----
    # DCPL, DCPL_VCO, DCPL_PFD, DCPL_XOSC caps placed near CC1200
    dcpl_caps = [
        (f"{prefix}_DCPL",      c_refs[9] if len(c_refs) > 9 else "CX1",
         c_values[9] if len(c_values) > 9 else "220nF"),
        (f"{prefix}_DCPL_VCO",  c_refs[10] if len(c_refs) > 10 else "CX2",
         c_values[10] if len(c_values) > 10 else "10nF"),
        (f"{prefix}_DCPL_PFD",  c_refs[11] if len(c_refs) > 11 else "CX3",
         c_values[11] if len(c_values) > 11 else "47nF"),
        (f"{prefix}_DCPL_XOSC", c_refs[12] if len(c_refs) > 12 else "CX4",
         c_values[12] if len(c_values) > 12 else "47nF"),
    ]
    dcpl_x_base = 180.34
    dcpl_y_base = 96.52
    for i, (net, ref, val) in enumerate(dcpl_caps):
        dx = dcpl_x_base + i * 12.7
        dy = dcpl_y_base
        elems.append(place_symbol(
            "Device:C", dx, dy, 0, ref, val,
            "Capacitor_SMD:C_0805_2012Metric", ["1", "2"], sheet_path
        ))
        elems.append(place_label(net, dx, dy + 3.81, 0))
        elems.append(pwr_gnd(dx, dy - 3.81, sheet_path))

    # ---- Collect needed lib symbols ----
    needed = {"RF:CC1200", "Device:R", "Device:C", "Device:L",
              "Device:Crystal_GND24", "Device:FerriteBead",
              "Connector:Conn_Coaxial", "Connector:TestPoint",
              "power:+3.3V", "power:GND"}

    return needed, elems


def generate_cc1200_uhf(cache):
    """Generate cc1200_uhf.kicad_sch"""
    r_refs = ["R1", "R2"]
    r_values = ["56k", "18R"]
    l_refs = [f"L{i}" for i in range(1, 9)]
    l_values = ["56nH", "56nH", "27nH", "27nH", "15nH", "43nH", "22nH", "15nH"]
    c_refs = [f"C{i}" for i in range(1, 26)]
    c_values = [
        "5.1pF", "56pF", "100pF", "39pF",   # C1-C4 matching
        "5.1pF", "6.2pF", "2.2pF", "5.1pF", # C5-C8 matching
        "1.5nF",                               # C9 PLL
        "220nF", "10nF", "47nF", "47nF",      # C10-C13 DCPL
        "47nF", "47nF", "47nF", "47nF",        # C14-C17 bypass
        "47nF", "47nF", "47nF", "47nF", "47nF",# C18-C22 bypass
        "15pF", "15pF",                         # C23-C24 XOSC
        "10nF",                                 # C25 bypass
    ]
    tp_refs = ["TP1", "TP2", "TP3", "TP4"]

    needed, elems = generate_cc1200_sheet(
        prefix="UHF",
        freq_label="432 MHz UHF",
        sheet_uuid=SHEET_UUIDS["cc1200_uhf"],
        page_num="3",
        cache=cache,
        u_ref="U1", y_ref="Y1", j_ref="J1",
        r_refs=r_refs, l_refs=l_refs, fb_ref="FB1",
        c_refs=c_refs, tp_refs=tp_refs,
        r_values=r_values, l_values=l_values, c_values=c_values,
        net_prefix="UHF_",
    )

    write_sch("cc1200_uhf.kicad_sch", SHEET_UUIDS["cc1200_uhf"], "3",
              needed, cache, elems)


def generate_cc1200_vhf(cache):
    """Generate cc1200_vhf.kicad_sch"""
    r_refs = ["R3", "R4"]
    r_values = ["56k", "22R"]
    l_refs = [f"L{i}" for i in range(9, 16)]
    l_values = ["270nH", "22nH", "120nH", "82nH", "180nH", "100nH", "100nH"]
    c_refs = [f"C{i}" for i in range(26, 51)]
    c_values = [
        "100pF", "18pF", "15pF", "1.5pF",     # C26-C29 matching
        "15pF", "15pF", "15pF", "100pF",       # C30-C33 matching
        "1.8nF",                                 # C34 PLL
        "220nF", "10nF", "47nF", "47nF",        # C35-C38 DCPL
        "47nF", "47nF", "47nF", "47nF",          # C39-C42 bypass
        "47nF", "47nF", "47nF", "47nF", "47nF",  # C43-C47 bypass
        "15pF", "15pF",                           # C48-C49 XOSC
        "10nF",                                   # C50 bypass
    ]
    tp_refs = ["TP5", "TP6", "TP7", "TP8"]

    needed, elems = generate_cc1200_sheet(
        prefix="VHF",
        freq_label="144 MHz VHF",
        sheet_uuid=SHEET_UUIDS["cc1200_vhf"],
        page_num="2",
        cache=cache,
        u_ref="U2", y_ref="Y2", j_ref="J2",
        r_refs=r_refs, l_refs=l_refs, fb_ref="FB2",
        c_refs=c_refs, tp_refs=tp_refs,
        r_values=r_values, l_values=l_values, c_values=c_values,
        net_prefix="VHF_",
    )

    write_sch("cc1200_vhf.kicad_sch", SHEET_UUIDS["cc1200_vhf"], "2",
              needed, cache, elems)


def generate_pico(cache):
    """Generate pico.kicad_sch"""
    sheet_uuid = SHEET_UUIDS["pico"]
    sheet_path = f"/{ROOT_UUID}/{sheet_uuid}"
    elems = []

    elems.append(place_text("Raspberry Pi Pico - SPI Host", 25.4, 25.4, 3.81))

    # Place Pico at center of sheet
    pico_x, pico_y = 127, 101.6
    all_pins = [str(i) for i in range(1, 41)]
    elems.append(place_symbol(
        "MCU_Module:RaspberryPi_Pico", pico_x, pico_y, 0, "U3", "RaspberryPi_Pico",
        "Module:RaspberryPi_Pico_Common_Unspecified",
        all_pins, sheet_path
    ))

    # Pin assignments with global labels
    # Left side pins (connection at pico_x + pin_rel_x, pico_y + pin_rel_y)
    gpio_map = {
        # pin_num: (net_name, is_global)
        "1":  ("UART_TX", True),          # GP0 -> Pico TX
        "2":  ("UART_RX", True),          # GP1 -> Pico RX
        "10": ("UHF_GPIO0", True),        # GP7
        "11": ("UHF_RESET", True),        # GP8
        "12": ("UHF_GPIO2", True),        # GP9
        "14": ("UHF_SCLK", True),         # GP10 - SPI1 SCK
        "15": ("UHF_MOSI", True),         # GP11 - SPI1 TX
        "16": ("UHF_MISO", True),         # GP12 - SPI1 RX
        "17": ("UHF_CSN", True),          # GP13 - SPI1 CS
        "21": ("VHF_MISO", True),         # GP16 - SPI0 RX
        "22": ("VHF_CSN", True),          # GP17 - SPI0 CS
        "24": ("VHF_SCLK", True),         # GP18 - SPI0 SCK
        "25": ("VHF_MOSI", True),         # GP19 - SPI0 TX
        "26": ("VHF_RESET", True),        # GP20
        "27": ("VHF_GPIO2", True),        # GP21
        "29": ("VHF_GPIO0", True),        # GP22
    }

    for pin, (net, is_global) in gpio_map.items():
        rel = PICO_PINS[pin]
        abs_x = pico_x + rel[0]
        abs_y = pico_y + rel[1]
        # Determine label angle: left-side pins -> 180, right-side -> 0
        if rel[0] < 0:
            angle = 180
        else:
            angle = 0
        if is_global:
            elems.append(place_global_label(net, abs_x, abs_y, angle))
        else:
            elems.append(place_label(net, abs_x, abs_y, angle))

    # No-connect pins (unused GPIOs)
    nc_pins = ["4", "5", "6", "7", "9", "19", "20", "30", "37",
               "31", "32", "34", "35", "40"]
    for pin in nc_pins:
        if pin in PICO_PINS:
            rel = PICO_PINS[pin]
            abs_x = pico_x + rel[0]
            abs_y = pico_y + rel[1]
            elems.append(place_no_connect(abs_x, abs_y))

    # Power connections
    # VSYS (pin 39) -> +5V
    vsys_x = pico_x + PICO_PINS["39"][0]
    vsys_y = pico_x + PICO_PINS["39"][1]  # Bug: should be pico_y, fix below
    vsys_y = pico_y + PICO_PINS["39"][1]
    elems.append(pwr_5v(vsys_x, vsys_y, sheet_path))

    # 3V3 (pin 36) -> +3.3V
    v33_x = pico_x + PICO_PINS["36"][0]
    v33_y = pico_y + PICO_PINS["36"][1]
    elems.append(pwr_33(v33_x, v33_y, sheet_path))

    # GND (pin 3) -> GND
    gnd_x = pico_x + PICO_PINS["3"][0]
    gnd_y = pico_y + PICO_PINS["3"][1]
    elems.append(pwr_gnd(gnd_x, gnd_y, sheet_path))

    # ---- Bypass caps near Pico ----
    # C51: 100nF on +3.3V
    c51_x, c51_y = 165.1, 63.5
    elems.append(place_symbol(
        "Device:C", c51_x, c51_y, 0, "C51", "100nF",
        "Capacitor_SMD:C_0402_1005Metric", ["1", "2"], sheet_path
    ))
    elems.append(pwr_33(c51_x, c51_y + 3.81, sheet_path))
    elems.append(pwr_gnd(c51_x, c51_y - 3.81, sheet_path))

    # C52: 100nF on +5V (VSYS)
    c52_x, c52_y = 86.36, 63.5
    elems.append(place_symbol(
        "Device:C", c52_x, c52_y, 0, "C52", "100nF",
        "Capacitor_SMD:C_0402_1005Metric", ["1", "2"], sheet_path
    ))
    elems.append(pwr_5v(c52_x, c52_y + 3.81, sheet_path))
    elems.append(pwr_gnd(c52_x, c52_y - 3.81, sheet_path))

    # Needed lib symbols
    needed = {"MCU_Module:RaspberryPi_Pico", "Device:C",
              "power:+3.3V", "power:GND", "power:+5V"}

    write_sch("pico.kicad_sch", sheet_uuid, "4", needed, cache, elems)


def generate_power(cache):
    """Generate power.kicad_sch"""
    sheet_uuid = SHEET_UUIDS["power"]
    sheet_path = f"/{ROOT_UUID}/{sheet_uuid}"
    elems = []

    elems.append(place_text("Power Supply: 24V -> 5V -> 3.3V", 25.4, 25.4, 3.81))

    # ---- Architecture notes ----
    elems.append(place_text("24V -> 5V: LM2596S-5 or MP1584EN DC-DC buck module\\n"
                            "5V -> 3.3V: AMS1117-3.3 LDO\\n"
                            "Schottky diode D1 recommended for reverse polarity protection",
                            25.4, 38.1, 1.27))

    # ---- 24V input connector ----
    j3_x, j3_y = 50.8, 76.2
    elems.append(place_symbol(
        "Connector:Conn_01x02_Pin", j3_x, j3_y, 0, "J3", "24V_IN",
        "Connector_PinHeader_2.54mm:PinHeader_1x02_P2.54mm_Vertical",
        ["1", "2"], sheet_path
    ))
    # Conn_01x02_Pin: pin 1 at (-2.54, 0), pin 2 at (-2.54, -2.54)
    # Actually let me check... Conn_01x02_Pin pins are typically on left side
    # For now just place power labels near the connector
    elems.append(pwr_24v(j3_x - 5.08, j3_y, sheet_path))
    elems.append(pwr_gnd(j3_x - 5.08, j3_y + 2.54, sheet_path))

    # ---- PWR_FLAG on +24V and GND to mark them as driven ----
    elems.append(pwr_flag(j3_x - 5.08, j3_y - 5.08, sheet_path))

    # ---- Input capacitor C53: 100uF on 24V ----
    c53_x, c53_y = 76.2, 76.2
    elems.append(place_symbol(
        "Device:C", c53_x, c53_y, 0, "C53", "100uF",
        "Capacitor_SMD:CP_Elec_8x10.5", ["1", "2"], sheet_path
    ))
    elems.append(pwr_24v(c53_x, c53_y + 3.81, sheet_path))
    elems.append(pwr_gnd(c53_x, c53_y - 3.81, sheet_path))

    # ---- 5V rail caps ----
    c54_x, c54_y = 114.3, 76.2
    elems.append(place_symbol(
        "Device:C", c54_x, c54_y, 0, "C54", "22uF",
        "Capacitor_SMD:C_1206_3216Metric", ["1", "2"], sheet_path
    ))
    elems.append(pwr_5v(c54_x, c54_y + 3.81, sheet_path))
    elems.append(pwr_gnd(c54_x, c54_y - 3.81, sheet_path))

    c55_x, c55_y = 127, 76.2
    elems.append(place_symbol(
        "Device:C", c55_x, c55_y, 0, "C55", "100nF",
        "Capacitor_SMD:C_0402_1005Metric", ["1", "2"], sheet_path
    ))
    elems.append(pwr_5v(c55_x, c55_y + 3.81, sheet_path))
    elems.append(pwr_gnd(c55_x, c55_y - 3.81, sheet_path))

    # ---- 3.3V rail caps ----
    c56_x, c56_y = 165.1, 76.2
    elems.append(place_symbol(
        "Device:C", c56_x, c56_y, 0, "C56", "10uF",
        "Capacitor_SMD:C_0805_2012Metric", ["1", "2"], sheet_path
    ))
    elems.append(pwr_33(c56_x, c56_y + 3.81, sheet_path))
    elems.append(pwr_gnd(c56_x, c56_y - 3.81, sheet_path))

    c57_x, c57_y = 177.8, 76.2
    elems.append(place_symbol(
        "Device:C", c57_x, c57_y, 0, "C57", "100nF",
        "Capacitor_SMD:C_0402_1005Metric", ["1", "2"], sheet_path
    ))
    elems.append(pwr_33(c57_x, c57_y + 3.81, sheet_path))
    elems.append(pwr_gnd(c57_x, c57_y - 3.81, sheet_path))

    c58_x, c58_y = 190.5, 76.2
    elems.append(place_symbol(
        "Device:C", c58_x, c58_y, 0, "C58", "22uF",
        "Capacitor_SMD:C_1206_3216Metric", ["1", "2"], sheet_path
    ))
    elems.append(pwr_33(c58_x, c58_y + 3.81, sheet_path))
    elems.append(pwr_gnd(c58_x, c58_y - 3.81, sheet_path))

    # ---- Power rail labels (just to show the rails clearly) ----
    elems.append(place_text("+24V rail", 63.5, 63.5, 1.27))
    elems.append(place_text("+5V rail (from DC-DC)", 101.6, 63.5, 1.27))
    elems.append(place_text("+3.3V rail (from LDO)", 152.4, 63.5, 1.27))

    needed = {"Device:C", "Connector:Conn_01x02_Pin",
              "power:+3.3V", "power:GND", "power:+5V", "power:+24V", "power:PWR_FLAG"}

    write_sch("power.kicad_sch", sheet_uuid, "5", needed, cache, elems)


def generate_pi_header(cache):
    """Generate pi_header.kicad_sch"""
    sheet_uuid = SHEET_UUIDS["pi_header"]
    sheet_path = f"/{ROOT_UUID}/{sheet_uuid}"
    elems = []

    elems.append(place_text("Raspberry Pi GPIO Header (2x20)", 25.4, 25.4, 3.81))

    # Place Pi header connector
    hdr_x, hdr_y = 127, 101.6
    all_pins = [str(i) for i in range(1, 41)]
    elems.append(place_symbol(
        "Connector:Raspberry_Pi_2_3", hdr_x, hdr_y, 0, "J4", "Raspberry_Pi_2_3",
        "Connector_PinSocket_2.54mm:PinSocket_2x20_P2.54mm_Vertical",
        all_pins, sheet_path
    ))

    # ---- Power connections ----
    # 5V (pins 2, 4) at relative (-5.08, 33.02) -- top
    v5_x = hdr_x + PI_HEADER_PINS["2"][0]
    v5_y = hdr_y + PI_HEADER_PINS["2"][1]
    elems.append(pwr_5v(v5_x, v5_y, sheet_path))

    # 3.3V (pins 1, 17) at relative (5.08, 33.02) -- top
    v33_x = hdr_x + PI_HEADER_PINS["1"][0]
    v33_y = hdr_y + PI_HEADER_PINS["1"][1]
    elems.append(pwr_33(v33_x, v33_y, sheet_path))

    # GND (pin 6 = primary) at relative (0, -33.02) -- bottom
    gnd_x = hdr_x + PI_HEADER_PINS["6"][0]
    gnd_y = hdr_y + PI_HEADER_PINS["6"][1]
    elems.append(pwr_gnd(gnd_x, gnd_y, sheet_path))

    # ---- UART connections ----
    # Pi GPIO14/TXD (pin 8) -> UART_RX label (Pi transmits, Pico receives)
    # Pi GPIO15/RXD (pin 10) -> UART_TX label (Pico transmits, Pi receives)
    pin8_x = hdr_x + PI_HEADER_PINS["8"][0]
    pin8_y = hdr_y + PI_HEADER_PINS["8"][1]
    elems.append(place_global_label("UART_RX", pin8_x, pin8_y, 180))

    pin10_x = hdr_x + PI_HEADER_PINS["10"][0]
    pin10_y = hdr_y + PI_HEADER_PINS["10"][1]
    elems.append(place_global_label("UART_TX", pin10_x, pin10_y, 180))

    # ---- No-connect on unused GPIO pins ----
    # Most GPIO pins are unused - add no-connect flags
    nc_header_pins = [
        "36", "11", "12", "35", "38", "40",
        "15", "16", "18", "22", "37", "13",
        "27", "28", "3", "5", "7", "29", "31",
        "26", "24", "21", "19", "23", "32", "33"
    ]
    for pin in nc_header_pins:
        if pin in PI_HEADER_PINS:
            rel = PI_HEADER_PINS[pin]
            abs_x = hdr_x + rel[0]
            abs_y = hdr_y + rel[1]
            elems.append(place_no_connect(abs_x, abs_y))

    needed = {"Connector:Raspberry_Pi_2_3",
              "power:+3.3V", "power:GND", "power:+5V"}

    write_sch("pi_header.kicad_sch", sheet_uuid, "6", needed, cache, elems)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main():
    print("Extracting symbol definitions from KiCad libraries...")
    cache = build_lib_symbols_cache()
    print(f"  Extracted {len(cache)} symbols")

    print("\nGenerating sub-schematics...")
    generate_cc1200_uhf(cache)
    generate_cc1200_vhf(cache)
    generate_pico(cache)
    generate_power(cache)
    generate_pi_header(cache)

    print("\nDone! Generated 5 sub-schematic files in:")
    print(f"  {OUTPUT_DIR}")


if __name__ == "__main__":
    main()
