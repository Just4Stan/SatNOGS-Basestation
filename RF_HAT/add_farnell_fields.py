#!/usr/bin/env python3
"""Add Farnell order codes as custom fields to KiCad schematic symbol instances."""

import re
import os

BASE = os.path.join(os.path.dirname(__file__), 'RF_HAT_out')

# Reference designator -> Farnell Belgium dp/ code
FARNELL = {
    # === Top-level sheet (RF_HAT.kicad_sch) ===
    'D1':  '2211954',   # Bourns CD0603-B0230 Schottky 30V 0603
    'R16': '9238727',   # 100k 0603 1% Yageo
    'R17': '2138429',   # 13.7k 0603 1% Vishay
    'U5':  '',           # LMR51420YFDDCR — not on Farnell (use Mouser/LCSC)
    'U4':  '',           # XT30PW-M — not on Farnell (use TME)
    'C22': '1650835',   # 100nF X7R 0603
    'C24': '4292940',   # 22uF 25V X5R 0805
    'C19': '3013463',   # 4.7uF 50V X5R 0805 Samsung
    'L18': '4142875',   # 4.7uH power inductor (ASPI-4030S-4R7M-T alt)
    # H1-H4: mounting holes, no component

    # === UHF CC1200 (CC1200_UHF_432MHz.kicad_sch) ===
    'U1':  '2345430',   # CC1200RHBR
    'J1':  '2112467',   # SMA edge-mount
    'Y1':  '1712847',   # 40 MHz crystal 3225
    'L1':  '1515375',   # 15nH
    'L2':  '2470386',   # 43nH
    'L3':  '1515382',   # 22nH
    'L4':  '3354667',   # 56nH
    'L5':  '1515375',   # 15nH
    'L6':  '1515383',   # 27nH
    'L7':  '3354667',   # 56nH
    'L8':  '1515383',   # 27nH
    'C1':  '1813415',   # 2.2pF C0G
    'C2':  '2280868',   # 39pF C0G
    'C3':  '2904766',   # 5.1pF C0G
    'C4':  '3759543',   # 6.0pF C0G (6.2pF sub)
    'C5':  '1572627',   # 56pF C0G
    'C6':  '1759022',   # 10nF X7R
    'C7':  '722080',    # 100pF C0G
    'C8':  '2904766',   # 5.1pF C0G
    'C9':  '2904766',   # 5.1pF C0G
    'C10': '2496882',   # 1nF C0G (DC block)
    'C12': '721980',    # 15pF C0G
    'C13': '721980',    # 15pF C0G
    'C14': '2495363',   # 47nF X7R
    'C15': '2495363',   # 47nF X7R
    'C16': '1759022',   # 10nF X7R
    'C17': '1828900',   # 220nF X7R
    'C18': '2672137',   # 1.5nF C0G (PLL)
    'C21': '2495363',   # 47nF X7R
    'C57': '2495363',   # 47nF X7R
    'C58': '2495363',   # 47nF X7R
    'C59': '2495363',   # 47nF X7R
    'C60': '2495363',   # 47nF X7R
    'C61': '2495363',   # 47nF X7R
    'C62': '2495363',   # 47nF X7R
    'C63': '2495363',   # 47nF X7R
    'C64': '2495363',   # 47nF X7R
    'R4':  '9238603',   # 10k 0603
    'R5':  '9238603',   # 10k 0603
    'R6':  '9238271',   # 18R 0603
    'R7':  '9238697',   # 56k 0603

    # === VHF CC1200 (CC1200_VHF_144MHz.kicad_sch) ===
    'U2':  '2345430',   # CC1200RHBR
    'J2':  '2112467',   # SMA edge-mount
    'Y2':  '1712847',   # 40 MHz crystal 3225
    'L9':  '9528032',   # 100nH
    'L10': '9528032',   # 100nH
    'L11': '1515390',   # 47nH
    'L12': '9528032',   # 100nH
    'L13': '3471564',   # 270nH
    'L14': '1515382',   # 22nH
    'L15': '3678407',   # 120nH
    'L16': '1515396',   # 82nH
    'C11': '721980',    # 15pF C0G
    'C20': '721980',    # 15pF C0G
    'C23': '1813422',   # 1.8nF C0G (PLL)
    'C25': '2495363',   # 47nF X7R
    'C26': '2495363',   # 47nF X7R
    'C27': '721980',    # 15pF C0G
    'C28': '2495363',   # 47nF X7R
    'C29': '2495363',   # 47nF X7R
    'C30': '2495363',   # 47nF X7R
    'C31': '1759022',   # 10nF X7R
    'C32': '721980',    # 15pF C0G
    'C33': '2495363',   # 47nF X7R
    'C34': '1828900',   # 220nF X7R
    'C35': '2495363',   # 47nF X7R
    'C36': '2495363',   # 47nF X7R
    'C37': '2495363',   # 47nF X7R
    'C38': '722080',    # 100pF C0G
    'C39': '722080',    # 100pF C0G
    'C40': '2495363',   # 47nF X7R
    'C41': '1759022',   # 10nF X7R
    'C42': '2495363',   # 47nF X7R
    'C43': '722080',    # 100pF C0G
    'C44': '721864',    # 1.5pF C0G
    'C45': '721980',    # 15pF C0G
    'C46': '721980',    # 15pF C0G
    'C47': '722080',    # 100pF C0G
    'R1':  '9238603',   # 10k 0603
    'R2':  '9238603',   # 10k 0603
    'R3':  '9238697',   # 56k 0603
    'R8':  '9238280',   # 22R 0603

    # === Pi Header (Pi_Header.kicad_sch) ===
    'J3':  '9838309',   # 2x20 pin header 2.54mm

    # === Pico (Pico_SPI_Host.kicad_sch) ===
    'U3':  '3643332',   # Raspberry Pi Pico
}

FILES = [
    'RF_HAT.kicad_sch',
    'CC1200_UHF_432MHz.kicad_sch',
    'CC1200_VHF_144MHz.kicad_sch',
    'Pi_Header.kicad_sch',
    'Pico_SPI_Host.kicad_sch',
]


def find_lib_symbols_end(lines):
    """Find the line index where (lib_symbols ...) block ends."""
    depth = 0
    started = False
    for i, line in enumerate(lines):
        if '(lib_symbols' in line and not started:
            started = True
            depth = line.count('(') - line.count(')')
            continue
        if started:
            depth += line.count('(') - line.count(')')
            if depth <= 0:
                return i
    return 0  # no lib_symbols block


def process_file(filepath):
    with open(filepath, 'r') as f:
        lines = f.readlines()

    lib_end = find_lib_symbols_end(lines)
    insertions = []  # list of (line_index, property_text)

    i = lib_end + 1
    while i < len(lines):
        line = lines[i]

        # Detect symbol instance start: a line with just "\t(symbol"
        if line.strip() == '(symbol' and line.startswith('\t'):
            # We're in a symbol instance. Find reference and insertion point.
            ref = None
            comp_x, comp_y = '0', '0'
            insert_before = None

            # Scan forward inside this symbol block
            depth = 1
            j = i + 1
            got_position = False

            while j < len(lines) and depth > 0:
                l = lines[j]
                depth += l.count('(') - l.count(')')

                # Get component position (first (at ...) after symbol start)
                if not got_position:
                    at_m = re.match(r'\s+\(at ([\d.-]+) ([\d.-]+)', l)
                    if at_m:
                        comp_x = at_m.group(1)
                        comp_y = at_m.group(2)
                        got_position = True

                # Get reference designator
                ref_m = re.search(r'\(property "Reference" "([^"]+)"', l)
                if ref_m:
                    ref = ref_m.group(1)

                # Check for existing Farnell property (skip if already present)
                if '"Farnell"' in l:
                    ref = None  # skip this component
                    break

                # Find first (pin line at depth 2 (direct child of symbol)
                if insert_before is None and re.match(r'\t\t\(pin ', l):
                    insert_before = j

                # Find (instances as fallback
                if insert_before is None and re.match(r'\t\t\(instances', l):
                    insert_before = j

                if depth <= 0:
                    break
                j += 1

            if ref and ref in FARNELL and insert_before is not None:
                code = FARNELL[ref]
                if code:  # skip empty codes
                    prop = (
                        f'\t\t(property "Farnell" "{code}"\n'
                        f'\t\t\t(at {comp_x} {comp_y} 0)\n'
                        f'\t\t\t(effects\n'
                        f'\t\t\t\t(font\n'
                        f'\t\t\t\t\t(size 1.27 1.27)\n'
                        f'\t\t\t\t)\n'
                        f'\t\t\t\t(hide yes)\n'
                        f'\t\t\t)\n'
                        f'\t\t)\n'
                    )
                    insertions.append((insert_before, prop, ref))

            i = j if j > i else i + 1
        else:
            i += 1

    # Apply insertions in reverse order so line numbers stay valid
    for line_idx, prop_text, ref in sorted(insertions, key=lambda x: x[0], reverse=True):
        lines.insert(line_idx, prop_text)

    with open(filepath, 'w') as f:
        f.writelines(lines)

    refs = [r for _, _, r in insertions]
    return refs


if __name__ == '__main__':
    total = 0
    for fname in FILES:
        path = os.path.join(BASE, fname)
        if not os.path.exists(path):
            print(f"SKIP {fname} (not found)")
            continue
        refs = process_file(path)
        total += len(refs)
        print(f"{fname}: added Farnell codes to {len(refs)} components: {', '.join(refs)}")

    print(f"\nTotal: {total} Farnell codes added")
    print("\nNot on Farnell (source elsewhere):")
    print("  U4 (XT30PW-M) — TME or AliExpress")
    print("  U5 (LMR51420YFDDCR) — Mouser or LCSC")
    print("  H1-H4 (mounting holes) — no component needed")
