#!/usr/bin/env python3
"""Add LCSC/JLCPCB order codes as custom fields to KiCad schematic symbol instances."""

import re
import os

BASE = os.path.join(os.path.dirname(__file__), 'RF_HAT_out')

# Reference designator -> LCSC part number (Cxxxxxxx)
LCSC = {
    # === Top-level sheet (RF_HAT.kicad_sch) ===
    # Components that already have LCSC in schematic are included for completeness;
    # the script will skip them if already present.
    'D1':  'C3759853',   # Bourns CD0603-B0340R Schottky 40V 0603
    'R16': 'C270364',    # 100k 0603 1% (already in schematic)
    'R17': 'C138175',    # 13.7k 0603 1% (already in schematic)
    'U5':  'C7296200',   # LMR51420YFDDCR buck converter
    'U4':  'C431092',    # XT30PW-M (already in schematic)
    'C22': 'C76939',     # 100nF X7R 0603 (already in schematic)
    'C24': 'C2762594',   # 22uF 0603 (already in schematic)
    'C19': 'C98192',     # 4.7uF 50V 0805 (already in schematic)
    'L18': 'C39846837',  # 4.7uH power inductor 3030 (already in schematic)
    # H1-H4: mounting holes, no component

    # === UHF CC1200 (CC1200_UHF_432MHz.kicad_sch) ===
    'U1':  'C122881',    # CC1200RHBR QFN-32
    'J1':  '',           # SMA edge-mount — hand solder
    'Y1':  'C5380316',   # 40 MHz crystal 3225
    'L1':  'C23651',     # 15nH Sunlord SDCL1608C15NJTDF (160K stock)
    'L2':  'C29683',     # 47nH Sunlord SDCL1608C47NJTDF (sub for 43nH)
    'L3':  'C98071',     # 22nH Murata LQW18AN22NJ00D
    'L4':  'C12831',     # 56nH Sunlord SDCL1608C56NJTDF
    'L5':  'C23651',     # 15nH Sunlord SDCL1608C15NJTDF (160K stock)
    'L6':  'C162550',    # 27nH Murata LQG18HN27NJ00D
    'L7':  'C12831',     # 56nH Sunlord SDCL1608C56NJTDF
    'L8':  'C162550',    # 27nH Murata LQG18HN27NJ00D
    'C1':  'C1651',      # 2.2pF C0G 0603 FH
    'C2':  'C445758',    # 39pF C0G 0603 TDK
    'C3':  'C61816',     # 5.1pF C0G 0603 FH 0603CG5R1C500NT
    'C4':  'C48579262',  # 6.0pF C0G 0603 (6.2pF sub)
    'C5':  'C338104',    # 56pF C0G 0603 TDK
    'C6':  'C57112',     # 10nF X7R 0603 FH
    'C7':  'C14858',     # 100pF C0G 0603 Samsung
    'C8':  'C61816',     # 5.1pF C0G 0603 FH 0603CG5R1C500NT
    'C9':  'C61816',     # 5.1pF C0G 0603 FH 0603CG5R1C500NT
    'C10': 'C342910',    # 1nF C0G 0603 TDK
    'C12': 'C1644',      # 15pF C0G 0603 Samsung (Basic)
    'C13': 'C1644',      # 15pF C0G 0603 Samsung (Basic)
    'C14': 'C1622',      # 47nF X7R 0603 Samsung (Basic)
    'C15': 'C1622',      # 47nF X7R 0603 Samsung (Basic)
    'C16': 'C57112',     # 10nF X7R 0603 FH
    'C17': 'C21120',     # 220nF X7R 0603 Samsung (Basic)
    'C18': 'C76600',     # 1.5nF C0G 0603 TDK
    'C21': 'C1622',      # 47nF X7R 0603 Samsung (Basic)
    'C57': 'C1622',      # 47nF X7R 0603 Samsung (Basic)
    'C58': 'C1622',      # 47nF X7R 0603 Samsung (Basic)
    'C59': 'C1622',      # 47nF X7R 0603 Samsung (Basic)
    'C60': 'C1622',      # 47nF X7R 0603 Samsung (Basic)
    'C61': 'C1622',      # 47nF X7R 0603 Samsung (Basic)
    'C62': 'C1622',      # 47nF X7R 0603 Samsung (Basic)
    'C63': 'C1622',      # 47nF X7R 0603 Samsung (Basic)
    'C64': 'C1622',      # 47nF X7R 0603 Samsung (Basic)
    'R4':  'C25804',     # 10k 0603 UNI-ROYAL (Basic)
    'R5':  'C25804',     # 10k 0603 UNI-ROYAL (Basic)
    'R6':  'C245924',    # 18R 0603 YAGEO
    'R7':  'C23206',     # 56k 0603 UNI-ROYAL

    # === VHF CC1200 (CC1200_VHF_144MHz.kicad_sch) ===
    'U2':  'C122881',    # CC1200RHBR QFN-32
    'J2':  '',           # SMA edge-mount — hand solder
    'Y2':  'C5380316',   # 40 MHz crystal 3225
    'L9':  'C98089',     # 100nH Murata LQW18ANR10J00D
    'L10': 'C98089',     # 100nH Murata LQW18ANR10J00D
    'L11': 'C29683',     # 47nH Sunlord SDCL1608C47NJTDF (129K stock)
    'L12': 'C98089',     # 100nH Murata LQW18ANR10J00D
    'L13': 'C668477',    # 270nH Murata LQW18ANR27G80D
    'L14': 'C98071',     # 22nH Murata LQW18AN22NJ00D
    'L15': 'C307612',    # 120nH Murata LQW18ANR12G80D
    'L16': 'C98085',     # 82nH Murata LQW18AN82NG00D
    'C11': 'C1644',      # 15pF C0G 0603 Samsung (Basic)
    'C20': 'C1644',      # 15pF C0G 0603 Samsung (Basic)
    'C23': 'C343008',    # 1.8nF C0G 0603 TDK 100V (better stock)
    'C25': 'C1622',      # 47nF X7R 0603 Samsung (Basic)
    'C26': 'C1622',      # 47nF X7R 0603 Samsung (Basic)
    'C27': 'C1644',      # 15pF C0G 0603 Samsung (Basic)
    'C28': 'C1622',      # 47nF X7R 0603 Samsung (Basic)
    'C29': 'C1622',      # 47nF X7R 0603 Samsung (Basic)
    'C30': 'C1622',      # 47nF X7R 0603 Samsung (Basic)
    'C31': 'C57112',     # 10nF X7R 0603 FH
    'C32': 'C1644',      # 15pF C0G 0603 Samsung (Basic)
    'C33': 'C1622',      # 47nF X7R 0603 Samsung (Basic)
    'C34': 'C21120',     # 220nF X7R 0603 Samsung (Basic)
    'C35': 'C1622',      # 47nF X7R 0603 Samsung (Basic)
    'C36': 'C1622',      # 47nF X7R 0603 Samsung (Basic)
    'C37': 'C1622',      # 47nF X7R 0603 Samsung (Basic)
    'C38': 'C14858',     # 100pF C0G 0603 Samsung
    'C39': 'C14858',     # 100pF C0G 0603 Samsung
    'C40': 'C1622',      # 47nF X7R 0603 Samsung (Basic)
    'C41': 'C57112',     # 10nF X7R 0603 FH
    'C42': 'C1622',      # 47nF X7R 0603 Samsung (Basic)
    'C43': 'C14858',     # 100pF C0G 0603 Samsung
    'C44': 'C695130',    # 1.5pF C0G 0603 TDK
    'C45': 'C1644',      # 15pF C0G 0603 Samsung (Basic)
    'C46': 'C1644',      # 15pF C0G 0603 Samsung (Basic)
    'C47': 'C14858',     # 100pF C0G 0603 Samsung
    'R1':  'C25804',     # 10k 0603 UNI-ROYAL (Basic)
    'R2':  'C25804',     # 10k 0603 UNI-ROYAL (Basic)
    'R3':  'C23206',     # 56k 0603 UNI-ROYAL
    'R8':  'C23345',     # 22R 0603 UNI-ROYAL (Basic)

    # === Pi Header (Pi_Header.kicad_sch) ===
    'J3':  'C50980',     # 2x20 pin header 2.54mm

    # === Pico (Pico_SPI_Host.kicad_sch) ===
    'U3':  '',           # Raspberry Pi Pico — hand solder, buy separately
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
    insertions = []  # list of (line_index, property_text, ref)

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

                # Check for existing LCSC property (skip if already present)
                if '"LCSC"' in l:
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

            if ref and ref in LCSC and insert_before is not None:
                code = LCSC[ref]
                if code:  # skip empty codes
                    prop = (
                        f'\t\t(property "LCSC" "{code}"\n'
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
        print(f"{fname}: added LCSC codes to {len(refs)} components: {', '.join(refs)}")

    print(f"\nTotal: {total} LCSC codes added")
    print("\nNot on LCSC (hand-solder):")
    print("  J1, J2 (SMA edge-mount) — hand solder")
    print("  U3 (Raspberry Pi Pico) — buy separately, hand solder")
    print("  H1-H4 (mounting holes) — no component needed")
    print("\nSubstitutions:")
    print("  L2 (43nH) -> 47nH Sunlord SDCL1608C47NJTDF (C29683)")
    print("  L1,L5 (15nH) -> Sunlord SDCL1608C15NJTDF (C23651)")
    print("  L4,L7 (56nH) -> Sunlord SDCL1608C56NJTDF (C12831)")
