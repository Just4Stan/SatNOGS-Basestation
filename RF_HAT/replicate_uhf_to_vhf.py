#!/usr/bin/env python3
"""Replicate UHF CC1200 component placement to VHF CC1200 on the RF HAT PCB.

Reads UHF instance 1 footprint positions/orientations from the PCB file and
applies the same relative layout to VHF components. The mapping was derived
automatically from the KiCad netlist by tracing CC1200 pin connections.

Workflow:
  1. In KiCad schematic editor:
     - Delete the "CC1200_UHF_432MHz1" sub-sheet (the duplicated UHF page)
  2. Tools → Update PCB from Schematic (adds VHF footprints, removes UHF inst.2)
  3. Save the PCB  (File → Save)
  4. Close KiCad   (avoids file conflicts)
  5. Run:  python3 replicate_uhf_to_vhf.py
  6. Reopen the PCB in KiCad — VHF components are now placed

Optional arguments:
  --offset-x X   Horizontal offset from UHF anchor U1 (default: -13.65)
  --offset-y Y   Vertical   offset from UHF anchor U1 (default: -0.26)
  --pcb PATH     Path to the PCB file
  --dry-run      Print what would change without writing
"""

import argparse
import re
import sys
import shutil
from pathlib import Path

# ──────────────────────────────────────────────────────────────────────
# UHF → VHF component mapping  (traced from KiCad netlist net names)
# ──────────────────────────────────────────────────────────────────────
UHF_TO_VHF = {
    # IC / connectors / crystal
    "U1":  "U2",
    "Y1":  "Y2",
    "J1":  "J2",
    # Resistors
    "R4":  "R1",   # CSN pull-up 10k
    "R5":  "R2",   # RESET pull-up 10k
    "R6":  "R8",   # PA feed resistor
    "R7":  "R3",   # RBIAS 56k
    # Inductors — TX matching
    "L1":  "L14",  # TX match L (parallel C_feedback)
    "L2":  "L15",  # TX match L (series)
    "L3":  "L16",  # TX match L (toward antenna)
    "L4":  "L13",  # PA supply choke
    # Inductors — RX / LNA
    "L5":  "L11",  # TRX coupling (balun → TRX_SW)
    "L6":  "L9",   # LNA_P bias shunt to GND
    "L7":  "L10",  # LNA bridge P ↔ N
    "L8":  "L12",  # LNA_N → balun center
    # Capacitors — TX matching
    "C1":  "C44",  # TX feedback (across L1)
    "C2":  "C39",  # TX series from PA
    "C3":  "C45",  # TRX coupling cap
    "C4":  "C46",  # TX shunt to GND
    "C5":  "C38",  # PA bias bypass
    "C10": "C47",  # DC block before SMA
    # Capacitors — RX / LNA
    "C8":  "C27",  # LNA_P coupling
    "C9":  "C32",  # LNA_N shunt to GND
    # Crystal load caps
    "C12": "C11",  # XOSC_Q1 load
    "C13": "C20",  # XOSC_Q2 load
    # Decoupling on dedicated CC1200 pins
    "C14": "C28",  # DCPL_PFD_CHP (pin 26)
    "C15": "C25",  # DCPL_XOSC   (pin 29)
    "C16": "C31",  # DCPL_VCO    (pin 21)
    "C17": "C34",  # DCPL         (pin 6)
    # PLL loop filter
    "C18": "C23",  # LPF0 ↔ LPF1
    # Power bypass — unique values
    "C6":  "C41",  # 10 nF bypass
    "C7":  "C43",  # 100 pF bypass
    # Power bypass — 47 nF bank (all electrically identical)
    "C21": "C26",
    "C57": "C29",
    "C58": "C30",
    "C59": "C33",
    "C60": "C35",
    "C61": "C36",
    "C62": "C37",
    "C63": "C40",
    "C64": "C42",
}

# ──────────────────────────────────────────────────────────────────────
# PCB footprint parser (minimal, works on KiCad 9 s-expression format)
# ──────────────────────────────────────────────────────────────────────

def find_footprint_blocks(pcb_text):
    """Yield (ref, start, end, at_match) for every top-level footprint."""
    # Footprints start with "(footprint " at top level (single tab indent)
    pattern = re.compile(r'^\t\(footprint ', re.MULTILINE)
    for m in pattern.finditer(pcb_text):
        start = m.start()
        # Walk forward counting parens to find the matching close
        depth = 0
        i = m.start()
        while i < len(pcb_text):
            if pcb_text[i] == '(':
                depth += 1
            elif pcb_text[i] == ')':
                depth -= 1
                if depth == 0:
                    end = i + 1
                    break
            i += 1
        else:
            continue

        block = pcb_text[start:end]

        # Extract reference — look for (property "Reference" "XX" ...)
        ref_m = re.search(r'\(property\s+"Reference"\s+"([^"]+)"', block)
        if not ref_m:
            continue
        ref = ref_m.group(1)

        # Extract (at X Y [angle]) — the FIRST (at ...) in the block is the footprint position
        at_m = re.search(r'\(at\s+([-\d.]+)\s+([-\d.]+)(?:\s+([-\d.]+))?\)', block)
        if not at_m:
            continue

        yield ref, start, end, at_m


def get_footprint_positions(pcb_text, refs):
    """Return {ref: (x, y, angle)} for requested references."""
    positions = {}
    for ref, _start, _end, at_m in find_footprint_blocks(pcb_text):
        if ref in refs:
            x = float(at_m.group(1))
            y = float(at_m.group(2))
            angle = float(at_m.group(3)) if at_m.group(3) else 0.0
            positions[ref] = (x, y, angle)
    return positions


def set_footprint_position(pcb_text, ref, new_x, new_y, new_angle):
    """Return pcb_text with the given footprint's (at ...) updated."""
    for fp_ref, start, end, at_m in find_footprint_blocks(pcb_text):
        if fp_ref != ref:
            continue

        block = pcb_text[start:end]
        # The at_match offsets are relative to the whole pcb_text — we need
        # the offset relative to the block start for replacement.
        at_abs_start = start + at_m.start()
        at_abs_end   = start + at_m.end()
        old_at = pcb_text[at_abs_start:at_abs_end]

        if new_angle == 0.0:
            new_at = f"(at {new_x:.6g} {new_y:.6g})"
        else:
            new_at = f"(at {new_x:.6g} {new_y:.6g} {new_angle:.6g})"

        return pcb_text[:at_abs_start] + new_at + pcb_text[at_abs_end:]

    print(f"  WARNING: footprint '{ref}' not found in PCB — skipped")
    return pcb_text


# ──────────────────────────────────────────────────────────────────────

def main():
    default_pcb = str(Path(__file__).parent / "RF_HAT_out" / "RF_HAT.kicad_pcb")

    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--pcb", default=default_pcb, help="Path to the PCB file")
    parser.add_argument("--offset-x", type=float, default=-13.65,
                        help="X offset from UHF anchor (default: -13.65, matches UHF inst.2)")
    parser.add_argument("--offset-y", type=float, default=-0.26,
                        help="Y offset from UHF anchor (default: -0.26)")
    parser.add_argument("--dry-run", action="store_true",
                        help="Print changes without writing the file")
    args = parser.parse_args()

    pcb_path = Path(args.pcb)
    if not pcb_path.exists():
        sys.exit(f"Error: PCB file not found: {pcb_path}")

    pcb_text = pcb_path.read_text()

    # 1. Read UHF instance 1 positions
    uhf_refs = set(UHF_TO_VHF.keys())
    uhf_pos = get_footprint_positions(pcb_text, uhf_refs)

    missing_uhf = uhf_refs - set(uhf_pos.keys())
    if missing_uhf:
        print(f"WARNING: UHF components not found in PCB: {sorted(missing_uhf)}")

    anchor = uhf_pos.get("U1")
    if not anchor:
        sys.exit("Error: UHF anchor U1 not found in PCB")

    ax, ay, _ = anchor
    print(f"UHF anchor U1 at ({ax}, {ay})")
    print(f"VHF target offset: ({args.offset_x}, {args.offset_y})")
    print(f"VHF anchor U2 will be at ({ax + args.offset_x:.4f}, {ay + args.offset_y:.4f})")
    print()

    # 2. Check which VHF components exist in the PCB
    vhf_refs = set(UHF_TO_VHF.values())
    vhf_pos = get_footprint_positions(pcb_text, vhf_refs)
    missing_vhf = vhf_refs - set(vhf_pos.keys())
    if missing_vhf:
        print(f"WARNING: VHF components not yet in PCB: {sorted(missing_vhf)}")
        print("  → Did you run 'Update PCB from Schematic' in KiCad first?")
        if len(missing_vhf) == len(vhf_refs):
            sys.exit("Error: No VHF components found. Run Update PCB from Schematic first.")
        print()

    # 3. Compute and apply new positions
    print(f"{'UHF':>5} → {'VHF':<5}  {'Function':<30}  {'New position'}")
    print("-" * 75)

    functions = {
        "U1": "CC1200 IC", "Y1": "40 MHz crystal", "J1": "SMA connector",
        "R4": "CSN pull-up", "R5": "RESET pull-up", "R6": "PA feed R", "R7": "RBIAS",
        "L1": "TX match L1", "L2": "TX match L2", "L3": "TX match L3",
        "L4": "PA choke", "L5": "TRX coupling L", "L6": "LNA bias shunt",
        "L7": "LNA bridge P-N", "L8": "LNA N→balun",
        "C1": "TX feedback C", "C2": "TX series C", "C3": "TRX coupling C",
        "C4": "TX shunt C", "C5": "PA bias bypass", "C6": "bypass 10nF",
        "C7": "bypass 100pF", "C8": "LNA_P coupling", "C9": "LNA_N shunt",
        "C10": "DC block", "C12": "XOSC Q1 load", "C13": "XOSC Q2 load",
        "C14": "DCPL PFD", "C15": "DCPL XOSC", "C16": "DCPL VCO",
        "C17": "DCPL (pin 6)", "C18": "PLL loop filter",
        "C21": "bypass 47nF", "C57": "bypass 47nF", "C58": "bypass 47nF",
        "C59": "bypass 47nF", "C60": "bypass 47nF", "C61": "bypass 47nF",
        "C62": "bypass 47nF", "C63": "bypass 47nF", "C64": "bypass 47nF",
    }

    changes = 0
    for uhf_ref in sorted(UHF_TO_VHF.keys(),
                          key=lambda r: (r[0], int(re.search(r'\d+', r).group()))):
        vhf_ref = UHF_TO_VHF[uhf_ref]

        if uhf_ref not in uhf_pos:
            continue
        if vhf_ref not in vhf_pos:
            continue

        ux, uy, uangle = uhf_pos[uhf_ref]
        # Relative offset from UHF anchor
        dx = ux - ax
        dy = uy - ay
        # New VHF position
        new_x = ax + args.offset_x + dx
        new_y = ay + args.offset_y + dy
        new_angle = uangle

        func = functions.get(uhf_ref, "bypass 47nF")
        print(f"{uhf_ref:>5} → {vhf_ref:<5}  {func:<30}  ({new_x:.4f}, {new_y:.4f}, {new_angle}°)")

        if not args.dry_run:
            pcb_text = set_footprint_position(pcb_text, vhf_ref, new_x, new_y, new_angle)
        changes += 1

    print(f"\n{changes} components {'would be ' if args.dry_run else ''}repositioned.")

    if not args.dry_run and changes > 0:
        # Backup
        backup = pcb_path.with_suffix(".kicad_pcb.bak")
        shutil.copy2(pcb_path, backup)
        print(f"Backup saved to {backup}")

        pcb_path.write_text(pcb_text)
        print(f"PCB written to {pcb_path}")
        print("\nOpen in KiCad to verify placement. VHF components should mirror UHF layout.")


if __name__ == "__main__":
    main()
