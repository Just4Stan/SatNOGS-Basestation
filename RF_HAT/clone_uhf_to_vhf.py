#!/usr/bin/env python3
"""
Clone the hand-arranged UHF CC1200 schematic to VHF,
changing only references, values, and net names.

Usage:
    python3 clone_uhf_to_vhf.py
"""

import re
import uuid as uuid_mod

UHF_PATH = "RF_HAT_out/CC1200_UHF_432MHz.kicad_sch"
VHF_PATH = "RF_HAT_out/CC1200_VHF_144MHz.kicad_sch"

# Root and sheet UUIDs (from root schematic)
ROOT_UUID = "300cf3d4-e317-44c8-acd8-5f9989779c7c"
UHF_SHEET_UUID = "4304a2bc-ba15-4549-9abd-1aeb0fc86918"
VHF_SHEET_UUID = "b5374fb6-6911-4d97-b6ed-89df84753480"

# Reference offsets by prefix
REF_OFFSETS = {
    'U': 1,
    'L': 8,
    'C': 27,
    'R': 7,
    'J': 1,
    'Y': 1,
    'FB': 1,
    'TP': 4,
}

# Value changes: UHF value → VHF value (only for components that differ)
VALUE_CHANGES = {
    # TX matching inductors
    '15nH': None,   # L1 and L5 both 15nH but need different VHF values - handle by ref
    '43nH': '120nH',  # L2
    '22nH': None,      # L3 - also ambiguous, handle by ref
    '56nH': None,      # L4, L7 - ambiguous, handle by ref
    '27nH': '100nH',   # L6, L8
    # Caps
    '2.2pF': '1.5pF',  # C1
    '39pF': '100pF',    # C2
    '5.1pF': '15pF',    # C3, C8, C9
    '6.2pF': '15pF',    # C4
    '56pF': '100pF',    # C5
    '1nF': '100pF',     # C10
    # PA bias
    '18R': '22R',       # R6
    # PLL
    '1.5nF': '1.8nF',   # C18
}

# Per-reference value overrides (UHF ref → VHF value)
# This handles ambiguous cases where multiple UHF components share a value
REF_VALUE_MAP = {
    'L1': '22nH',     # TX match 1
    'L2': '120nH',    # TX match 2
    'L3': '82nH',     # TX match 3
    'L4': '270nH',    # PA RF choke
    'L5': '47nH',     # TRX coupling
    'L6': '100nH',    # LNA_P bias
    'L7': '180nH',    # LNA bridge
    'L8': '100nH',    # LNA_N to balun
    'C1': '1.5pF',    # feedback
    'C2': '100pF',    # series
    'C3': '15pF',     # TRX coupling
    'C4': '15pF',     # shunt
    'C5': '100pF',    # PA bias bypass
    'C6': '10nF',     # same
    'C7': '100pF',    # same
    'C8': '15pF',     # LNA_P coupling
    'C9': '15pF',     # LNA_N shunt
    'C10': '100pF',   # DC block
    'C18': '1.8nF',   # PLL filter
    'R6': '22R',      # PA bias
}


def offset_ref(ref):
    """Convert UHF reference to VHF reference."""
    m = re.match(r'^([A-Z]+)(\d+)$', ref)
    if not m:
        return ref
    prefix = m.group(1)
    num = int(m.group(2))
    offset = REF_OFFSETS.get(prefix, 0)
    return f"{prefix}{num + offset}"


def get_vhf_value(uhf_ref, uhf_value):
    """Get VHF value for a component, given its UHF ref and value."""
    if uhf_ref in REF_VALUE_MAP:
        return REF_VALUE_MAP[uhf_ref]
    return uhf_value  # Keep same value (bypass caps, crystal, etc.)


def main():
    with open(UHF_PATH) as f:
        content = f.read()

    # Step 1: Replace all UUIDs with new ones
    uuid_map = {}
    def replace_uuid(m):
        old = m.group(1)
        if old not in uuid_map:
            uuid_map[old] = str(uuid_mod.uuid4())
        return f'"{uuid_map[old]}"'

    content = re.sub(r'"([0-9a-f]{8}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{12})"',
                     replace_uuid, content)

    # Step 2: Fix sheet hierarchy paths (UHF sheet UUID → VHF sheet UUID)
    # The instance paths contain the root UUID and sheet UUID
    content = content.replace(uuid_map.get(UHF_SHEET_UUID, UHF_SHEET_UUID),
                              VHF_SHEET_UUID)
    # Also replace any remaining references to the UHF sheet UUID
    content = content.replace(UHF_SHEET_UUID, VHF_SHEET_UUID)

    # Make sure root UUID is correct
    if ROOT_UUID in uuid_map:
        content = content.replace(uuid_map[ROOT_UUID], ROOT_UUID)

    # Step 3: Replace hierarchical label names (UHF_* → VHF_*)
    content = re.sub(r'(hierarchical_label\s+")UHF_', r'\1VHF_', content)

    # Step 4: Replace component references and values
    # Find each symbol block and update ref + value
    result = []
    idx = 0
    ref_pattern = re.compile(r'(\(property\s+"Reference"\s+")([A-Z]+\d+)(")')
    val_pattern = re.compile(r'(\(property\s+"Value"\s+")([^"]+)(")')
    instance_ref_pattern = re.compile(r'(\(reference\s+")([A-Z]+\d+)(")')

    # Process symbol blocks
    sym_start = 0
    while True:
        pos = content.find('\t(symbol\n\t\t(lib_id', sym_start)
        if pos == -1:
            break
        # Find end of this symbol block
        depth = 0
        end = pos
        for i in range(pos, len(content)):
            if content[i] == '(':
                depth += 1
            elif content[i] == ')':
                depth -= 1
                if depth == 0:
                    end = i + 1
                    break
        block = content[pos:end]

        # Extract current reference
        ref_m = re.search(r'\(property\s+"Reference"\s+"([A-Z]+\d+)"', block)
        if ref_m:
            uhf_ref = ref_m.group(1)
            vhf_ref = offset_ref(uhf_ref)

            # Get current value
            val_m = re.search(r'\(property\s+"Value"\s+"([^"]+)"', block)
            uhf_val = val_m.group(1) if val_m else ""
            vhf_val = get_vhf_value(uhf_ref, uhf_val)

            # Replace reference everywhere in block
            block = ref_pattern.sub(
                lambda m: m.group(1) + offset_ref(m.group(2)) + m.group(3), block)
            block = instance_ref_pattern.sub(
                lambda m: m.group(1) + offset_ref(m.group(2)) + m.group(3), block)

            # Replace value
            if vhf_val != uhf_val:
                block = val_pattern.sub(
                    lambda m: m.group(1) + vhf_val + m.group(3), block, count=1)

        result.append(content[sym_start:pos])
        result.append(block)
        sym_start = end

    result.append(content[sym_start:])
    content = ''.join(result)

    # Step 5: Replace remaining UHF refs in instance blocks outside symbols
    # (e.g., in (instances ...) at the end of file)
    content = instance_ref_pattern.sub(
        lambda m: m.group(1) + offset_ref(m.group(2)) + m.group(3), content)

    # Step 6: Update sheet title
    content = content.replace('"CC1200_UHF_432MHz"', '"CC1200_VHF_144MHz"')

    # Step 7: Replace any remaining UHF_ net references in labels, wires, etc.
    # (net labels, local labels)
    content = re.sub(r'(\(label\s+")UHF_', r'\1VHF_', content)
    content = re.sub(r'(\(global_label\s+")UHF_', r'\1VHF_', content)
    # Also in net_name properties
    content = content.replace('"UHF_', '"VHF_')

    # But don't change the sheet filename in references
    content = content.replace('VHF_432MHz', 'UHF_432MHz')  # Undo accidental change

    with open(VHF_PATH, 'w') as f:
        f.write(content)

    print(f"Cloned {UHF_PATH} → {VHF_PATH}")
    print(f"Replaced {len(uuid_map)} UUIDs")

    # Print value changes for verification
    print("\nValue changes applied:")
    for uhf_ref, vhf_val in sorted(REF_VALUE_MAP.items()):
        vhf_ref = offset_ref(uhf_ref)
        print(f"  {uhf_ref} → {vhf_ref}: {vhf_val}")


if __name__ == "__main__":
    main()
