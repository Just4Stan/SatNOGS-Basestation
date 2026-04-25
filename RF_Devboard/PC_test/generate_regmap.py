import re
import json
from pathlib import Path

HEADER_FILENAME = "cc120x_spi.h"

OUT_ALL = "cc1200_regmap_all.json"
OUT_PROFILE = "cc1200_regmap_profile.json"

DEFINE_RE = re.compile(
    r'^\s*#define\s+CC120X_([A-Z0-9_]+)\s+(0x[0-9A-Fa-f]+)\b'
)

def is_register_address(addr: int) -> bool:
    # Normal regs
    if 0x00 <= addr <= 0xFF:
        return True
    # Extended regs (TI encoding): 0x2Fxx
    if 0x2F00 <= addr <= 0x2FFF:
        return True
    return False

# Remove obvious non-profile stuff (status/test/AES/etc.)
IGNORE_PREFIXES = (
    "RSSI", "MARC", "LQI", "PQT_", "DEM_", "FREQOFF_EST", "AGC_GAIN",
    "CFM_", "RNDGEN", "MAGN", "ANG", "CHFILT_", "GPIO_STATUS",
    "MODEM_STATUS", "MARC_STATUS",
    "RXFIRST", "TXFIRST", "RXLAST", "TXLAST", "NUM_TXBYTES", "NUM_RXBYTES",
    "FIFO_NUM_", "RXFIFO_",
    "ATEST", "ADC_TEST", "DVC_TEST", "FSRF_TEST", "PRE_TEST", "PRE_OVR",
    "XOSC_TEST", "MDM_TEST",
    "AES", "AES_KEY", "AES_BUFFER",
    "PARTNUMBER", "PARTVERSION", "SERIAL_STATUS"
)

IGNORE_EXACT = {
    "SRES", "SFSTXON", "SXOFF", "SCAL", "SRX", "STX", "SIDLE",
    "SWOR", "SPWD", "SFRX", "SFTX", "SWORRST", "SNOP",
    "SINGLE_TXFIFO", "BURST_TXFIFO", "SINGLE_RXFIFO", "BURST_RXFIFO",
    "STATE_IDLE", "STATE_RX", "STATE_TX", "STATE_FSTXON",
    "STATE_CALIBRATE", "STATE_SETTLING",
    "STATE_RXFIFO_ERROR", "STATE_TXFIFO_ERROR",
    "SAFC",
    "ASK_SOFT_RX_DATA",
    "PA_IFAMP_TEST",
    "BIST",
    "IRQ0F",
    "IRQ0M",
}


def is_profile_register(name: str) -> bool:
    if name in IGNORE_EXACT:
        return False
    if any(name.startswith(p) for p in IGNORE_PREFIXES):
        return False
    return True

def write_json(path: Path, data: dict):
    with open(path, "w", encoding="utf-8") as f:
        json.dump(dict(sorted(data.items())), f, indent=4)

def generate():
    script_dir = Path(__file__).parent
    header_path = script_dir / HEADER_FILENAME

    if not header_path.exists():
        print(f"ERROR: {HEADER_FILENAME} not found in {script_dir}")
        return

    reg_all = {}
    reg_profile = {}

    skipped_nonreg = 0
    skipped_profile = 0

    with open(header_path, "r", encoding="utf-8", errors="ignore") as f:
        for line in f:
            m = DEFINE_RE.match(line)
            if not m:
                continue

            name = m.group(1)
            addr = int(m.group(2), 16)

            if not is_register_address(addr):
                skipped_nonreg += 1
                continue

            reg_all[name] = addr

            if is_profile_register(name):
                reg_profile[name] = addr
            else:
                skipped_profile += 1

    out_all_path = script_dir / OUT_ALL
    out_profile_path = script_dir / OUT_PROFILE

    write_json(out_all_path, reg_all)
    write_json(out_profile_path, reg_profile)

    print("SUCCESS:")
    print(f"  all     : {len(reg_all)} entries → {out_all_path}")
    print(f"  profile : {len(reg_profile)} entries → {out_profile_path}")
    print("Skipped:")
    print(f"  non-register defines (masks/etc.)          : {skipped_nonreg}")
    print(f"  register-ish but not profile-applicable    : {skipped_profile}")

if __name__ == "__main__":
    generate()
