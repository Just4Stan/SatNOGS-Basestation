#!/usr/bin/env python3
"""
VHF Packet Transmitter — sends packets on the CC1200 VHF radio (145 MHz).

This is the uplink path for commanding the AetherSpace CubeSat at 145.9 MHz.
Connects to the RF HAT Pico over /dev/serial0 (Pi UART), loads the VHF SmartRF
profile, and transmits one or more packets.

Usage:
    python3 transmit.py "48 65 6C 6C 6F"             # hex bytes
    python3 transmit.py --ascii "Hello"               # ASCII string
    python3 transmit.py --file cmd.bin                 # binary file
    python3 transmit.py --freq 145.9 "DE ON4SAT"      # custom freq (MHz)
    python3 transmit.py --repeat 5 --interval 2 "AA"  # repeat 5x, 2s apart
"""

import sys
import time
import signal
import argparse
import datetime
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
from rf_hat import (
    CC1200Link, RADIO_VHF, STATE_TX, STATE_IDLE,
    freq_to_regs,
)

DEFAULT_PORT = "/dev/serial0"
DEFAULT_FREQ_MHZ = 145.9
DEFAULT_PROFILE = str(Path(__file__).parent / "configs" / "smartrf_vhf_145.txt")


def timestamp() -> str:
    return datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]


def parse_payload(args) -> bytes:
    """Parse payload from CLI arguments."""
    if args.file:
        with open(args.file, "rb") as f:
            return f.read()
    if args.ascii:
        return args.payload.encode("utf-8") if args.payload else b""
    # Default: hex string like "48 65 6C 6C 6F" or "48656C6C6F"
    if args.payload:
        cleaned = args.payload.replace(" ", "").replace(":", "").replace("-", "")
        return bytes.fromhex(cleaned)
    return b""


def main():
    parser = argparse.ArgumentParser(description="CC1200 VHF packet transmitter")
    parser.add_argument("payload", nargs="?", default="",
                        help="Hex bytes (e.g. '48 65 6C 6C 6F') or ASCII with --ascii")
    parser.add_argument("--port", default=DEFAULT_PORT, help="Serial port")
    parser.add_argument("--freq", type=float, default=DEFAULT_FREQ_MHZ,
                        help="TX frequency in MHz (default: 145.9)")
    parser.add_argument("--profile", default=DEFAULT_PROFILE,
                        help="SmartRF config file path")
    parser.add_argument("--no-profile", action="store_true",
                        help="Skip profile loading (assume already configured)")
    parser.add_argument("--ascii", action="store_true",
                        help="Treat payload as ASCII string instead of hex")
    parser.add_argument("--file", default=None,
                        help="Read payload from a binary file")
    parser.add_argument("--repeat", type=int, default=1,
                        help="Number of times to transmit (default: 1)")
    parser.add_argument("--interval", type=float, default=1.0,
                        help="Seconds between repeated transmissions (default: 1.0)")
    parser.add_argument("--power", type=int, default=None,
                        help="PA_CFG1 value (0x00-0x7F, higher = more power)")
    args = parser.parse_args()

    data = parse_payload(args)
    if not data:
        print("ERROR: No payload specified. Use hex string, --ascii, or --file.")
        sys.exit(1)

    if len(data) > 128:
        print(f"WARNING: Payload is {len(data)} bytes — CC1200 TX FIFO is 128 bytes.")
        print("         Only the first 128 bytes will be sent.")
        data = data[:128]

    def log(msg: str):
        print(f"[{timestamp()}] {msg}")

    # Graceful shutdown
    running = True
    def on_signal(sig, frame):
        nonlocal running
        running = False
    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)

    link = CC1200Link()

    log(f"Opening {args.port}...")
    try:
        link.open(args.port)
    except Exception as e:
        log(f"FATAL: Cannot open {args.port}: {e}")
        sys.exit(1)

    time.sleep(0.1)

    # Ping
    pong = link.ping()
    if pong is None:
        log("FATAL: No response to PING — check UART wiring")
        link.close()
        sys.exit(1)
    log(f"PING OK: {pong!r}")

    # Get info
    info = link.get_info()
    if info:
        log(f"Radio: active={info.active} count={info.count} "
            f"part=0x{info.part:02X} ver=0x{info.ver:02X}")
    else:
        log("WARNING: GET_INFO failed")

    # Select VHF radio
    if not link.select_radio(RADIO_VHF):
        log("FATAL: Failed to select VHF radio")
        link.close()
        sys.exit(1)
    log("Selected VHF radio (index 1)")

    # Load SmartRF profile
    if not args.no_profile:
        log(f"Loading SmartRF profile: {args.profile}")
        ok, msg = link.apply_smartrf_config(args.profile)
        if not ok:
            log(f"FATAL: Profile apply failed: {msg}")
            link.close()
            sys.exit(1)
        log(f"Profile: {msg}")

    # Override frequency
    freq_hz = args.freq * 1e6
    log(f"Setting frequency: {args.freq:.3f} MHz")
    if not link.set_frequency(freq_hz):
        log("WARNING: Failed to set frequency registers")

    # Verify frequency
    actual = link.get_frequency()
    if actual:
        log(f"Frequency readback: {actual/1e6:.6f} MHz")

    # Override PA power if requested
    if args.power is not None:
        pa_val = args.power & 0x7F
        if link.write_reg(0x2B, pa_val):  # PA_CFG1 is reg 0x2B
            log(f"PA_CFG1 set to 0x{pa_val:02X}")
        else:
            log("WARNING: Failed to set PA_CFG1")

    # Transmit
    log(f"Payload ({len(data)} bytes): {data.hex(' ')}")
    log(f"Transmitting {args.repeat}x with {args.interval}s interval...")

    try:
        for i in range(args.repeat):
            if not running:
                break

            ok = link.tx_packet(data)
            if ok:
                log(f"TX [{i+1}/{args.repeat}] OK — {len(data)} bytes sent")
            else:
                log(f"TX [{i+1}/{args.repeat}] FAILED")

            if i < args.repeat - 1 and running:
                time.sleep(args.interval)

    finally:
        log("Returning to IDLE")
        link.set_state(STATE_IDLE)
        link.close()


if __name__ == "__main__":
    main()
