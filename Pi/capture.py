#!/usr/bin/env python3
"""
UHF Packet Capture — receives packets on the CC1200 UHF radio (432 MHz).

This is the primary receive path for the AetherSpace CubeSat downlink at 433 MHz.
Connects to the RF HAT Pico over /dev/serial0 (Pi UART), loads the SmartRF profile,
enables RX streaming, and logs all received packets.

Usage:
    python3 capture.py                                  # defaults (433 MHz)
    python3 capture.py --freq 435.0                     # specific frequency
    python3 capture.py --log packets.log                # log to file
    python3 capture.py --port /dev/ttyUSB0              # alternate serial port
    python3 capture.py --profile configs/smartrf_uhf_435.txt  # custom profile
"""

import sys
import time
import signal
import argparse
import datetime
from pathlib import Path

# Add parent dir to path for imports when running from Pi/ directory
sys.path.insert(0, str(Path(__file__).parent))
from rf_hat import (
    CC1200Link, RADIO_UHF, STATE_RX, STATE_IDLE,
    EVT_RX_DATA, EVT_ERROR, freq_to_regs,
)

DEFAULT_PORT = "/dev/serial0"
DEFAULT_FREQ_MHZ = 433.0
DEFAULT_PROFILE = str(Path(__file__).parent / "configs" / "smartrf_uhf_435.txt")
METRICS_INTERVAL_S = 5.0


def timestamp() -> str:
    return datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]


def main():
    parser = argparse.ArgumentParser(description="CC1200 UHF packet capture")
    parser.add_argument("--port", default=DEFAULT_PORT, help="Serial port")
    parser.add_argument("--freq", type=float, default=DEFAULT_FREQ_MHZ,
                        help="RX frequency in MHz (default: 433.0)")
    parser.add_argument("--profile", default=DEFAULT_PROFILE,
                        help="SmartRF config file path")
    parser.add_argument("--log", default=None,
                        help="Log file path (packets are appended)")
    parser.add_argument("--duration", type=float, default=0,
                        help="Capture duration in seconds (0 = indefinite)")
    parser.add_argument("--no-profile", action="store_true",
                        help="Skip profile loading (assume already configured)")
    args = parser.parse_args()

    logfile = None
    if args.log:
        logfile = open(args.log, "a", encoding="utf-8")

    def log(msg: str):
        line = f"[{timestamp()}] {msg}"
        print(line)
        if logfile:
            logfile.write(line + "\n")
            logfile.flush()

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

    # Select UHF radio
    if not link.select_radio(RADIO_UHF):
        log("FATAL: Failed to select UHF radio")
        link.close()
        sys.exit(1)
    log("Selected UHF radio (index 0)")

    # Load SmartRF profile
    if not args.no_profile:
        log(f"Loading SmartRF profile: {args.profile}")
        ok, msg = link.apply_smartrf_config(args.profile)
        if not ok:
            log(f"FATAL: Profile apply failed: {msg}")
            link.close()
            sys.exit(1)
        log(f"Profile: {msg}")

    # Override frequency if different from profile default
    freq_hz = args.freq * 1e6
    log(f"Setting frequency: {args.freq:.3f} MHz")
    if not link.set_frequency(freq_hz):
        log("WARNING: Failed to set frequency registers")

    # Verify frequency
    actual = link.get_frequency()
    if actual:
        log(f"Frequency readback: {actual/1e6:.6f} MHz")

    # Enter RX
    if not link.set_state(STATE_RX):
        log("WARNING: Failed to enter RX state")
    log("Entering RX mode")

    # Enable streaming
    if not link.set_streaming(True):
        log("WARNING: Failed to enable streaming")
    log("RX streaming enabled — waiting for packets...")

    # Capture loop
    start_time = time.time()
    last_metrics = time.time()
    pkt_count = 0
    total_bytes = 0

    try:
        while running:
            if args.duration > 0 and (time.time() - start_time) >= args.duration:
                log(f"Duration {args.duration}s reached")
                break

            # Process events
            for evt_type, body in link.pop_events():
                if evt_type == EVT_RX_DATA and body:
                    n = body[0]
                    data = body[1:1 + n]
                    pkt_count += 1
                    total_bytes += n
                    hex_str = data.hex(" ")
                    log(f"RX [{pkt_count}] {n} bytes: {hex_str}")

                elif evt_type == EVT_ERROR and body:
                    code = body[0]
                    log(f"EVT_ERROR code=0x{code:02X}")

            # Periodic metrics
            now = time.time()
            if now - last_metrics >= METRICS_INTERVAL_S:
                last_metrics = now
                m = link.get_metrics()
                if m:
                    rssi_str = f"{m.rssi_dbm:.1f} dBm" if m.rssi_dbm is not None else "N/A"
                    elapsed = now - start_time
                    log(f"METRICS: MARC=0x{m.marc:02X} RXBYTES={m.rxbytes} "
                        f"RSSI={rssi_str} | "
                        f"total: {pkt_count} pkts, {total_bytes} bytes, "
                        f"{elapsed:.0f}s elapsed")

            time.sleep(0.01)

    finally:
        log(f"Stopping capture: {pkt_count} packets, {total_bytes} bytes")
        link.set_streaming(False)
        link.set_state(STATE_IDLE)
        link.close()
        if logfile:
            logfile.close()


if __name__ == "__main__":
    main()
