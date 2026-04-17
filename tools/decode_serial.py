#!/usr/bin/env python3
"""
Decode oversampled CC1200 serial mode captures.

The CC1200 in blind mode (SYNC_MODE=0) outputs at the internal demodulator
sample rate (~8x symbol rate for 9600 bps, ~16x for 4800 bps). This script:
1. Auto-detects the oversampling factor from run-length statistics
2. Decimates to symbol rate using majority voting
3. Runs NRZ-I decoding
4. Runs G3RUH descrambling (17-bit LFSR, taps 12+17)
5. Searches for HDLC 0x7E flags and extracts AX.25 frames
6. Validates CRC-16/CCITT

Usage:
    python3 decode_serial.py <file.raw> [--baud 9600] [--oversample auto]
"""

import sys
import os
import struct
import argparse
from collections import Counter


def bytes_to_bits(data: bytes) -> list:
    """Convert byte array to list of bits (MSB first)."""
    bits = []
    for b in data:
        for i in range(7, -1, -1):
            bits.append((b >> i) & 1)
    return bits


def detect_oversample(bits: list, expected_baud: int = 9600) -> int:
    """Auto-detect oversampling factor from run-length distribution."""
    # Measure average run length in first 50k bits
    sample = bits[:min(50000, len(bits))]
    if len(sample) < 100:
        return 8  # default

    runs = []
    current = sample[0]
    run = 1
    for b in sample[1:]:
        if b == current:
            run += 1
        else:
            runs.append(run)
            current = b
            run = 1
    runs.append(run)

    if not runs:
        return 8

    # Modal run length ≈ oversampling factor
    rc = Counter(runs)
    # Find the peak in the 2-20 range
    peak = max(range(2, min(21, max(rc.keys()) + 1)),
               key=lambda x: rc.get(x, 0), default=8)

    # Also check average
    avg = sum(runs) / len(runs)

    # Use the nearest power-of-2-ish value
    candidates = [4, 6, 7, 8, 10, 12, 14, 15, 16]
    best = min(candidates, key=lambda c: abs(c - avg))

    print(f"  Run length stats: avg={avg:.1f}, mode peak={peak}")
    print(f"  Auto-detected oversample factor: {best}")
    return best


def decimate(bits: list, factor: int) -> list:
    """Decimate bit stream by majority voting over windows."""
    out = []
    for i in range(0, len(bits) - factor + 1, factor):
        window = bits[i:i + factor]
        ones = sum(window)
        out.append(1 if ones > factor // 2 else 0)
    return out


def nrzi_decode(bits: list) -> list:
    """NRZ-I decode: output 0 when bit changes, 1 when same."""
    out = []
    prev = 0
    for b in bits:
        out.append(0 if b != prev else 1)
        prev = b
    return out


def g3ruh_descramble(bits: list) -> list:
    """G3RUH descramble using 17-bit LFSR (taps at 12 and 17)."""
    shift_reg = 0
    out = []
    for b in bits:
        # Feedback taps: bit 12 and bit 17 (0-indexed: 11 and 16)
        fb = ((shift_reg >> 11) ^ (shift_reg >> 16)) & 1
        out_bit = b ^ fb
        out.append(out_bit)
        shift_reg = ((shift_reg << 1) | b) & 0x1FFFF  # 17-bit mask
    return out


def hdlc_extract(bits: list) -> list:
    """Extract HDLC frames from bit stream.

    Searches for 0x7E flags, performs bit destuffing, validates CRC-16/CCITT.
    Returns list of (frame_bytes, crc_valid) tuples.
    """
    FLAG = 0x7E  # 01111110
    frames = []

    # Search for flag sequences
    i = 0
    while i < len(bits) - 8:
        # Look for flag: 01111110
        byte_val = 0
        for j in range(8):
            byte_val = (byte_val << 1) | bits[i + j]

        if byte_val == FLAG:
            # Found flag — collect bits until next flag
            i += 8  # skip flag
            frame_bits = []
            consecutive_ones = 0
            abort = False

            while i < len(bits):
                bit = bits[i]
                i += 1

                if bit == 1:
                    consecutive_ones += 1
                    if consecutive_ones == 7:
                        # Abort sequence (7+ ones)
                        abort = True
                        break
                    if consecutive_ones == 6:
                        # Check next bit — if 0, it's a flag
                        # We already consumed the bit, check if we have 01111110
                        # Actually, 6 ones followed by... let's check
                        # We have 6 consecutive ones. The previous bits include these.
                        # Remove the 5 ones we added (the 6th is part of flag detection)
                        # This is a flag — end of frame
                        # Remove the last 5 one-bits (they were part of the closing flag)
                        for _ in range(5):
                            if frame_bits:
                                frame_bits.pop()
                        break
                    frame_bits.append(bit)
                else:
                    if consecutive_ones == 5:
                        # Bit stuffing — skip this zero
                        consecutive_ones = 0
                        continue
                    consecutive_ones = 0
                    frame_bits.append(bit)

            if abort or len(frame_bits) < 24:  # minimum: 14 addr + 1 ctrl + 2 CRC + padding
                continue

            # Convert bits to bytes
            frame_bytes = bytearray()
            for j in range(0, len(frame_bits) - 7, 8):
                byte_val = 0
                for k in range(8):
                    byte_val = (byte_val << 1) | frame_bits[j + k]
                frame_bytes.append(byte_val)

            if len(frame_bytes) >= 3:
                # CRC-16/CCITT check (last 2 bytes)
                crc_valid = check_crc16(frame_bytes)
                frames.append((bytes(frame_bytes[:-2]), crc_valid))
        else:
            i += 1

    return frames


def check_crc16(frame: bytes) -> bool:
    """Check CRC-16/CCITT (poly 0x8408, init 0xFFFF)."""
    crc = 0xFFFF
    for byte in frame:
        crc ^= byte
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0x8408
            else:
                crc >>= 1
    return crc == 0xF0B8  # residual for correct CRC


def decode_ax25_address(data: bytes) -> str:
    """Decode AX.25 address field (6 chars + SSID byte)."""
    if len(data) < 7:
        return "?"
    callsign = ""
    for i in range(6):
        c = (data[i] >> 1) & 0x7F
        if c != 0x20:  # space
            callsign += chr(c)
    ssid = (data[6] >> 1) & 0x0F
    if ssid:
        callsign += f"-{ssid}"
    return callsign


def main():
    parser = argparse.ArgumentParser(description="Decode oversampled CC1200 serial captures")
    parser.add_argument("files", nargs="+", help="Raw dump files")
    parser.add_argument("--baud", type=int, default=9600, help="Symbol rate (bps)")
    parser.add_argument("--oversample", default="auto", help="Oversampling factor (auto or integer)")
    parser.add_argument("--skip-decimate", action="store_true", help="Skip decimation (data already at symbol rate)")
    args = parser.parse_args()

    total_frames = 0

    for path in args.files:
        name = os.path.basename(path)
        print(f"\n{'='*60}")
        print(f"File: {name}")
        print(f"{'='*60}")

        with open(path, "rb") as f:
            data = f.read()

        if len(data) == 0:
            print("  EMPTY")
            continue

        # Check for all-zeros
        if data == bytes(len(data)):
            print(f"  ALL ZEROS ({len(data)} bytes) — captured before byte extraction fix")
            continue

        print(f"  Size: {len(data)} bytes ({len(data)*8} bits)")

        # Convert to bits
        bits = bytes_to_bits(data)

        # Decimate if oversampled
        if not args.skip_decimate:
            if args.oversample == "auto":
                factor = detect_oversample(bits, args.baud)
            else:
                factor = int(args.oversample)

            print(f"  Decimating by {factor}x: {len(bits)} → {len(bits)//factor} bits")
            bits = decimate(bits, factor)

        print(f"  Symbol-rate bits: {len(bits)} ({len(bits)/args.baud:.1f}s of data)")

        # NRZ-I decode
        nrzi_bits = nrzi_decode(bits)

        # G3RUH descramble
        descrambled = g3ruh_descramble(nrzi_bits)

        # Search for HDLC frames
        frames = hdlc_extract(descrambled)

        print(f"\n  Found {len(frames)} HDLC frames")

        for i, (frame, crc_ok) in enumerate(frames):
            crc_str = "CRC OK" if crc_ok else "CRC FAIL"
            print(f"\n  Frame {i+1}: {len(frame)} bytes [{crc_str}]")
            print(f"    Hex: {frame[:32].hex()}")

            if len(frame) >= 14:
                dst = decode_ax25_address(frame[0:7])
                src = decode_ax25_address(frame[7:14])
                print(f"    Dst: {dst}  Src: {src}")

            if crc_ok:
                total_frames += 1

        # Also try without NRZ-I (some sats don't use it)
        if not frames:
            print("\n  Trying without NRZ-I...")
            descrambled2 = g3ruh_descramble(bits)  # skip NRZ-I
            frames2 = hdlc_extract(descrambled2)
            if frames2:
                print(f"  Found {len(frames2)} frames WITHOUT NRZ-I!")
                for i, (frame, crc_ok) in enumerate(frames2):
                    print(f"    Frame {i+1}: {len(frame)} bytes [{'CRC OK' if crc_ok else 'CRC FAIL'}]")
                    print(f"    Hex: {frame[:32].hex()}")

        # Also try raw (no descramble) — for non-G3RUH sats
        if not frames:
            print("\n  Trying raw HDLC (no G3RUH)...")
            frames3 = hdlc_extract(nrzi_bits)
            if frames3:
                print(f"  Found {len(frames3)} raw HDLC frames!")
                for i, (frame, crc_ok) in enumerate(frames3):
                    print(f"    Frame {i+1}: {len(frame)} bytes [{'CRC OK' if crc_ok else 'CRC FAIL'}]")
                    print(f"    Hex: {frame[:32].hex()}")

        # Stats
        flag_count = 0
        for i in range(len(descrambled) - 8):
            byte_val = 0
            for j in range(8):
                byte_val = (byte_val << 1) | descrambled[i + j]
            if byte_val == 0x7E:
                flag_count += 1
        print(f"\n  0x7E flags found after descramble: {flag_count}")

    print(f"\n{'='*60}")
    print(f"TOTAL CRC-valid frames: {total_frames}")


if __name__ == "__main__":
    main()
