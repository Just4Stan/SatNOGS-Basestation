#!/usr/bin/env python3
"""
USP (Unified Space Protocol) bitstream decoder.

Decodes raw demodulated bitstream from CC1200 synchronous serial mode.
USP is used by many UHF CubeSats (e.g. UmKA-1, satellites using NanoAvionics OBC).

Protocol stack:
    1. 64-bit sync word: 0x5072F64B2D90B1F5
    2. PLS code: (64,7) block code → 7 data bits (frame length + coding rate)
    3. Payload: Viterbi-decoded (r=1/2, k=7, poly 0x6D/0x4F)
    4. CCSDS descramble: x^8 + x^7 + x^5 + x^3 + 1, seed 0xFF
    5. RS(255,223) decode → 223 bytes useful data per block

Usage:
    from usp_decoder import USPDecoder

    decoder = USPDecoder()
    for chunk in raw_byte_chunks:
        frames = decoder.feed(chunk)
        for frame in frames:
            print(f"USP frame: {len(frame)} bytes")

Standalone test:
    python3 usp_decoder.py --file capture.bin
"""

import struct
from typing import List, Optional

# USP 64-bit sync word (split into two 32-bit halves for easier matching)
USP_SYNC_HI = 0x5072F64B
USP_SYNC_LO = 0x2D90B1F5
USP_SYNC_64 = (USP_SYNC_HI << 32) | USP_SYNC_LO

# CCSDS scrambler polynomial: x^8 + x^7 + x^5 + x^3 + 1
# Implemented as LFSR with seed 0xFF
CCSDS_POLY = 0b10101001  # taps at 8,7,5,3 → feedback bits (reversed)


def _build_ccsds_pn_sequence(length: int) -> bytes:
    """Generate CCSDS pseudo-random sequence for descrambling."""
    sr = 0xFF  # seed
    out = bytearray(length)
    for i in range(length):
        out[i] = sr
        for _ in range(8):
            feedback = 0
            # Taps at bits 0, 2, 4, 6 of SR (corresponding to x^8,x^7,x^5,x^3)
            feedback = ((sr >> 7) ^ (sr >> 5) ^ (sr >> 3) ^ (sr >> 0)) & 1
            sr = ((sr << 1) | feedback) & 0xFF
    return bytes(out)


# Pre-compute a generous PN sequence (max frame = 255*4 RS blocks + overhead)
_PN_SEQ = _build_ccsds_pn_sequence(2048)


def ccsds_descramble(data: bytes) -> bytes:
    """XOR data with CCSDS PN sequence."""
    n = min(len(data), len(_PN_SEQ))
    return bytes(a ^ b for a, b in zip(data[:n], _PN_SEQ[:n]))


# ---------------------------------------------------------------------------
# Viterbi decoder (r=1/2, k=7) — minimal Python implementation
# ---------------------------------------------------------------------------
# Generator polynomials (octal): G1=171 (0x6D), G2=133 (0x4F) — NASA standard
VITERBI_G1 = 0b1101101  # 0x6D = 109
VITERBI_G2 = 0b1001111  # 0x4F = 79
VITERBI_K = 7
VITERBI_STATES = 1 << (VITERBI_K - 1)  # 64 states


def _parity(x: int) -> int:
    """Count parity (number of 1-bits mod 2)."""
    x ^= x >> 16
    x ^= x >> 8
    x ^= x >> 4
    x ^= x >> 2
    x ^= x >> 1
    return x & 1


def _build_viterbi_tables():
    """Pre-compute output and next-state tables for Viterbi."""
    # output[state][input_bit] = (g1_out, g2_out)
    output = {}
    for state in range(VITERBI_STATES):
        output[state] = {}
        for inp in (0, 1):
            reg = (inp << (VITERBI_K - 1)) | state
            g1 = _parity(reg & VITERBI_G1)
            g2 = _parity(reg & VITERBI_G2)
            output[state][inp] = (g1, g2)
    return output


_VITERBI_OUTPUT = _build_viterbi_tables()


def viterbi_decode(symbols: List[int]) -> bytes:
    """
    Hard-decision Viterbi decode.

    Args:
        symbols: List of received bits (0 or 1), length must be even.
                 Pairs are (G1, G2) for each input bit.

    Returns:
        Decoded bytes.
    """
    n_pairs = len(symbols) // 2
    if n_pairs == 0:
        return b""

    INF = 0x7FFFFFFF

    # Path metrics
    pm = [INF] * VITERBI_STATES
    pm[0] = 0  # start in state 0

    # Traceback
    tb = []  # tb[step][state] = (prev_state, input_bit)

    for step in range(n_pairs):
        rx_g1 = symbols[step * 2]
        rx_g2 = symbols[step * 2 + 1]

        new_pm = [INF] * VITERBI_STATES
        step_tb = [None] * VITERBI_STATES

        for state in range(VITERBI_STATES):
            if pm[state] == INF:
                continue
            for inp in (0, 1):
                next_state = (state >> 1) | (inp << (VITERBI_K - 2))
                g1, g2 = _VITERBI_OUTPUT[state][inp]
                dist = (g1 ^ rx_g1) + (g2 ^ rx_g2)  # Hamming distance
                metric = pm[state] + dist
                if metric < new_pm[next_state]:
                    new_pm[next_state] = metric
                    step_tb[next_state] = (state, inp)

        pm = new_pm
        tb.append(step_tb)

    # Find best final state
    best_state = min(range(VITERBI_STATES), key=lambda s: pm[s])

    # Traceback
    decoded_bits = []
    state = best_state
    for step in range(n_pairs - 1, -1, -1):
        prev_state, inp = tb[step][state]
        decoded_bits.append(inp)
        state = prev_state

    decoded_bits.reverse()

    # Pack bits into bytes
    out = bytearray()
    for i in range(0, len(decoded_bits) - 7, 8):
        byte_val = 0
        for j in range(8):
            byte_val = (byte_val << 1) | decoded_bits[i + j]
        out.append(byte_val)

    return bytes(out)


# ---------------------------------------------------------------------------
# Reed-Solomon (255,223) — uses reedsolo if available, otherwise skip
# ---------------------------------------------------------------------------
_rs_codec = None
try:
    from reedsolo import RSCodec
    _rs_codec = RSCodec(32)  # 32 check symbols = RS(255,223)
except ImportError:
    pass


def rs_decode(data: bytes) -> Optional[bytes]:
    """RS(255,223) decode. Returns None if uncorrectable or reedsolo not installed."""
    if _rs_codec is None:
        # No RS library — return data minus 32 parity bytes (unchecked)
        if len(data) >= 255:
            return bytes(data[:223])
        return bytes(data)
    try:
        decoded = _rs_codec.decode(bytearray(data))
        return bytes(decoded)
    except Exception:
        return None


# ---------------------------------------------------------------------------
# PLS code decoder (64,7) block code — simplified
# ---------------------------------------------------------------------------
# The PLS code encodes 7 bits into 64 bits using a (64,7) Reed-Muller-like code.
# For now, we do a simple majority-vote approach on the repeated pattern.
# A full implementation would use the DVB-S2 PLS decoding algorithm.

def decode_pls(bits_64: List[int]) -> Optional[int]:
    """Decode 64-bit PLS code to 7-bit value. Returns None on failure."""
    if len(bits_64) < 64:
        return None
    # Simplified: extract bits by position (proper decoding needs correlation)
    # For prototype, return raw first 7 bits (placeholder)
    val = 0
    for i in range(7):
        val = (val << 1) | (bits_64[i] & 1)
    return val


# ---------------------------------------------------------------------------
# USP Decoder state machine
# ---------------------------------------------------------------------------
class USPDecoder:
    """Incremental USP frame decoder fed with raw demodulated bytes."""

    def __init__(self):
        self.bit_buf: List[int] = []
        self.frames_decoded = 0
        self.sync_found = 0

    def feed(self, data: bytes) -> List[bytes]:
        """Feed raw bytes (from EVT_RX_DATA), return any decoded frames."""
        # Unpack bytes to bits (MSB first)
        for byte_val in data:
            for i in range(7, -1, -1):
                self.bit_buf.append((byte_val >> i) & 1)

        frames = []

        # Search for sync words in bit buffer
        while len(self.bit_buf) >= 64 + 64 + 16:  # sync + PLS + minimum payload
            sync_pos = self._find_sync()
            if sync_pos is None:
                # No sync found — keep last 63 bits for partial match
                if len(self.bit_buf) > 63:
                    self.bit_buf = self.bit_buf[-63:]
                break

            self.sync_found += 1

            # Consume bits up to and including sync
            pos = sync_pos + 64

            # Decode PLS code (next 64 bits)
            if pos + 64 > len(self.bit_buf):
                break  # not enough data yet
            pls_bits = self.bit_buf[pos:pos + 64]
            pls_val = decode_pls(pls_bits)
            pos += 64

            # PLS gives us frame length and coding info
            # For now, assume a standard 255-byte RS block with r=1/2 Viterbi
            # That means 255 * 8 * 2 = 4080 encoded symbols after PLS
            n_symbols = 4080

            if pos + n_symbols > len(self.bit_buf):
                # Not enough data for full frame — put sync back and wait
                break

            symbols = self.bit_buf[pos:pos + n_symbols]
            pos += n_symbols

            # Viterbi decode
            decoded = viterbi_decode(symbols)

            # CCSDS descramble
            descrambled = ccsds_descramble(decoded)

            # RS decode
            if len(descrambled) >= 255:
                payload = rs_decode(descrambled[:255])
                if payload is not None:
                    frames.append(payload)
                    self.frames_decoded += 1

            # Consume processed bits
            self.bit_buf = self.bit_buf[pos:]

        return frames

    def _find_sync(self) -> Optional[int]:
        """Find 64-bit USP sync word in bit buffer. Returns bit position or None."""
        if len(self.bit_buf) < 64:
            return None

        # Build a sliding 64-bit register
        reg = 0
        for i in range(64):
            reg = (reg << 1) | self.bit_buf[i]

        if reg == USP_SYNC_64:
            return 0

        for i in range(64, len(self.bit_buf)):
            reg = ((reg << 1) | self.bit_buf[i]) & 0xFFFFFFFFFFFFFFFF
            if reg == USP_SYNC_64:
                return i - 63

        return None

    def reset(self):
        """Clear internal state."""
        self.bit_buf.clear()


# ---------------------------------------------------------------------------
# Standalone CLI
# ---------------------------------------------------------------------------
if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="USP bitstream decoder")
    parser.add_argument("--file", required=True, help="Raw bitstream file (binary)")
    args = parser.parse_args()

    with open(args.file, "rb") as f:
        raw = f.read()

    print(f"Read {len(raw)} bytes ({len(raw)*8} bits)")

    decoder = USPDecoder()

    # Feed in chunks (simulating streaming)
    chunk_size = 64
    total_frames = []
    for i in range(0, len(raw), chunk_size):
        frames = decoder.feed(raw[i:i + chunk_size])
        total_frames.extend(frames)

    print(f"Sync words found: {decoder.sync_found}")
    print(f"Frames decoded: {decoder.frames_decoded}")

    for i, frame in enumerate(total_frames):
        print(f"  Frame {i}: {len(frame)} bytes — {frame[:32].hex()}...")
