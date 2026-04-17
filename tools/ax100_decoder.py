#!/usr/bin/env python3
"""
GomSpace AX.100 Mode 5 bitstream decoder.

Decodes raw demodulated bitstream from CC1200 synchronous serial mode.
AX.100 Mode 5 is used by many CubeSats with GomSpace radios (e.g. AAUSAT-4,
GomX-1, many ESA OPS-SAT-like missions).

Protocol stack:
    1. 32-bit sync word: 0x930B51DE
    2. Golay(24,12) encoded header → 12 data bits (packet length)
    3. CCSDS descramble: x^8 + x^7 + x^5 + x^3 + 1, seed 0xFF
    4. RS(255,223) decode
    5. CSP (CubeSat Space Protocol) payload

Usage:
    from ax100_decoder import AX100Decoder

    decoder = AX100Decoder()
    for chunk in raw_byte_chunks:
        frames = decoder.feed(chunk)
        for frame in frames:
            print(f"AX.100 frame: {len(frame)} bytes")

Standalone test:
    python3 ax100_decoder.py --file capture.bin
"""

from typing import List, Optional

# AX.100 Mode 5 sync word (32 bits)
AX100_SYNC = 0x930B51DE

# CCSDS scrambler — same as USP
CCSDS_POLY = 0b10101001


def _build_ccsds_pn_sequence(length: int) -> bytes:
    """Generate CCSDS pseudo-random sequence for descrambling."""
    sr = 0xFF
    out = bytearray(length)
    for i in range(length):
        out[i] = sr
        for _ in range(8):
            feedback = ((sr >> 7) ^ (sr >> 5) ^ (sr >> 3) ^ (sr >> 0)) & 1
            sr = ((sr << 1) | feedback) & 0xFF
    return bytes(out)


_PN_SEQ = _build_ccsds_pn_sequence(2048)


def ccsds_descramble(data: bytes) -> bytes:
    """XOR data with CCSDS PN sequence."""
    n = min(len(data), len(_PN_SEQ))
    return bytes(a ^ b for a, b in zip(data[:n], _PN_SEQ[:n]))


# ---------------------------------------------------------------------------
# Golay(24,12) decoder
# ---------------------------------------------------------------------------
# Extended Golay code: 12 data bits → 24 coded bits, corrects up to 3 errors.
# Generator matrix (systematic form): G = [I_12 | P] where P is the parity matrix.

_GOLAY_PARITY = [
    0x8ED, 0x1DB, 0x3B5, 0x769, 0xED1, 0xDA3,
    0xB47, 0x68F, 0xD1D, 0xA3B, 0x477, 0xFFE,
]


def _golay_syndrome(codeword_24: int) -> int:
    """Compute syndrome for a 24-bit Golay codeword."""
    # Systematic encoding: bits [23:12] = data, bits [11:0] = parity
    s = 0
    for i in range(12):
        if codeword_24 & (1 << (23 - i)):
            s ^= _GOLAY_PARITY[i]
    parity_received = codeword_24 & 0xFFF
    return s ^ parity_received


def _popcount(x: int) -> int:
    """Count number of set bits."""
    c = 0
    while x:
        c += x & 1
        x >>= 1
    return c


def golay_decode(bits_24: int) -> Optional[int]:
    """
    Decode a 24-bit extended Golay codeword.

    Returns the 12-bit data word, or None if uncorrectable (>3 errors).
    """
    s = _golay_syndrome(bits_24)
    if s == 0:
        return (bits_24 >> 12) & 0xFFF

    # Try 1-bit error patterns in data half
    if _popcount(s) <= 3:
        # Error is in parity bits only — data is correct
        return (bits_24 >> 12) & 0xFFF

    for i in range(12):
        test = s ^ _GOLAY_PARITY[i]
        if _popcount(test) <= 2:
            # Single error in data bit i, plus up to 2 in parity
            data = (bits_24 >> 12) & 0xFFF
            data ^= (1 << (11 - i))
            return data

    # Try 2-bit error patterns in data
    for i in range(12):
        for j in range(i + 1, 12):
            test = s ^ _GOLAY_PARITY[i] ^ _GOLAY_PARITY[j]
            if _popcount(test) <= 1:
                data = (bits_24 >> 12) & 0xFFF
                data ^= (1 << (11 - i)) | (1 << (11 - j))
                return data

    return None  # uncorrectable


# ---------------------------------------------------------------------------
# Reed-Solomon (255,223) — uses reedsolo if available
# ---------------------------------------------------------------------------
_rs_codec = None
try:
    from reedsolo import RSCodec
    _rs_codec = RSCodec(32)
except ImportError:
    pass


def rs_decode(data: bytes) -> Optional[bytes]:
    """RS(255,223) decode. Returns None if uncorrectable."""
    if _rs_codec is None:
        if len(data) >= 255:
            return bytes(data[:223])
        return bytes(data)
    try:
        decoded = _rs_codec.decode(bytearray(data))
        return bytes(decoded)
    except Exception:
        return None


# ---------------------------------------------------------------------------
# AX.100 Mode 5 decoder state machine
# ---------------------------------------------------------------------------
class AX100Decoder:
    """Incremental AX.100 Mode 5 frame decoder fed with raw demodulated bytes."""

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

        while len(self.bit_buf) >= 32 + 24 + 8:  # sync + Golay header + min payload
            sync_pos = self._find_sync()
            if sync_pos is None:
                if len(self.bit_buf) > 31:
                    self.bit_buf = self.bit_buf[-31:]
                break

            self.sync_found += 1
            pos = sync_pos + 32

            # Decode Golay(24,12) header → packet length
            if pos + 24 > len(self.bit_buf):
                break

            header_bits = self.bit_buf[pos:pos + 24]
            header_24 = 0
            for b in header_bits:
                header_24 = (header_24 << 1) | (b & 1)
            pos += 24

            pkt_len = golay_decode(header_24)
            if pkt_len is None or pkt_len == 0 or pkt_len > 255:
                # Bad header — skip this sync and keep searching
                self.bit_buf = self.bit_buf[sync_pos + 32:]
                continue

            # Read pkt_len bytes of payload
            n_bits = pkt_len * 8
            if pos + n_bits > len(self.bit_buf):
                # Not enough data — wait for more
                break

            payload_bits = self.bit_buf[pos:pos + n_bits]
            pos += n_bits

            # Pack bits to bytes
            payload = bytearray()
            for i in range(0, len(payload_bits) - 7, 8):
                byte_val = 0
                for j in range(8):
                    byte_val = (byte_val << 1) | payload_bits[i + j]
                payload.append(byte_val)

            # CCSDS descramble
            descrambled = ccsds_descramble(bytes(payload))

            # RS decode (if payload is 255 bytes = full RS block)
            if len(descrambled) >= 255:
                decoded = rs_decode(descrambled[:255])
                if decoded is not None:
                    frames.append(decoded)
                    self.frames_decoded += 1
            else:
                # Short frame — no RS, just raw CSP data
                frames.append(descrambled)
                self.frames_decoded += 1

            # Consume processed bits
            self.bit_buf = self.bit_buf[pos:]

        return frames

    def _find_sync(self) -> Optional[int]:
        """Find 32-bit AX.100 sync word in bit buffer."""
        if len(self.bit_buf) < 32:
            return None

        reg = 0
        for i in range(32):
            reg = (reg << 1) | self.bit_buf[i]

        if reg == AX100_SYNC:
            return 0

        for i in range(32, len(self.bit_buf)):
            reg = ((reg << 1) | self.bit_buf[i]) & 0xFFFFFFFF
            if reg == AX100_SYNC:
                return i - 31

        return None

    def reset(self):
        """Clear internal state."""
        self.bit_buf.clear()


# ---------------------------------------------------------------------------
# Standalone CLI
# ---------------------------------------------------------------------------
if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="AX.100 Mode 5 bitstream decoder")
    parser.add_argument("--file", required=True, help="Raw bitstream file (binary)")
    args = parser.parse_args()

    with open(args.file, "rb") as f:
        raw = f.read()

    print(f"Read {len(raw)} bytes ({len(raw)*8} bits)")

    decoder = AX100Decoder()

    chunk_size = 64
    total_frames = []
    for i in range(0, len(raw), chunk_size):
        frames = decoder.feed(raw[i:i + chunk_size])
        total_frames.extend(frames)

    print(f"Sync words found: {decoder.sync_found}")
    print(f"Frames decoded: {decoder.frames_decoded}")

    for i, frame in enumerate(total_frames):
        print(f"  Frame {i}: {len(frame)} bytes — {frame[:32].hex()}...")
