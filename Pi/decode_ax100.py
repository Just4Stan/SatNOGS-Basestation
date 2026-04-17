#!/usr/bin/env python3
"""
AX.100 Mode 5 (ASM+Golay) decoder for CC1200 raw captures.

Decodes GOMspace NanoCom AX100 frames from raw CC1200 FIFO output.
The CC1200 matched sync word 0x930B51DE and delivered 255-byte fixed-length
chunks. This script attempts Golay(24,12) + CCSDS descramble + RS(255,223)
on each chunk.

Frame format (after sync word, which CC1200 strips):
  [3B Golay header] [payload + 32B RS parity]  (scrambled)

Usage:
  python3 decode_ax100.py <rawfile> [--scan] [--verbose]

References:
  - gr-satellites: github.com/daniestevez/gr-satellites
  - Daniel Estevez: destevez.net/2018/03/ty-2-decoded/
"""

import sys
import os
import struct
import argparse
from collections import Counter

# ---------------------------------------------------------------------------
# Golay(24,12) codec — syndrome decoding per Morelos-Zaragoza / gr-satellites
# ---------------------------------------------------------------------------

# Parity-check rows: 12 x 24-bit values
_GOLAY_H = [
    0x8008ED, 0x4001DB, 0x2003B5, 0x100769,
    0x080ED1, 0x040DA3, 0x020B47, 0x01068F,
    0x008D1D, 0x004A3B, 0x002477, 0x001FFE,
]

# Extract B matrix (lower 12 bits of each H row)
_GOLAY_B = [h & 0xFFF for h in _GOLAY_H]


def _popcount(x):
    return bin(x).count('1')


def _golay_syndrome(word24):
    """Compute 12-bit syndrome from 24-bit received word."""
    s = 0
    for i in range(12):
        if _popcount(word24 & _GOLAY_H[i]) & 1:
            s |= (1 << (11 - i))
    return s


def _golay_mult_by_b(s12):
    """Multiply 12-bit vector by B matrix → 12-bit result."""
    r = 0
    for i in range(12):
        if s12 & (1 << (11 - i)):
            r ^= _GOLAY_B[i]
    return r


def golay_decode(data3):
    """Decode 3-byte Golay(24,12) codeword.

    Returns (data12, errors_corrected) or (None, -1) if uncorrectable.
    data12 is the 12-bit information word.
    """
    word = (data3[0] << 16) | (data3[1] << 8) | data3[2]

    s = _golay_syndrome(word)
    if s == 0:
        return word & 0xFFF, 0

    # Try: error in parity bits only (upper 12)
    if _popcount(s) <= 3:
        e = s << 12
        corrected = word ^ e
        return corrected & 0xFFF, _popcount(s)

    # Try: 1 data bit error + parity errors
    for i in range(12):
        t = s ^ _GOLAY_B[i]
        if _popcount(t) <= 2:
            e = (t << 12) | (1 << (11 - i))
            corrected = word ^ e
            return corrected & 0xFFF, _popcount(t) + 1

    # Compute q = s * B
    q = _golay_mult_by_b(s)

    # Try: error in data bits only (lower 12)
    if _popcount(q) <= 3:
        e = q
        corrected = word ^ e
        return corrected & 0xFFF, _popcount(q)

    # Try: 1 parity bit error + data errors
    for i in range(12):
        t = q ^ _GOLAY_B[i]
        if _popcount(t) <= 2:
            e = (1 << (11 - i + 12)) | t
            corrected = word ^ e
            return corrected & 0xFFF, _popcount(t) + 1

    return None, -1


# ---------------------------------------------------------------------------
# CCSDS synchronous scrambler (x^8 + x^7 + x^5 + x^3 + 1, seed=0xFF)
# ---------------------------------------------------------------------------

def _generate_ccsds_pn(length):
    """Generate CCSDS PN sequence bytes. Repeats every 255 bytes."""
    # LFSR: taps at positions 8, 7, 5, 3 (x^8+x^7+x^5+x^3+1)
    # Shift register x[1..8], all init to 1
    x = [0] + [1] * 8  # x[0] unused, x[1..8] = all 1s

    pn_bytes = []
    for _ in range(length):
        byte_val = 0
        for bit in range(8):
            # Output is x[1] (MSB first)
            byte_val = (byte_val << 1) | x[1]
            # Feedback: x[8] ^ x[6] ^ x[4] ^ x[1]
            # Note: polynomial 1+x^3+x^5+x^7+x^8 → taps from high end
            # gr-satellites: feedback = x[1] ^ x[4] ^ x[6] ^ x[8]
            # Actually looking at gr-satellites randomizer.c more carefully:
            # The shift register is x[1..8], output x[1], feedback into x[8]
            # feedback = (x[8] + x[6] + x[4] + x[1]) mod 2
            # But wait - let me match the CCSDS standard exactly.
            # CCSDS 131.0-B-4: h(x) = x^8 + x^7 + x^5 + x^3 + 1
            # The LFSR has taps at: x^8, x^7, x^5, x^3 (and +1 is the input)
            # Output = MSB, feedback = XOR of tapped positions
            fb = x[8] ^ x[6] ^ x[4] ^ x[1]
            # Shift left
            for j in range(1, 8):
                x[j] = x[j + 1]
            x[8] = fb
        pn_bytes.append(byte_val)
    return bytes(pn_bytes)


# Pre-compute 255 bytes of PN (it repeats after 255)
_CCSDS_PN = _generate_ccsds_pn(255)


def ccsds_descramble(data):
    """XOR data with CCSDS PN sequence (cyclic, period 255)."""
    pn_len = len(_CCSDS_PN)
    result = bytearray(len(data))
    for i in range(len(data)):
        result[i] = data[i] ^ _CCSDS_PN[i % pn_len]
    return bytes(result)


# ---------------------------------------------------------------------------
# CCSDS Reed-Solomon(255,223) — dual basis
# ---------------------------------------------------------------------------
# The CCSDS RS code uses a dual-basis representation. We need to convert
# between conventional and dual basis before/after using reedsolo.
# Conversion tables from CCSDS 131.0-B-4 / gr-satellites.

# Conventional-to-dual (Berlekamp) basis conversion table
_TAL1TAB = [
    0x00, 0xCC, 0xAC, 0x60, 0x79, 0xB5, 0xD5, 0x19,
    0x11, 0xDD, 0xBD, 0x71, 0x68, 0xA4, 0xC4, 0x08,
    0xE0, 0x2C, 0x4C, 0x80, 0x99, 0x55, 0x35, 0xF9,
    0xF1, 0x3D, 0x5D, 0x91, 0x88, 0x44, 0x24, 0xE8,
    0x10, 0xDC, 0xBC, 0x70, 0x69, 0xA5, 0xC5, 0x09,
    0x01, 0xCD, 0xAD, 0x61, 0x78, 0xB4, 0xD4, 0x18,
    0xF0, 0x3C, 0x5C, 0x90, 0x89, 0x45, 0x25, 0xE9,
    0xE1, 0x2D, 0x4D, 0x81, 0x98, 0x54, 0x34, 0xF8,
    0xD0, 0x1C, 0x7C, 0xB0, 0xA9, 0x65, 0x05, 0xC9,
    0xC1, 0x0D, 0x6D, 0xA1, 0xB8, 0x74, 0x14, 0xD8,
    0x30, 0xFC, 0x9C, 0x50, 0x49, 0x85, 0xE5, 0x29,
    0x21, 0xED, 0x8D, 0x41, 0x58, 0x94, 0xF4, 0x38,
    0xC0, 0x0C, 0x6C, 0xA0, 0xB9, 0x75, 0x15, 0xD9,
    0xD1, 0x1D, 0x7D, 0xB1, 0xA8, 0x64, 0x04, 0xC8,
    0x20, 0xEC, 0x8C, 0x40, 0x59, 0x95, 0xF5, 0x39,
    0x31, 0xFD, 0x9D, 0x51, 0x48, 0x84, 0xE4, 0x28,
    0xB1, 0x7D, 0x1D, 0xD1, 0xC8, 0x04, 0x64, 0xA8,
    0xA0, 0x6C, 0x0C, 0xC0, 0xD9, 0x15, 0x75, 0xB9,
    0x51, 0x9D, 0xFD, 0x31, 0x28, 0xE4, 0x84, 0x48,
    0x40, 0x8C, 0xEC, 0x20, 0x39, 0xF5, 0x95, 0x59,
    0xA1, 0x6D, 0x0D, 0xC1, 0xD8, 0x14, 0x74, 0xB8,
    0xB0, 0x7C, 0x1C, 0xD0, 0xC9, 0x05, 0x65, 0xA9,
    0x41, 0x8D, 0xED, 0x21, 0x38, 0xF4, 0x94, 0x58,
    0x50, 0x9C, 0xFC, 0x30, 0x29, 0xE5, 0x85, 0x49,
    0x61, 0xAD, 0xCD, 0x01, 0x18, 0xD4, 0xB4, 0x78,
    0x70, 0xBC, 0xDC, 0x10, 0x09, 0xC5, 0xA5, 0x69,
    0x81, 0x4D, 0x2D, 0xE1, 0xF8, 0x34, 0x54, 0x98,
    0x90, 0x5C, 0x3C, 0xF0, 0xE9, 0x25, 0x45, 0x89,
    0x71, 0xBD, 0xDD, 0x11, 0x08, 0xC4, 0xA4, 0x68,
    0x60, 0xAC, 0xCC, 0x00, 0x19, 0xD5, 0xB5, 0x79,
    0x91, 0x5D, 0x3D, 0xF1, 0xE8, 0x24, 0x44, 0x88,
    0x80, 0x4C, 0x2C, 0xE0, 0xF9, 0x35, 0x55, 0x99,
]

# Dual-to-conventional basis conversion table
_TAL0TAB = [
    0x00, 0x7B, 0xAF, 0xD4, 0x99, 0xE2, 0x36, 0x4D,
    0xFA, 0x81, 0x55, 0x2E, 0x63, 0x18, 0xCC, 0xB7,
    0x86, 0xFD, 0x29, 0x52, 0x1F, 0x64, 0xB0, 0xCB,
    0x7C, 0x07, 0xD3, 0xA8, 0xE5, 0x9E, 0x4A, 0x31,
    0xEC, 0x97, 0x43, 0x38, 0x75, 0x0E, 0xDA, 0xA1,
    0x16, 0x6D, 0xB9, 0xC2, 0x8F, 0xF4, 0x20, 0x5B,
    0x6A, 0x11, 0xC5, 0xBE, 0xF3, 0x88, 0x5C, 0x27,
    0x90, 0xEB, 0x3F, 0x44, 0x09, 0x72, 0xA6, 0xDD,
    0xEF, 0x94, 0x40, 0x3B, 0x76, 0x0D, 0xD9, 0xA2,
    0x15, 0x6E, 0xBA, 0xC1, 0x8C, 0xF7, 0x23, 0x58,
    0x69, 0x12, 0xC6, 0xBD, 0xF0, 0x8B, 0x5F, 0x24,
    0x93, 0xE8, 0x3C, 0x47, 0x0A, 0x71, 0xA5, 0xDE,
    0x03, 0x78, 0xAC, 0xD7, 0x9A, 0xE1, 0x35, 0x4E,
    0xF9, 0x82, 0x56, 0x2D, 0x60, 0x1B, 0xCF, 0xB4,
    0x85, 0xFE, 0x2A, 0x51, 0x1C, 0x67, 0xB3, 0xC8,
    0x7F, 0x04, 0xD0, 0xAB, 0xE6, 0x9D, 0x49, 0x32,
    0x8D, 0xF6, 0x22, 0x59, 0x14, 0x6F, 0xBB, 0xC0,
    0x77, 0x0C, 0xD8, 0xA3, 0xEE, 0x95, 0x41, 0x3A,
    0x0B, 0x70, 0xA4, 0xDF, 0x92, 0xE9, 0x3D, 0x46,
    0xF1, 0x8A, 0x5E, 0x25, 0x68, 0x13, 0xC7, 0xBC,
    0x61, 0x1A, 0xCE, 0xB5, 0xF8, 0x83, 0x57, 0x2C,
    0x9B, 0xE0, 0x34, 0x4F, 0x02, 0x79, 0xAD, 0xD6,
    0xE7, 0x9C, 0x48, 0x33, 0x7E, 0x05, 0xD1, 0xAA,
    0x1D, 0x66, 0xB2, 0xC9, 0x84, 0xFF, 0x2B, 0x50,
    0x62, 0x19, 0xCD, 0xB6, 0xFB, 0x80, 0x54, 0x2F,
    0x98, 0xE3, 0x37, 0x4C, 0x01, 0x7A, 0xAE, 0xD5,
    0xE4, 0x9F, 0x4B, 0x30, 0x7D, 0x06, 0xD2, 0xA9,
    0x1E, 0x65, 0xB1, 0xCA, 0x87, 0xFC, 0x28, 0x53,
    0x0E, 0x75, 0xA1, 0xDA, 0x97, 0xEC, 0x38, 0x43,
    0xF4, 0x8F, 0x5B, 0x20, 0x6D, 0x16, 0xC2, 0xB9,
    0x88, 0xF3, 0x27, 0x5C, 0x11, 0x6A, 0xBE, 0xC5,
    0x72, 0x09, 0xDD, 0xA6, 0xEB, 0x90, 0x44, 0x3F,
]


def _conv_to_dual(data):
    """Convert conventional basis to dual (Berlekamp) basis."""
    return bytes(_TAL1TAB[b] for b in data)


def _dual_to_conv(data):
    """Convert dual (Berlekamp) basis to conventional basis."""
    return bytes(_TAL0TAB[b] for b in data)


def rs_decode_ccsds(data, nsym=32):
    """Decode shortened CCSDS RS(255,223) codeword.

    data: payload + 32 bytes RS parity (in CCSDS dual basis)
    Returns (decoded_payload, errors_corrected) or (None, -1) on failure.
    """
    import reedsolo

    rx_len = len(data)
    if rx_len < nsym + 1:
        return None, -1

    # Convert from dual to conventional basis
    conv_data = bytearray(_dual_to_conv(data))

    # Pad with zeros at front for shortened code
    pad = 255 - rx_len
    padded = bytearray(pad) + conv_data

    # reedsolo RS(255,223) with CCSDS parameters:
    # field generator: x^8 + x^7 + x^2 + x + 1 = 0x187
    # code generator fcr=112, prim=11
    try:
        # prim = field polynomial (0x187 for CCSDS), generator = primitive element (11)
        rs = reedsolo.RSCodec(nsym, nsize=255, fcr=112, prim=0x187,
                              generator=11, c_exp=8)
        # reedsolo expects the message as-is with parity at end
        decoded = rs.decode(bytes(padded))
        # decoded is (data, remainder, errata_pos)
        payload = bytes(decoded[0])[pad:]  # strip padding
        nerrs = len(decoded[2]) if len(decoded) > 2 else 0
        # Convert back to dual basis
        payload = _conv_to_dual(payload)
        return payload, nerrs
    except Exception:
        return None, -1


def rs_decode_ccsds_bruteforce(data, nsym=32):
    """Try multiple RS parameter combinations."""
    import reedsolo

    rx_len = len(data)
    if rx_len < nsym + 1:
        return None, -1, ""

    # Try with and without basis conversion
    for basis_name, convert_in, convert_out in [
        ("dual→conv", _dual_to_conv, _conv_to_dual),
        ("no-convert", lambda x: x, lambda x: x),
    ]:
        conv_data = bytearray(convert_in(data))
        pad = 255 - rx_len
        padded = bytearray(pad) + conv_data

        # Try different RS parameter sets
        # prim = field polynomial, generator = primitive element
        param_sets = [
            # CCSDS standard: field poly 0x187, prim elem 11, fcr 112
            {"nsym": nsym, "fcr": 112, "prim": 0x187, "generator": 11},
            # Default reedsolo: field poly 0x11D, prim elem 2, fcr 0
            {"nsym": nsym, "fcr": 0, "prim": 0x11D, "generator": 2},
            # CCSDS with fcr=0
            {"nsym": nsym, "fcr": 0, "prim": 0x187, "generator": 11},
            # CCSDS with fcr=1
            {"nsym": nsym, "fcr": 1, "prim": 0x187, "generator": 11},
            # Standard with fcr=1
            {"nsym": nsym, "fcr": 1, "prim": 0x11D, "generator": 2},
            # AX.25-style: field poly 0x11D, prim 2, fcr 112
            {"nsym": nsym, "fcr": 112, "prim": 0x11D, "generator": 2},
        ]

        for params in param_sets:
            try:
                rs = reedsolo.RSCodec(params["nsym"], nsize=255,
                                      fcr=params["fcr"],
                                      prim=params["prim"],
                                      generator=params["generator"],
                                      c_exp=8)
                decoded = rs.decode(bytes(padded))
                payload = bytes(decoded[0])[pad:]
                nerrs = len(decoded[2]) if len(decoded) > 2 else 0
                desc = f"{basis_name}, fcr={params['fcr']}, prim={params['prim']}"
                payload = convert_out(payload)
                return payload, nerrs, desc
            except Exception:
                continue

    return None, -1, ""


# ---------------------------------------------------------------------------
# Frame scanning — search for valid Golay headers at every byte offset
# ---------------------------------------------------------------------------

def scan_for_frames(raw_data, verbose=False):
    """Scan raw data byte-by-byte for valid Golay headers.

    More robust than fixed 255-byte chunking — handles misalignment.
    """
    frames = []
    i = 0
    while i < len(raw_data) - 3:
        data12, errs = golay_decode(raw_data[i:i + 3])
        if data12 is not None:
            length = data12 & 0xFF  # lower 8 bits = frame length
            flags = (data12 >> 8) & 0xF

            # Valid AX.100 frame length: includes RS parity (32 bytes)
            # So total = CSP payload + 32. Min ~36 (4 byte CSP + 32 RS),
            # max 255. Golay errors <= 3.
            if 36 <= length <= 255 and errs <= 3:
                end = i + 3 + length
                if end <= len(raw_data):
                    frame_data = raw_data[i + 3:end]

                    # Descramble
                    descrambled = ccsds_descramble(frame_data)

                    # Try RS decode
                    payload, rs_errs = rs_decode_ccsds(descrambled)
                    if payload is not None:
                        frames.append({
                            'offset': i,
                            'golay_errs': errs,
                            'length': length,
                            'flags': flags,
                            'rs_errs': rs_errs,
                            'payload': payload,
                            'raw': frame_data,
                            'descrambled': descrambled,
                        })
                        if verbose:
                            print(f"  [DECODED] offset={i}, len={length}, "
                                  f"golay_errs={errs}, rs_errs={rs_errs}")
                        i = end  # skip past this frame
                        continue
                    else:
                        # Try bruteforce RS params
                        payload, rs_errs, desc = rs_decode_ccsds_bruteforce(
                            descrambled)
                        if payload is not None:
                            frames.append({
                                'offset': i,
                                'golay_errs': errs,
                                'length': length,
                                'flags': flags,
                                'rs_errs': rs_errs,
                                'payload': payload,
                                'raw': frame_data,
                                'descrambled': descrambled,
                                'rs_params': desc,
                            })
                            if verbose:
                                print(f"  [DECODED-ALT] offset={i}, len={length}, "
                                      f"golay_errs={errs}, rs_errs={rs_errs}, {desc}")
                            i = end
                            continue

                        # Try without descrambling (some sats disable scrambler)
                        payload, rs_errs = rs_decode_ccsds(frame_data)
                        if payload is not None:
                            frames.append({
                                'offset': i,
                                'golay_errs': errs,
                                'length': length,
                                'flags': flags,
                                'rs_errs': rs_errs,
                                'payload': payload,
                                'raw': frame_data,
                                'descrambled': frame_data,
                                'no_scrambler': True,
                            })
                            if verbose:
                                print(f"  [DECODED-NOSCR] offset={i}, len={length}")
                            i = end
                            continue

                if verbose and length >= 36:
                    print(f"  [golay-ok] offset={i}, len={length}, errs={errs}, "
                          f"flags=0x{flags:X} (RS failed)")
        i += 1
    return frames


# ---------------------------------------------------------------------------
# Fixed-chunk analysis (CC1200 delivered 255-byte blocks)
# ---------------------------------------------------------------------------

def analyze_chunks(raw_data, chunk_size=255, verbose=False):
    """Process raw data as fixed 255-byte chunks from CC1200."""
    n_chunks = len(raw_data) // chunk_size
    remainder = len(raw_data) % chunk_size

    print(f"File: {len(raw_data)} bytes = {n_chunks} chunks of {chunk_size}"
          f" + {remainder} remainder bytes")
    print()

    golay_ok = 0
    golay_fail = 0
    rs_ok = 0
    frames = []

    for idx in range(n_chunks):
        chunk = raw_data[idx * chunk_size:(idx + 1) * chunk_size]
        data12, errs = golay_decode(chunk[:3])

        if data12 is None:
            golay_fail += 1
            if verbose:
                print(f"  chunk {idx:3d}: Golay FAIL  "
                      f"header={chunk[0]:02X} {chunk[1]:02X} {chunk[2]:02X}")
            continue

        length = data12 & 0xFF
        flags = (data12 >> 8) & 0xF
        golay_ok += 1

        if verbose:
            print(f"  chunk {idx:3d}: Golay OK  len={length:3d}  "
                  f"errs={errs}  flags=0x{flags:X}  "
                  f"header={chunk[0]:02X} {chunk[1]:02X} {chunk[2]:02X}")

        if length < 36 or length > 252:
            continue

        # Extract frame data (after 3-byte Golay header, 'length' bytes)
        frame_data = chunk[3:3 + length]

        # Descramble
        descrambled = ccsds_descramble(frame_data)

        # Try RS decode
        payload, rs_errs = rs_decode_ccsds(descrambled)
        if payload is not None:
            rs_ok += 1
            frames.append({
                'chunk': idx,
                'golay_errs': errs,
                'length': length,
                'flags': flags,
                'rs_errs': rs_errs,
                'payload': payload,
            })
            print(f"  *** DECODED chunk {idx}: {len(payload)} bytes, "
                  f"RS corrected {rs_errs} errors")
        else:
            # Try bruteforce
            payload, rs_errs, desc = rs_decode_ccsds_bruteforce(descrambled)
            if payload is not None:
                rs_ok += 1
                frames.append({
                    'chunk': idx,
                    'golay_errs': errs,
                    'length': length,
                    'flags': flags,
                    'rs_errs': rs_errs,
                    'payload': payload,
                    'rs_params': desc,
                })
                print(f"  *** DECODED chunk {idx}: {len(payload)} bytes, "
                      f"RS({desc}) corrected {rs_errs}")

    print()
    print(f"Summary: {golay_ok}/{n_chunks} Golay OK, {rs_ok} RS decoded")
    return frames


# ---------------------------------------------------------------------------
# Cross-capture pattern analysis
# ---------------------------------------------------------------------------

def find_common_patterns(data1, data2, min_len=4):
    """Find byte patterns that appear in both captures."""
    print(f"\nSearching for common {min_len}+ byte patterns...")

    # Build set of all n-grams from data1
    patterns1 = set()
    for length in range(min_len, min(16, min(len(data1), len(data2)))):
        for i in range(len(data1) - length):
            patterns1.add(data1[i:i + length])

    # Check which appear in data2
    matches = []
    for length in range(min_len, min(16, min(len(data1), len(data2)))):
        for i in range(len(data2) - length):
            pat = data2[i:i + length]
            if pat in patterns1:
                # Find where it appears in data1
                pos1 = data1.find(pat)
                matches.append((length, pat, pos1, i))

    if not matches:
        print("  No common patterns found.")
        return

    # Sort by length descending, deduplicate overlapping
    matches.sort(key=lambda x: -x[0])
    seen = set()
    unique = []
    for length, pat, pos1, pos2 in matches:
        key = (pos1, pos2)
        if key not in seen:
            seen.add(key)
            unique.append((length, pat, pos1, pos2))
            if len(unique) >= 20:
                break

    print(f"  Found {len(unique)} unique common patterns (showing top 20):")
    for length, pat, pos1, pos2 in unique[:20]:
        hex_str = pat.hex()
        print(f"    len={length:2d}  data1@{pos1:5d}  data2@{pos2:5d}  "
              f"{hex_str[:32]}{'...' if len(hex_str) > 32 else ''}")


# ---------------------------------------------------------------------------
# Golay header statistics
# ---------------------------------------------------------------------------

def golay_stats(raw_data, chunk_size=255):
    """Analyze Golay decode success rate and length distribution."""
    n_chunks = len(raw_data) // chunk_size
    lengths = []
    errors = []

    for idx in range(n_chunks):
        chunk = raw_data[idx * chunk_size:(idx + 1) * chunk_size]
        data12, errs = golay_decode(chunk[:3])
        if data12 is not None:
            lengths.append(data12 & 0xFF)
            errors.append(errs)

    if not lengths:
        print("No valid Golay headers found in any chunk.")
        return

    print(f"\nGolay Header Statistics ({len(lengths)}/{n_chunks} chunks decoded):")
    print(f"  Error distribution: {Counter(errors).most_common()}")
    print(f"  Length distribution: {Counter(lengths).most_common(10)}")

    # Filter to plausible AX.100 lengths (36-255 = payload + 32 RS parity)
    plausible = [l for l in lengths if 36 <= l <= 255]
    print(f"  Plausible frame lengths (36-255): {len(plausible)}/{len(lengths)}")
    if plausible:
        print(f"  Plausible length values: {Counter(plausible).most_common(10)}")


# ---------------------------------------------------------------------------
# Byte-level entropy analysis per chunk
# ---------------------------------------------------------------------------

def chunk_entropy(raw_data, chunk_size=255):
    """Compute per-chunk Shannon entropy to identify signal vs noise."""
    import math
    n_chunks = len(raw_data) // chunk_size

    print(f"\nPer-chunk entropy (lower = more structured):")
    for idx in range(min(n_chunks, 30)):  # show first 30
        chunk = raw_data[idx * chunk_size:(idx + 1) * chunk_size]
        freq = Counter(chunk)
        ent = -sum((c / chunk_size) * math.log2(c / chunk_size)
                    for c in freq.values())
        bar = '#' * int(ent * 4)
        marker = ' <-- low entropy' if ent < 6.0 else ''
        print(f"  chunk {idx:3d}: H={ent:.2f} {bar}{marker}")
    if n_chunks > 30:
        print(f"  ... ({n_chunks - 30} more chunks)")


# ---------------------------------------------------------------------------
# Signal vs noise diagnosis
# ---------------------------------------------------------------------------

def diagnose(raw_data, filename=""):
    """Determine if capture contains signal or noise."""
    import math

    n = len(raw_data)
    if n == 0:
        print("  Empty file.")
        return "empty"

    # Basic stats
    avg = sum(raw_data) / n
    ones = sum(bin(b).count('1') for b in raw_data) / (n * 8)
    freq = Counter(raw_data)
    ent = -sum((c / n) * math.log2(c / n) for c in freq.values())
    ff_count = freq.get(0xFF, 0)
    zero_count = freq.get(0x00, 0)

    # Check if descrambled data matches ~PN (= noise)
    inv_pn = bytes(b ^ 0xFF for b in _CCSDS_PN[:min(n, 255)])
    pn_match = sum(1 for a, b in zip(raw_data, inv_pn) if a == b)
    pn_pct = pn_match / min(n, 255) * 100

    # Check for AX.25 patterns (alternating bits)
    ax25_bytes = sum(1 for b in raw_data if b in (0x55, 0xAA, 0x7E))
    ax25_pct = ax25_bytes / n * 100

    print(f"  Size: {n} bytes")
    print(f"  Entropy: {ent:.2f} bits/byte")
    print(f"  Avg byte: {avg:.1f}, ones: {ones:.1%}")
    print(f"  0xFF bytes: {ff_count}/{n} ({ff_count/n*100:.0f}%)")
    print(f"  0x00 bytes: {zero_count}/{n} ({zero_count/n*100:.0f}%)")
    print(f"  AX.25 pattern bytes (55/AA/7E): {ax25_bytes}/{n} ({ax25_pct:.0f}%)")

    # Classify
    if zero_count == n:
        verdict = "EMPTY (all zeros — no data captured)"
        category = "empty"
    elif ff_count / n > 0.5:
        verdict = "NOISE (>50% 0xFF — false sync triggers on noise floor)"
        category = "noise"
    elif avg > 200 and ones > 0.75:
        verdict = "LIKELY NOISE (high byte avg, heavy 1-bit bias)"
        category = "noise"
    elif ax25_pct > 5:
        verdict = "AX.25 FRAGMENT (alternating-bit patterns, probably mid-stream capture)"
        category = "ax25"
    elif 100 < avg < 160 and 0.35 < ones < 0.65:
        verdict = "POSSIBLE SIGNAL (byte distribution near random — could be encoded data)"
        category = "signal"
    elif ent > 7.5:
        verdict = "HIGH ENTROPY (near-random, possibly encrypted/compressed/signal)"
        category = "signal"
    else:
        verdict = f"STRUCTURED (H={ent:.1f}, avg={avg:.0f} — real RF capture)"
        category = "structured"

    print(f"  VERDICT: {verdict}")
    return category


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description='AX.100 Mode 5 decoder for CC1200 raw captures')
    parser.add_argument('rawfile', nargs='+', help='Raw capture file(s)')
    parser.add_argument('--scan', action='store_true',
                        help='Byte-by-byte scan (slower, finds misaligned frames)')
    parser.add_argument('--verbose', '-v', action='store_true')
    parser.add_argument('--entropy', action='store_true',
                        help='Show per-chunk entropy analysis')
    parser.add_argument('--patterns', action='store_true',
                        help='Find common patterns between captures')
    parser.add_argument('--stats', action='store_true',
                        help='Show Golay header statistics')
    parser.add_argument('--diagnose', action='store_true',
                        help='Signal vs noise diagnosis')
    parser.add_argument('--all', action='store_true',
                        help='Run all analyses')
    args = parser.parse_args()

    if args.all:
        args.scan = args.entropy = args.patterns = args.stats = True
        args.diagnose = True

    all_data = {}
    all_frames = {}

    for filepath in args.rawfile:
        print(f"\n{'='*70}")
        print(f"FILE: {os.path.basename(filepath)}")
        print(f"{'='*70}")

        with open(filepath, 'rb') as f:
            raw = f.read()
        all_data[filepath] = raw

        if args.diagnose:
            category = diagnose(raw, filepath)
            print()

        print(f"Size: {len(raw)} bytes ({len(raw)/255:.1f} chunks of 255)")

        if args.stats:
            golay_stats(raw)

        if args.entropy:
            chunk_entropy(raw)

        # Fixed-chunk analysis
        print(f"\n--- Fixed 255-byte chunk analysis ---")
        frames_chunk = analyze_chunks(raw, verbose=args.verbose)
        all_frames[filepath] = frames_chunk

        # Byte-by-byte scan
        if args.scan:
            print(f"\n--- Byte-by-byte scan ---")
            frames_scan = scan_for_frames(raw, verbose=args.verbose)
            print(f"Scan found {len(frames_scan)} decoded frames")
            for f in frames_scan:
                print(f"  offset={f['offset']}, len={f['length']}, "
                      f"golay_errs={f['golay_errs']}, rs_errs={f['rs_errs']}")
                print(f"  payload ({len(f['payload'])} bytes): "
                      f"{f['payload'][:32].hex()}")
                if len(f['payload']) > 32:
                    print(f"  ...")

        # Print decoded frames
        for f in frames_chunk:
            print(f"\n--- Decoded frame (chunk {f.get('chunk', '?')}) ---")
            payload = f['payload']
            print(f"  Length: {len(payload)} bytes")
            print(f"  Hex: {payload.hex()}")
            # Try to interpret as CSP header (first 4 bytes)
            if len(payload) >= 4:
                csp_hdr = struct.unpack('>I', payload[:4])[0]
                csp_prio = (csp_hdr >> 30) & 0x3
                csp_src = (csp_hdr >> 25) & 0x1F
                csp_dst = (csp_hdr >> 20) & 0x1F
                csp_dport = (csp_hdr >> 14) & 0x3F
                csp_sport = (csp_hdr >> 8) & 0x3F
                csp_flags = csp_hdr & 0xFF
                print(f"  CSP: prio={csp_prio} src={csp_src} dst={csp_dst} "
                      f"dport={csp_dport} sport={csp_sport} flags=0x{csp_flags:02X}")
                if len(payload) > 4:
                    print(f"  CSP data ({len(payload)-4} bytes): "
                          f"{payload[4:36].hex()}"
                          f"{'...' if len(payload) > 36 else ''}")

    # Cross-capture pattern analysis
    if args.patterns and len(args.rawfile) >= 2:
        paths = list(all_data.keys())
        find_common_patterns(all_data[paths[0]], all_data[paths[1]])

    # Summary
    total_decoded = sum(len(f) for f in all_frames.values())
    print(f"\n{'='*70}")
    print(f"TOTAL: {total_decoded} frames decoded across {len(args.rawfile)} files")
    if total_decoded == 0:
        print("\nNo frames decoded. Possible reasons:")
        print("  1. CC1200 FIXED 255-byte mode captured noise after sync match")
        print("  2. Sync threshold too loose (24 bits instead of 4 errors)")
        print("  3. Frame alignment doesn't match 255-byte boundaries")
        print("  4. Try --scan for byte-by-byte search")
        print("  5. Try --stats to see Golay header decode rates")
        print("  6. Data may need bit-inversion or byte-reversal")
    print(f"{'='*70}")


if __name__ == '__main__':
    main()
