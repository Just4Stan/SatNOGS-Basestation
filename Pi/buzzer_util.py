"""
Standalone buzzer helper — sends one beep command to the RF HAT Pico.

Lightweight: only depends on pyserial. Does NOT import rf_hat.py.
Used by wifi_provision.py and dashboard.py which can't load the full
CC1200Link stack (different process, no persistent serial connection).

Usage:
    from buzzer_util import beep
    beep(0x01)              # READY pattern
    beep(0x06, "/dev/serial0")  # WIFI_OK pattern
"""

import struct

try:
    import serial
except ImportError:
    serial = None

# Pattern IDs — must match firmware (main.cpp buzzer_start)
READY      = 0x01
AOS        = 0x02
LOS        = 0x03
PACKET     = 0x04
ERROR      = 0x05
WIFI_OK    = 0x06
GPS_OK     = 0x07
SETUP_DONE = 0x08

# Protocol constants (subset of rf_hat.py / proto.h)
MSG_BUZZER = 0x50


def _crc16_ccitt(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= (b << 8)
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def _cobs_encode(data: bytes) -> bytes:
    out = bytearray()
    idx = 0
    while idx < len(data):
        block_start = len(out)
        out.append(0)  # placeholder for code byte
        code = 1
        while idx < len(data) and data[idx] != 0 and code < 0xFF:
            out.append(data[idx])
            idx += 1
            code += 1
        if idx < len(data) and data[idx] == 0:
            idx += 1
        out[block_start] = code
    return bytes(out)


def _build_frame(pattern: int) -> bytes:
    """Build a COBS-encoded MSG_BUZZER frame with CRC-16."""
    body = bytes([pattern & 0xFF])
    hdr = struct.pack("<BBH", MSG_BUZZER, 0, len(body))
    crc = _crc16_ccitt(hdr + body)
    payload = hdr + body + struct.pack("<H", crc)
    return _cobs_encode(payload) + b"\x00"


def beep(pattern: int, port: str = "/dev/serial0", baud: int = 115200):
    """Send a single buzzer command to RF HAT. Best-effort, never raises."""
    if serial is None:
        return
    try:
        ser = serial.Serial(port, baud, timeout=0.1)
        ser.write(_build_frame(pattern))
        ser.flush()
        ser.close()
    except Exception:
        pass
