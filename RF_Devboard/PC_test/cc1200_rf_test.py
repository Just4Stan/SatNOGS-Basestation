#!/usr/bin/env python3
"""
cc1200_rf_test.py  -  Bidirectional RF integration test for two CC1200 boards.

Usage:
    python cc1200_rf_test.py --board-a COM3 --board-b COM4
    python cc1200_rf_test.py --board-a COM3 --board-b COM4 --profile smartrf_config.txt --packets 20 --out report.json

Both boards are configured with the same SmartRF profile, then tested in both
directions:
    Phase 1: Board A  -TX->  Board B receives
    Phase 2: Board B  -TX->  Board A receives
"""

import argparse
import json
import math
import re
import struct
import sys
import time
import threading
from dataclasses import dataclass, field, asdict
from pathlib import Path
from typing import List, Optional, Tuple


# ---------------------------------------------------------------------------
# Protocol constants (must match firmware)
# ---------------------------------------------------------------------------
MSG_PING          = 0x01
MSG_GET_INFO      = 0x02
MSG_SET_STATE     = 0x03
MSG_SET_STREAMING = 0x04
MSG_GET_METRICS   = 0x05

MSG_READ_REG      = 0x10
MSG_WRITE_REG     = 0x11
MSG_READ_EXT      = 0x12
MSG_WRITE_EXT     = 0x13

MSG_RX_READ       = 0x20
MSG_STROBE        = 0x21

MSG_TX_WRITE      = 0x22
MSG_TX_SEND       = 0x23
MSG_TX_FLUSH      = 0x24

MSG_SET_FREQ_WORD = 0x34

RSP_FLAG          = 0x80
EVT_RX_DATA       = 0xE0
EVT_ERROR         = 0xE1

APPEND_STATUS_LEN = 2  # CC1200 PKT_CFG1 APPEND_STATUS: RSSI byte + LQI/CRC byte

# CC1200 frequency tuning — extended register space (SWRU346B Table 5)
CC1200_EXTADDR_FREQ2 = 0x0C
CC1200_EXTADDR_FREQ1 = 0x0D
CC1200_EXTADDR_FREQ0 = 0x0E
CC1200_XOSC_HZ    = 40_000_000
CC1200_LO_DIV_UHF = 8          # 410–480 MHz band (FS_CFG.FSD_BANDSELECT = 0b0100)

EXT_RSSI1         = 0x71
EXT_RSSI0         = 0x72
EXT_MARCSTATE     = 0x73
EXT_LQI_VAL       = 0x74
EXT_NUM_TXBYTES   = 0xD6
EXT_NUM_RXBYTES   = 0xD7

STATE_IDLE        = 0
STATE_RX          = 1
STATE_TX          = 2


# ---------------------------------------------------------------------------
# COBS + CRC16
# ---------------------------------------------------------------------------
def cobs_encode(data: bytes) -> bytes:
    out = bytearray()
    out.append(0)
    code_index = 0
    code = 1
    for b in data:
        if b == 0:
            out[code_index] = code
            code_index = len(out)
            out.append(0)
            code = 1
        else:
            out.append(b)
            code += 1
            if code == 0xFF:
                out[code_index] = code
                code_index = len(out)
                out.append(0)
                code = 1
    out[code_index] = code
    return bytes(out)


def cobs_decode(data: bytes) -> bytes:
    out = bytearray()
    i = 0
    n = len(data)
    while i < n:
        code = data[i]
        if code == 0:
            raise ValueError("Bad COBS (code=0)")
        i += 1
        for _ in range(code - 1):
            if i >= n:
                raise ValueError("Bad COBS (overrun)")
            out.append(data[i])
            i += 1
        if code != 0xFF and i < n:
            out.append(0)
    return bytes(out)


def crc16_ccitt_false(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= (b << 8)
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def build_frame(msg_type: int, seq: int, body: bytes) -> bytes:
    hdr = struct.pack("<BBH", msg_type, seq & 0xFF, len(body))
    crc = crc16_ccitt_false(hdr + body)
    payload = hdr + body + struct.pack("<H", crc)
    return cobs_encode(payload) + b"\x00"


def parse_frame(raw_cobs: bytes) -> Optional[Tuple[int, int, bytes]]:
    dec = cobs_decode(raw_cobs)
    if len(dec) < 6:
        return None
    msg_type, seq, ln = struct.unpack_from("<BBH", dec, 0)
    if len(dec) != 4 + ln + 2:
        return None
    body = dec[4:4 + ln]
    rx_crc = struct.unpack_from("<H", dec, 4 + ln)[0]
    if rx_crc != crc16_ccitt_false(dec[:4 + ln]):
        return None
    return msg_type, seq, body


# ---------------------------------------------------------------------------
# Link layer
# ---------------------------------------------------------------------------
@dataclass
class Metrics:
    marc: int = 0
    rxbytes: int = 0
    rssi_raw_1db: int = 0
    rssi_dbm_x10: int = -32768
    last_doppler_retune_us: int = 0
    max_doppler_retune_us: int = 0

    @property
    def rssi_dbm(self) -> Optional[float]:
        if self.rssi_dbm_x10 == -32768:
            return None
        return self.rssi_dbm_x10 / 10.0


@dataclass
class RegWrite:
    is_ext: bool
    addr: int
    value: int
    name: str


class CC1200Link:
    def __init__(self, name: str = "?"):
        self.name = name
        self.ser = None
        self.rxbuf = bytearray()
        self.seq = 1
        self.lock = threading.Lock()
        self.running = False
        self.rx_thread: Optional[threading.Thread] = None
        self.event_queue: List[Tuple[int, bytes]] = []
        self.rsp_data = {}

    def open(self, port: str, baud: int = 115200) -> None:
        import serial
        self.close()
        self.ser = serial.Serial(port, baudrate=baud, timeout=0.02)
        self.running = True
        self.rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self.rx_thread.start()

    def close(self) -> None:
        self.running = False
        if self.rx_thread:
            self.rx_thread.join(timeout=0.5)
            self.rx_thread = None
        if self.ser:
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None
        with self.lock:
            self.rxbuf.clear()
            self.event_queue.clear()
            self.rsp_data.clear()

    def is_open(self) -> bool:
        return self.ser is not None and self.ser.is_open

    def _rx_loop(self):
        while self.running and self.ser:
            try:
                data = self.ser.read(256)
                if data:
                    # Undo Pico USB-CDC CRLF translation: 0x0a -> 0x0d 0x0a
                    data = data.replace(b'\x0d\x0a', b'\x0a')
                    with self.lock:
                        self.rxbuf.extend(data)

                while True:
                    with self.lock:
                        if b"\x00" not in self.rxbuf:
                            break
                        frame, _, rest = self.rxbuf.partition(b"\x00")
                        self.rxbuf = bytearray(rest)

                    if not frame:
                        continue
                    try:
                        parsed = parse_frame(frame)
                    except Exception:
                        parsed = None
                    if not parsed:
                        continue

                    msg_type, seq, body = parsed
                    if msg_type >= 0xE0 or not (msg_type & RSP_FLAG):
                        with self.lock:
                            self.event_queue.append((msg_type, body))
                    else:
                        with self.lock:
                            self.rsp_data[seq] = body

            except Exception:
                time.sleep(0.05)

    def pop_events(self) -> List[Tuple[int, bytes]]:
        with self.lock:
            ev = list(self.event_queue)
            self.event_queue.clear()
        return ev

    def send_cmd(self, msg_type: int, body: bytes = b"", timeout_s: float = 1.0) -> Optional[bytes]:
        if not self.ser:
            return None
        seq = self.seq & 0xFF
        self.seq = (self.seq + 1) & 0xFF
        pkt = build_frame(msg_type, seq, body)
        try:
            self.ser.write(pkt)
            self.ser.flush()
        except Exception:
            return None
        t0 = time.time()
        while time.time() - t0 < timeout_s:
            with self.lock:
                if seq in self.rsp_data:
                    return self.rsp_data.pop(seq)
            time.sleep(0.005)
        return None

    def ping(self) -> bool:
        return self.send_cmd(MSG_PING) is not None

    def get_info(self) -> Optional[Tuple[int, int]]:
        b = self.send_cmd(MSG_GET_INFO)
        if b and len(b) == 2:
            return b[0], b[1]
        return None

    def set_state(self, state: int) -> bool:
        b = self.send_cmd(MSG_SET_STATE, bytes([state & 0xFF]))
        return bool(b and len(b) >= 1 and b[0] == 1)

    def set_streaming(self, on: bool) -> bool:
        b = self.send_cmd(MSG_SET_STREAMING, bytes([1 if on else 0]))
        return bool(b and len(b) >= 1 and b[0] in (0, 1))

    def get_metrics(self) -> Optional[Metrics]:
        b = self.send_cmd(MSG_GET_METRICS)
        if not b:
            return None
        if len(b) == 3:
            return Metrics(marc=b[0], rxbytes=b[1], rssi_raw_1db=struct.unpack_from("<b", b, 2)[0])
        if len(b) == 5:
            return Metrics(marc=b[0], rxbytes=b[1], rssi_raw_1db=struct.unpack_from("<b", b, 2)[0],
                           rssi_dbm_x10=struct.unpack_from("<h", b, 3)[0])
        if len(b) >= 36 and b[0] == 2:
            return Metrics(marc=b[4], rxbytes=b[5],
                           rssi_dbm_x10=struct.unpack_from("<h", b, 8)[0],
                           last_doppler_retune_us=struct.unpack_from("<I", b, 28)[0],
                           max_doppler_retune_us=struct.unpack_from("<I", b, 32)[0])
        if len(b) >= 28 and b[0] == 2:
            return Metrics(marc=b[4], rxbytes=b[5],
                           rssi_dbm_x10=struct.unpack_from("<h", b, 8)[0])
        return None

    def read_reg(self, addr: int) -> Optional[int]:
        b = self.send_cmd(MSG_READ_REG, bytes([addr & 0xFF]))
        return b[1] if (b and len(b) == 2 and b[0] == 1) else None

    def write_reg(self, addr: int, val: int) -> bool:
        b = self.send_cmd(MSG_WRITE_REG, bytes([addr & 0xFF, val & 0xFF]))
        return bool(b and len(b) >= 1 and b[0] == 1)

    def read_ext(self, addr: int) -> Optional[int]:
        b = self.send_cmd(MSG_READ_EXT, bytes([addr & 0xFF]))
        return b[1] if (b and len(b) == 2 and b[0] == 1) else None

    def write_ext(self, addr: int, val: int) -> bool:
        b = self.send_cmd(MSG_WRITE_EXT, bytes([addr & 0xFF, val & 0xFF]))
        return bool(b and len(b) >= 1 and b[0] == 1)

    def tx_flush(self) -> bool:
        b = self.send_cmd(MSG_TX_FLUSH)
        return bool(b and len(b) >= 1 and b[0] == 1)

    def tx_write(self, data: bytes, flush_first: bool = True, force_idle: bool = True) -> Tuple[bool, int]:
        flags = 0
        if flush_first:
            flags |= 0x01
        if force_idle:
            flags |= 0x02
        if len(data) > 255:
            data = data[:255]
        body = bytes([flags, len(data)]) + data
        b = self.send_cmd(MSG_TX_WRITE, body, timeout_s=1.0)
        if b and len(b) == 2 and b[0] in (0, 1):
            return (b[0] == 1), int(b[1])
        return False, 0

    def tx_send(self, go_idle: bool = False, go_rx: bool = False, flush_before: bool = False) -> bool:
        flags = 0
        if go_idle:
            flags |= 0x01
        if go_rx:
            flags |= 0x02
        if flush_before:
            flags |= 0x04
        b = self.send_cmd(MSG_TX_SEND, bytes([flags]), timeout_s=1.0)
        return bool(b and len(b) >= 1 and b[0] == 1)


# ---------------------------------------------------------------------------
# SmartRF profile loading
# ---------------------------------------------------------------------------
EXPORT_LINE_RE = re.compile(r'^\s*(0x[0-9A-Fa-f]{1,2})\s*,\s*//\s*([A-Z0-9_]+)\b')


def load_regmap_json(path: str) -> dict:
    with open(path, "r", encoding="utf-8") as f:
        data = json.load(f)
    return {k: int(v) for k, v in data.items()}


def compile_smartrf_config(export_path: str, regmap: dict) -> Tuple[List[RegWrite], str]:
    with open(export_path, "r", encoding="utf-8", errors="ignore") as f:
        lines = f.readlines()
    writes: List[RegWrite] = []
    seen = kept = ignored = 0
    for line in lines:
        m = EXPORT_LINE_RE.match(line)
        if not m:
            continue
        seen += 1
        val = int(m.group(1), 16)
        name = m.group(2).strip()
        ti_addr = regmap.get(name)
        if ti_addr is None:
            ignored += 1
            continue
        is_ext = ti_addr >= 0x2F00
        writes.append(RegWrite(is_ext=is_ext, addr=ti_addr & 0xFF, value=val, name=name))
        kept += 1
    return writes, f"parsed {seen} lines, compiled {kept} writes, ignored {ignored}"


def apply_profile_to_board(link: CC1200Link, writes: List[RegWrite]) -> Tuple[bool, str]:
    if not link.set_state(STATE_IDLE):
        return False, "failed to enter IDLE before profile apply"
    time.sleep(0.05)
    for w in writes:
        ok = False
        for _ in range(3):
            ok = link.write_ext(w.addr, w.value) if w.is_ext else link.write_reg(w.addr, w.value)
            if ok:
                break
            time.sleep(0.005)
        if not ok:
            space = "EXT" if w.is_ext else "REG"
            return False, f"{space} write failed at 0x{w.addr:02X}={w.value:02X} ({w.name})"
        time.sleep(0.001)
    return True, f"applied {len(writes)} register writes"


def freq_mhz_to_word(freq_mhz: float) -> Tuple[int, int, int]:
    """Compute CC1200 FREQ2/FREQ1/FREQ0 for UHF band (LO_DIV=8, XOSC=40 MHz)."""
    word = round(freq_mhz * 1e6 * CC1200_LO_DIV_UHF / CC1200_XOSC_HZ * 65536) & 0xFFFFFF
    return (word >> 16) & 0xFF, (word >> 8) & 0xFF, word & 0xFF


def set_freq_mhz(link: CC1200Link, freq_mhz: float) -> bool:
    """Tune CC1200 to freq_mhz by directly writing FREQ2/1/0 (radio should be IDLE)."""
    f2, f1, f0 = freq_mhz_to_word(freq_mhz)
    return (
        link.write_ext(CC1200_EXTADDR_FREQ2, f2) and
        link.write_ext(CC1200_EXTADDR_FREQ1, f1) and
        link.write_ext(CC1200_EXTADDR_FREQ0, f0)
    )


# ---------------------------------------------------------------------------
# Test logic
# ---------------------------------------------------------------------------
@dataclass
class PacketResult:
    index: int
    payload_hex: str
    received: bool
    payload_match: bool
    received_hex: str
    rssi_dbm: Optional[float]
    lqi: Optional[int]
    latency_ms: Optional[float]


@dataclass
class PhaseResult:
    tx_board: str
    rx_board: str
    packets_sent: int
    packets_received: int
    packets_matched: int
    rssi_values: List[float] = field(default_factory=list)
    lqi_values: List[int] = field(default_factory=list)
    latency_values: List[float] = field(default_factory=list)
    per_packet: List[PacketResult] = field(default_factory=list)

    @property
    def packet_loss_pct(self) -> float:
        if self.packets_sent == 0:
            return 100.0
        return 100.0 * (self.packets_sent - self.packets_received) / self.packets_sent

    @property
    def rssi_avg(self) -> Optional[float]:
        return sum(self.rssi_values) / len(self.rssi_values) if self.rssi_values else None

    @property
    def rssi_min(self) -> Optional[float]:
        return min(self.rssi_values) if self.rssi_values else None

    @property
    def rssi_max(self) -> Optional[float]:
        return max(self.rssi_values) if self.rssi_values else None

    @property
    def lqi_avg(self) -> Optional[float]:
        return sum(self.lqi_values) / len(self.lqi_values) if self.lqi_values else None

    @property
    def lqi_min(self) -> Optional[int]:
        return min(self.lqi_values) if self.lqi_values else None

    @property
    def lqi_max(self) -> Optional[int]:
        return max(self.lqi_values) if self.lqi_values else None

    @property
    def latency_avg(self) -> Optional[float]:
        return sum(self.latency_values) / len(self.latency_values) if self.latency_values else None

    @property
    def latency_min(self) -> Optional[float]:
        return min(self.latency_values) if self.latency_values else None

    @property
    def latency_max(self) -> Optional[float]:
        return max(self.latency_values) if self.latency_values else None

    @property
    def latency_stddev(self) -> Optional[float]:
        if len(self.latency_values) < 2:
            return None
        avg = self.latency_avg
        return math.sqrt(sum((x - avg) ** 2 for x in self.latency_values) / len(self.latency_values))


def wait_for_rx_event(
    link: CC1200Link, timeout_s: float, expected_len: int = 0
) -> Tuple[Optional[bytes], Optional[bytes]]:
    """Accumulate EVT_RX_DATA chunks until expected_len + APPEND_STATUS bytes received or timeout.

    Returns (payload_bytes, append_status_bytes) where append_status_bytes is the
    2 CC1200 status bytes (RSSI + LQI/CRC_OK) appended after each packet.
    """
    drain_len = expected_len + APPEND_STATUS_LEN if expected_len else 0
    buf = bytearray()
    t0 = time.time()
    while time.time() - t0 < timeout_s:
        for (evt_type, body) in link.pop_events():
            if evt_type == EVT_RX_DATA and body:
                n = body[0]
                buf.extend(body[1:1 + n])
        if drain_len and len(buf) >= drain_len:
            return bytes(buf[:expected_len]), bytes(buf[expected_len:drain_len])
        if not expected_len and buf:
            return bytes(buf), b""
        time.sleep(0.01)
    # Timeout: return whatever payload arrived; status bytes may not have arrived yet
    if expected_len and len(buf) >= expected_len:
        return bytes(buf[:expected_len]), bytes(buf[expected_len:expected_len + APPEND_STATUS_LEN])
    return (bytes(buf) if buf else None), b""


def run_phase(
    tx: CC1200Link,
    rx: CC1200Link,
    payloads: List[bytes],
    rx_timeout_s: float,
    inter_packet_s: float,
) -> PhaseResult:
    result = PhaseResult(
        tx_board=tx.name,
        rx_board=rx.name,
        packets_sent=len(payloads),
        packets_received=0,
        packets_matched=0,
    )

    # Prepare RX board
    rx.pop_events()
    if not rx.set_state(STATE_IDLE):
        log(f"  [WARN] {rx.name}: failed to enter IDLE before RX setup")
    if not rx.set_streaming(True):
        log(f"  [WARN] {rx.name}: SET_STREAMING failed")
    if not rx.set_state(STATE_RX):
        log(f"  [WARN] {rx.name}: failed to enter RX")

    # Prepare TX board
    if not tx.set_state(STATE_IDLE):
        log(f"  [WARN] {tx.name}: failed to enter IDLE before TX")

    time.sleep(0.1)

    for i, payload in enumerate(payloads):
        # Variable-length mode: first byte in TX FIFO is the packet length
        framed = bytes([len(payload)]) + payload
        ok_write, written = tx.tx_write(framed, flush_first=True, force_idle=True)
        if not ok_write:
            log(f"  [{tx.name}] pkt {i+1}: TX_WRITE failed")
            result.per_packet.append(PacketResult(
                index=i + 1, payload_hex=payload.hex(" "),
                received=False, payload_match=False, received_hex="",
                rssi_dbm=None, lqi=None, latency_ms=None,
            ))
            continue

        t_send = time.time()
        ok_send = tx.tx_send(go_idle=False, go_rx=False)
        if not ok_send:
            log(f"  [{tx.name}] pkt {i+1}: TX_SEND failed")
            result.per_packet.append(PacketResult(
                index=i + 1, payload_hex=payload.hex(" "),
                received=False, payload_match=False, received_hex="",
                rssi_dbm=None, lqi=None, latency_ms=None,
            ))
            continue

        # Accumulate chunks; +1 for the leading length byte the radio prepends on RX
        rx_data, append_status = wait_for_rx_event(rx, timeout_s=rx_timeout_s, expected_len=len(payload) + 1)
        latency_ms = (time.time() - t_send) * 1000.0 if rx_data is not None else None

        # Strip leading length byte before comparing to sent payload
        rx_payload = rx_data[1:] if (rx_data and len(rx_data) > 1) else b""
        received = len(rx_payload) > 0
        matched = received and (rx_payload == payload)

        # Extract LQI from APPEND_STATUS byte 2 (bits [6:0]).
        # CC1200 LQI is an error metric: 0 = best link, higher = worse (SWRU346B §6.13).
        lqi = (append_status[1] & 0x7F) if len(append_status) >= 2 else None
        if lqi is not None:
            result.lqi_values.append(lqi)
        if latency_ms is not None:
            result.latency_values.append(latency_ms)

        # Read RSSI from RX board after packet
        metrics = rx.get_metrics()
        rssi = metrics.rssi_dbm if metrics else None
        if rssi is not None:
            result.rssi_values.append(rssi)

        if received:
            result.packets_received += 1
        if matched:
            result.packets_matched += 1

        status = "OK" if matched else ("RX_MISMATCH" if received else "LOST")
        rssi_str = f"{rssi:.1f} dBm" if rssi is not None else "n/a"
        lqi_str = f"{lqi}" if lqi is not None else "n/a"
        lat_str = f"{latency_ms:.0f} ms" if latency_ms is not None else "-"
        log(f"  [{tx.name}->{rx.name}] pkt {i+1}/{len(payloads)}: {status}  "
            f"RSSI={rssi_str}  LQI={lqi_str}  latency={lat_str}")

        result.per_packet.append(PacketResult(
            index=i + 1,
            payload_hex=payload.hex(" "),
            received=received,
            payload_match=matched,
            received_hex=rx_payload.hex(" ") if rx_payload else "",
            rssi_dbm=rssi,
            lqi=lqi,
            latency_ms=latency_ms,
        ))

        if inter_packet_s > 0:
            time.sleep(inter_packet_s)

    # Put both boards back to idle
    rx.set_streaming(False)
    rx.set_state(STATE_IDLE)
    tx.set_state(STATE_IDLE)

    return result


# ---------------------------------------------------------------------------
# Console helpers
# ---------------------------------------------------------------------------
_log_lines: List[str] = []


def log(msg: str):
    print(msg)
    _log_lines.append(msg)


def print_phase_summary(phase: PhaseResult):
    log(f"\n  {'-'*56}")
    log(f"  Direction : {phase.tx_board} -> {phase.rx_board}")
    log(f"  Sent      : {phase.packets_sent}")
    log(f"  Received  : {phase.packets_received}  (loss {phase.packet_loss_pct:.1f}%)")
    log(f"  Matched   : {phase.packets_matched}")
    if phase.rssi_values:
        log(f"  RSSI      : avg={phase.rssi_avg:.1f}  min={phase.rssi_min:.1f}  max={phase.rssi_max:.1f}  dBm")
    else:
        log(f"  RSSI      : n/a")
    if phase.lqi_values:
        log(f"  LQI       : avg={phase.lqi_avg:.1f}  min={phase.lqi_min}  max={phase.lqi_max}  (0=best, 127=worst)")
    else:
        log(f"  LQI       : n/a")
    if phase.latency_values:
        log(f"  Latency   : avg={phase.latency_avg:.1f}  min={phase.latency_min:.1f}  "
            f"max={phase.latency_max:.1f}  stddev={phase.latency_stddev:.1f}  ms")
    else:
        log(f"  Latency   : n/a")


# ---------------------------------------------------------------------------
# Report helpers
# ---------------------------------------------------------------------------
def phase_to_dict(p: PhaseResult) -> dict:
    return {
        "tx_board": p.tx_board,
        "rx_board": p.rx_board,
        "packets_sent": p.packets_sent,
        "packets_received": p.packets_received,
        "packets_matched": p.packets_matched,
        "packet_loss_pct": round(p.packet_loss_pct, 2),
        "rssi_avg_dbm": round(p.rssi_avg, 2) if p.rssi_avg is not None else None,
        "rssi_min_dbm": round(p.rssi_min, 2) if p.rssi_min is not None else None,
        "rssi_max_dbm": round(p.rssi_max, 2) if p.rssi_max is not None else None,
        "lqi_avg": round(p.lqi_avg, 1) if p.lqi_avg is not None else None,
        "lqi_min": p.lqi_min,
        "lqi_max": p.lqi_max,
        "latency_avg_ms": round(p.latency_avg, 2) if p.latency_avg is not None else None,
        "latency_min_ms": round(p.latency_min, 2) if p.latency_min is not None else None,
        "latency_max_ms": round(p.latency_max, 2) if p.latency_max is not None else None,
        "latency_stddev_ms": round(p.latency_stddev, 2) if p.latency_stddev is not None else None,
        "per_packet": [asdict(r) for r in p.per_packet],
    }


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------
def build_payloads(count: int, size: int) -> List[bytes]:
    payloads = []
    for i in range(count):
        # Each packet: 2-byte index, then incrementing bytes to fill size
        idx = i & 0xFFFF
        base = bytes([idx >> 8, idx & 0xFF])
        body = bytes([(idx + j) & 0xFF for j in range(size - 2)]) if size > 2 else b""
        payloads.append((base + body)[:size])
    return payloads


def main():
    parser = argparse.ArgumentParser(description="CC1200 bidirectional RF integration test")
    parser.add_argument("--board-a", required=True, metavar="PORT", help="Serial port for board A (e.g. COM3)")
    parser.add_argument("--board-b", required=True, metavar="PORT", help="Serial port for board B (e.g. COM4)")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--profile", metavar="FILE", help="SmartRF Studio config.txt export to apply to both boards")
    parser.add_argument("--regmap", metavar="FILE", help="cc1200_regmap_profile.json (auto-detected if omitted)")
    parser.add_argument("--packets", type=int, default=100, help="Packets to send per direction (default 100)")
    parser.add_argument("--size", type=int, default=16, help="Payload size in bytes (default 16, max 255)")
    parser.add_argument("--timeout", type=float, default=1.0, help="Per-packet RX timeout in seconds (default 1.0)")
    parser.add_argument("--gap", type=float, default=0.2, help="Inter-packet gap in seconds (default 0.2)")
    parser.add_argument("--out", metavar="FILE", help="Write JSON report to this file")
    parser.add_argument("--notes", metavar="TEXT", default="", help="Free-text test conditions note written into the report")
    parser.add_argument("--freqs", type=float, nargs="+", metavar="MHz", default=[435.0],
                        help="Frequencies to sweep in MHz (default: 435). Example: --freqs 430 435 437 440")
    args = parser.parse_args()

    args.size = max(2, min(255, args.size))

    # Locate regmap
    regmap_path = args.regmap
    if not regmap_path:
        default = Path(__file__).parent / "cc1200_regmap_profile.json"
        if default.exists():
            regmap_path = str(default)

    # -----------------------------------------------------------------------
    log("=" * 60)
    log("  CC1200 Bidirectional RF Integration Test")
    log("=" * 60)
    log(f"  Board A : {args.board_a}")
    log(f"  Board B : {args.board_b}")
    log(f"  Packets : {args.packets} per direction  |  Size: {args.size} bytes")
    log(f"  RX timeout: {args.timeout} s  |  Gap: {args.gap} s")
    log(f"  Freqs     : {', '.join(f'{f} MHz' for f in args.freqs)}")

    import serial

    board_a = CC1200Link(name="A")
    board_b = CC1200Link(name="B")

    try:
        # ------------------------------------------------------------------
        # Connect
        # ------------------------------------------------------------------
        log("\n[1] Connecting to boards...")
        try:
            board_a.open(args.board_a, args.baud)
        except Exception as e:
            log(f"  FAIL: cannot open {args.board_a}: {e}")
            sys.exit(1)
        try:
            board_b.open(args.board_b, args.baud)
        except Exception as e:
            log(f"  FAIL: cannot open {args.board_b}: {e}")
            sys.exit(1)

        time.sleep(0.2)

        # Ping
        for name, lnk in [("A", board_a), ("B", board_b)]:
            if not lnk.ping():
                log(f"  FAIL: board {name} ({lnk.ser.port}) did not respond to PING")
                sys.exit(1)
            log(f"  Board {name}: PING OK")

        # GET_INFO
        board_info = {}
        for name, lnk in [("A", board_a), ("B", board_b)]:
            info = lnk.get_info()
            if info:
                part, ver = info
                board_info[name] = {"part": f"0x{part:02X}", "version": f"0x{ver:02X}"}
                log(f"  Board {name}: PART=0x{part:02X}  VER=0x{ver:02X}")
            else:
                board_info[name] = {}
                log(f"  Board {name}: GET_INFO failed (continuing)")

        # ------------------------------------------------------------------
        # Apply SmartRF profile (if provided)
        # ------------------------------------------------------------------
        profile_summary = "none"
        profile_writes: List[RegWrite] = []

        if args.profile:
            log(f"\n[2] Loading SmartRF profile: {args.profile}")
            if not regmap_path:
                log("  FAIL: regmap JSON not found. Pass --regmap or place cc1200_regmap_profile.json next to this script.")
                sys.exit(1)

            regmap = load_regmap_json(regmap_path)
            writes, summary = compile_smartrf_config(args.profile, regmap)
            profile_writes = writes
            log(f"  Profile: {summary}")
            profile_summary = summary

            for name, lnk in [("A", board_a), ("B", board_b)]:
                log(f"  Applying to board {name}...")
                ok, msg = apply_profile_to_board(lnk, writes)
                if not ok:
                    log(f"  FAIL on board {name}: {msg}")
                    sys.exit(1)
                log(f"  Board {name}: {msg}")

        else:
            log("\n[2] No profile specified -- using existing board configuration")

        # ------------------------------------------------------------------
        # Build test payloads (shared across all frequencies)
        # ------------------------------------------------------------------
        payloads = build_payloads(args.packets, args.size)

        # ------------------------------------------------------------------
        # Frequency sweep
        # ------------------------------------------------------------------
        freq_results = []

        for fi, freq_mhz in enumerate(args.freqs):
            log(f"\n{'='*60}")
            log(f"  FREQUENCY {fi + 1}/{len(args.freqs)}: {freq_mhz} MHz")
            log(f"{'='*60}")

            # Re-apply full profile to reset all registers, then override FREQ
            if profile_writes:
                log(f"  Re-applying profile...")
                for name, lnk in [("A", board_a), ("B", board_b)]:
                    ok, msg = apply_profile_to_board(lnk, profile_writes)
                    if not ok:
                        log(f"  FAIL on board {name}: {msg}")
                        sys.exit(1)

            log(f"  Tuning to {freq_mhz} MHz...")
            for name, lnk in [("A", board_a), ("B", board_b)]:
                if not set_freq_mhz(lnk, freq_mhz):
                    log(f"  FAIL: could not tune board {name} to {freq_mhz} MHz")
                    sys.exit(1)
            log(f"  Tuned OK")
            time.sleep(0.2)

            # Phase 1: A -> B
            log(f"\n  Phase 1: A -> B  ({args.packets} pkts @ {freq_mhz} MHz)")
            phase1 = run_phase(board_a, board_b, payloads, args.timeout, args.gap)
            print_phase_summary(phase1)

            time.sleep(0.5)

            # Phase 2: B -> A
            log(f"\n  Phase 2: B -> A  ({args.packets} pkts @ {freq_mhz} MHz)")
            phase2 = run_phase(board_b, board_a, payloads, args.timeout, args.gap)
            print_phase_summary(phase2)

            # Per-frequency totals
            f_sent = phase1.packets_sent + phase2.packets_sent
            f_rx   = phase1.packets_received + phase2.packets_received
            f_ok   = phase1.packets_matched + phase2.packets_matched
            f_loss = 100.0 * (f_sent - f_rx) / f_sent if f_sent else 100.0
            f_pass = (f_rx == f_sent and f_ok == f_sent)

            all_rssi = phase1.rssi_values + phase2.rssi_values
            all_lqi  = phase1.lqi_values  + phase2.lqi_values

            log(f"\n  {freq_mhz} MHz: {f_rx}/{f_sent} received  {f_ok} matched  "
                f"{'PASS' if f_pass else 'FAIL'}")
            if all_rssi:
                log(f"  RSSI avg={sum(all_rssi)/len(all_rssi):.1f}  "
                    f"min={min(all_rssi):.1f}  max={max(all_rssi):.1f}  dBm")
            if all_lqi:
                log(f"  LQI  avg={sum(all_lqi)/len(all_lqi):.1f}  "
                    f"min={min(all_lqi)}  max={max(all_lqi)}  (0=best, 127=worst)")

            freq_results.append({
                "freq_mhz": freq_mhz,
                "phase1_A_to_B": phase_to_dict(phase1),
                "phase2_B_to_A": phase_to_dict(phase2),
                "overall": {
                    "total_sent": f_sent,
                    "total_received": f_rx,
                    "total_matched": f_ok,
                    "packet_loss_pct": round(f_loss, 2),
                    "rssi_avg_dbm": round(sum(all_rssi) / len(all_rssi), 2) if all_rssi else None,
                    "rssi_min_dbm": round(min(all_rssi), 2) if all_rssi else None,
                    "rssi_max_dbm": round(max(all_rssi), 2) if all_rssi else None,
                    "lqi_avg": round(sum(all_lqi) / len(all_lqi), 1) if all_lqi else None,
                    "lqi_min": min(all_lqi) if all_lqi else None,
                    "lqi_max": max(all_lqi) if all_lqi else None,
                    "passed": f_pass,
                },
            })

        # ------------------------------------------------------------------
        # Overall summary across all frequencies
        # ------------------------------------------------------------------
        total_sent = sum(r["overall"]["total_sent"] for r in freq_results)
        total_rx   = sum(r["overall"]["total_received"] for r in freq_results)
        total_ok   = sum(r["overall"]["total_matched"] for r in freq_results)
        total_loss = 100.0 * (total_sent - total_rx) / total_sent if total_sent else 100.0
        all_passed = all(r["overall"]["passed"] for r in freq_results)

        log(f"\n{'='*60}")
        log("  OVERALL SUMMARY")
        log(f"{'='*60}")
        log(f"  Frequencies tested     : {', '.join(f'{f} MHz' for f in args.freqs)}")
        log(f"  Total packets sent     : {total_sent}")
        log(f"  Total packets received : {total_rx}  (loss {total_loss:.1f}%)")
        log(f"  Total payload matches  : {total_ok}")
        log(f"\n  RESULT: {'PASS' if all_passed else 'FAIL'}")
        log(f"{'='*60}\n")

        # ------------------------------------------------------------------
        # JSON report
        # ------------------------------------------------------------------
        if args.out:
            report = {
                "timestamp": time.strftime("%Y-%m-%dT%H:%M:%S"),
                "notes": args.notes,
                "board_a_port": args.board_a,
                "board_b_port": args.board_b,
                "board_info": board_info,
                "profile": args.profile or "none",
                "profile_summary": profile_summary,
                "packet_count": args.packets,
                "payload_size": args.size,
                "frequency_sweep": freq_results,
                "overall": {
                    "total_sent": total_sent,
                    "total_received": total_rx,
                    "total_matched": total_ok,
                    "packet_loss_pct": round(total_loss, 2),
                    "passed": all_passed,
                },
            }
            with open(args.out, "w", encoding="utf-8") as f:
                json.dump(report, f, indent=2)
            log(f"  Report saved to: {args.out}")

    finally:
        board_a.close()
        board_b.close()


if __name__ == "__main__":
    main()
