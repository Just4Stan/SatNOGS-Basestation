#!/usr/bin/env python3
"""
RF HAT Protocol Library — talks to the RF HAT Pico over UART (/dev/serial0).

Extracted from Robbe Lehaen's cc1200_gui.py, adapted for Pi UART operation
and dual-radio (UHF RX + VHF TX) ground station use.

Usage:
    from rf_hat import CC1200Link, RADIO_UHF, RADIO_VHF

    link = CC1200Link()
    link.open("/dev/serial0")
    link.select_radio(RADIO_UHF)
    link.apply_smartrf_config("configs/smartrf_uhf_435.txt")
    link.set_state(STATE_RX)
    link.set_streaming(True)
    # ... poll link.pop_events() for EVT_RX_DATA ...
"""

import re
import json
import math
import time
import struct
import threading
from pathlib import Path
from dataclasses import dataclass
from typing import Optional, Tuple, List

import serial

# ---------------------------------------------------------------------------
# Protocol constants (must match firmware proto.h)
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

MSG_PROFILE_CLEAR = 0x30
MSG_PROFILE_BEGIN = 0x31
MSG_PROFILE_CHUNK = 0x32
MSG_PROFILE_APPLY = 0x33
MSG_SET_FREQ_WORD = 0x34

MSG_SET_SERIAL_MODE = 0x35

MSG_SELECT_RADIO  = 0x40

MSG_BUZZER        = 0x50

RSP_FLAG          = 0x80

EVT_RX_DATA       = 0xE0
EVT_ERROR         = 0xE1

STATE_IDLE = 0
STATE_RX   = 1
STATE_TX   = 2

RADIO_UHF = 0  # CC1200 #0 on SPI1 (432 MHz)
RADIO_VHF = 1  # CC1200 #1 on SPI0 (144 MHz)

# CC1200 extended register addresses (status)
EXT_RSSI1       = 0x71
EXT_RSSI0       = 0x72
EXT_MARCSTATE   = 0x73
EXT_LQI_VAL     = 0x74
EXT_FREQOFF_EST1 = 0x77
EXT_FREQOFF_EST0 = 0x78
EXT_NUM_RXBYTES = 0xD7

# CC1200 frequency registers (extended)
EXT_FREQ2       = 0x0C
EXT_FREQ1       = 0x0D
EXT_FREQ0       = 0x0E
EXT_FS_CFG      = None  # FS_CFG is a normal register at addr 0x21

# Crystal frequency (RF HAT uses 40 MHz)
F_XOSC_HZ = 40_000_000

# CC1200 modulation-critical register addresses (normal space)
REG_SYNC3         = 0x04
REG_SYNC2         = 0x05
REG_SYNC1         = 0x06
REG_SYNC0         = 0x07
REG_SYNC_CFG1     = 0x08
REG_DEVIATION_M   = 0x0A
REG_MODCFG_DEV_E  = 0x0B
REG_CHAN_BW        = 0x10
REG_SYMBOL_RATE2   = 0x13
REG_SYMBOL_RATE1   = 0x14
REG_SYMBOL_RATE0   = 0x15
REG_PKT_CFG2      = 0x26
REG_PKT_CFG1      = 0x27
REG_PKT_CFG0      = 0x28
REG_RFEND_CFG1    = 0x29
REG_RFEND_CFG0    = 0x2A
REG_PKT_LEN       = 0x2E
REG_FS_CFG        = 0x21

# Modulation format encoding (CC1200 datasheet, MODCFG_DEV_E[5:3])
MOD_FORMAT_MAP = {
    "2-FSK":  0b001,
    "2-GFSK": 0b000,
    "ASK":    0b011,
    "OOK":    0b011,
    "4-FSK":  0b101,
    "4-GFSK": 0b100,
}

# Band select → LO divider mapping (CC1200 datasheet Table 12)
BAND_SELECT = {
    0x02: {"lo_div": 4,  "f_min": 820e6, "f_max": 960e6,  "label": "820-960 MHz"},
    0x04: {"lo_div": 8,  "f_min": 410e6, "f_max": 480e6,  "label": "410-480 MHz"},
    0x06: {"lo_div": 12, "f_min": 273e6, "f_max": 320e6,  "label": "273-320 MHz"},
    0x0A: {"lo_div": 20, "f_min": 164e6, "f_max": 192e6,  "label": "164-192 MHz"},
    0x0B: {"lo_div": 24, "f_min": 136e6, "f_max": 160e6,  "label": "136-160 MHz"},
}


# ---------------------------------------------------------------------------
# COBS + CRC-16 (identical to firmware)
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
    try:
        dec = cobs_decode(raw_cobs)
    except ValueError:
        return None
    if len(dec) < 6:
        return None
    msg_type, seq, ln = struct.unpack_from("<BBH", dec, 0)
    if len(dec) != 4 + ln + 2:
        return None
    body = dec[4:4 + ln]
    rx_crc = struct.unpack_from("<H", dec, 4 + ln)[0]
    calc = crc16_ccitt_false(dec[:4 + ln])
    if rx_crc != calc:
        return None
    return msg_type, seq, body


# ---------------------------------------------------------------------------
# Frequency helpers
# ---------------------------------------------------------------------------
def band_for_freq(freq_hz: float) -> Optional[int]:
    """Return the FSD_BANDSELECT value for a given frequency."""
    for bs, info in BAND_SELECT.items():
        if info["f_min"] <= freq_hz <= info["f_max"]:
            return bs
    return None


def freq_to_regs(freq_hz: float) -> Tuple[int, int, int, int]:
    """
    Compute CC1200 frequency register values for a given frequency.
    Returns (fs_cfg, freq2, freq1, freq0).

    fs_cfg: FS_CFG register value (normal reg 0x21)
    freq2/1/0: FREQ registers (extended 0x0C/0x0D/0x0E)
    """
    bs = band_for_freq(freq_hz)
    if bs is None:
        raise ValueError(f"Frequency {freq_hz/1e6:.3f} MHz not in any CC1200 band")
    lo_div = BAND_SELECT[bs]["lo_div"]
    # FS_CFG: bit 4 = FS_LOCK_EN (1), bits 3:0 = FSD_BANDSELECT
    fs_cfg = 0x10 | bs
    # FREQ = round(f_rf * lo_div * 2^16 / f_xosc)
    freq_word = round(freq_hz * lo_div * 65536 / F_XOSC_HZ)
    freq2 = (freq_word >> 16) & 0xFF
    freq1 = (freq_word >> 8) & 0xFF
    freq0 = freq_word & 0xFF
    return fs_cfg, freq2, freq1, freq0


def regs_to_freq(fs_cfg: int, freq2: int, freq1: int, freq0: int) -> float:
    """Decode CC1200 frequency registers back to Hz."""
    bs = fs_cfg & 0x0F
    info = BAND_SELECT.get(bs)
    if info is None:
        return 0.0
    lo_div = info["lo_div"]
    freq_word = (freq2 << 16) | (freq1 << 8) | freq0
    return freq_word * F_XOSC_HZ / (lo_div * 65536)


def symbol_rate_to_regs(bps: float) -> Tuple[int, int, int]:
    """
    Compute CC1200 SYMBOL_RATE2/1/0 register values from symbol rate in bps.

    Formula (CC1200 datasheet section 8.7.1):
        rate = (2^20 + SRATE_M) * 2^SRATE_E / 2^39 * f_xosc

    Where SRATE_E is bits [7:4] of SYMBOL_RATE2,
          SRATE_M is {SYMBOL_RATE2[3:0], SYMBOL_RATE1, SYMBOL_RATE0} (20 bits).

    Returns (SYMBOL_RATE2, SYMBOL_RATE1, SYMBOL_RATE0).
    """
    # Find the best exponent
    best_e = 0
    best_m = 0
    best_err = float("inf")

    for e in range(16):
        # SRATE_M = rate * 2^39 / (2^SRATE_E * f_xosc) - 2^20
        m_float = bps * (2 ** 39) / ((2 ** e) * F_XOSC_HZ) - (2 ** 20)
        m = int(round(m_float))
        m = max(0, min(0xFFFFF, m))  # clamp to 20 bits

        # Compute actual rate for error check
        actual = ((2 ** 20) + m) * (2 ** e) * F_XOSC_HZ / (2 ** 39)
        err = abs(actual - bps)
        if err < best_err:
            best_err = err
            best_e = e
            best_m = m

    sr2 = ((best_e & 0xF) << 4) | ((best_m >> 16) & 0xF)
    sr1 = (best_m >> 8) & 0xFF
    sr0 = best_m & 0xFF
    return sr2, sr1, sr0


def deviation_to_regs(hz: float) -> Tuple[int, int]:
    """
    Compute CC1200 DEVIATION_M and DEV_E from deviation frequency in Hz.

    Formula (CC1200 datasheet section 8.7.2):
        f_dev = f_xosc / 2^24 * (256 + DEV_M) * 2^DEV_E

    Where DEVIATION_M register = DEV_M (8 bits, addr 0x0A),
          DEV_E = MODCFG_DEV_E[2:0] (3 bits in reg 0x0B).

    Returns (DEVIATION_M, DEV_E).
    """
    best_e = 0
    best_m = 0
    best_err = float("inf")

    for e in range(8):
        # DEV_M = f_dev * 2^24 / (f_xosc * 2^DEV_E) - 256
        m_float = hz * (2 ** 24) / (F_XOSC_HZ * (2 ** e)) - 256
        m = int(round(m_float))
        m = max(0, min(255, m))

        actual = F_XOSC_HZ / (2 ** 24) * (256 + m) * (2 ** e)
        err = abs(actual - hz)
        if err < best_err:
            best_err = err
            best_e = e
            best_m = m

    return best_m, best_e


def chan_bw_to_reg(bw_khz: float) -> int:
    """
    Compute CC1200 CHAN_BW register value from RX filter bandwidth in kHz.

    Formula (CC1200 datasheet section 8.7.3):
        BW = f_xosc / (CHFILT_BYPASS * 8 * (BB_CHP_SET + 1) * 2^ADC_DEC_FACTOR)

    CHAN_BW register format: {ADC_CIC_DECFACT[1:0], 00, BB_CHP_SET[3:0]}
        ADC_CIC_DECFACT = bits 7:6
        BB_CHP_SET = bits 3:0

    We use the simplified SmartRF-style lookup table approach.
    """
    bw_hz = bw_khz * 1000
    best_reg = 0x8E  # default ~29.8 kHz
    best_err = float("inf")

    for adc_dec in range(4):
        for bb_chp in range(16):
            # BW = f_xosc / (8 * (bb_chp + 1) * 2^(adc_dec + 1))
            calc_bw = F_XOSC_HZ / (8.0 * (bb_chp + 1) * (2 ** (adc_dec + 1)))
            err = abs(calc_bw - bw_hz)
            if err < best_err:
                best_err = err
                best_reg = ((adc_dec & 0x3) << 6) | (bb_chp & 0xF)

    return best_reg


def doppler_shift(freq_hz: float, range_rate_m_s: float) -> float:
    """
    Compute Doppler-shifted frequency.
    range_rate_m_s: positive = satellite moving away (red shift), negative = approaching.
    """
    c = 299_792_458.0
    return freq_hz * (1.0 - range_rate_m_s / c)


# ---------------------------------------------------------------------------
# SmartRF config parsing
# ---------------------------------------------------------------------------
@dataclass
class RegWrite:
    is_ext: bool
    addr: int
    value: int
    name: str


EXPORT_LINE_RE = re.compile(r'^\s*(0x[0-9A-Fa-f]{1,2})\s*,\s*//\s*([A-Z0-9_]+)\b')


def load_regmap(path: str) -> dict:
    with open(path, "r", encoding="utf-8") as f:
        data = json.load(f)
    return {k: int(v) for k, v in data.items()}


def compile_smartrf_config(export_path: str, regmap: dict) -> Tuple[List[RegWrite], str]:
    """Parse SmartRF Studio export and compile to register write list."""
    with open(export_path, "r", encoding="utf-8", errors="ignore") as f:
        lines = f.readlines()

    seen = kept = ignored = 0
    writes: List[RegWrite] = []

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
        addr8 = ti_addr & 0xFF
        writes.append(RegWrite(is_ext=is_ext, addr=addr8, value=val, name=name))
        kept += 1

    summary = f"Parsed {seen} lines, {kept} writes, {ignored} ignored"
    return writes, summary


# ---------------------------------------------------------------------------
# Data types
# ---------------------------------------------------------------------------
@dataclass
class Metrics:
    marc: int = 0
    rxbytes: int = 0
    rssi_raw_1db: int = 0
    rssi_dbm_x10: int = -32768  # -32768 = invalid
    # Extended fields (28-byte response)
    version: int = 0
    flags: int = 0
    desired_state: int = 0
    chip_state_raw: int = 0
    txbytes: int = 0
    last_rx_chunk: int = 0
    rx_total_bytes: int = 0
    tx_total_bytes: int = 0
    rx_overflow_count: int = 0
    rx_fifo_err_count: int = 0
    tx_fifo_err_count: int = 0
    uptime_ms: int = 0

    @property
    def rssi_dbm(self) -> Optional[float]:
        if self.rssi_dbm_x10 == -32768:
            return None
        return self.rssi_dbm_x10 / 10.0

    @property
    def streaming(self) -> bool:
        return bool(self.flags & 0x01)

    @property
    def serial_mode(self) -> bool:
        return bool(self.flags & 0x10)


@dataclass
class RadioInfo:
    active: int = 0
    count: int = 0
    part: int = 0
    ver: int = 0


# ---------------------------------------------------------------------------
# CC1200 Link — threaded serial protocol handler
# ---------------------------------------------------------------------------
class CC1200Link:
    def __init__(self):
        self.ser: Optional[serial.Serial] = None
        self.rxbuf = bytearray()
        self.seq = 1

        self.lock = threading.Lock()
        self.running = False
        self.rx_thread: Optional[threading.Thread] = None

        self.event_queue: List[Tuple[int, bytes]] = []
        self.rsp_data = {}

        self.last_metrics = Metrics()
        self._active_radio = 0

    def open(self, port: str = "/dev/serial0", baud: int = 115200) -> None:
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
                data = self.ser.read(512)
                if data:
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
                    parsed = parse_frame(frame)
                    if not parsed:
                        continue

                    msg_type, seq, body = parsed
                    if msg_type >= 0xE0:
                        with self.lock:
                            self.event_queue.append((msg_type, body))
                    elif msg_type & RSP_FLAG:
                        with self.lock:
                            self.rsp_data[seq] = body
                    else:
                        with self.lock:
                            self.event_queue.append((msg_type, body))

            except Exception:
                time.sleep(0.05)

    def pop_events(self) -> List[Tuple[int, bytes]]:
        with self.lock:
            ev = list(self.event_queue)
            self.event_queue.clear()
        return ev

    def send_cmd(self, msg_type: int, body: bytes = b"",
                 timeout_s: float = 1.0) -> Optional[bytes]:
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

    # ----- High-level commands -----

    def ping(self) -> Optional[bytes]:
        return self.send_cmd(MSG_PING)

    def get_info(self) -> Optional[RadioInfo]:
        b = self.send_cmd(MSG_GET_INFO)
        if b and len(b) >= 4:
            return RadioInfo(active=b[0], count=b[1], part=b[2], ver=b[3])
        return None

    def select_radio(self, index: int) -> bool:
        b = self.send_cmd(MSG_SELECT_RADIO, bytes([index & 0xFF]))
        if b and len(b) >= 1:
            self._active_radio = b[0]
            return True
        return False

    def set_state(self, state: int) -> bool:
        b = self.send_cmd(MSG_SET_STATE, bytes([state & 0xFF]))
        return bool(b and len(b) >= 1 and b[0] == 1)

    def set_streaming(self, on: bool) -> bool:
        b = self.send_cmd(MSG_SET_STREAMING, bytes([1 if on else 0]))
        return bool(b and len(b) >= 1)

    def get_metrics(self) -> Optional[Metrics]:
        b = self.send_cmd(MSG_GET_METRICS)
        if not b:
            return None
        if len(b) >= 28 and b[0] == 2:
            # 28-byte extended response
            m = Metrics(
                version=b[0],
                flags=b[1],
                desired_state=b[2],
                chip_state_raw=b[3],
                marc=b[4],
                rxbytes=b[5],
                txbytes=b[6],
                last_rx_chunk=b[7],
                rssi_dbm_x10=struct.unpack_from("<h", b, 8)[0],
                rx_total_bytes=struct.unpack_from("<I", b, 10)[0],
                tx_total_bytes=struct.unpack_from("<I", b, 14)[0],
                rx_overflow_count=struct.unpack_from("<H", b, 18)[0],
                rx_fifo_err_count=struct.unpack_from("<H", b, 20)[0],
                tx_fifo_err_count=struct.unpack_from("<H", b, 22)[0],
                uptime_ms=struct.unpack_from("<I", b, 24)[0],
            )
            if m.rssi_dbm_x10 != -32768:
                m.rssi_raw_1db = max(-128, min(127, m.rssi_dbm_x10 // 10))
        elif len(b) >= 5:
            # Legacy 5-byte response (fallback)
            m = Metrics(
                marc=b[0], rxbytes=b[1],
                rssi_raw_1db=struct.unpack_from("<b", b, 2)[0],
                rssi_dbm_x10=struct.unpack_from("<h", b, 3)[0],
            )
        else:
            return None
        self.last_metrics = m
        return m

    # ----- Register access -----

    def read_reg(self, addr: int) -> Optional[int]:
        b = self.send_cmd(MSG_READ_REG, bytes([addr & 0xFF]))
        if b and len(b) == 2 and b[0] == 1:
            return b[1]
        return None

    def write_reg(self, addr: int, val: int) -> bool:
        b = self.send_cmd(MSG_WRITE_REG, bytes([addr & 0xFF, val & 0xFF]))
        return bool(b and len(b) >= 1 and b[0] == 1)

    def read_ext(self, addr: int) -> Optional[int]:
        b = self.send_cmd(MSG_READ_EXT, bytes([addr & 0xFF]))
        if b and len(b) == 2 and b[0] == 1:
            return b[1]
        return None

    def write_ext(self, addr: int, val: int) -> bool:
        b = self.send_cmd(MSG_WRITE_EXT, bytes([addr & 0xFF, val & 0xFF]))
        return bool(b and len(b) >= 1 and b[0] == 1)

    def strobe(self, cmd: int) -> Optional[int]:
        b = self.send_cmd(MSG_STROBE, bytes([cmd & 0xFF]))
        if b and len(b) >= 1:
            return b[0]
        return None

    # ----- TX -----

    def tx_flush(self) -> bool:
        b = self.send_cmd(MSG_TX_FLUSH)
        return bool(b and len(b) >= 1 and b[0] == 1)

    def tx_write(self, data: bytes, flush_first: bool = True,
                 force_idle: bool = True) -> Tuple[bool, int]:
        flags = 0
        if flush_first:
            flags |= 0x01
        if force_idle:
            flags |= 0x02
        if len(data) > 255:
            data = data[:255]
        body = bytes([flags, len(data)]) + data
        b = self.send_cmd(MSG_TX_WRITE, body)
        if b and len(b) == 2:
            return (b[0] == 1), int(b[1])
        return (False, 0)

    def tx_send(self, go_idle: bool = False, go_rx: bool = True,
                flush_before: bool = False) -> bool:
        flags = 0
        if go_idle:
            flags |= 0x01
        if go_rx:
            flags |= 0x02
        if flush_before:
            flags |= 0x04
        b = self.send_cmd(MSG_TX_SEND, bytes([flags]))
        return bool(b and len(b) >= 1 and b[0] == 1)

    def tx_packet(self, data: bytes) -> bool:
        """Write data to TX FIFO and transmit. Returns to IDLE after."""
        ok, written = self.tx_write(data, flush_first=True, force_idle=True)
        if not ok:
            return False
        return self.tx_send(go_idle=True)

    # ----- Buzzer (on Pico GP2) -----

    BUZZER_READY  = 0x01
    BUZZER_AOS    = 0x02
    BUZZER_LOS    = 0x03
    BUZZER_PACKET = 0x04
    BUZZER_ERROR  = 0x05

    def buzzer(self, pattern: int) -> bool:
        """Trigger a buzzer beep pattern on the RF HAT Pico.
        Pattern IDs: 0x01=ready, 0x02=AOS, 0x03=LOS, 0x04=packet, 0x05=error."""
        b = self.send_cmd(MSG_BUZZER, bytes([pattern & 0xFF]))
        return bool(b and len(b) >= 1 and b[0] == 1)

    # ----- Serial mode (raw bit streaming via PIO) -----

    def set_serial_mode(self, enable: bool, mode: int = 1) -> bool:
        """Enable/disable CC1200 synchronous serial mode + PIO bit streaming.

        When enabled, the CC1200 demodulates FSK/GFSK and outputs raw bits
        via GPIO, the RP2040 PIO captures them, and they arrive as EVT_RX_DATA
        events — same as normal FIFO streaming but with raw demodulated bits
        instead of decoded packets.

        This is used for protocols that the CC1200 hardware packet engine
        cannot decode natively.

        Args:
            enable: True to enter serial mode, False to return to FIFO mode.
            mode: 1=raw bitstream, 2=AX.25 G3RUH decode (NRZ-I + descramble + HDLC)
        Returns:
            True if firmware confirmed the mode switch.
        """
        if enable:
            b = self.send_cmd(MSG_SET_SERIAL_MODE, bytes([mode & 0xFF]))
        else:
            b = self.send_cmd(MSG_SET_SERIAL_MODE, bytes([0]))
        return bool(b and len(b) >= 1 and b[0] == 1)

    # ----- Frequency control -----

    def set_frequency_word(self, word: int, force_cal: bool = False) -> bool:
        """Atomic FREQ register update via firmware. Doppler-safe."""
        word &= 0xFFFFFF
        f2 = (word >> 16) & 0xFF
        f1 = (word >> 8) & 0xFF
        f0 = word & 0xFF
        flags = 0x01 if force_cal else 0x00
        b = self.send_cmd(MSG_SET_FREQ_WORD, bytes([f2, f1, f0, flags]))
        return bool(b and len(b) >= 1 and b[0] == 1)

    def set_frequency(self, freq_hz: float, force_cal: bool = False) -> bool:
        """Set CC1200 carrier frequency. Uses atomic freq word command."""
        fs_cfg, freq2, freq1, freq0 = freq_to_regs(freq_hz)
        # FS_CFG only changes when LO divider changes (band switch) — write separately
        self.write_reg(0x21, fs_cfg)
        word = (freq2 << 16) | (freq1 << 8) | freq0
        return self.set_frequency_word(word, force_cal=force_cal)

    def get_frequency(self) -> Optional[float]:
        """Read back the current CC1200 frequency."""
        fs_cfg = self.read_reg(0x21)
        freq2 = self.read_ext(EXT_FREQ2)
        freq1 = self.read_ext(EXT_FREQ1)
        freq0 = self.read_ext(EXT_FREQ0)
        if None in (fs_cfg, freq2, freq1, freq0):
            return None
        return regs_to_freq(fs_cfg, freq2, freq1, freq0)

    # ----- Modulation reconfiguration -----

    def configure_modulation(self, mod_format: str = "2-GFSK",
                              symbol_rate_bps: int = 2400,
                              deviation_hz: int = 5000,
                              rx_bw_khz: float = 25.0,
                              sync_word: Optional[str] = None,
                              freq_hz: Optional[float] = None,
                              sync_threshold: int = 4,
                              pkt_length: Optional[int] = None,
                              variable_length: bool = False,
                              enable_pn9: bool = False,
                              enable_crc16: bool = False) -> bool:
        """
        Reconfigure modulation-critical CC1200 registers without full profile reload.

        Puts radio in IDLE, writes ~15 registers, optionally sets frequency.
        Much faster than apply_smartrf_config() (~100ms vs ~500ms).

        Args:
            mod_format: "2-GFSK", "2-FSK", "4-GFSK", "4-FSK", "ASK", "OOK"
            symbol_rate_bps: Symbol rate in bits per second
            deviation_hz: Frequency deviation in Hz
            rx_bw_khz: RX channel filter bandwidth in kHz
            sync_word: Hex string sync word (e.g. "7A0E" or "7E7E"), or None to keep current
            freq_hz: Carrier frequency in Hz, or None to keep current
            sync_threshold: SYNC_THR value (max allowed bit errors = THR/2). Default 4.
            pkt_length: Fixed packet length in bytes, or None to keep current
            variable_length: If True, use variable-length packet mode (length byte prefix)
            enable_pn9: If True, enable PN9 data whitening (PKT_CFG0 bit 6)
            enable_crc16: If True, enable CRC-16 append/check (PKT_CFG1 bits [1:0]=01)

        Returns True if all writes succeeded.
        """
        # Validate modulation format
        fmt_bits = MOD_FORMAT_MAP.get(mod_format)
        if fmt_bits is None:
            return False

        # Put radio in IDLE and flush FIFO before reconfiguring
        self.set_state(STATE_IDLE)
        time.sleep(0.02)
        # Confirm IDLE (poll MARCSTATE up to 50ms)
        for _ in range(10):
            marc = self.read_ext(0x73)
            if marc is not None and (marc & 0x1F) == 1:  # IDLE
                break
            time.sleep(0.005)
        self.strobe(0x3A)  # SFRX — flush RX FIFO

        ok = True

        # Symbol rate registers
        sr2, sr1, sr0 = symbol_rate_to_regs(symbol_rate_bps)
        ok = ok and self.write_reg(REG_SYMBOL_RATE2, sr2)
        ok = ok and self.write_reg(REG_SYMBOL_RATE1, sr1)
        ok = ok and self.write_reg(REG_SYMBOL_RATE0, sr0)

        # Deviation
        dev_m, dev_e = deviation_to_regs(deviation_hz)
        ok = ok and self.write_reg(REG_DEVIATION_M, dev_m)

        # MODCFG_DEV_E: [7] MODEM_MODE=0, [6] MOD_FORMAT_MSB, [5:3] MOD_FORMAT, [2:0] DEV_E
        # For 2-GFSK: fmt_bits=0b000, for 2-FSK: fmt_bits=0b001, etc.
        modcfg = ((fmt_bits & 0x7) << 3) | (dev_e & 0x7)
        ok = ok and self.write_reg(REG_MODCFG_DEV_E, modcfg)

        # RX channel bandwidth
        chan_bw = chan_bw_to_reg(rx_bw_khz)
        ok = ok and self.write_reg(REG_CHAN_BW, chan_bw)

        # Sync word (up to 32 bits) + SYNC_THR
        if sync_word is not None:
            sw_bytes = bytes.fromhex(sync_word)
            if len(sw_bytes) == 2:
                # 16-bit sync word → write to SYNC1/SYNC0, clear SYNC3/SYNC2
                ok = ok and self.write_reg(REG_SYNC3, 0x00)
                ok = ok and self.write_reg(REG_SYNC2, 0x00)
                ok = ok and self.write_reg(REG_SYNC1, sw_bytes[0])
                ok = ok and self.write_reg(REG_SYNC0, sw_bytes[1])
                # SYNC_CFG1: 16-bit sync word mode + sync threshold
                sync_cfg1 = self.read_reg(REG_SYNC_CFG1)
                if sync_cfg1 is not None:
                    sync_cfg1 = (sync_cfg1 & 0xE0) | (sync_threshold & 0x1F)
                    sync_cfg1 = (sync_cfg1 & 0xE3) | (0x04 << 2)  # 16-bit mode
                    ok = ok and self.write_reg(REG_SYNC_CFG1, sync_cfg1)
            elif len(sw_bytes) == 4:
                # 32-bit sync word
                ok = ok and self.write_reg(REG_SYNC3, sw_bytes[0])
                ok = ok and self.write_reg(REG_SYNC2, sw_bytes[1])
                ok = ok and self.write_reg(REG_SYNC1, sw_bytes[2])
                ok = ok and self.write_reg(REG_SYNC0, sw_bytes[3])
                # SYNC_CFG1: 32-bit sync word mode + sync threshold
                sync_cfg1 = self.read_reg(REG_SYNC_CFG1)
                if sync_cfg1 is not None:
                    sync_cfg1 = (sync_cfg1 & 0xE0) | (sync_threshold & 0x1F)
                    sync_cfg1 = (sync_cfg1 & 0xE3) | (0x06 << 2)  # 32-bit mode
                    ok = ok and self.write_reg(REG_SYNC_CFG1, sync_cfg1)
        else:
            # No sync word known — use caller-specified threshold (default 4).
            # SYNC_THR=4 → max 2 bit errors in 32-bit sync (SYNC_ERROR < THR/2).
            sync_cfg1 = self.read_reg(REG_SYNC_CFG1)
            if sync_cfg1 is not None:
                sync_cfg1 = (sync_cfg1 & 0xE0) | (sync_threshold & 0x1F)
                ok = ok and self.write_reg(REG_SYNC_CFG1, sync_cfg1)

        # Packet engine config (PKT_CFG0, PKT_CFG1, PKT_LEN)
        # Only write when explicitly configured — otherwise keep SmartRF defaults
        if pkt_length is not None or variable_length or enable_pn9 or enable_crc16:
            # PKT_CFG0: [6] WHITE_DATA (PN9), [5] PKT_FORMAT=00 (FIFO),
            #           [4:0] LENGTH_CONFIG: 00=fixed, 01=variable
            pkt_cfg0 = self.read_reg(REG_PKT_CFG0) or 0x00
            if enable_pn9:
                pkt_cfg0 |= 0x40   # WHITE_DATA = 1
            else:
                pkt_cfg0 &= ~0x40  # WHITE_DATA = 0
            if variable_length:
                pkt_cfg0 = (pkt_cfg0 & 0xE0) | 0x20  # LENGTH_CONFIG = 01 (variable)
            else:
                pkt_cfg0 = (pkt_cfg0 & 0xE0) | 0x00  # LENGTH_CONFIG = 00 (fixed)
            ok = ok and self.write_reg(REG_PKT_CFG0, pkt_cfg0)

            # PKT_CFG1: [1:0] CRC_CFG: 00=off, 01=CRC16 (ITU-T)
            pkt_cfg1 = self.read_reg(REG_PKT_CFG1) or 0x00
            if enable_crc16:
                pkt_cfg1 = (pkt_cfg1 & 0xFC) | 0x01  # CRC16
            else:
                pkt_cfg1 = (pkt_cfg1 & 0xFC) | 0x00  # CRC off
            ok = ok and self.write_reg(REG_PKT_CFG1, pkt_cfg1)

            # PKT_LEN: fixed packet length (or max for variable mode)
            if pkt_length is not None:
                ok = ok and self.write_reg(REG_PKT_LEN, pkt_length & 0xFF)

        # RFEND_CFG1: RXOFF_MODE — stay in RX after good packet
        # bits [5:4] = 0b11 → return to RX after packet
        rfend1 = self.read_reg(REG_RFEND_CFG1)
        if rfend1 is not None:
            rfend1 = (rfend1 & 0xCF) | 0x30  # RXOFF_MODE = 11 → RX
            ok = ok and self.write_reg(REG_RFEND_CFG1, rfend1)

        # RFEND_CFG0: return to RX after bad CRC too
        rfend0 = self.read_reg(REG_RFEND_CFG0)
        if rfend0 is not None:
            rfend0 = (rfend0 & 0xCF) | 0x30  # RXOFF_MODE = 11 → RX
            ok = ok and self.write_reg(REG_RFEND_CFG0, rfend0)

        # Frequency (if specified)
        if freq_hz is not None:
            ok = ok and self.set_frequency(freq_hz)

        return ok

    # ----- SmartRF profile loading -----

    def apply_smartrf_config(self, config_path: str,
                             regmap_path: Optional[str] = None) -> Tuple[bool, str]:
        """
        Load and apply a SmartRF Studio register export to the active CC1200.
        Uses direct register writes (not the bulk profile system).
        """
        if regmap_path is None:
            regmap_path = str(Path(__file__).parent / "configs" / "cc1200_regmap_profile.json")

        try:
            regmap = load_regmap(regmap_path)
        except Exception as e:
            return False, f"Failed to load regmap: {e}"

        try:
            writes, summary = compile_smartrf_config(config_path, regmap)
        except Exception as e:
            return False, f"Failed to parse config: {e}"

        if not writes:
            return False, "No register writes compiled from config"

        # Force IDLE before writing registers
        if not self.set_state(STATE_IDLE):
            return False, "Failed to set IDLE"
        time.sleep(0.05)

        failed = []
        for w in writes:
            ok = False
            for _ in range(3):
                if w.is_ext:
                    ok = self.write_ext(w.addr, w.value)
                else:
                    ok = self.write_reg(w.addr, w.value)
                if ok:
                    break
                time.sleep(0.005)
            if not ok:
                failed.append(w)

        if failed:
            names = ", ".join(w.name for w in failed[:5])
            return False, f"{summary}; {len(failed)} writes failed: {names}"

        return True, summary
