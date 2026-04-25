import time
import struct
import threading
import re
import json
from pathlib import Path
from dataclasses import dataclass
from typing import Optional, Tuple, List

import serial
import serial.tools.list_ports

import customtkinter as ctk
from tkinter import filedialog


# -----------------------------
# Protocol constants (must match firmware)
# -----------------------------
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

# CC1200 EXT status regs
EXT_RSSI1       = 0x71
EXT_RSSI0       = 0x72
EXT_MARCSTATE   = 0x73
EXT_LQI_VAL     = 0x74
EXT_NUM_TXBYTES = 0xD6
EXT_NUM_RXBYTES = 0xD7

STATE_IDLE = 0
STATE_RX   = 1
STATE_TX   = 2


# -----------------------------
# COBS + CRC16
# -----------------------------
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
    calc = crc16_ccitt_false(dec[:4 + ln])
    if rx_crc != calc:
        return None
    return msg_type, seq, body


# -----------------------------
# Link layer
# -----------------------------
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

    def open(self, port: str, baud: int = 115200) -> None:
        self.close()
        self.ser = serial.Serial(port, baudrate=baud, timeout=0.02)
        self.running = True
        self.rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self.rx_thread.start()

    def close(self) -> None:
        self.running = False
        if self.rx_thread:
            self.rx_thread.join(timeout=0.2)
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
            time.sleep(0.01)
        return None

    def ping(self) -> Optional[bytes]:
        return self.send_cmd(MSG_PING, b"")

    def get_info(self) -> Optional[Tuple[int, int]]:
        b = self.send_cmd(MSG_GET_INFO, b"")
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
        b = self.send_cmd(MSG_GET_METRICS, b"")
        if not b:
            return None

        # Old firmware: 3-byte legacy
        if len(b) == 3:
            marc = b[0]
            rxbytes = b[1]
            rssi_raw_1db = struct.unpack_from("<b", b, 2)[0]
            m = Metrics(marc=marc, rxbytes=rxbytes, rssi_raw_1db=rssi_raw_1db, rssi_dbm_x10=-32768)
            self.last_metrics = m
            return m

        # Old intermediate firmware: 5-byte legacy
        if len(b) == 5:
            marc = b[0]
            rxbytes = b[1]
            rssi_raw_1db = struct.unpack_from("<b", b, 2)[0]
            rssi_dbm_x10 = struct.unpack_from("<h", b, 3)[0]
            m = Metrics(marc=marc, rxbytes=rxbytes, rssi_raw_1db=rssi_raw_1db, rssi_dbm_x10=rssi_dbm_x10)
            self.last_metrics = m
            return m

        # New firmware v2: 36-byte metrics
        if len(b) >= 36 and b[0] == 2:
            marc = b[4]
            rxbytes = b[5]
            rssi_dbm_x10 = struct.unpack_from("<h", b, 8)[0]
            last_retune_us = struct.unpack_from("<I", b, 28)[0]
            max_retune_us = struct.unpack_from("<I", b, 32)[0]
            m = Metrics(
                marc=marc,
                rxbytes=rxbytes,
                rssi_raw_1db=0,
                rssi_dbm_x10=rssi_dbm_x10,
                last_doppler_retune_us=last_retune_us,
                max_doppler_retune_us=max_retune_us,
            )
            self.last_metrics = m
            return m

        # Earlier v2 without timing
        if len(b) >= 28 and b[0] == 2:
            marc = b[4]
            rxbytes = b[5]
            rssi_dbm_x10 = struct.unpack_from("<h", b, 8)[0]
            m = Metrics(marc=marc, rxbytes=rxbytes, rssi_raw_1db=0, rssi_dbm_x10=rssi_dbm_x10)
            self.last_metrics = m
            return m

        return None

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
        if b and len(b) == 1:
            return b[0]
        return None

    # -----------------
    # TX helpers
    # -----------------
    def tx_flush(self) -> bool:
        b = self.send_cmd(MSG_TX_FLUSH, b"")
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
        return (False, 0)

    def tx_send(self, go_idle: bool = True, go_rx: bool = False, flush_before: bool = False) -> bool:
        flags = 0
        if go_idle:
            flags |= 0x01
        if go_rx:
            flags |= 0x02
        if flush_before:
            flags |= 0x04

        b = self.send_cmd(MSG_TX_SEND, bytes([flags]), timeout_s=1.0)
        return bool(b and len(b) >= 1 and b[0] == 1)

    @staticmethod
    def parse_hex_string(s: str) -> Optional[bytes]:
        s = s.strip()
        if not s:
            return b""
        s = s.replace("0x", "").replace("0X", "")
        s = s.replace(":", " ").replace(",", " ").replace("\n", " ").replace("\t", " ")
        s = re.sub(r"\s+", " ", s).strip()

        if " " in s:
            parts = s.split(" ")
            try:
                return bytes(int(p, 16) & 0xFF for p in parts if p)
            except Exception:
                return None

        if len(s) % 2 != 0:
            return None
        try:
            return bytes(int(s[i:i + 2], 16) & 0xFF for i in range(0, len(s), 2))
        except Exception:
            return None

    def tx_hex_test(self, hex_str: str) -> Tuple[bool, str]:
        data = self.parse_hex_string(hex_str)
        if data is None:
            return False, "Bad hex string"

        marc0 = self.read_ext(EXT_MARCSTATE)
        tx0 = self.read_ext(EXT_NUM_TXBYTES)
        rx0 = self.read_ext(EXT_NUM_RXBYTES)

        ok, written = self.tx_write(data, flush_first=True, force_idle=True)
        if not ok:
            return False, "TX_WRITE failed"

        tx1 = self.read_ext(EXT_NUM_TXBYTES)

        ok2 = self.tx_send(go_idle=False, go_rx=False, flush_before=False)
        if not ok2:
            return False, f"TX_WRITE ok (written={written}), but TX_SEND failed"

        marc1 = self.read_ext(EXT_MARCSTATE)
        tx2 = self.read_ext(EXT_NUM_TXBYTES)

        msg = (
            f"TX sent: {written} byte(s) | "
            f"MARC: 0x{(marc0 or 0):02X} -> 0x{(marc1 or 0):02X} | "
            f"TXBYTES: 0x{(tx0 or 0):02X} -> 0x{(tx1 or 0):02X} -> 0x{(tx2 or 0):02X} | "
            f"RXBYTES(before)=0x{(rx0 or 0):02X}"
        )
        return True, msg

    # -----------------
    # Frequency / Doppler helpers
    # -----------------
    @staticmethod
    def freq_to_regs(freq_hz: float, f_xtal_hz: float = 40_000_000.0) -> int:
        if freq_hz < 0:
            freq_hz = 0.0
        word = int(round((freq_hz * (1 << 16)) / f_xtal_hz))
        return word & 0xFFFFFF

    @staticmethod
    def regs_to_freq(word: int, f_xtal_hz: float = 40_000_000.0) -> float:
        word &= 0xFFFFFF
        return (word * f_xtal_hz) / float(1 << 16)

    def set_frequency_word(self, word: int, force_cal: bool = False) -> bool:
        word &= 0xFFFFFF
        f2 = (word >> 16) & 0xFF
        f1 = (word >> 8) & 0xFF
        f0 = word & 0xFF
        flags = 0x01 if force_cal else 0x00

        b = self.send_cmd(MSG_SET_FREQ_WORD, bytes([f2, f1, f0, flags]), timeout_s=0.5)
        return bool(b and len(b) >= 1 and b[0] == 1)

    def set_frequency_hz(self, freq_hz: float, force_cal: bool = False) -> Tuple[bool, int]:
        word = self.freq_to_regs(freq_hz)
        ok = self.set_frequency_word(word, force_cal=force_cal)
        return ok, word


# -----------------------------
# SmartRF export parsing
# -----------------------------
EXPORT_LINE_RE = re.compile(r'^\s*(0x[0-9A-Fa-f]{1,2})\s*,\s*//\s*([A-Z0-9_]+)\b')


def load_regmap_json(path: str) -> dict:
    with open(path, "r", encoding="utf-8") as f:
        data = json.load(f)
    return {k: int(v) for k, v in data.items()}


def compile_smartrf_config(export_path: str, regmap: dict) -> Tuple[List[RegWrite], str]:
    with open(export_path, "r", encoding="utf-8", errors="ignore") as f:
        lines = f.readlines()

    seen = 0
    kept = 0
    ignored = 0
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

        if ti_addr >= 0x2F00:
            is_ext = True
            addr8 = ti_addr & 0xFF
        else:
            is_ext = False
            addr8 = ti_addr & 0xFF

        writes.append(RegWrite(is_ext=is_ext, addr=addr8, value=val, name=name))
        kept += 1

    summary = f"Parsed {seen} config lines, compiled {kept} writes, ignored {ignored} non-profile lines"
    return writes, summary


# -----------------------------
# GUI
# -----------------------------
class App(ctk.CTk):
    def __init__(self):
        super().__init__()
        ctk.set_appearance_mode("System")
        ctk.set_default_color_theme("blue")

        self.title("CC1200 USB + RF Frontend Tester (RX + TX + Doppler)")
        self.geometry("1220x860")

        self.link = CC1200Link()

        self.busy = False
        self.profile_writes: Optional[List[RegWrite]] = None

        script_dir = Path(__file__).parent
        self.regmap_path = str(script_dir / "cc1200_regmap_profile.json")
        self.regmap: Optional[dict] = None
        self._load_regmap()

        top = ctk.CTkFrame(self)
        top.pack(fill="x", padx=10, pady=10)

        self.port_var = ctk.StringVar(value="")
        self.port_menu = ctk.CTkOptionMenu(top, variable=self.port_var, values=["(refresh)"])
        self.port_menu.pack(side="left", padx=8, pady=8)

        self.btn_refresh = ctk.CTkButton(top, text="Refresh ports", command=self.refresh_ports)
        self.btn_refresh.pack(side="left", padx=8)

        self.btn_connect = ctk.CTkButton(top, text="Connect", command=self.toggle_connect)
        self.btn_connect.pack(side="left", padx=8)

        self.conn_label = ctk.CTkLabel(top, text="Disconnected")
        self.conn_label.pack(side="left", padx=12)

        mid = ctk.CTkFrame(self)
        mid.pack(fill="both", expand=True, padx=10, pady=(0, 10))

        left = ctk.CTkScrollableFrame(mid, width=380)
        left.pack(side="left", fill="y", padx=10, pady=10)

        right = ctk.CTkFrame(mid)
        right.pack(side="right", fill="both", expand=True, padx=10, pady=10)

        ctk.CTkLabel(left, text="Controls", font=ctk.CTkFont(size=16, weight="bold")).pack(pady=(8, 4))

        self.btn_ping = ctk.CTkButton(left, text="PING", command=self.do_ping)
        self.btn_ping.pack(fill="x", padx=8, pady=6)

        self.btn_info = ctk.CTkButton(left, text="GET_INFO", command=self.do_info)
        self.btn_info.pack(fill="x", padx=8, pady=6)

        self.btn_idle = ctk.CTkButton(left, text="SET_STATE: IDLE", command=lambda: self.do_state(STATE_IDLE))
        self.btn_idle.pack(fill="x", padx=8, pady=6)

        self.btn_rx = ctk.CTkButton(left, text="SET_STATE: RX", command=lambda: self.do_state(STATE_RX))
        self.btn_rx.pack(fill="x", padx=8, pady=6)

        self.stream_var = ctk.IntVar(value=0)
        self.chk_stream = ctk.CTkCheckBox(
            left,
            text="Streaming (push EVT_RX_DATA)",
            variable=self.stream_var,
            command=self.do_stream_toggle,
        )
        self.chk_stream.pack(anchor="w", padx=10, pady=(10, 6))

        # ---- TX Test ----
        ctk.CTkLabel(left, text="TX Test (Send Hex)", font=ctk.CTkFont(size=14, weight="bold")).pack(pady=(12, 4))
        tx_frame = ctk.CTkFrame(left)
        tx_frame.pack(fill="x", padx=8, pady=6)

        self.tx_hex = ctk.CTkEntry(tx_frame, placeholder_text="Hex bytes (e.g. DE AD BE EF or DEADBEEF)")
        self.tx_hex.pack(fill="x", padx=8, pady=(8, 6))

        tx_btns = ctk.CTkFrame(tx_frame)
        tx_btns.pack(fill="x", padx=8, pady=(0, 8))
        ctk.CTkButton(tx_btns, text="TX Flush", command=self.do_tx_flush).pack(side="left", expand=True, fill="x", padx=(0, 4))
        ctk.CTkButton(tx_btns, text="Send Hex", command=self.do_tx_send_hex).pack(side="left", expand=True, fill="x", padx=(4, 0))

        # ---- Doppler Test ----
        ctk.CTkLabel(left, text="Doppler Test", font=ctk.CTkFont(size=14, weight="bold")).pack(pady=(12, 4))
        dop_frame = ctk.CTkFrame(left)
        dop_frame.pack(fill="x", padx=8, pady=6)

        self.base_freq_mhz = ctk.CTkEntry(dop_frame, placeholder_text="Base frequency MHz (e.g. 433.000000)")
        self.base_freq_mhz.pack(fill="x", padx=8, pady=(8, 4))

        self.shift_hz = ctk.CTkEntry(dop_frame, placeholder_text="Doppler shift Hz (e.g. -3200)")
        self.shift_hz.pack(fill="x", padx=8, pady=(0, 4))

        self.step_hz = ctk.CTkEntry(dop_frame, placeholder_text="Step Hz (e.g. 500)")
        self.step_hz.pack(fill="x", padx=8, pady=(0, 8))

        self.force_cal_var = ctk.IntVar(value=0)
        self.chk_force_cal = ctk.CTkCheckBox(
            dop_frame,
            text="Force calibration on retune",
            variable=self.force_cal_var
        )
        self.chk_force_cal.pack(anchor="w", padx=10, pady=(0, 8))

        dop_btns1 = ctk.CTkFrame(dop_frame)
        dop_btns1.pack(fill="x", padx=8, pady=(0, 6))
        ctk.CTkButton(dop_btns1, text="Apply Base", command=self.do_apply_base_freq).pack(
            side="left", expand=True, fill="x", padx=(0, 4)
        )
        ctk.CTkButton(dop_btns1, text="Apply Base + Shift", command=self.do_apply_doppler).pack(
            side="left", expand=True, fill="x", padx=(4, 0)
        )

        dop_btns2 = ctk.CTkFrame(dop_frame)
        dop_btns2.pack(fill="x", padx=8, pady=(0, 8))
        ctk.CTkButton(dop_btns2, text="- Step", command=lambda: self.do_nudge_doppler(-1)).pack(
            side="left", expand=True, fill="x", padx=(0, 4)
        )
        ctk.CTkButton(dop_btns2, text="+ Step", command=lambda: self.do_nudge_doppler(+1)).pack(
            side="left", expand=True, fill="x", padx=(4, 0)
        )

        self.doppler_status = ctk.CTkLabel(dop_frame, text="Target: ---")
        self.doppler_status.pack(anchor="w", padx=10, pady=(2, 8))

        self.base_freq_mhz.insert(0, "433.000000")
        self.shift_hz.insert(0, "0")
        self.step_hz.insert(0, "500")

        # ---- Profile loader/apply ----
        ctk.CTkLabel(left, text="Profile (SmartRF config.txt)", font=ctk.CTkFont(size=14, weight="bold")).pack(pady=(12, 4))

        prof_frame = ctk.CTkFrame(left)
        prof_frame.pack(fill="x", padx=8, pady=6)

        self.profile_entry = ctk.CTkEntry(prof_frame, placeholder_text="Select SmartRF config.txt (export)")
        self.profile_entry.pack(fill="x", padx=8, pady=(8, 6))

        prof_btns = ctk.CTkFrame(prof_frame)
        prof_btns.pack(fill="x", padx=8, pady=(0, 8))

        self.btn_browse_profile = ctk.CTkButton(prof_btns, text="Browse...", command=self.browse_profile)
        self.btn_browse_profile.pack(side="left", expand=True, fill="x", padx=(0, 4))

        self.btn_apply_profile = ctk.CTkButton(prof_btns, text="Apply Profile to Board", command=self.apply_profile)
        self.btn_apply_profile.pack(side="left", expand=True, fill="x", padx=(4, 0))

        regmap_status = "OK" if self.regmap else "MISSING"
        self.profile_status = ctk.CTkLabel(left, text=f"Regmap: {regmap_status}; Profile: none loaded")
        self.profile_status.pack(anchor="w", padx=10, pady=(2, 8))

        # ---- Metrics ----
        ctk.CTkLabel(left, text="Metrics", font=ctk.CTkFont(size=14, weight="bold")).pack(pady=(8, 4))
        self.btn_metrics_once = ctk.CTkButton(left, text="GET_METRICS (once)", command=self.do_metrics_once)
        self.btn_metrics_once.pack(fill="x", padx=8, pady=6)

        self.poll_var = ctk.IntVar(value=1)
        self.chk_poll = ctk.CTkCheckBox(left, text="Poll metrics (4 Hz)", variable=self.poll_var)
        self.chk_poll.pack(anchor="w", padx=10, pady=(4, 8))

        self.rssi_label = ctk.CTkLabel(left, text="RSSI: ---")
        self.rssi_label.pack(anchor="w", padx=10, pady=(8, 2))

        self.rssi_bar = ctk.CTkProgressBar(left)
        self.rssi_bar.set(0.0)
        self.rssi_bar.pack(fill="x", padx=10, pady=(0, 4))

        self.retune_label = ctk.CTkLabel(left, text="Retune: last --- us | max --- us")
        self.retune_label.pack(anchor="w", padx=10, pady=(0, 10))

        self.btn_snap = ctk.CTkButton(left, text="DEBUG SNAPSHOT", command=lambda: self.debug_snapshot("SNAP"))
        self.btn_snap.pack(fill="x", padx=8, pady=6)

        # ---- Register ops ----
        ctk.CTkLabel(left, text="Register Access", font=ctk.CTkFont(size=14, weight="bold")).pack(pady=(10, 4))

        reg_frame = ctk.CTkFrame(left)
        reg_frame.pack(fill="x", padx=8, pady=6)

        self.reg_addr = ctk.CTkEntry(reg_frame, placeholder_text="Reg addr (e.g. 0x00)")
        self.reg_addr.pack(fill="x", padx=8, pady=(8, 4))

        self.reg_val = ctk.CTkEntry(reg_frame, placeholder_text="Value (e.g. 0x12)")
        self.reg_val.pack(fill="x", padx=8, pady=(0, 8))

        btns = ctk.CTkFrame(reg_frame)
        btns.pack(fill="x", padx=8, pady=(0, 8))
        ctk.CTkButton(btns, text="Read REG", command=self.do_read_reg).pack(side="left", expand=True, fill="x", padx=(0, 4))
        ctk.CTkButton(btns, text="Write REG", command=self.do_write_reg).pack(side="left", expand=True, fill="x", padx=(4, 0))

        ext_frame = ctk.CTkFrame(left)
        ext_frame.pack(fill="x", padx=8, pady=6)

        self.ext_addr = ctk.CTkEntry(ext_frame, placeholder_text="EXT addr (e.g. 0x71 for RSSI1)")
        self.ext_addr.pack(fill="x", padx=8, pady=(8, 4))

        self.ext_val = ctk.CTkEntry(ext_frame, placeholder_text="Value (e.g. 0x01)")
        self.ext_val.pack(fill="x", padx=8, pady=(0, 8))

        btns2 = ctk.CTkFrame(ext_frame)
        btns2.pack(fill="x", padx=8, pady=(0, 8))
        ctk.CTkButton(btns2, text="Read EXT", command=self.do_read_ext).pack(side="left", expand=True, fill="x", padx=(0, 4))
        ctk.CTkButton(btns2, text="Write EXT", command=self.do_write_ext).pack(side="left", expand=True, fill="x", padx=(4, 0))

        # ---- Right log ----
        ctk.CTkLabel(right, text="Console / Events", font=ctk.CTkFont(size=16, weight="bold")).pack(pady=(8, 4))

        self.log = ctk.CTkTextbox(right, width=780, height=700)
        self.log.pack(fill="both", expand=True, padx=10, pady=10)
        self.log.configure(state="disabled")

        bottom = ctk.CTkFrame(right)
        bottom.pack(fill="x", padx=10, pady=(0, 10))
        self.btn_clear = ctk.CTkButton(bottom, text="Clear log", command=self.clear_log)
        self.btn_clear.pack(side="left")

        self.status_label = ctk.CTkLabel(bottom, text="Ready")
        self.status_label.pack(side="left", padx=10)

        self._last_poll = 0.0

        self.refresh_ports()
        self.after(50, self.ui_tick)

    def debug_snapshot(self, tag="SNAP"):
        if not self.ensure_connected():
            return

        marc = self.link.read_ext(EXT_MARCSTATE)
        rx = self.link.read_ext(EXT_NUM_RXBYTES)
        tx = self.link.read_ext(EXT_NUM_TXBYTES)
        r1 = self.link.read_ext(EXT_RSSI1)
        r0 = self.link.read_ext(EXT_RSSI0)
        lqi = self.link.read_ext(EXT_LQI_VAL)

        rx_n = (rx or 0) & 0x7F
        rx_ovf = 1 if ((rx or 0) & 0x80) else 0
        tx_n = (tx or 0) & 0x7F
        tx_ovf = 1 if ((tx or 0) & 0x80) else 0

        self.log_line(
            f"[{tag}] MARC=0x{(marc or 0):02X}  "
            f"RXBYTES=0x{(rx or 0):02X} (n={rx_n},ovf={rx_ovf})  "
            f"TXBYTES=0x{(tx or 0):02X} (n={tx_n},ovf={tx_ovf})  "
            f"RSSI1=0x{(r1 or 0):02X} RSSI0=0x{(r0 or 0):02X}  "
            f"LQI=0x{(lqi or 0):02X}"
        )

    def _load_regmap(self):
        try:
            self.regmap = load_regmap_json(self.regmap_path)
        except Exception:
            self.regmap = None

    def log_line(self, s: str):
        self.log.configure(state="normal")
        self.log.insert("end", s + "\n")
        self.log.see("end")
        self.log.configure(state="disabled")

    def clear_log(self):
        self.log.configure(state="normal")
        self.log.delete("1.0", "end")
        self.log.configure(state="disabled")

    def set_status(self, s: str):
        self.status_label.configure(text=s)

    def refresh_ports(self):
        ports = []
        for p in serial.tools.list_ports.comports():
            ports.append(f"{p.device}  ({p.description})")
        if not ports:
            ports = ["(no ports found)"]
        self.port_menu.configure(values=ports)
        if ports and self.port_var.get() not in ports:
            self.port_var.set(ports[0])

    def _selected_device(self) -> Optional[str]:
        val = self.port_var.get()
        if not val or val.startswith("(no"):
            return None
        return val.split()[0]

    def toggle_connect(self):
        if self.link.is_open():
            self.link.close()
            self.conn_label.configure(text="Disconnected")
            self.btn_connect.configure(text="Connect")
            self.set_status("Disconnected")
            self.log_line("[UI] Disconnected")
            return

        dev = self._selected_device()
        if not dev:
            self.set_status("No serial port selected")
            return
        try:
            self.link.open(dev, 115200)
            self.conn_label.configure(text=f"Connected: {dev}")
            self.btn_connect.configure(text="Disconnect")
            self.set_status("Connected")
            self.log_line(f"[UI] Connected to {dev}")
        except Exception as e:
            self.set_status(f"Connect failed: {e}")
            self.log_line(f"[ERR] Connect failed: {e}")

    def ensure_connected(self) -> bool:
        if not self.link.is_open():
            self.set_status("Not connected")
            return False
        return True

    # ---- Profile UI ----
    def browse_profile(self):
        path = filedialog.askopenfilename(
            title="Select SmartRF config.txt (export)",
            filetypes=[
                ("Text files", "*.txt;*.cfg;*.ini"),
                ("C files", "*.c;*.h"),
                ("All files", "*.*"),
            ],
        )
        if not path:
            return

        self.profile_entry.delete(0, "end")
        self.profile_entry.insert(0, path)

        if not self.regmap:
            self.profile_writes = None
            self.profile_status.configure(text=f"Regmap missing: {Path(self.regmap_path).name} not loaded")
            self.log_line(f"[PROFILE] regmap missing/unloaded: {self.regmap_path}")
            return

        try:
            writes, summary = compile_smartrf_config(path, self.regmap)
        except Exception as e:
            self.profile_writes = None
            self.profile_status.configure(text=f"Profile load failed: {e}")
            self.log_line(f"[PROFILE] load failed: {e}")
            return

        self.profile_writes = writes
        regmap_status = "OK" if self.regmap else "MISSING"
        self.profile_status.configure(text=f"Regmap: {regmap_status}; Profile: {summary}")
        self.log_line(f"[PROFILE] {summary}")
        self.log_line(f"[PROFILE] Source: {path}")

        n_ext = sum(1 for w in writes if w.is_ext)
        n_norm = len(writes) - n_ext
        self.log_line(f"[PROFILE] breakdown: normal={n_norm} ext={n_ext}")
        for w in writes[:12]:
            space = "EXT " if w.is_ext else "NORM"
            self.log_line(f"[PROFILE] {space} 0x{w.addr:02X} = 0x{w.value:02X}   // {w.name}")
        if len(writes) > 12:
            self.log_line(f"[PROFILE] ... ({len(writes) - 12} more)")

    def apply_profile(self):
        if not self.ensure_connected():
            return
        if not self.profile_writes:
            self.set_status("No profile loaded")
            self.log_line("[PROFILE] No profile loaded")
            return

        self.busy = True
        try:
            _ = self.link.pop_events()
            ok, msg = self.apply_profile_writes(self.profile_writes)
            if ok:
                self.log_line(f"[PROFILE] {msg}")
                self.set_status("Profile applied")
            else:
                self.log_line(f"[PROFILE] APPLY FAILED: {msg}")
                self.set_status("Profile apply failed")
        finally:
            self.busy = False

    def apply_profile_writes(self, writes: List[RegWrite]) -> Tuple[bool, str]:
        if not writes:
            return False, "No profile writes compiled"

        if not self.link.set_state(STATE_IDLE):
            return False, "Failed to set IDLE before applying profile"
        time.sleep(0.05)

        for w in writes:
            ok = False
            for _attempt in range(3):
                ok = self.link.write_ext(w.addr, w.value) if w.is_ext else self.link.write_reg(w.addr, w.value)
                if ok:
                    break
                time.sleep(0.005)

            if not ok:
                space = "EXT" if w.is_ext else "REG"
                return False, f"{space} write failed at 0x{w.addr:02X} = 0x{w.value:02X} ({w.name})"

            time.sleep(0.001)

        return True, f"Applied profile writes: {len(writes)} (order preserved from SmartRF export)"

    # ---- General actions ----
    def do_ping(self):
        if not self.ensure_connected():
            return
        b = self.link.ping()
        self.log_line(f"[PING] rsp={b!r}")
        self.set_status("PING done")

    def do_info(self):
        if not self.ensure_connected():
            return
        info = self.link.get_info()
        if not info:
            self.log_line("[INFO] failed")
            self.set_status("GET_INFO failed")
            return
        part, ver = info
        self.log_line(f"[INFO] PART=0x{part:02X} VER=0x{ver:02X}")
        self.set_status("GET_INFO done")

    def do_state(self, state: int):
        if not self.ensure_connected():
            return
        ok = self.link.set_state(state)
        label = {STATE_IDLE: "IDLE", STATE_RX: "RX", STATE_TX: "TX"}.get(state, f"{state}")
        self.log_line(f"[STATE] {label} -> {'OK' if ok else 'FAIL'}")
        self.set_status("SET_STATE done")

    def do_stream_toggle(self):
        if not self.ensure_connected():
            return
        on = bool(self.stream_var.get())
        ok = self.link.set_streaming(on)
        self.log_line(f"[STREAM] set {1 if on else 0} -> {'OK' if ok else 'FAIL'}")
        self.set_status("Streaming updated")

    # ---- Metrics ----
    def do_metrics_once(self):
        if not self.ensure_connected():
            return
        m = self.link.get_metrics()
        if not m:
            self.log_line("[METRICS] failed")
            self.set_status("GET_METRICS failed")
            return

        self.log_line(self._format_metrics_line(m))
        self.set_status("GET_METRICS done")
        self.update_rssi_ui(m)
        self.update_retune_ui(m)

    def _format_metrics_line(self, m: Metrics) -> str:
        rssi_str = "INVALID" if m.rssi_dbm is None else f"{m.rssi_dbm:.1f} dBm"
        return (
            f"[METRICS] MARC=0x{m.marc:02X} RXBYTES=0x{m.rxbytes:02X} "
            f"RSSI={rssi_str} RETUNE_LAST={m.last_doppler_retune_us} us "
            f"RETUNE_MAX={m.max_doppler_retune_us} us"
        )

    def update_rssi_ui(self, m: Metrics):
        if m.rssi_dbm is None:
            self.rssi_label.configure(text="RSSI: INVALID")
            self.rssi_bar.set(0.0)
            return

        dbm = m.rssi_dbm
        self.rssi_label.configure(text=f"RSSI: {dbm:.1f} dBm")
        v = (dbm + 120.0) / 100.0
        if v < 0.0:
            v = 0.0
        if v > 1.0:
            v = 1.0
        self.rssi_bar.set(v)

    def update_retune_ui(self, m: Metrics):
        self.retune_label.configure(
            text=f"Retune: last {m.last_doppler_retune_us} us | max {m.max_doppler_retune_us} us"
        )

    # ---- Parsing helpers ----
    def parse_int(self, text: str) -> Optional[int]:
        try:
            return int(text.strip(), 0)
        except Exception:
            return None

    def _parse_float(self, text: str) -> Optional[float]:
        try:
            return float(text.strip())
        except Exception:
            return None

    def _get_force_cal(self) -> bool:
        return bool(self.force_cal_var.get())

    def _current_base_hz(self) -> Optional[float]:
        mhz = self._parse_float(self.base_freq_mhz.get())
        if mhz is None:
            return None
        return mhz * 1e6

    def _current_shift_hz(self) -> float:
        hz = self._parse_float(self.shift_hz.get())
        if hz is None:
            return 0.0
        return hz

    def _current_step_hz(self) -> float:
        hz = self._parse_float(self.step_hz.get())
        if hz is None or hz <= 0:
            return 500.0
        return hz

    # ---- Register actions ----
    def do_read_reg(self):
        if not self.ensure_connected():
            return
        a = self.parse_int(self.reg_addr.get())
        if a is None:
            self.set_status("Bad reg addr")
            return
        v = self.link.read_reg(a)
        if v is None:
            self.log_line(f"[READ_REG] 0x{a:02X} -> FAIL")
        else:
            self.log_line(f"[READ_REG] 0x{a:02X} -> 0x{v:02X}")
        self.set_status("Read reg done")

    def do_write_reg(self):
        if not self.ensure_connected():
            return
        a = self.parse_int(self.reg_addr.get())
        v = self.parse_int(self.reg_val.get())
        if a is None or v is None:
            self.set_status("Bad addr/val")
            return
        ok = self.link.write_reg(a, v)
        self.log_line(f"[WRITE_REG] 0x{a:02X} = 0x{v:02X} -> {'OK' if ok else 'FAIL'}")
        self.set_status("Write reg done")

    def do_read_ext(self):
        if not self.ensure_connected():
            return
        a = self.parse_int(self.ext_addr.get())
        if a is None:
            self.set_status("Bad ext addr")
            return
        v = self.link.read_ext(a)
        if v is None:
            self.log_line(f"[READ_EXT] 0x{a:02X} -> FAIL")
        else:
            self.log_line(f"[READ_EXT] 0x{a:02X} -> 0x{v:02X}")
        self.set_status("Read ext done")

    def do_write_ext(self):
        if not self.ensure_connected():
            return
        a = self.parse_int(self.ext_addr.get())
        v = self.parse_int(self.ext_val.get())
        if a is None or v is None:
            self.set_status("Bad addr/val")
            return
        ok = self.link.write_ext(a, v)
        self.log_line(f"[WRITE_EXT] 0x{a:02X} = 0x{v:02X} -> {'OK' if ok else 'FAIL'}")
        self.set_status("Write ext done")

    # ---- TX actions ----
    def do_tx_flush(self):
        if not self.ensure_connected():
            return
        ok = self.link.tx_flush()
        self.log_line(f"[TX] FLUSH -> {'OK' if ok else 'FAIL'}")
        self.set_status("TX flush done")

    def do_tx_send_hex(self):
        if not self.ensure_connected():
            return
        s = self.tx_hex.get()
        ok, msg = self.link.tx_hex_test(s)
        self.log_line(f"[TX] {msg}" if ok else f"[TX] FAIL: {msg}")
        self.set_status("TX send done" if ok else "TX send failed")

    # ---- Doppler actions ----
    def _apply_frequency_hz(self, freq_hz: float, label: str):
        if not self.ensure_connected():
            return

        force_cal = self._get_force_cal()
        ok, word = self.link.set_frequency_hz(freq_hz, force_cal=force_cal)

        # Read back metrics so the message includes firmware-side retune time
        metrics = self.link.get_metrics()
        last_us = metrics.last_doppler_retune_us if metrics else 0
        max_us = metrics.max_doppler_retune_us if metrics else 0

        mhz = freq_hz / 1e6
        freq_back = self.link.regs_to_freq(word)
        freq_back_mhz = freq_back / 1e6
        err_hz = freq_back - freq_hz

        if ok:
            self.log_line(
                f"[DOPPLER] {label}: target={mhz:.6f} MHz  "
                f"word=0x{word:06X}  applied={freq_back_mhz:.6f} MHz  "
                f"error={err_hz:+.1f} Hz  "
                f"time={last_us} us  max={max_us} us  "
                f"{'(SCAL requested)' if force_cal else ''}"
            )
            self.doppler_status.configure(
                text=f"Target: {mhz:.6f} MHz | Word: 0x{word:06X} | Time: {last_us} us"
            )
            self.set_status("Frequency retune sent")
            if metrics:
                self.update_retune_ui(metrics)
        else:
            self.log_line(
                f"[DOPPLER] FAIL: {label}: target={mhz:.6f} MHz  word=0x{word:06X}"
            )
            self.set_status("Frequency retune failed")

    def do_apply_base_freq(self):
        base_hz = self._current_base_hz()
        if base_hz is None:
            self.set_status("Bad base frequency")
            self.log_line("[DOPPLER] Bad base frequency")
            return
        self._apply_frequency_hz(base_hz, "Base")

    def do_apply_doppler(self):
        base_hz = self._current_base_hz()
        if base_hz is None:
            self.set_status("Bad base frequency")
            self.log_line("[DOPPLER] Bad base frequency")
            return

        shift_hz = self._current_shift_hz()
        target_hz = base_hz + shift_hz

        self._apply_frequency_hz(
            target_hz,
            f"Base+Shift (base={base_hz/1e6:.6f} MHz, shift={shift_hz:+.1f} Hz)"
        )

    def do_nudge_doppler(self, direction: int):
        shift = self._current_shift_hz()
        step = self._current_step_hz()
        new_shift = shift + (direction * step)

        self.shift_hz.delete(0, "end")
        self.shift_hz.insert(0, f"{new_shift:.1f}")

        self.do_apply_doppler()

    # ---- UI tick ----
    def ui_tick(self):
        try:
            if self.link.is_open() and not self.busy:
                for (t, body) in self.link.pop_events():
                    if t == EVT_RX_DATA and body:
                        n = body[0]
                        data = body[1:1 + n]
                        self.log_line(f"[EVT_RX_DATA] n={n}  {data.hex(' ')}")
                    elif t == EVT_ERROR and body:
                        code = body[0]
                        self.log_line(f"[EVT_ERROR] code=0x{code:02X} body={body.hex(' ')}")
                    else:
                        self.log_line(f"[EVT] type=0x{t:02X} body={body.hex(' ')}")

                if self.poll_var.get():
                    now = time.time()
                    if now - self._last_poll > 0.25:
                        self._last_poll = now
                        m = self.link.get_metrics()
                        if m:
                            self.update_rssi_ui(m)
                            self.update_retune_ui(m)
                            self.log_line(self._format_metrics_line(m))

        finally:
            self.after(50, self.ui_tick)


def main():
    app = App()
    app.mainloop()


if __name__ == "__main__":
    main()