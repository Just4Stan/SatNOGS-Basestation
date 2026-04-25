import time
import struct
import threading
from typing import Optional, Tuple

import serial
import serial.tools.list_ports


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

RSP_FLAG          = 0x80

STATE_IDLE = 0
STATE_RX   = 1


# -----------------------------
# COBS + CRC16 (same as firmware)
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
# CC1200 EXT status block addresses (SWRU346B)
# -----------------------------
EXT_RSSI1         = 0x71
EXT_RSSI0         = 0x72
EXT_MARCSTATE     = 0x73
EXT_LQI_VAL       = 0x74
EXT_PQT_SYNC_ERR  = 0x75
EXT_DEM_STATUS    = 0x76
EXT_FREQOFF_EST1  = 0x77
EXT_FREQOFF_EST0  = 0x78
EXT_AGC_GAIN3     = 0x79
EXT_AGC_GAIN2     = 0x7A
EXT_AGC_GAIN1     = 0x7B
EXT_AGC_GAIN0     = 0x7C

EXT_NUM_RXBYTES   = 0xD7

# MARCSTATE (low 5 bits typically state ID; you’ve observed 0x41=IDLE-ish, 0x6D=RX-ish)
RX_MARC_OBSERVED  = 0x6D


def sign_extend_12(v12: int) -> int:
    v12 &= 0x0FFF
    if v12 & 0x0800:
        return v12 - 0x1000
    return v12

def decode_rssi_dbm_from_ext(rssi1_u8: int, rssi0_u8: int) -> Optional[float]:
    # RSSI[11:0] = { RSSI1[7:0], RSSI0[3:0] }, 0.0625 dB per LSB
    rssi12_u = ((rssi1_u8 & 0xFF) << 4) | (rssi0_u8 & 0x0F)
    rssi12 = sign_extend_12(rssi12_u)
    # -128 dBm invalid => -2048 raw
    if rssi12 == -2048:
        return None
    return rssi12 / 16.0

def i8(x: int) -> int:
    return struct.unpack("<b", bytes([x & 0xFF]))[0]


class Link:
    def __init__(self):
        self.ser: Optional[serial.Serial] = None
        self.rxbuf = bytearray()
        self.seq = 1
        self.lock = threading.Lock()
        self.running = False
        self.rx_thread: Optional[threading.Thread] = None
        self.rsp_data = {}  # seq -> body

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
            self.rsp_data.clear()

    def _rx_loop(self):
        while self.running and self.ser:
            try:
                data = self.ser.read(256)
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

                    try:
                        parsed = parse_frame(frame)
                    except Exception:
                        parsed = None
                    if not parsed:
                        continue

                    msg_type, seq, body = parsed
                    if msg_type & RSP_FLAG:
                        with self.lock:
                            self.rsp_data[seq] = body
            except Exception:
                time.sleep(0.05)

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

    def set_state(self, rx: bool) -> bool:
        b = self.send_cmd(MSG_SET_STATE, bytes([STATE_RX if rx else STATE_IDLE]))
        return bool(b and len(b) >= 1 and b[0] == 1)

    def read_ext(self, addr: int) -> Optional[int]:
        b = self.send_cmd(MSG_READ_EXT, bytes([addr & 0xFF]))
        if b and len(b) == 2 and b[0] == 1:
            return b[1]
        return None

    def get_metrics_raw(self) -> Optional[bytes]:
        # firmware currently returns 3 bytes: marc, rxbytes, rssi(status raw) :contentReference[oaicite:2]{index=2}
        return self.send_cmd(MSG_GET_METRICS, b"")


def pick_port() -> Optional[str]:
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        return None
    print("Available ports:")
    for i, p in enumerate(ports):
        print(f"  [{i}] {p.device} ({p.description})")
    while True:
        s = input("Select port index: ").strip()
        try:
            idx = int(s)
            if 0 <= idx < len(ports):
                return ports[idx].device
        except Exception:
            pass
        print("Bad selection.")


def force_rx(link: Link) -> bool:
    if not link.set_state(True):
        return False
    time.sleep(0.05)
    marc = link.read_ext(EXT_MARCSTATE)
    # You’ve seen RX as 0x6D; accept that. If it isn’t, we’ll still proceed but warn.
    return marc is not None


def diag_loop(link: Link, seconds: float = 20.0, hz: float = 4.0):
    period = 1.0 / hz
    t_end = time.time() + seconds

    rssi_ext_min = None
    rssi_ext_max = None
    rssi_ext_changes = 0
    last_ext = None

    print("\n--- DIAG LOOP (FORCES RX) ---")
    print("Do these while it runs:")
    print("  1) Touch antenna / feedline with finger")
    print("  2) Unscrew antenna")
    print("  3) Put a transmitter (SDR / handheld) near it and key on/off")
    print("We print BOTH:")
    print("  - EXT RSSI (RSSI1/RSSI0 decoded)")
    print("  - METRICS RSSI (firmware status RSSI byte from GET_METRICS)")
    print("If METRICS changes but EXT doesn't -> EXT mapping/decoding is wrong.")
    print("If neither changes -> RX chain likely not actually running.\n")

    while time.time() < t_end:
        marc = link.read_ext(EXT_MARCSTATE) or 0x00

        # If we are not in RX-ish state, re-strobe RX
        if marc != RX_MARC_OBSERVED:
            link.set_state(True)
            time.sleep(0.02)
            marc = link.read_ext(EXT_MARCSTATE) or marc

        rxbytes = link.read_ext(EXT_NUM_RXBYTES)
        rssi1 = link.read_ext(EXT_RSSI1)
        rssi0 = link.read_ext(EXT_RSSI0)

        dem = link.read_ext(EXT_DEM_STATUS)
        f1 = link.read_ext(EXT_FREQOFF_EST1)
        f0 = link.read_ext(EXT_FREQOFF_EST0)

        g3 = link.read_ext(EXT_AGC_GAIN3)
        g2 = link.read_ext(EXT_AGC_GAIN2)
        g1 = link.read_ext(EXT_AGC_GAIN1)
        g0 = link.read_ext(EXT_AGC_GAIN0)

        lqi = link.read_ext(EXT_LQI_VAL)
        pqt = link.read_ext(EXT_PQT_SYNC_ERR)

        # EXT RSSI decode
        rssi_ext_dbm = None
        if rssi1 is not None and rssi0 is not None:
            rssi_ext_dbm = decode_rssi_dbm_from_ext(rssi1, rssi0)

        # Track movement
        if rssi_ext_dbm is not None:
            if rssi_ext_min is None or rssi_ext_dbm < rssi_ext_min: rssi_ext_min = rssi_ext_dbm
            if rssi_ext_max is None or rssi_ext_dbm > rssi_ext_max: rssi_ext_max = rssi_ext_dbm
            if last_ext is not None and abs(rssi_ext_dbm - last_ext) >= 0.5:
                rssi_ext_changes += 1
            last_ext = rssi_ext_dbm

        # Firmware metrics (status RSSI byte)
        met = link.get_metrics_raw()
        met_str = "n/a"
        if met and len(met) >= 3:
            met_marc = met[0]
            met_rxbytes = met[1]
            met_rssi_raw = i8(met[2])
            met_str = f"MARC=0x{met_marc:02X} RXBYTES=0x{met_rxbytes:02X} RSSI_RAW(i8)={met_rssi_raw}"

        rssi_ext_str = "INVALID" if rssi_ext_dbm is None else f"{rssi_ext_dbm:7.2f} dBm"

        print(
            f"MARC=0x{marc:02X}  RXBYTES=0x{(rxbytes or 0):02X}  "
            f"RSSI1=0x{(rssi1 or 0):02X} RSSI0=0x{(rssi0 or 0):02X}  EXT_RSSI={rssi_ext_str}  "
            f"AGC=[{(g3 or 0):02X} {(g2 or 0):02X} {(g1 or 0):02X} {(g0 or 0):02X}]  "
            f"DEM=0x{(dem or 0):02X}  FREQOFF=0x{(f1 or 0):02X}{(f0 or 0):02X}  "
            f"LQI=0x{(lqi or 0):02X}  PQT_SYNC_ERR=0x{(pqt or 0):02X}  |  METRICS: {met_str}"
        )

        time.sleep(period)

    print("\n--- SUMMARY ---")
    if rssi_ext_min is None:
        print("EXT RSSI never became valid.")
    else:
        print(f"EXT RSSI min/max: {rssi_ext_min:.2f} .. {rssi_ext_max:.2f} dBm")
        print(f"EXT RSSI >=0.5 dB changes: {rssi_ext_changes} times")
    print("If EXT and METRICS RSSI never change -> RX chain not really alive, or RF path is dead.\n")


def main():
    port = pick_port()
    if not port:
        print("No serial ports found.")
        return

    link = Link()
    link.open(port, 115200)
    print(f"\n[OK] Connected to {port}")

    if not force_rx(link):
        print("[ERR] Failed to enter RX.")
        link.close()
        return

    # Run loop
    diag_loop(link, seconds=30.0, hz=4.0)
    link.close()


if __name__ == "__main__":
    main()
