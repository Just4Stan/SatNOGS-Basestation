#!/usr/bin/env python3
"""RF loopback test — V1 board ↔ RF HAT over 435 MHz air link.

Tests both directions:
  A) RF HAT TX → V1 RX
  B) V1 TX → RF HAT RX

Uses the verified SmartRF profile from Pi/configs/smartrf_uhf_435.txt.
"""

import json, os, re, serial, struct, sys, time, glob

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
SMARTRF_PATH = os.path.join(SCRIPT_DIR, "Pi", "configs", "smartrf_uhf_435.txt")
REGMAP_PATH = os.path.join(SCRIPT_DIR, "Pi", "configs", "cc1200_regmap_profile.json")
EXPORT_LINE_RE = re.compile(r"^\s*0x([0-9A-Fa-f]{2}),\s*//\s*(\S+)")

# ---- COBS ----
def cobs_encode(data):
    out = bytearray()
    idx = 0
    while idx <= len(data):
        end = idx
        while end < len(data) and data[end] != 0 and (end - idx) < 254:
            end += 1
        out.append(end - idx + 1)
        out.extend(data[idx:end])
        if end < len(data) and data[end] == 0:
            end += 1
        idx = end
        if idx >= len(data):
            break
    return bytes(out)

def cobs_decode(data):
    out = bytearray()
    idx = 0
    while idx < len(data):
        code = data[idx]; idx += 1
        if code == 0: break
        for _ in range(code - 1):
            if idx < len(data): out.append(data[idx]); idx += 1
        if code < 0xFF and idx < len(data): out.append(0)
    if out and out[-1] == 0: out = out[:-1]
    return bytes(out)

# ---- CRC-16 ----
def crc16(data):
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021 if crc & 0x8000 else crc << 1) & 0xFFFF
    return crc

# ---- Frame I/O ----
def build_frame(msg_type, seq, body=b''):
    header = struct.pack('<BBH', msg_type, seq, len(body))
    payload = header + body
    payload += struct.pack('<H', crc16(payload))
    return cobs_encode(payload) + b'\x00'

def recv_frame(ser, timeout=2.0):
    buf = bytearray()
    deadline = time.time() + timeout
    while time.time() < deadline:
        chunk = ser.read(ser.in_waiting or 1)
        if chunk:
            buf.extend(chunk)
            while 0x00 in buf:
                idx = buf.index(0x00)
                frame = bytes(buf[:idx]); buf = buf[idx+1:]
                if len(frame) < 2: continue
                dec = cobs_decode(frame)
                if len(dec) < 6: continue
                mt, sq = dec[0], dec[1]
                plen = struct.unpack('<H', dec[2:4])[0]
                if 4 + plen + 2 > len(dec): continue
                body = dec[4:4+plen]
                if struct.unpack('<H', dec[4+plen:6+plen])[0] != crc16(dec[:4+plen]): continue
                return mt, sq, body
    return None, None, None

def send_cmd(ser, msg_type, seq, body=b'', timeout=2.0):
    ser.write(build_frame(msg_type, seq & 0xFF, body)); ser.flush()
    return recv_frame(ser, timeout)

# ---- Messages ----
MSG_PING=0x01; MSG_GET_INFO=0x02; MSG_SET_STATE=0x03; MSG_SET_STREAMING=0x04
MSG_GET_METRICS=0x05; MSG_WRITE_REG=0x11; MSG_WRITE_EXT=0x13; MSG_STROBE=0x21
MSG_TX_WRITE=0x22; MSG_TX_SEND=0x23; MSG_TX_FLUSH=0x24; MSG_SELECT_RADIO=0x40
SIDLE=0x36; SCAL=0x33; SFRX=0x3A; SFTX=0x3B; SRX=0x34; STX=0x35

# ---- SmartRF ----
# Registers that are read-only or status (skip during config)
RO_REGS = {
    "RSSI1","RSSI0","MARCSTATE","LQI_VAL","PQT_SYNC_ERR","DEM_STATUS",
    "FREQOFF_EST1","FREQOFF_EST0","AGC_GAIN3","AGC_GAIN2","AGC_GAIN1",
    "AGC_GAIN0","CFM_RX_DATA_OUT","CFM_TX_DATA_IN","ASK_SOFT_RX_DATA",
    "RNDGEN","MAGN2","MAGN1","MAGN0","ANG1","ANG0","CHFILT_I2","CHFILT_I1",
    "CHFILT_I0","CHFILT_Q2","CHFILT_Q1","CHFILT_Q0","GPIO_STATUS",
    "FSCAL_CTRL","PHASE_ADJUST","PARTNUMBER","PARTVERSION","SERIAL_STATUS",
    "MODEM_STATUS1","MODEM_STATUS0","MARC_STATUS1","MARC_STATUS0",
    "PA_IFAMP_TEST","FSRF_TEST","PRE_TEST","PRE_OVR","ADC_TEST","DVC_TEST",
    "ATEST","ATEST_LVDS","ATEST_MODE","XOSC_TEST1","XOSC_TEST0","AES",
    "MDM_TEST","RXFIRST","TXFIRST","RXLAST","TXLAST","NUM_TXBYTES",
    "NUM_RXBYTES","FIFO_NUM_TXBYTES","FIFO_NUM_RXBYTES","RXFIFO_PRE_BUF",
    "WOR_TIME1","WOR_TIME0","WOR_CAPTURE1","WOR_CAPTURE0","BIST",
    "DCFILTOFFSET_I1","DCFILTOFFSET_I0","DCFILTOFFSET_Q1","DCFILTOFFSET_Q0",
    "IQIE_I1","IQIE_I0","IQIE_Q1","IQIE_Q0",
}
for i in range(16): RO_REGS.add(f"AES_KEY{i}"); RO_REGS.add(f"AES_BUFFER{i}")

def load_smartrf():
    with open(REGMAP_PATH) as f: regmap = json.load(f)
    with open(SMARTRF_PATH) as f: lines = f.readlines()
    regs = []
    for line in lines:
        m = EXPORT_LINE_RE.match(line)
        if not m: continue
        val = int(m.group(1), 16); name = m.group(2).strip()
        if name in RO_REGS: continue
        ti_addr = regmap.get(name)
        if ti_addr is None: continue
        is_ext = ti_addr >= 0x2F00
        regs.append((1 if is_ext else 0, ti_addr & 0xFF, val))
    return regs

def write_config(ser, seq, regs):
    """Write registers with 3 retries each (matches GUI apply_profile_writes)."""
    fails = 0
    for is_ext, addr, val in regs:
        msg = MSG_WRITE_EXT if is_ext else MSG_WRITE_REG
        ok = False
        for _ in range(3):
            mt, _, body = send_cmd(ser, msg, seq & 0xFF, bytes([addr, val]), timeout=0.3)
            seq = (seq + 1) & 0xFF
            if mt is not None and body and body[0] == 1:
                ok = True; break
            time.sleep(0.005)
        if not ok: fails += 1
    return fails, seq & 0xFF

def identify(ser):
    ser.reset_input_buffer(); time.sleep(0.2)
    ser.read(ser.in_waiting or 1); time.sleep(0.1); ser.reset_input_buffer()
    mt, _, body = send_cmd(ser, MSG_PING, 0xAA, timeout=1.0)
    if mt is None: return None, "no PING response"
    mt, _, body = send_cmd(ser, MSG_GET_INFO, 0xAB, timeout=1.0)
    if mt is None: return None, "no GET_INFO response"
    if len(body) == 4: return "rf_hat", f"part=0x{body[2]:02X} ver=0x{body[3]:02X}"
    if len(body) == 2: return "v1", f"part=0x{body[0]:02X} ver=0x{body[1]:02X}"
    return "unknown", f"body={body.hex()}"

def check_rx(ser, expected, timeout=3.0):
    """Check for EVT_RX_DATA events containing expected payload."""
    buf = bytearray()
    deadline = time.time() + timeout
    while time.time() < deadline:
        chunk = ser.read(ser.in_waiting or 1)
        if chunk:
            buf.extend(chunk)
            while 0x00 in buf:
                idx = buf.index(0x00)
                frame = bytes(buf[:idx]); buf = buf[idx+1:]
                if len(frame) < 2: continue
                dec = cobs_decode(frame)
                if len(dec) < 6: continue
                mt = dec[0]
                plen = struct.unpack('<H', dec[2:4])[0]
                if 4+plen+2 > len(dec): continue
                body = dec[4:4+plen]
                if struct.unpack('<H', dec[4+plen:6+plen])[0] != crc16(dec[:4+plen]): continue
                if mt == 0xE0 and len(body) >= 2:  # EVT_RX_DATA
                    n = body[0]; rx = body[1:1+n]
                    print(f"    RX: {n} bytes: {rx.hex()}")
                    # Check if expected payload appears anywhere in received data
                    if expected in rx:
                        return True
                    # Variable-length: [pkt_len] [payload] [2-byte status]
                    if len(rx) >= 1:
                        pkt_len = rx[0]
                        pkt = rx[1:1+pkt_len]
                        if pkt == expected:
                            return True
        time.sleep(0.01)
    return False

# ---- Main ----
def main():
    ports = sorted(glob.glob('/dev/cu.usbmodem*'))
    if len(ports) < 2:
        print(f"Need 2 ports, found {len(ports)}: {ports}"); sys.exit(1)

    print(f"Ports: {ports}\n")
    boards = {}
    for port in ports:
        try:
            ser = serial.Serial(port, 115200, timeout=0.5); time.sleep(0.3)
            bt, info = identify(ser); ser.close()
            print(f"  {port}: {bt or 'UNKNOWN'} — {info}")
            if bt: boards[bt] = port
        except Exception as e:
            print(f"  {port}: ERROR — {e}")

    print()
    if 'rf_hat' not in boards or 'v1' not in boards:
        print("Need both RF HAT and V1 board."); sys.exit(1)

    regs = load_smartrf()
    print(f"SmartRF: {len(regs)} registers from {os.path.basename(SMARTRF_PATH)}\n")

    hat = serial.Serial(boards['rf_hat'], 115200, timeout=0.5)
    v1 = serial.Serial(boards['v1'], 115200, timeout=0.5)
    time.sleep(0.3); hat.reset_input_buffer(); v1.reset_input_buffer()
    sh = 1; sv = 1

    print("=" * 60)
    print("RF LOOPBACK TEST — 435 MHz, 2-GFSK, 2.4 kbps")
    print(f"  RF HAT: {boards['rf_hat']}")
    print(f"  V1:     {boards['v1']}")
    print("=" * 60 + "\n")

    # Select UHF on RF HAT
    send_cmd(hat, MSG_SELECT_RADIO, sh, bytes([0x00])); sh += 1

    # Configure both
    for label, ser, is_hat in [("RF HAT", hat, True), ("V1", v1, False)]:
        sq = sh if is_hat else sv
        # IDLE before config — matches GUI workflow
        send_cmd(ser, MSG_SET_STATE, sq, bytes([0])); sq += 1
        time.sleep(0.05)
        f, sq = write_config(ser, sq, regs)
        print(f"  Config {label}: {len(regs)} regs, {f} failures")
        send_cmd(ser, MSG_STROBE, sq, bytes([SCAL])); sq += 1
        if is_hat: sh = sq
        else: sv = sq
    time.sleep(0.3)

    test_data = b'\xDE\xAD\xBE\xEF\xCA\xFE\x01\x02\x03\x04'
    # Variable-length mode (PKT_CFG0=0x20): first FIFO byte = packet length field
    tx_fifo = bytes([len(test_data)]) + test_data
    results = {}

    # ======== TEST A: RF HAT TX → V1 RX ========
    print(f"\n--- TEST A: RF HAT TX → V1 RX ---")

    # V1 → RX
    send_cmd(v1, MSG_STROBE, sv, bytes([SFRX])); sv += 1
    send_cmd(v1, MSG_SET_STATE, sv, bytes([1])); sv += 1
    send_cmd(v1, MSG_SET_STREAMING, sv, bytes([1])); sv += 1
    time.sleep(0.05)

    # RF HAT → TX x3
    for i in range(3):
        # flags 0x03 = SFTX (flush) + SIDLE (force idle before write) — matches GUI
        send_cmd(hat, MSG_TX_WRITE, sh, bytes([0x03, len(tx_fifo)]) + tx_fifo); sh += 1
        # flags 0x01 = SIDLE after TX — matches GUI tx_send(go_idle=True)
        send_cmd(hat, MSG_TX_SEND, sh, bytes([0x01])); sh += 1
        print(f"  HAT TX {i+1}/3")
        time.sleep(0.5)

    results['A'] = check_rx(v1, test_data, timeout=3.0)
    print(f"  Result: {'PASS' if results['A'] else 'FAIL'}")

    # ======== TEST B: V1 TX → RF HAT RX ========
    print(f"\n--- TEST B: V1 TX → RF HAT RX ---")

    # RF HAT → RX
    send_cmd(hat, MSG_STROBE, sh, bytes([SIDLE])); sh += 1
    send_cmd(hat, MSG_STROBE, sh, bytes([SFRX])); sh += 1
    send_cmd(hat, MSG_SET_STATE, sh, bytes([1])); sh += 1
    send_cmd(hat, MSG_SET_STREAMING, sh, bytes([1])); sh += 1
    time.sleep(0.05)

    # V1 → TX x3 (same flow as GUI: tx_write with flush+idle, then tx_send with go_idle)
    for i in range(3):
        send_cmd(v1, MSG_TX_WRITE, sv, bytes([0x03, len(tx_fifo)]) + tx_fifo); sv += 1
        send_cmd(v1, MSG_TX_SEND, sv, bytes([0x01])); sv += 1
        print(f"  V1 TX {i+1}/3")
        time.sleep(0.5)

    results['B'] = check_rx(hat, test_data, timeout=3.0)
    if not results['B']:
        # Debug
        mt, _, body = send_cmd(hat, MSG_GET_METRICS, sh, timeout=1.0); sh += 1
        if body and len(body) >= 28 and body[0] == 2:
            print(f"  HAT: MARC=0x{body[4]:02X} RXBYTES={body[5]} RSSI={struct.unpack_from('<h',body,8)[0]/10:.1f}dBm")
    print(f"  Result: {'PASS' if results['B'] else 'FAIL'}")

    # ======== Summary ========
    print(f"\n{'='*60}")
    print(f"  TEST A (HAT→V1): {'PASS' if results['A'] else 'FAIL'}")
    print(f"  TEST B (V1→HAT): {'PASS' if results['B'] else 'FAIL'}")
    if all(results.values()):
        print("  *** BIDIRECTIONAL LOOPBACK VERIFIED ***")
    print(f"{'='*60}")

    hat.close(); v1.close()
    return 0 if any(results.values()) else 1

if __name__ == '__main__':
    sys.exit(main())
