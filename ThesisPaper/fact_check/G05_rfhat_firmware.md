# G5 — RF HAT Firmware

## Claim 1 — COBS + CRC-16/CCITT-FALSE at 115200
**Location:** ch05_firmware_software.tex:L97
**Quote:** "Binary protocol: COBS framing + CRC-16/CCITT-FALSE over UART 115200."
**Verdict:** OK
**Computed/Evidence:** config.h:L15 `UART_BAUD 115200`; crc16.h:L9 "CRC-16/CCITT-FALSE: poly=0x1021, init=0xFFFF, xorout=0x0000"; cobs.h declares encode/decode.

## Claim 2 — Message types and opcodes
**Location:** ch05_firmware_software.tex:L99-110
**Quote:** "SELECT_RADIO (0x40) ... SET_SERIAL_MODE (0x35) ... BUZZER (0x50) ... EVT_RX_DATA (0xE0)"
**Verdict:** OK
**Computed/Evidence:** proto.h:L42 MSG_SELECT_RADIO=0x40, L39 MSG_SET_SERIAL_MODE=0x35, L45 MSG_BUZZER=0x50, L49 EVT_RX_DATA=0xE0. READ/WRITE_REG=0x10/0x11, READ/WRITE_EXT=0x12/0x13, PROFILE_BEGIN/CHUNK/APPLY=0x31/0x32/0x33, SET_STREAMING=0x04, TX_WRITE/SEND=0x22/0x23, PING=0x01 (PONG implied via RSP_FLAG=0x80 OR).
**Note:** The claim says "PING/PONG" — PONG is not a distinct enum; response uses MSG_RSP_FLAG (0x80) OR'd with PING → 0x81. Accurate semantically.

## Claim 3 — SPI assignments and 5 MHz clock
**Location:** ch05_firmware_software.tex:L112
**Quote:** "SPI1 = UHF (GP10-13), SPI0 = VHF (GP16-19), 5 MHz clock"
**Verdict:** OK
**Computed/Evidence:** config.h:L18-23 UHF on spi1, GP10=SCK, GP11=MOSI, GP12=MISO, GP13=CSN, baud 5 MHz. L30-35 VHF on spi0, GP18=SCK, GP19=MOSI, GP16=MISO, GP17=CSN, baud 5 MHz.

## Claim 4 — 4x WS2812B NeoPixels on GP4
**Location:** ch05_firmware_software.tex:L114
**Quote:** "4x WS2812B NeoPixels on GP4"
**Verdict:** OK
**Computed/Evidence:** config.h:L48 `PIN_NEOPIXEL 4`, L49 `NEOPIXEL_COUNT 4`.

## Claim 5 — Buzzer on GP5
**Location:** ch05_firmware_software.tex:L114
**Quote:** "Non-blocking buzzer tone sequencer on GP5"
**Verdict:** OK
**Computed/Evidence:** config.h:L52 `PIN_BUZZER 5`. (main.cpp:L35 header comment says "on GP2" — stale comment; actual pin is GP5 via config.h.)

## Claim 6 — CC1200 serial-mode register config
**Location:** ch05_firmware_software.tex:L119-121
**Quote:** "PKT_FORMAT=0x01, FIFO_EN=0, SYNC_MODE=0x00"
**Verdict:** OK
**Computed/Evidence:** proto.c:L697 `pkt_cfg2 = (pkt_cfg2 & 0xFC) | 0x01` (PKT_FORMAT=01); L701 `mdmcfg1 &= ~(1u << 6)` (FIFO_EN bit cleared); L716 `sync_cfg1 &= 0x1F` clears SYNC_MODE bits[7:5] → blind mode.

## Claim 7 — PIO program snippet
**Location:** ch05_firmware_software.tex:L125-131
**Quote:** "wait 1 pin 2 / in pins, 1 / wait 0 pin 2"
**Verdict:** OK
**Computed/Evidence:** serial_rx.pio.h:L27-29 opcodes 0x20a2 (wait 1 pin 2), 0x4001 (in pins, 1), 0x2022 (wait 0 pin 2). Runtime patch at L79-81 substitutes `clk_offset = pin_clk - pin_rx = 9 - 7 = 2`, preserving the `pin 2` form.

## Claim 8 — ~7-8x oversampling ratio
**Location:** ch05_firmware_software.tex:L135
**Quote:** "CC1200 outputs samples at ~7-8x the configured symbol rate in blind mode"
**Verdict:** PARTIAL
**Computed/Evidence:** proto.c:L96-98 comment: "CC1200 blind mode outputs ~7-8x symbol rate". proto.c:L710-711 comment in serial_mode_enable says "~15x symbol rate". Two different figures in the firmware itself.
**Note:** Firmware measures the ratio at runtime (L899 `measured_bps = words * 80`), so the actual value is determined empirically rather than by datasheet spec; thesis figure should be verified against a measured pass.

## Claim 9 — 256-byte ring buffer, flush every 50ms or 32 bytes
**Location:** ch05_firmware_software.tex:L140
**Quote:** "256-byte ring buffer, forwarded to the Pi ... every 50 ms or 32 bytes"
**Verdict:** OK
**Computed/Evidence:** proto.c:L81 `SERIAL_BUF_SIZE 256`; L640 `bool timeout = (now - g_serial_last_send_ms) >= 50`; L643 `if (avail >= 32 || (avail > 0 && timeout))`.

## Claim 10 — G3RUH descrambler poly x^17+x^12+1
**Location:** ch05_firmware_software.tex:L143
**Quote:** "G3RUH descrambler: 17-bit LFSR, polynomial x^17 + x^12 + 1"
**Verdict:** OK
**Computed/Evidence:** ax25_decode.c:L42-56 `g3ruh_descramble`: tap at delay 12 (`sr >> 11`), tap at delay 17 (`sr >> 16`), 17-bit mask `0x1FFFF`. Comment L46 "polynomial x^17 + x^12 + 1".

## Claim 11 — NRZ-I decoder
**Location:** ch05_firmware_software.tex:L144
**Quote:** "NRZ-I decoder: differential to absolute"
**Verdict:** OK
**Computed/Evidence:** ax25_decode.c:L60-64 `nrzi_decode`: same-bit=1, transition=0.

## Claim 12 — HDLC deframer details
**Location:** ch05_firmware_software.tex:L145
**Quote:** "0x7E flag, bit-stuffing after 5 ones, LSB-first, CRC-16/CCITT (poly 0x8408, residual 0xF0B8), min frame 17 bytes"
**Verdict:** OK
**Computed/Evidence:** ax25_decode.c:L92-117 tracks 6 consecutive 1s for flag = 0x7E (01111110); L110-113 destuffs 0 after 5 ones; L99 `bit_buf = (bit_buf >> 1) | 0x80` → LSB-first byte assembly; L20 `crc ^= 0x8408`; L133 `if (crc == 0xF0B8)`; L120 `d->frame_len >= 17`.

## Claim 13 — station.py serial-mode selection and AX.100 sync word
**Location:** ch05_firmware_software.tex:L150
**Quote:** "station.py selects serial mode for >=4800 bps FSK/GFSK, FIFO mode with AX.100 sync word 0x930B51DE"
**Verdict:** OK
**Computed/Evidence:** Pi/station.py:L126 `sync_word = "930B51DE"  # GOMspace AX.100 Mode 5/6`; L133 `elif "G3RUH" in combined or ("AX.25" in combined and baud >= 4800) ...`; L159 `if baud >= 4800 and cc_fmt in ("2-FSK", "2-GFSK")` selects serial mode as unknown-protocol fallback.

## Claim 14 — CRC context disambiguation (0x8408 vs 0x1021)
**Location:** ch05_firmware_software.tex:L145 (0x8408) vs L156 (0x1021)
**Quote:** caption "CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF)"; body "CRC-16/CCITT (poly 0x8408, residual 0xF0B8)"
**Verdict:** OK
**Computed/Evidence:** Two different CRCs for two different contexts. COBS frame CRC (host↔firmware transport) uses CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF) per crc16.h:L9 and crc16.c. AX.25 HDLC FCS inside the demodulated bitstream uses reflected CRC-16/CCITT-FALSE (poly 0x8408 = bit-reversed 0x1021) per ax25_decode.c:L20. Both claims are correct for their respective contexts.
**Note:** Might be worth a one-sentence footnote in the thesis noting 0x8408 is the reflected form of 0x1021 to avoid reader confusion.

## Claim 15 — Appendix A RF HAT Pico pin table
**Location:** appendix_a_pinout.tex:L42-63
**Verdict:** OK
**Computed/Evidence:** Every row matches config.h:
- GP0/GP1 UART0 TX/RX ↔ PIN_UART_TX/RX
- GP4 NeoPixel ↔ PIN_NEOPIXEL; GP5 Buzzer ↔ PIN_BUZZER
- UHF (SPI1): GP6=GPIO3, GP7=GPIO0, GP8=RESET, GP9=GPIO2, GP10=SCK, GP11=MOSI, GP12=MISO, GP13=CSN
- VHF (SPI0): GP16=MISO, GP17=CSN, GP18=SCK, GP19=MOSI, GP20=RESET, GP21=GPIO2, GP22=GPIO0, GP26=GPIO3

## Claim 16 — Appendix B COBS opcode table
**Location:** appendix_b_protocol.tex:L37-73
**Verdict:** OK
**Computed/Evidence:** All 24 entries match proto.h msg_type_t enum (0x01 PING, 0x02 GET_INFO, 0x03 SET_STATE, 0x04 SET_STREAMING, 0x05 GET_METRICS, 0x10-0x13 REG/EXT, 0x20 RX_READ, 0x21 STROBE, 0x22-0x24 TX_*, 0x30-0x33 PROFILE_*, 0x34 SET_FREQ_WORD, 0x35 SET_SERIAL_MODE, 0x40 SELECT_RADIO, 0x50 BUZZER, 0xE0 EVT_RX_DATA, 0xE1 EVT_ERROR). BUZZER pattern range "0x01–0x08" matches main.cpp:L38-45 (8 defined patterns).
**Note:** ch05:L107 lists only 5 patterns (ready, AOS, LOS, packet, error); appendix B correctly says 0x01–0x08. Minor inconsistency — ch05 omits wifi_ok (0x06), gps_ok (0x07), setup_done (0x08).

## Claim 17 — CC1200 part=0x20 and version=0x11 verified at init
**Location:** ch06_results.tex:L114
**Quote:** "SPI readback confirms part ID 0x20 and version 0x11"
**Verdict:** PARTIAL
**Computed/Evidence:** main.cpp:L180-186 reads PARTNUMBER (ext 0x8F) and PARTVERSION (ext 0x90) but only validates `part == 0x20`. Version is printed for diagnostics, not checked. The claim that version reads as 0x11 is not enforced in firmware; the only evidence is any boot log or empirical observation on the deployed hardware.
**Note:** Part=0x20 claim is firmware-enforced and correct. Version=0x11 claim is plausible (standard CC1200 PARTVERSION) but not validated in code, so "SPI readback confirms version 0x11" should be backed by a boot-log excerpt or rephrased as "reads as 0x11 on the deployed chip".
