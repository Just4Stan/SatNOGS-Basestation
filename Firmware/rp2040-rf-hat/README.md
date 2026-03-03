# RF HAT Firmware — Dual CC1200 Controller

PlatformIO firmware for the RF HAT Pico, controlling two TI CC1200 sub-GHz transceivers
(UHF 432 MHz + VHF 144 MHz) and communicating with the Raspberry Pi 3A+ over UART.

Based on [CC1200RXFrontend](https://github.com/RobbeLehaen/CC1200RXFrontend) by Robbe Lehaen,
adapted for the RF HAT dual-radio hardware.

See the [Firmware README](../README.md) for how this fits into the full ground station system.

## Architecture

```
Pi 3A+ (satnogs-client + packet capture)
  │
  ├── USB serial (/dev/ttyACM0) ──→ Motor Pico   (rotator, hamlib/rotctld)
  │
  └── UART (GPIO14/15, 115200)  ──→ RF HAT Pico  (this firmware)
                                      ├── SPI1 → CC1200 UHF (432 MHz)
                                      └── SPI0 → CC1200 VHF (144 MHz)
```

## Quick Start

```bash
cd Firmware/rp2040-rf-hat
pio run                    # build → rf_hat_firmware.uf2 (copied to repo root)
pio run -t upload          # flash via BOOTSEL
pio device monitor         # USB debug serial (115200)
```

## Hardware Pin Map (from `rf_hat_circuit.py`)

| Function         | GPIO | Pico Pin | Notes              |
|------------------|------|----------|--------------------|
| **UART TX**      | GP0  | 1        | → Pi GPIO15 (RXD)  |
| **UART RX**      | GP1  | 2        | ← Pi GPIO14 (TXD)  |
| **UHF SCLK**     | GP10 | 14       | SPI1               |
| **UHF MOSI**     | GP11 | 15       | SPI1               |
| **UHF MISO**     | GP12 | 16       | SPI1               |
| **UHF CSN**      | GP13 | 17       | SPI1               |
| **UHF RESET**    | GP8  | 11       |                    |
| **UHF GPIO0**    | GP7  | 10       |                    |
| **UHF GPIO2**    | GP9  | 12       |                    |
| **UHF GPIO3**    | GP6  | 9        |                    |
| **VHF SCLK**     | GP18 | 24       | SPI0               |
| **VHF MOSI**     | GP19 | 25       | SPI0               |
| **VHF MISO**     | GP16 | 21       | SPI0               |
| **VHF CSN**      | GP17 | 22       | SPI0               |
| **VHF RESET**    | GP20 | 26       |                    |
| **VHF GPIO0**    | GP22 | 29       |                    |
| **VHF GPIO2**    | GP21 | 27       |                    |
| **VHF GPIO3**    | GP26 | 31       |                    |
| **Buzzer**       | GP4  | 6        | Passive piezo via NPN/MOSFET |
| **NeoPixel**     | GP5  | 7        | 4x WS2812B status LEDs |
| **LED**          | GP25 | —        | Pico onboard       |

## How the Codebase Works

### CC1200 Driver (`lib/cc1200_driver/`)

Hardware-agnostic CC1200 SPI driver, originally from Robbe's CC1200RXFrontend.
Bug-fixed in this version (see [Code Review Notes](#code-review-notes--bug-fixes)).

| File | Lines | Purpose |
|------|-------|---------|
| `cc1200_hal.h` | ~60 | HAL struct definition + inline helpers |
| `cc1200_hal.c` | 106 | RP2040 SPI + GPIO initialization, chip-select with SO-ready wait, reset pulse, raw SPI transfer |
| `cc1200.h` | ~100 | CC1200 API: register/FIFO/strobe/extended access, command enums |
| `cc1200.c` | 239 | All CC1200 SPI transactions: single reg, burst, extended, FIFO, strobes |

**Key types:**

- **`cc1200_hal_t`** — HAL configuration struct:
  - `spi_inst_t* spi` — Pico SDK SPI instance (spi0 or spi1)
  - `pin_sck/mosi/miso/csn/reset` — GPIO assignments
  - `baud_hz` — SPI clock rate (5 MHz)
  - `so_ready_timeout_us` — max wait time for MISO low after CSN assert (5 ms)
  - `cfg_burst_interbyte_delay_cycles` — computed at init for >=100 ns delay between burst writes

- **`cc1200_t`** — Device handle. Contains a `cc1200_hal_t` member. One per physical CC1200.

- **`cc1200_status_t`** — Decoded chip status byte:
  - `chip_ready` — bit 7 inverted (0 = ready)
  - `state` — bits 6:4 (IDLE, RX, TX, FSTXON, CALIBRATE, SETTLING, RXFIFO_ERROR, TXFIFO_ERROR)
  - `fifo_bytes` — bits 3:0 (number of bytes available in FIFO)

- **`cc1200_cmd_t`** — Strobe commands: `SRES` (0x30), `SRX` (0x34), `STX` (0x35), `SIDLE` (0x36), `SFRX` (0x3A), `SFTX` (0x3B), etc.

**CC1200 SPI protocol (from SWRS123D datasheet):**

The CC1200 uses a byte-oriented SPI interface. Every transaction starts by pulling CSN low
and waiting for SO (MISO) to go low — this is the "SO-ready" handshake indicating the CC1200's
internal state machine is ready. The first byte sent is a header that encodes the operation:

```
Header byte: [R/W(1)] [BURST(1)] [ADDR(6)]

Bit 7: 1 = read, 0 = write
Bit 6: 1 = burst (auto-increment address), 0 = single
Bits 5:0: register address
```

Register spaces:
- **Standard registers** (0x00-0x2E): 2-byte transfer (header + data)
- **Extended registers** (via address 0x2F): 3-byte transfer (0x2F header + ext_addr + data)
- **Strobes** (0x30-0x3D): 1-byte command, returns status byte
- **FIFO** (0x3F): burst read/write for packet data

For burst writes to the config register space (0x00-0x2E), the CC1200 requires a minimum
100 ns inter-byte delay. The HAL computes the required NOP count at init time based on the
system clock frequency.

**`cc1200_hal_init()`** — Initializes the SPI peripheral, configures GPIO functions, sets up CSN
as a GPIO output (initially high), configures the reset pin, and computes the burst delay.

**`cc1200_hal_select()`** — Pulls CSN low, then polls MISO until it reads 0 (or timeout).
This is critical — the CC1200 holds MISO high while its crystal is starting or while it's
processing the previous command.

**`cc1200_begin()`** — Full reset sequence: hardware reset pulse (1ms low, 1ms post), then
send SRES strobe, then wait for SO-ready again. After this, the CC1200 is in IDLE state with
default register values.

### Firmware (`src/`)

| File | Lines | Purpose |
|------|-------|---------|
| `main.cpp` | ~230 | Arduino entry point: init UART + both CC1200s + NeoPixel + buzzer + protocol handler, main loop |
| `config.h` | ~60 | Pin definitions, SPI baud rates, radio constants |
| `proto.c` | 542 | Full binary protocol handler: COBS frame parsing, command dispatch, profile staging, RX streaming, TX helpers |
| `proto.h` | 63 | Message type enums and API |
| `cobs.c` | ~80 | COBS encoder/decoder |
| `cobs.h` | ~20 | COBS API |
| `crc16.c` | ~30 | CRC-16/CCITT-FALSE implementation |
| `crc16.h` | ~15 | CRC API |

#### Entry Point (`main.cpp`)

`setup()` runs:

1. **USB serial** (115200 baud) — debug output only, not used for protocol
2. **Onboard LED** configured for heartbeat
3. **NeoPixel** (4x WS2812B on GP3) — all white during init
4. **Buzzer** (GP2 PWM) — hardware PWM initialized
5. **UART0** initialized at 115200 baud, GP0 TX, GP1 RX, 8N1, FIFO enabled
6. **UHF CC1200** initialized on SPI1 — `init_radio()` configures the HAL, calls `cc1200_begin()` (hardware reset + SRES strobe), then reads part number register (ext addr 0x8F) to verify CC1200 responds (expected: 0x20)
7. **VHF CC1200** initialized on SPI0 — same sequence
8. **Protocol handler** initialized with both radio instances + buzzer callback
9. **NeoPixel** → green + triple beep if both radios OK; red + error beep if either failed

`loop()` runs:
- **500ms heartbeat**: toggle LED + update NeoPixel color (green=idle, blue=RX streaming, red=fault)
- **`buzzer_tick()`**: advance non-blocking buzzer tone sequencer
- **`proto_poll()`**: read UART bytes, decode COBS frames, dispatch commands, stream RX data
- **1ms yield**: `delay(1)` for USB serial processing

#### Protocol Handler (`proto.c`)

This is the heart of the firmware. It implements a binary request/response protocol over UART
using COBS framing.

**Frame structure on the wire:**
```
┌──────────────────────────────┐
│ COBS_ENCODE(payload) │ 0x00  │   ← wire bytes
└──────────────────────────────┘
         │
         ▼ (after COBS decode)
┌──────┬─────┬──────┬──────────┬───────┐
│ type │ seq │ len  │   body   │ crc16 │   ← payload
│  1B  │ 1B  │ 2B   │  len B   │  2B   │
│      │     │  LE  │          │  LE   │
└──────┴─────┴──────┴──────────┴───────┘
```

**COBS (Consistent Overhead Byte Stuffing):** Replaces all 0x00 bytes in the payload with
distance-to-next-zero markers, then uses a 0x00 byte as the frame delimiter. This guarantees
the delimiter never appears inside the frame — no escaping ambiguity, low overhead (~0.4%).

**`proto_poll()`** — Called from the main loop. Reads bytes from UART0:
- Non-zero bytes are accumulated in `g_rx_enc[]` (max 512 bytes)
- A 0x00 byte triggers end-of-frame processing:
  1. COBS-decode the accumulated bytes into `g_rx_dec[]`
  2. Validate minimum frame size (6 bytes: type + seq + len + crc)
  3. Verify payload length matches encoded length field
  4. Compute CRC-16 over the header+body and compare with the received CRC
  5. If valid, call `handle_cmd()` with the message type, sequence, and body
- After processing commands, calls `maybe_stream_rx()` to push any pending RX FIFO data

**`handle_cmd()`** — Giant switch statement dispatching on message type:

- **`MSG_PING` (0x01)**: Returns "PONG" — basic connectivity test
- **`MSG_GET_INFO` (0x02)**: Returns active radio index, radio count, CC1200 part number and version
- **`MSG_SELECT_RADIO` (0x40)**: Switches active radio (0=UHF, 1=VHF). Resets streaming and clears any staged profile.
- **`MSG_SET_STATE` (0x03)**: Sends SIDLE, SRX, or STX strobe to the active CC1200
- **`MSG_SET_STREAMING` (0x04)**: Enables/disables automatic RX FIFO push (via `maybe_stream_rx()`)
- **`MSG_GET_METRICS` (0x05)**: Reads MARCSTATE, RX FIFO count, and RSSI (12-bit, converted to dBm x10)
- **`MSG_READ_REG`/`MSG_WRITE_REG` (0x10/0x11)**: Standard register access (0x00-0x2E)
- **`MSG_READ_EXT`/`MSG_WRITE_EXT` (0x12/0x13)**: Extended register access
- **`MSG_RX_READ` (0x20)**: Read up to N bytes from RX FIFO
- **`MSG_STROBE` (0x21)**: Send a strobe command (validated to 0x30-0x3D range)
- **`MSG_TX_WRITE` (0x22)**: Write data to TX FIFO with optional IDLE/flush before write
- **`MSG_TX_SEND` (0x23)**: Send STX strobe with optional post-TX state (IDLE or RX)
- **`MSG_TX_FLUSH` (0x24)**: Flush TX FIFO
- **`MSG_PROFILE_*` (0x30-0x33)**: Bulk register configuration (see below)

**Profile system:**
The profile system allows loading a complete SmartRF Studio register configuration in chunks,
then applying it atomically:

1. `PROFILE_CLEAR` — reset staging buffer
2. `PROFILE_BEGIN` — start a new profile load (up to 768 entries)
3. `PROFILE_CHUNK` — append register entries: each is `[type(1) addr(1) val(1)]` where type bit 0 selects extended (1) vs standard (0) register space
4. `PROFILE_APPLY` — force IDLE, poll MARCSTATE to confirm, write all staged registers, then optionally flush FIFOs and/or enter RX

**RX streaming (`maybe_stream_rx()`):**
When streaming is enabled, this function polls `NUM_RXBYTES` and reads the RX FIFO in 64-byte
chunks, sending each as an `EVT_RX_DATA` event frame. This allows the Pi to receive packets
without polling — the Pico pushes data as it arrives.

**RSSI calculation (`read_metrics()`):**
The CC1200 provides a 12-bit signed RSSI value across two registers (RSSI1 and RSSI0[7:4]).
The firmware sign-extends this, scales it to dBm x10 (divide by 16, multiply by 10), and also
provides a 1-byte integer dBm approximation. An RSSI of -2048 means "not valid" (not yet measured).

**`send_frame()`** — Builds a response/event frame:
1. Assemble payload: type + seq + len(LE) + body + CRC-16(LE)
2. COBS-encode into `g_tx_enc[]`
3. Write encoded bytes + 0x00 delimiter to UART0

### PC/Pi Test Tools (`tools/`)

| File                       | Purpose                                          |
|----------------------------|--------------------------------------------------|
| `cc1200_gui.py`            | Python GUI (CustomTkinter) for interactive CC1200 control: register read/write, profile load, RX streaming, RSSI display |
| `cc1200_diag.py`           | CLI diagnostic tool: ping, info, register dump, profile load, RX test |
| `generate_regmap.py`       | Generates `cc1200_regmap_all.json` from TI's `cc120x_spi.h` header file |
| `cc120x_spi.h`            | TI CC120X register definitions (source for generate_regmap.py) |
| `cc1200_regmap_all.json`   | Complete register name→address map (generated) |
| `cc1200_regmap_profile.json` | Profile-applicable registers only (subset) |
| `smartrf_config.txt`       | SmartRF Studio register export (435 MHz, 2.4 kbps, 2-GFSK) |

The Python tools implement the same COBS protocol and can talk to the Pico over either
USB serial (original Robbe setup) or UART serial (via a USB-UART adapter or directly from the Pi).

## Binary Protocol (COBS over UART)

### Frame Format

```
Wire: COBS_ENCODE(payload) + 0x00 delimiter

Payload: type(1) + seq(1) + len(2 LE) + body(len) + crc16(2 LE)
```

CRC-16/CCITT-FALSE: poly=0x1021, init=0xFFFF, no refin/refout, xorout=0.

### Message Types

**Control:**
| Code | Name              | Body (request)      | Body (response)           |
|------|-------------------|----------------------|---------------------------|
| 0x01 | PING              | —                    | "PONG"                    |
| 0x02 | GET_INFO          | —                    | active(1) count(1) part(1) ver(1) |
| 0x03 | SET_STATE         | state(1): 0=IDLE 1=RX 2=TX | ok(1)               |
| 0x04 | SET_STREAMING     | enable(1)            | enabled(1)                |
| 0x05 | GET_METRICS       | —                    | marc(1) rxbytes(1) rssi_1db(i8) rssi_dbm_x10(i16 LE) |
| 0x40 | SELECT_RADIO      | index(1): 0=UHF 1=VHF | active(1)              |
| 0x50 | BUZZER            | pattern(1)           | ok(1)                   |

BUZZER pattern IDs: 0x01=ready (triple beep), 0x02=AOS (rising), 0x03=LOS (falling), 0x04=packet (short), 0x05=error (low).

**Register I/O:**
| Code | Name        | Body (request)     | Body (response)     |
|------|-------------|---------------------|---------------------|
| 0x10 | READ_REG    | addr(1)            | ok(1) value(1)      |
| 0x11 | WRITE_REG   | addr(1) value(1)   | ok(1)               |
| 0x12 | READ_EXT    | ext_addr(1)        | ok(1) value(1)      |
| 0x13 | WRITE_EXT   | ext_addr(1) value(1) | ok(1)             |

**FIFO / Strobes:**
| Code | Name      | Body                | Response              |
|------|-----------|----------------------|-----------------------|
| 0x20 | RX_READ   | max_bytes(1)        | count(1) data(count)  |
| 0x21 | STROBE    | cmd(1) [0x30-0x3D]  | status_raw(1)         |

**TX:**
| Code | Name      | Body                           | Response              |
|------|-----------|--------------------------------|-----------------------|
| 0x22 | TX_WRITE  | flags(1) count(1) data(count)  | ok(1) written(1)      |
| 0x23 | TX_SEND   | flags(1)                       | ok(1)                 |
| 0x24 | TX_FLUSH  | —                              | ok(1)                 |

TX_WRITE flags: bit 0 = flush TX FIFO before write, bit 1 = IDLE before write.
TX_SEND flags: bit 0 = IDLE after TX, bit 1 = RX after TX, bit 2 = flush TX FIFO first.

**Profile (bulk register config from SmartRF Studio):**
| Code | Name           | Body                              | Response                    |
|------|----------------|------------------------------------|-----------------------------|
| 0x30 | PROFILE_CLEAR  | —                                 | ok(1)                       |
| 0x31 | PROFILE_BEGIN  | expected_count(2 LE)              | ok(1)                       |
| 0x32 | PROFILE_CHUNK  | count(1) [type(1) addr(1) val(1)] x count | ok(1) total(2 LE)  |
| 0x33 | PROFILE_APPLY  | flags(1)                          | ok(1)                       |

PROFILE_CHUNK type field: bit 0 = 1 for extended register, 0 for standard register.
PROFILE_APPLY flags: bit 0 = enter RX, bit 1 = IDLE, bit 2 = flush RX FIFO, bit 3 = flush TX FIFO.

**Responses** have `type | 0x80` and mirror the request `seq`.
**Async events** use `seq=0`: `0xE0` EVT_RX_DATA, `0xE1` EVT_ERROR.

### Typical Usage Sequence

```
1. PING                        → verify connectivity
2. GET_INFO                    → check CC1200 part number
3. SELECT_RADIO(0)             → select UHF
4. PROFILE_CLEAR               → reset profile buffer
5. PROFILE_BEGIN(count)         → start profile load
6. PROFILE_CHUNK(entries...)    → send register values (multiple chunks)
7. PROFILE_APPLY(0x05)         → apply profile, flush RX, enter RX
8. SET_STREAMING(1)            → enable RX data push
9. ... receive EVT_RX_DATA ... → packets arrive asynchronously
10. GET_METRICS                → check RSSI, MARCSTATE
```

## SmartRF Register Config (435 MHz)

The included `smartrf_config.txt` configures:

| Parameter         | Value           |
|-------------------|-----------------|
| Frequency         | 435.000 MHz     |
| Modulation        | 2-GFSK          |
| Symbol rate       | ~2.4 ksps       |
| Deviation         | ~5.0 kHz        |
| RX bandwidth      | ~29.8 kHz       |
| TX power          | +14 dBm (max)   |
| Packet format     | Variable-length, CRC-16 |
| Sync word         | 16-bit: 0x7A0E  |
| Preamble          | 3 bytes of 0xAA |

The modulation index (~4.17) is higher than typical for 2-GFSK at this data rate. This is
intentional for the AetherSpace CubeSat link budget — wider deviation improves sensitivity
at the cost of occupied bandwidth.

## Code Review Notes / Bug Fixes

Issues found in the original CC1200RXFrontend code. All except #2 have been **fixed** in this version:

1. **`send_frame()` buffer overflow** (FIXED) — COBS output could exceed `TX_ENC_MAX` (was 512)
   for payloads near max size. Fixed: increased to 520 and capped body_len at `sizeof(g_tx_payload) - 8`.

2. **`cc1200_read_status()` broken** (NOT FIXED — function unused) — Uses standard-space header
   for status registers that live in extended space. The function exists in the driver but is never
   called. Use `cc1200_read_ext()` instead for reading status registers.

3. **`profile_apply()` didn't confirm IDLE** (FIXED) — After SIDLE strobe, registers were written
   immediately without confirming the CC1200 had actually entered IDLE state. Fixed: now polls
   MARCSTATE for up to 5ms after SIDLE strobe.

4. **Burst inter-byte delay borderline** (FIXED) — The 100 ns inter-byte delay for config register
   burst writes was exactly at the CC1200 minimum (12 NOPs at 125 MHz = 96 ns). Fixed: added
   +2 cycle margin to guarantee >=100 ns.

5. **No strobe range validation** (FIXED) — `MSG_STROBE` accepted any byte value. Sending a
   value outside 0x30-0x3D would trigger an unintended register read/write instead of a command
   strobe. Fixed: range check added, returns error for invalid values.

6. **Register address check too loose** (FIXED) — `cc1200_read_reg()`/`cc1200_write_reg()` used
   `(addr & 0xC0u) != 0` which allowed addresses 0x2F-0x3F through (including the extended access
   prefix and strobes). Fixed: changed to `addr > 0x2Eu`.

## SatNOGS Integration

SatNOGS is SDR-centric (GNU Radio + SoapySDR). The CC1200 is a **hardware packet modem**, not an SDR —
it does not produce IQ samples. Integration options:

1. **Recommended**: Run `satnogs-client` for rotator scheduling + pointing. Capture CC1200 packets
   independently via a Python script on the Pi (reading UART). Submit decoded frames to
   [SatNOGS DB](https://db.satnogs.org/) via the SiDS API.

2. **Tighter**: Use satnogs-client pre/post observation scripts to automate CC1200 tuning and
   packet capture around scheduled passes.

3. **Doppler**: Write a minimal `rigctld` shim that translates `set_freq` hamlib commands to
   CC1200 frequency register writes via the UART protocol.

4. **Hybrid**: Add an RTL-SDR (~8 EUR) for standard SatNOGS waterfall/audio/IQ artifacts.
   Use CC1200 as the primary packet decoder (hardware demod = more reliable for narrow-band FSK).
