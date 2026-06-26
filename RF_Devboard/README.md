# CC1200RXFrontend

Firmware for an RP2040-based CC1200 “RF frontend” that controls a TI **CC1200** sub-GHz transceiver and exposes a **binary protocol over USB CDC**. The intended use case is a **SatNOGS groundstation** host that configures the CC1200, switches RX/TX state, monitors metrics, ingests received bytes (either polled or streamed), and can perform basic **TX test transmissions**.

Key features:
- USB CDC **binary protocol** (COBS framing + CRC16) for control, telemetry, optional RX streaming, and basic TX commands
- CC1200 SPI driver supporting normal + extended registers, FIFO reads/writes, and command strobes
- Host reference implementation and GUI in `PC_test/cc1200_gui.py` (includes a **Send Hex** TX test)
- RP2040 LED status indication

> Critical constraint: USB CDC stdio is used for the binary protocol. Printing debug text over the same interface will corrupt the stream.

---

## Repository layout

- `src/main.c`  
  System init, CC1200 bring-up, protocol polling loop.

- `src/usb_proto.[ch]`  
  Binary protocol framing/parsing and command dispatch. Handles RX streaming, metrics collection, and TX FIFO / TX start commands.

- `src/usb_cobs.[ch]`  
  COBS encoder/decoder used by the transport.

- `src/crc16.[ch]`  
  CRC-16/CCITT-FALSE implementation used by the transport.

- `src/config.h`  
  RP2040 pin mapping + SPI configuration.

- `src/leds.[ch]`  
  RGB status LED control and heartbeat.

- `CC1200_driver/cc1200.[ch]`  
  CC1200 low-level driver: register access, FIFO access, strobes.

- `CC1200_driver/cc1200_hal.[ch]`  
  Pico SDK SPI/GPIO HAL for CC1200. Includes SO(MISO)-ready wait on CS assert.

- `src/cc1200_profile.c`  
  Helper for applying a prepared list of register writes.

- `PC_test/cc1200_gui.py`  
  Reference host-side implementation and GUI (serial framing/parsing, SmartRF export parsing, register I/O, state control, RX ingest, **TX “Send Hex” test**).

---

## Hardware assumptions

### Default RP2040 pin mapping

Defined in `src/config.h`:

**LEDs**
- `GPIO12` : Blue
- `GPIO13` : Green
- `GPIO14` : Red
- `GPIO15` : Extra / heartbeat

**CC1200 (SPI0)**
- `GPIO18` : SCK
- `GPIO19` : MOSI
- `GPIO20` : MISO (also used for SO-ready)
- `GPIO21` : CSN
- `GPIO22` : CC1200 GPIO0
- `GPIO23` : CC1200 GPIO2
- `GPIO24` : CC1200 GPIO3
- `GPIO25` : CC1200 RESET

SPI frequency defaults to **5 MHz**. SO-ready timeout defaults to **5000 µs**.

### SPI mode and SO-ready behavior

- SPI is configured for **Mode 0**, 8-bit, MSB-first.
- On CSN assert (low), the HAL waits for CC1200 **SO (wired to MISO)** to go low, with a timeout. If SO does not go ready, the transfer is aborted.

---

## Firmware lifecycle

At boot:
1. Initializes USB CDC stdio and LEDs.
2. Initializes CC1200 HAL (SPI + GPIO) from `config.h`.
3. Resets CC1200 and verifies communication.
4. Optionally reads CC1200 identification fields (part/version) to validate the transceiver.
5. Enters a polling loop:
   - toggles heartbeat
   - processes incoming USB frames and emits responses/events
   - (optionally) drains RX FIFO and emits RX streaming events
   - sleeps briefly

---

## USB CDC binary protocol

### Transport framing

Frames are transmitted as:
- `COBS(payload) + 0x00` delimiter

Payload layout (little-endian where applicable):
- `type`  : `u8`
- `seq`   : `u8`
- `len`   : `u16` length of `body`
- `body`  : `len` bytes
- `crc16` : `u16` CRC-16/CCITT-FALSE computed over `type..body` (header + body, excluding delimiter)

CRC parameters:
- Polynomial: `0x1021`
- Init: `0xFFFF`
- XOROUT: `0x0000`
- RefIn/RefOut: false (CCITT-FALSE)

### Message types

#### Control / status
- `0x01` `MSG_PING`
- `0x02` `MSG_GET_INFO`
- `0x03` `MSG_SET_STATE`
  - `STATE_IDLE = 0`
  - `STATE_RX   = 1`
  - `STATE_TX   = 2` *(TX strobe; mainly useful for quick testing)*
- `0x04` `MSG_SET_STREAMING` (enable/disable async RX events)
- `0x05` `MSG_GET_METRICS`

#### Register I/O
- `0x10` `MSG_READ_REG`
- `0x11` `MSG_WRITE_REG`
- `0x12` `MSG_READ_EXT`
- `0x13` `MSG_WRITE_EXT`

#### FIFO / strobes
- `0x20` `MSG_RX_READ` (pull-mode RX FIFO read)
- `0x21` `MSG_STROBE`

#### TX (FIFO + transmit)
These commands provide a minimal TX path suitable for basic test transmissions. Actual on-air formatting depends on the active CC1200 packet configuration (SmartRF profile).

- `0x24` `MSG_TX_FLUSH`  
  Flush TX FIFO (`SFTX`).

  Request body: *(empty)*  
  Response body:
  - `ok` : `u8` (1 on success)

- `0x22` `MSG_TX_WRITE`  
  Write bytes into TX FIFO.

  Request body:
  - `flags` : `u8`
    - bit0: flush TX FIFO before writing (`SFTX`)
    - bit1: force IDLE before touching TX path (`SIDLE`)
  - `count` : `u8`
  - `data`  : `count` bytes

  Response body:
  - `ok`      : `u8` (1 on success)
  - `written` : `u8` (bytes written)

- `0x23` `MSG_TX_SEND`  
  Trigger transmit (`STX`) after data is staged in TX FIFO.

  Request body:
  - `flags` : `u8` *(optional; if omitted, defaults to 0)*
    - bit0: strobe `SIDLE` after `STX`
    - bit1: strobe `SRX` after `STX` (RX wins if both bit0+bit1 set)
    - bit2: flush TX FIFO before `STX` (`SFTX`) *(mostly useful for “empty send” sanity tests)*

  Response body:
  - `ok` : `u8` (1 on success)

> Note: These TX commands are “raw FIFO + STX”. If your profile uses the CC1200 packet engine with a length byte, CRC, whitening, etc., the bytes you write must match what the configured packet engine expects.

#### Profile staging (bulk apply)
- `0x30` `MSG_PROFILE_CLEAR`
- `0x31` `MSG_PROFILE_BEGIN`
- `0x32` `MSG_PROFILE_CHUNK`
- `0x33` `MSG_PROFILE_APPLY`

#### Responses and events
- Responses set `type |= 0x80` (response flag) and mirror the request `seq`.
- Async events:
  - `0xE0` `EVT_RX_DATA`
  - `0xE1` `EVT_ERROR`
  Async events typically use `seq = 0`.

### Error handling

- Malformed frames and CRC failures emit `EVT_ERROR`.
- Command-level errors (invalid args, SPI failures, bounds issues, overflow conditions) also emit `EVT_ERROR`.

---

## CC1200 control and data path

### Driver capabilities

The CC1200 driver supports:
- Normal register read/write
- Burst access
- Extended register access via the CC1200 “extended address” mechanism
- RX/TX FIFO burst reads/writes
- Command strobes (e.g., `SIDLE`, `SRX`, `SFRX`, `SFTX`, `STX`)

### RX streaming (push mode)

When streaming is enabled with `MSG_SET_STREAMING`:
- Firmware polls the CC1200 RX FIFO byte count and drains available bytes in chunks (up to 64 bytes per event).
- Each chunk is emitted as `EVT_RX_DATA`.

`EVT_RX_DATA` body format:
- `count` : `u8`
- `data`  : `count` bytes

This is the preferred ingestion mode for a groundstation pipeline that continuously forwards RX bytes to a demod/decoder stage.

### Pull-mode RX FIFO read

`MSG_RX_READ` requests a read of up to `max_bytes` from RX FIFO and returns:

Response body:
- `count` : `u8`
- `data`  : `count` bytes

### TX test path (raw FIFO + STX)

A basic TX sequence is:
1. `MSG_TX_FLUSH` *(optional but recommended for tests)*
2. `MSG_TX_WRITE` with `flags.bit0=1` to flush and `flags.bit1=1` to force `SIDLE`
3. `MSG_TX_SEND` to strobe `STX` (optionally return to `IDLE` or `RX` after)

The reference GUI includes a **TX Test (Send Hex)** panel that performs exactly this sequence.

---

## Metrics

`MSG_GET_METRICS` reads CC1200 status values and returns a compact metrics block, including:
- MARC state
- RX FIFO byte count
- RSSI (reconstructed from CC1200 RSSI registers)

Returned body (current format):
- `marc`        : `u8`
- `rxbytes`     : `u8` (masked to 7 bits by firmware)
- `rssi_raw_1db`: `i8` (coarse display value)
- `rssi_dbm_x10`: `i16` (dBm * 10, little-endian; `-32768` indicates invalid)

The Python GUI supports both legacy (short) and current (extended) metrics response formats.

---

## Profile configuration (SmartRF exports)

Two mechanisms exist:

1. **Direct register writes**  
   The host writes registers (normal and extended) one-by-one using `MSG_WRITE_REG` / `MSG_WRITE_EXT`.
   This is implemented in the reference GUI and is straightforward but more transaction-heavy.

2. **Bulk staging and apply** (recommended for SatNOGS integration)  
   The host stages register operations in RAM and applies them in one commit:
   - `MSG_PROFILE_CLEAR` clears staged entries
   - `MSG_PROFILE_BEGIN` starts a staging session (optional expected count)
   - `MSG_PROFILE_CHUNK` appends entries
   - `MSG_PROFILE_APPLY` writes all staged entries (firmware strobes `SIDLE` first)

`MSG_PROFILE_CHUNK` body format:
- `count` : `u8`
- Repeated `count` times:
  - `type` : `u8` (bit0 = 1 means extended register write, 0 means normal)
  - `addr` : `u8`
  - `val`  : `u8`

`MSG_PROFILE_APPLY` may accept an apply-flags byte to strobe after writing:
- bit0: strobe `SRX`
- bit1: strobe `SIDLE`
- bit2: strobe `SFRX`
- bit3: strobe `SFTX`

---

## Building (Pico SDK)

### Dependencies
- Raspberry Pi Pico SDK (PICO_SDK_PATH must be set)
- CMake
- Ninja (optional but used by example)

### Example build (Windows)

```bat
cd C:\pico\satnogs_rx_frontend
rmdir /s /q build
mkdir build
cd build

set PICO_SDK_PATH=C:\pico\pico-sdk

cmake -G "Ninja" ..
cmake --build .