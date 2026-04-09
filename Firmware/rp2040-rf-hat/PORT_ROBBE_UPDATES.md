# Porting Robbe's CC1200RXFrontend Updates (c10096a → dee48e9)

**Date**: 2026-04-01
**Source**: `/Users/stan/Documents/GitHub/CC1200RXFrontend/`
**Target**: `Firmware/rp2040-rf-hat/src/proto.c` + `proto.h` + `Pi/rf_hat.py`

Robbe pushed 5 commits that add Doppler retune, a radio FSM, metrics v2, and RX robustness. Our code is in `proto.c`/`proto.h` (ported from his `usb_proto.c`/`usb_proto.h`). Key difference: ours uses UART (not USB CDC) and supports dual-radio + buzzer (MSG_SELECT_RADIO, MSG_BUZZER). Those features are NOT in Robbe's code — preserve them during the port.

---

## Summary of Changes

| Feature | Robbe's File | Impact | Priority |
|---------|-------------|--------|----------|
| Radio FSM + desired state | usb_proto.c | Medium refactor in proto.c | HIGH — needed for Doppler |
| MSG_SET_FREQ_WORD (0x34) | usb_proto.c/h | New message handler + header enum | HIGH — needed for Doppler |
| Metrics v2 (28 bytes) | usb_proto.c | Replace `read_metrics()` with `build_metrics_v2()` | MEDIUM |
| RX overflow/FIFO error handling | usb_proto.c | Update `maybe_stream_rx()` + `MSG_RX_READ` | HIGH — prevents lockups |
| RX-sticky streaming | usb_proto.c | `maybe_stream_rx()` calls `ensure_radio_state()` | HIGH |
| Cumulative counters | usb_proto.c | New globals for rx/tx bytes, error counts | MEDIUM |
| GUI Doppler panel | cc1200_gui.py | New UI + `set_frequency_word()` / `set_frequency_hz()` | LOW (GUI only) |

---

## Task 1: Update `proto.h` — Add MSG_SET_FREQ_WORD

**File**: `Firmware/rp2040-rf-hat/src/proto.h`

Add to the `msg_type_t` enum, after `MSG_PROFILE_APPLY = 0x33`:

```c
MSG_SET_FREQ_WORD   = 0x34,
```

No other header changes needed. The `proto_state_t` enum already exists.

---

## Task 2: Update `proto.c` — Core Protocol Changes

**File**: `Firmware/rp2040-rf-hat/src/proto.c`

### 2a. Add new defines (top of file, after existing ext addr defines)

```c
#define CC1200_EXTADDR_NUM_TXBYTES  0xD6
#define CC1200_EXTADDR_FREQ2        0x0C
#define CC1200_EXTADDR_FREQ1        0x0D
#define CC1200_EXTADDR_FREQ0        0x0E
```

### 2b. Add radio FSM state + desired state tracking (after `static bool g_streaming`)

```c
static proto_state_t g_desired_state = STATE_IDLE;

typedef enum {
    RADIO_FSM_IDLE = 0,
    RADIO_FSM_RX,
    RADIO_FSM_TX,
    RADIO_FSM_DOPPLER_RETUNE,
} radio_fsm_state_t;

static radio_fsm_state_t g_radio_fsm = RADIO_FSM_IDLE;

// Doppler retune state
static volatile bool     g_freq_retune_pending = false;
static volatile bool     g_freq_retune_need_cal = false;
static volatile uint32_t g_pending_freq_word = 0;
static uint32_t          g_last_applied_freq_word = 0;
```

### 2c. Add cumulative counters (after FSM state)

```c
static uint32_t g_rx_total_bytes = 0;
static uint32_t g_tx_total_bytes = 0;
static uint16_t g_rx_overflow_count = 0;
static uint16_t g_rx_fifo_err_count = 0;
static uint16_t g_tx_fifo_err_count = 0;
static bool     g_rx_overflow_seen = false;
static bool     g_rx_fifo_err_seen = false;
static bool     g_tx_fifo_err_seen = false;
static uint8_t  g_last_rx_chunk = 0;
```

### 2d. Add helper functions (before `send_frame`)

```c
static inline uint32_t uptime_ms_now(void) {
    return to_ms_since_boot(get_absolute_time());
}

static inline uint32_t abs_u32_diff(uint32_t a, uint32_t b) {
    return (a >= b) ? (a - b) : (b - a);
}
```

### 2e. Add `ensure_radio_state()` function

This is the key new function — it recovers from FIFO errors and keeps the radio in the desired state. **IMPORTANT**: Use `active_radio()` instead of Robbe's `g_radio` since we support dual radios.

```c
static inline cc1200_status_t radio_status(void) {
    return cc1200_strobe(active_radio(), CC1200_CMD_SNOP);
}

static inline void set_desired_state(proto_state_t st) {
    g_desired_state = st;
}

static void ensure_radio_state(void) {
    cc1200_t* radio = active_radio();
    if (!radio) return;

    cc1200_status_t st = cc1200_strobe(radio, CC1200_CMD_SNOP);

    if (st.state == CC1200_STATE_RX_FIFO_ERR) {
        g_rx_fifo_err_seen = true;
        g_rx_fifo_err_count++;
        (void)cc1200_strobe(radio, CC1200_CMD_SFRX);
        if (g_desired_state == STATE_RX)
            (void)cc1200_strobe(radio, CC1200_CMD_SRX);
        else
            (void)cc1200_strobe(radio, CC1200_CMD_SIDLE);
        return;
    }
    if (st.state == CC1200_STATE_TX_FIFO_ERR) {
        g_tx_fifo_err_seen = true;
        g_tx_fifo_err_count++;
        (void)cc1200_strobe(radio, CC1200_CMD_SFTX);
        if (g_desired_state == STATE_RX)
            (void)cc1200_strobe(radio, CC1200_CMD_SRX);
        else if (g_desired_state == STATE_TX)
            (void)cc1200_strobe(radio, CC1200_CMD_STX);
        else
            (void)cc1200_strobe(radio, CC1200_CMD_SIDLE);
        return;
    }

    if (g_desired_state == STATE_RX) {
        if (st.state != CC1200_STATE_RX)
            (void)cc1200_strobe(radio, CC1200_CMD_SRX);
    } else if (g_desired_state == STATE_TX) {
        // TX is explicit, do nothing
    } else {
        if (st.state != CC1200_STATE_IDLE)
            (void)cc1200_strobe(radio, CC1200_CMD_SIDLE);
    }
}
```

**NOTE**: Check that `CC1200_STATE_RX_FIFO_ERR`, `CC1200_STATE_TX_FIFO_ERR`, `CC1200_STATE_RX`, `CC1200_STATE_IDLE` are defined in `cc1200.h`. If not, add them — values come from status byte bits [6:4]:
- IDLE = 0x00
- RX = 0x01
- TX = 0x02
- RX_FIFO_ERR = 0x06
- TX_FIFO_ERR = 0x07

Check `cc1200.h` for the actual enum/define names Robbe used.

### 2f. Add `perform_freq_retune()` function

This does the actual FREQ register update. Uses `cc1200_write_ext_burst()` which already exists in our driver.

```c
static bool perform_freq_retune(void) {
    cc1200_t* radio = active_radio();
    if (!radio) return false;

    uint32_t w = g_pending_freq_word & 0x00FFFFFFu;
    uint8_t freq_bytes[3];
    freq_bytes[0] = (uint8_t)((w >> 16) & 0xFFu);
    freq_bytes[1] = (uint8_t)((w >>  8) & 0xFFu);
    freq_bytes[2] = (uint8_t)( w        & 0xFFu);

    (void)cc1200_strobe(radio, CC1200_CMD_SIDLE);

    bool ok = cc1200_write_ext_burst(radio, CC1200_EXTADDR_FREQ2, freq_bytes, 3);
    if (!ok) return false;

    if (g_freq_retune_need_cal) {
        (void)cc1200_strobe(radio, CC1200_CMD_SCAL);
        g_freq_retune_need_cal = false;
    }

    (void)cc1200_strobe(radio, CC1200_CMD_SRX);

    g_last_applied_freq_word = w;
    g_freq_retune_pending = false;
    return true;
}
```

### 2g. Replace `maybe_stream_rx()` entirely

The new version handles RX overflow detection, Doppler retune during streaming, and keeps the radio in RX ("RX-sticky"). Copy from Robbe's version but replace `g_radio` with `active_radio()`.

Key differences from our current version:
- Checks `rxbytes & 0x80` for overflow flag → flush + re-enter RX
- Calls `ensure_radio_state()` to auto-recover
- Defers Doppler retune into the FSM
- Tracks `g_rx_total_bytes` and `g_last_rx_chunk`
- Breaks out of drain loop if a new Doppler update arrives

### 2h. Replace `read_metrics()` with `build_metrics_v2()`

The v2 response is 28 bytes instead of 5. See Robbe's `build_metrics_v2()`. Adapt to use `active_radio()`.

Response layout:
```
[0]    version = 2
[1]    flags (streaming, overflow, fifo_err)
[2]    desired_state
[3]    chip_state (from SNOP)
[4]    marcstate
[5]    rxbytes
[6]    txbytes
[7]    last_rx_chunk
[8-9]  rssi_dbm_x10 (int16 LE)
[10-13] rx_total_bytes (uint32 LE)
[14-17] tx_total_bytes (uint32 LE)
[18-19] rx_overflow_count (uint16 LE)
[20-21] rx_fifo_err_count (uint16 LE)
[22-23] tx_fifo_err_count (uint16 LE)
[24-27] uptime_ms (uint32 LE)
```

### 2i. Update `handle_cmd()` cases

**MSG_SET_STATE**: Add `set_desired_state()` and `g_radio_fsm` tracking:
```c
case MSG_SET_STATE:
    if (body[0] == STATE_RX) {
        set_desired_state(STATE_RX);
        g_radio_fsm = RADIO_FSM_RX;
        (void)cc1200_strobe(radio, CC1200_CMD_SRX);
    } else if (body[0] == STATE_TX) {
        set_desired_state(STATE_TX);
        g_radio_fsm = RADIO_FSM_TX;
        (void)cc1200_strobe(radio, CC1200_CMD_STX);
    } else {
        set_desired_state(STATE_IDLE);
        g_radio_fsm = RADIO_FSM_IDLE;
        (void)cc1200_strobe(radio, CC1200_CMD_SIDLE);
    }
```

**MSG_SET_STREAMING**: When enabling, auto-enter RX:
```c
case MSG_SET_STREAMING:
    g_streaming = (body[0] != 0);
    if (g_streaming) {
        set_desired_state(STATE_RX);
        g_radio_fsm = RADIO_FSM_RX;
        (void)cc1200_strobe(radio, CC1200_CMD_SRX);
    }
```

**MSG_GET_METRICS**: Change buffer to 64 bytes, call `build_metrics_v2()`.

**MSG_RX_READ**: Add overflow check (rxbytes & 0x80) before reading FIFO. Track `g_rx_total_bytes`.

**MSG_TX_WRITE**: Track `g_tx_total_bytes` on success.

**MSG_PROFILE_APPLY**: Add `ensure_radio_state()` fallback when no post-apply flags set.

**MSG_SET_FREQ_WORD** (NEW handler): Add the entire new case block from Robbe's code. Body: [FREQ2, FREQ1, FREQ0, flags?]. Queues retune; FSM applies it safely.

### 2j. Update `proto_init()`

Reset all new state variables (counters, FSM, retune pending, etc.). See Robbe's `usb_proto_init()`.

### 2k. Update `MSG_SELECT_RADIO` handler

Also reset `g_desired_state = STATE_IDLE`, `g_radio_fsm = RADIO_FSM_IDLE`, and `g_freq_retune_pending = false` on radio switch.

---

## Task 3: Update `Pi/rf_hat.py` — Pi-Side Protocol

### 3a. Add `MSG_SET_FREQ_WORD = 0x34` constant

### 3b. Update `get_metrics()` to handle v2 response

Add parsing for the 28-byte v2 format (check `b[0] == 2`). The existing v1 parsing should remain as fallback. See Robbe's GUI code for the exact decode:

```python
if len(b) >= 28 and b[0] == 2:
    marc = b[4]
    rxbytes = b[5]
    rssi_dbm_x10 = struct.unpack_from("<h", b, 8)[0]
    # ... etc
```

### 3c. Add `set_frequency_word()` method to CC1200Link

```python
def set_frequency_word(self, word: int, force_cal: bool = False) -> bool:
    word &= 0xFFFFFF
    f2 = (word >> 16) & 0xFF
    f1 = (word >> 8) & 0xFF
    f0 = word & 0xFF
    flags = 0x01 if force_cal else 0x00
    b = self._send_cmd(MSG_SET_FREQ_WORD, bytes([f2, f1, f0, flags]))
    return bool(b and len(b) >= 1 and b[0] == 1)
```

### 3d. Update `set_frequency()` to use MSG_SET_FREQ_WORD

Currently `set_frequency()` does 3 separate `write_ext()` calls for FREQ2/FREQ1/FREQ0. Replace with a single `set_frequency_word()` call. This is atomic and Doppler-safe (won't glitch mid-update).

**Before** (current, in rf_hat.py ~line 668):
```python
def set_frequency(self, freq_hz):
    freq_word = round(freq_hz * lo_div * 65536 / F_XOSC_HZ)
    ok  = self.write_ext(EXT_FREQ2, (freq_word >> 16) & 0xFF)
    ok &= self.write_ext(EXT_FREQ1, (freq_word >> 8) & 0xFF)
    ok &= self.write_ext(EXT_FREQ0, freq_word & 0xFF)
    return ok
```

**After**:
```python
def set_frequency(self, freq_hz):
    lo_div = ...  # keep existing LO divider logic
    freq_word = round(freq_hz * lo_div * 65536 / F_XOSC_HZ)
    return self.set_frequency_word(freq_word & 0xFFFFFF)
```

This is critical because `station.py` calls `set_frequency()` at 2 Hz for Doppler correction during passes. The old 3-register approach could glitch the frequency mid-update.

### 3e. Add `strobe()` method (Robbe added it to his GUI)

```python
def strobe(self, cmd: int) -> Optional[int]:
    b = self._send_cmd(MSG_STROBE, bytes([cmd & 0xFF]))
    if b and len(b) == 1:
        return b[0]
    return None
```

---

## Task 4: Update `Pi/rf_hat.py` GUI tools (LOW PRIORITY)

The `tools/cc1200_gui.py` changes are cosmetic + Doppler test panel. Not needed for bringup but useful for bench testing. Copy Robbe's GUI changes into `Firmware/rp2040-rf-hat/tools/cc1200_gui.py` if it exists there, or note that the canonical GUI is in `CC1200RXFrontend/PC_test/cc1200_gui.py`.

---

## What NOT to Change

- **UART I/O** (`uart_write_bytes`, `uart_is_readable`, `uart_getc`): Keep our UART transport. Robbe uses `fwrite(stdout)` / `getchar_timeout_us()` for USB CDC.
- **Dual-radio support** (`g_radios[]`, `g_active`, `active_radio()`, `MSG_SELECT_RADIO`): Robbe's code uses a single `g_radio` pointer. Keep our array + active_radio() pattern.
- **MSG_BUZZER handler**: Not in Robbe's code. Keep ours.
- **MSG_GET_INFO response**: Ours returns `[active_idx, radio_count, part, ver]` (4 bytes). Robbe's returns `[part, ver]` (2 bytes). Keep ours.
- **Strobe range validation** in MSG_STROBE: Robbe removed it. Keep our `0x30-0x3D` check.
- **Profile apply MARCSTATE polling**: We added a 50×100us poll loop for IDLE confirmation. Keep it — Robbe still doesn't have it.

---

## Build & Test

After porting:
1. `cd Firmware/rp2040-rf-hat && pio run` — must compile clean
2. Flash to Pico, connect via UART
3. Test with `Pi/rf_hat.py`:
   - `ping()` → "PONG"
   - `get_info()` → part=0x20
   - `get_metrics()` → should return 28-byte v2 response
   - `set_frequency_word(freq_to_regs(433e6))` → should ACK
   - `set_streaming(True)` → should auto-enter RX
4. Test Doppler retune: call `set_frequency()` rapidly (10× per second) while streaming — should not crash or lose RX state

---

## Dependency Order

```
proto.h (Task 1)
    └── proto.c (Task 2) — depends on header
         └── Pi/rf_hat.py (Task 3) — depends on firmware protocol
              └── GUI tools (Task 4) — optional
```
