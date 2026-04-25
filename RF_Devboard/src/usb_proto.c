#include "usb_proto.h"

#include <stdio.h>
#include <string.h>

#include "pico/stdlib.h"

#include "usb_cobs.h"
#include "crc16.h"

// CC1200 EXT status block regs used for telemetry/metrics
#define CC1200_EXTADDR_RSSI1        0x71
#define CC1200_EXTADDR_RSSI0        0x72
#define CC1200_EXTADDR_MARCSTATE    0x73
#define CC1200_EXTADDR_NUM_TXBYTES  0xD6
#define CC1200_EXTADDR_NUM_RXBYTES  0xD7

// CC1200 frequency word registers (extended register space)
#define CC1200_EXTADDR_FREQ2        0x0C
#define CC1200_EXTADDR_FREQ1        0x0D
#define CC1200_EXTADDR_FREQ0        0x0E

// Framing: COBS(payload) + 0x00 delimiter
// Payload: type(1) seq(1) len(2 LE) body(len) crc16(2 LE)
#define RX_ENC_MAX   512
#define RX_DEC_MAX   512
#define TX_ENC_MAX   512

// If the requested frequency jump exceeds this many synth-word LSBs,
// request a calibration during retune.
// Tune this threshold later during on-air testing.
#define FREQ_RETUNE_CAL_THRESHOLD_WORDS  64u

static cc1200_t* g_radio = NULL;
static usb_proto_hooks_t g_hooks = (usb_proto_hooks_t){0};

static bool g_streaming = false;
static proto_state_t g_desired_state = STATE_IDLE;

// RX accumulator for COBS frames
static uint8_t g_rx_enc[RX_ENC_MAX];
static size_t  g_rx_enc_len = 0;

// Working buffers
static uint8_t g_rx_dec[RX_DEC_MAX];
static uint8_t g_tx_payload[RX_DEC_MAX];
static uint8_t g_tx_enc[TX_ENC_MAX];

static inline void hook_usb_activity(void) { if (g_hooks.on_usb_activity) g_hooks.on_usb_activity(); }
static inline void hook_error(void)        { if (g_hooks.on_error) g_hooks.on_error(); }

// -----------------------------
// Firmware radio FSM
// -----------------------------
typedef enum {
    RADIO_FSM_IDLE = 0,
    RADIO_FSM_RX,
    RADIO_FSM_TX,
    RADIO_FSM_DOPPLER_RETUNE,
} radio_fsm_state_t;

static radio_fsm_state_t g_radio_fsm = RADIO_FSM_IDLE;

// Pending retune request (newest value wins)
static volatile bool     g_freq_retune_pending = false;
static volatile bool     g_freq_retune_need_cal = false;
static volatile uint32_t g_pending_freq_word = 0;       // only low 24 bits used
static uint32_t          g_last_applied_freq_word = 0;  // only low 24 bits used
static uint32_t          g_freq_retune_count = 0;

// Doppler retune timing
static uint32_t g_last_doppler_retune_us = 0;
static uint32_t g_max_doppler_retune_us  = 0;

// -----------------------------
// Metrics / counters
// -----------------------------
static uint32_t g_rx_total_bytes = 0;
static uint32_t g_tx_total_bytes = 0;
static uint16_t g_rx_overflow_count = 0;
static uint16_t g_rx_fifo_err_count = 0;
static uint16_t g_tx_fifo_err_count = 0;

static bool     g_rx_overflow_seen = false;
static bool     g_rx_fifo_err_seen = false;
static bool     g_tx_fifo_err_seen = false;

static uint8_t  g_last_rx_chunk = 0;

static inline int16_t sign_extend_12(uint16_t v12)
{
    v12 &= 0x0FFFu;
    if (v12 & 0x0800u) return (int16_t)(v12 - 0x1000u);
    return (int16_t)v12;
}

static inline uint32_t uptime_ms_now(void)
{
    return to_ms_since_boot(get_absolute_time());
}

static inline uint32_t uptime_us_now(void)
{
    return (uint32_t)to_us_since_boot(get_absolute_time());
}

static inline uint32_t abs_u32_diff(uint32_t a, uint32_t b)
{
    return (a >= b) ? (a - b) : (b - a);
}

// -----------------------------
// Framing helpers
// -----------------------------
static void send_frame(uint8_t type, uint8_t seq, const uint8_t* body, uint16_t body_len)
{
    if (body_len > (uint16_t)(sizeof(g_tx_payload) - 6)) return;

    g_tx_payload[0] = type;
    g_tx_payload[1] = seq;
    g_tx_payload[2] = (uint8_t)(body_len & 0xFF);
    g_tx_payload[3] = (uint8_t)((body_len >> 8) & 0xFF);

    if (body_len && body)
        memcpy(&g_tx_payload[4], body, body_len);

    uint16_t crc = crc16_ccitt_false(g_tx_payload, (size_t)(4 + body_len));
    g_tx_payload[4 + body_len + 0] = (uint8_t)(crc & 0xFF);
    g_tx_payload[4 + body_len + 1] = (uint8_t)((crc >> 8) & 0xFF);

    size_t payload_len = (size_t)(6 + body_len);
    size_t enc_len = cobs_encode(g_tx_enc, sizeof(g_tx_enc), g_tx_payload, payload_len);
    if (enc_len == 0) return;

    fwrite(g_tx_enc, 1, enc_len, stdout);
    fputc(0x00, stdout);
    fflush(stdout);
}

static void send_error(uint8_t code, uint8_t seq)
{
    uint8_t b[2] = { code, 0 };
    send_frame(EVT_ERROR, seq, b, (uint16_t)sizeof(b));
}

// -----------------------------
// Radio state helpers
// -----------------------------
static inline cc1200_status_t radio_status(void)
{
    return cc1200_strobe(g_radio, CC1200_CMD_SNOP);
}

static inline void set_desired_state(proto_state_t st)
{
    g_desired_state = st;
}

static void ensure_radio_state(void)
{
    if (!g_radio) return;

    cc1200_status_t st = radio_status();

    if (st.state == CC1200_STATE_RX_FIFO_ERR)
    {
        g_rx_fifo_err_seen = true;
        g_rx_fifo_err_count++;
        (void)cc1200_strobe(g_radio, CC1200_CMD_SFRX);

        if (g_desired_state == STATE_RX)
            (void)cc1200_strobe(g_radio, CC1200_CMD_SRX);
        else
            (void)cc1200_strobe(g_radio, CC1200_CMD_SIDLE);

        return;
    }

    if (st.state == CC1200_STATE_TX_FIFO_ERR)
    {
        g_tx_fifo_err_seen = true;
        g_tx_fifo_err_count++;
        (void)cc1200_strobe(g_radio, CC1200_CMD_SFTX);

        if (g_desired_state == STATE_RX)
            (void)cc1200_strobe(g_radio, CC1200_CMD_SRX);
        else if (g_desired_state == STATE_TX)
            (void)cc1200_strobe(g_radio, CC1200_CMD_STX);
        else
            (void)cc1200_strobe(g_radio, CC1200_CMD_SIDLE);

        return;
    }

    if (g_desired_state == STATE_RX)
    {
        if (st.state != CC1200_STATE_RX)
            (void)cc1200_strobe(g_radio, CC1200_CMD_SRX);
    }
    else if (g_desired_state == STATE_TX)
    {
        // TX is explicit only
    }
    else
    {
        if (st.state != CC1200_STATE_IDLE)
            (void)cc1200_strobe(g_radio, CC1200_CMD_SIDLE);
    }
}

// -----------------------------
// Profile staging
// -----------------------------
typedef struct {
    uint8_t is_ext;
    uint8_t addr;
    uint8_t val;
} prof_entry_t;

#ifndef PROFILE_MAX_ENTRIES
#define PROFILE_MAX_ENTRIES  768
#endif

static prof_entry_t g_prof[PROFILE_MAX_ENTRIES];
static uint16_t     g_prof_count = 0;
static bool         g_prof_active = false;

static void profile_clear(void)
{
    g_prof_count = 0;
    g_prof_active = false;
}

static bool profile_begin(uint16_t expected_count)
{
    (void)expected_count;
    g_prof_count = 0;
    g_prof_active = true;
    return true;
}

static bool profile_chunk_append(const uint8_t* body, uint16_t len)
{
    if (!g_prof_active) return false;
    if (!body || len < 1) return false;

    uint8_t count = body[0];
    uint16_t need = (uint16_t)(1u + (uint16_t)3u * (uint16_t)count);
    if (len != need) return false;

    const uint8_t* p = &body[1];
    for (uint8_t i = 0; i < count; i++)
    {
        if (g_prof_count >= PROFILE_MAX_ENTRIES) return false;

        uint8_t type = p[0];
        uint8_t addr = p[1];
        uint8_t val  = p[2];
        p += 3;

        g_prof[g_prof_count].is_ext = (type & 0x01u) ? 1u : 0u;
        g_prof[g_prof_count].addr   = addr;
        g_prof[g_prof_count].val    = val;
        g_prof_count++;
    }
    return true;
}

static bool profile_apply(cc1200_t* dev)
{
    if (!dev) return false;
    if (!g_prof_active) return false;

    (void)cc1200_strobe(dev, CC1200_CMD_SIDLE);

    for (uint16_t i = 0; i < g_prof_count; i++)
    {
        bool ok = false;
        if (g_prof[i].is_ext)
            ok = cc1200_write_ext(dev, g_prof[i].addr, g_prof[i].val);
        else
            ok = cc1200_write_reg(dev, g_prof[i].addr, g_prof[i].val);

        if (!ok) return false;
    }
    return true;
}

// -----------------------------
// Doppler retune
// -----------------------------
static bool perform_freq_retune(void)
{
    if (!g_radio) return false;

    uint32_t t0_us = uptime_us_now();

    uint32_t w = g_pending_freq_word & 0x00FFFFFFu;
    uint8_t freq_bytes[3];
    freq_bytes[0] = (uint8_t)((w >> 16) & 0xFFu); // FREQ2
    freq_bytes[1] = (uint8_t)((w >>  8) & 0xFFu); // FREQ1
    freq_bytes[2] = (uint8_t)( w        & 0xFFu); // FREQ0

    // Leave RX for the shortest safe window
    (void)cc1200_strobe(g_radio, CC1200_CMD_SIDLE);

    // Atomic retune: one burst write for all 3 bytes
    bool ok = cc1200_write_ext_burst(g_radio, CC1200_EXTADDR_FREQ2, freq_bytes, 3);
    if (!ok)
    {
        uint32_t dt_us = uptime_us_now() - t0_us;
        g_last_doppler_retune_us = dt_us;
        if (dt_us > g_max_doppler_retune_us) g_max_doppler_retune_us = dt_us;
        return false;
    }

    // Optional calibration only when needed
    if (g_freq_retune_need_cal)
    {
        (void)cc1200_strobe(g_radio, CC1200_CMD_SCAL);
        g_freq_retune_need_cal = false;
    }

    // Resume RX immediately
    (void)cc1200_strobe(g_radio, CC1200_CMD_SRX);

    uint32_t dt_us = uptime_us_now() - t0_us;
    g_last_doppler_retune_us = dt_us;
    if (dt_us > g_max_doppler_retune_us) g_max_doppler_retune_us = dt_us;

    g_last_applied_freq_word = w;
    g_freq_retune_pending = false;
    g_freq_retune_count++;
    return true;
}

// -----------------------------
// RX streaming
// -----------------------------
static void maybe_stream_rx(void)
{
    if (!g_streaming || !g_radio) return;

    set_desired_state(STATE_RX);

    if (g_freq_retune_pending)
        g_radio_fsm = RADIO_FSM_DOPPLER_RETUNE;

    if (g_radio_fsm == RADIO_FSM_DOPPLER_RETUNE)
    {
        if (perform_freq_retune())
        {
            g_radio_fsm = RADIO_FSM_RX;
        }
        else
        {
            // Recover RX and try again on the next loop
            (void)cc1200_strobe(g_radio, CC1200_CMD_SRX);
            g_radio_fsm = RADIO_FSM_RX;
        }
        return;
    }

    ensure_radio_state();

    uint8_t rxbytes = 0;
    if (!cc1200_read_ext(g_radio, CC1200_EXTADDR_NUM_RXBYTES, &rxbytes))
        return;

    if (rxbytes & 0x80u)
    {
        g_rx_overflow_seen = true;
        g_rx_overflow_count++;
        (void)cc1200_strobe(g_radio, CC1200_CMD_SFRX);
        (void)cc1200_strobe(g_radio, CC1200_CMD_SRX);
        return;
    }

    uint8_t n = (uint8_t)(rxbytes & 0x7Fu);
    if (n == 0)
    {
        ensure_radio_state();
        return;
    }

    while (n > 0)
    {
        uint8_t chunk = (n > 64u) ? 64u : n;
        uint8_t buf[64];

        if (!cc1200_fifo_read(g_radio, buf, chunk))
            break;

        g_last_rx_chunk = chunk;
        g_rx_total_bytes += (uint32_t)chunk;

        uint8_t body[1 + 64];
        body[0] = chunk;
        memcpy(&body[1], buf, chunk);

        send_frame(EVT_RX_DATA, 0, body, (uint16_t)(1u + chunk));

        if (!cc1200_read_ext(g_radio, CC1200_EXTADDR_NUM_RXBYTES, &rxbytes))
            break;

        if (rxbytes & 0x80u)
        {
            g_rx_overflow_seen = true;
            g_rx_overflow_count++;
            (void)cc1200_strobe(g_radio, CC1200_CMD_SFRX);
            break;
        }

        n = (uint8_t)(rxbytes & 0x7Fu);

        // If a newer retune request arrived while draining, handle it next loop
        if (g_freq_retune_pending)
            break;
    }

    ensure_radio_state();
}

// -----------------------------
// TX helpers
// -----------------------------
static bool tx_write_fifo(uint8_t flags, const uint8_t* data, uint8_t count)
{
    if (!g_radio) return false;

    if (flags & 0x02u)
        (void)cc1200_strobe(g_radio, CC1200_CMD_SIDLE);

    if (flags & 0x01u)
        (void)cc1200_strobe(g_radio, CC1200_CMD_SFTX);

    if (count == 0) return true;

    bool ok = cc1200_fifo_write(g_radio, data, (size_t)count);
    if (ok) g_tx_total_bytes += (uint32_t)count;
    return ok;
}

static bool tx_send(uint8_t flags)
{
    if (!g_radio) return false;

    if (flags & 0x04u)
        (void)cc1200_strobe(g_radio, CC1200_CMD_SFTX);

    (void)cc1200_strobe(g_radio, CC1200_CMD_STX);

    if (flags & 0x01u) (void)cc1200_strobe(g_radio, CC1200_CMD_SIDLE);
    if (flags & 0x02u) (void)cc1200_strobe(g_radio, CC1200_CMD_SRX);

    return true;
}

// -----------------------------
// Metrics builder
// -----------------------------
// MSG_GET_METRICS response body (v2):
//   [0]  version = 2
//   [1]  flags:
//        bit0 streaming_enabled
//        bit1 rx_overflow_seen_since_last_metrics
//        bit2 rx_fifo_err_seen_since_last_metrics
//        bit3 tx_fifo_err_seen_since_last_metrics
//   [2]  desired_state (proto_state_t)
//   [3]  chip_state (status byte decoded state, cc1200_state_t)
//   [4]  marcstate
//   [5]  rxbytes_now
//   [6]  txbytes_now
//   [7]  last_rx_chunk_bytes
//   [8..9]   rssi_dbm_x10 (int16 LE), -32768 means invalid
//   [10..13] rx_total_bytes (uint32 LE)
//   [14..17] tx_total_bytes (uint32 LE)
//   [18..19] rx_overflow_count (uint16 LE)
//   [20..21] rx_fifo_err_count (uint16 LE)
//   [22..23] tx_fifo_err_count (uint16 LE)
//   [24..27] uptime_ms (uint32 LE)
//   [28..31] last_doppler_retune_us (uint32 LE)
//   [32..35] max_doppler_retune_us  (uint32 LE)
// Total: 36 bytes
static bool build_metrics_v2(uint8_t* out, uint16_t* out_len)
{
    if (!g_radio || !out || !out_len) return false;

    uint8_t marc = 0;
    uint8_t rxbytes = 0;
    uint8_t txbytes = 0;
    uint8_t rssi1 = 0;
    uint8_t rssi0 = 0;

    if (!cc1200_read_ext(g_radio, CC1200_EXTADDR_MARCSTATE, &marc)) return false;

    (void)cc1200_read_ext(g_radio, CC1200_EXTADDR_NUM_RXBYTES, &rxbytes);
    (void)cc1200_read_ext(g_radio, CC1200_EXTADDR_NUM_TXBYTES, &txbytes);
    (void)cc1200_read_ext(g_radio, CC1200_EXTADDR_RSSI1, &rssi1);
    (void)cc1200_read_ext(g_radio, CC1200_EXTADDR_RSSI0, &rssi0);

    int16_t rssi_dbm_x10 = (int16_t)-32768;
    if (rssi0 & 0x01u)  // RSSI_VALID flag
    {
        uint16_t rssi12_u = ((uint16_t)rssi1 << 4) | (uint16_t)((rssi0 >> 4) & 0x0Fu);
        int16_t  rssi12   = sign_extend_12(rssi12_u);
        if (rssi12 != (int16_t)-2048)
        {
            // RSSI_OFFSET per SWRU346B Table 22: 99 dBm at DVGA_GAIN=00, -18 dBm per step
            uint8_t mdmcfg1 = 0;
            (void)cc1200_read_reg(g_radio, 0x13, &mdmcfg1);
            int16_t rssi_offset_x10 = 990 - (int16_t)((mdmcfg1 >> 6) & 0x03u) * 180;
            rssi_dbm_x10 = (int16_t)((rssi12 * 10) / 16) - rssi_offset_x10;
        }
    }

    cc1200_status_t st = radio_status();

    uint8_t flags = 0;
    if (g_streaming) flags |= (1u << 0);
    if (g_rx_overflow_seen) flags |= (1u << 1);
    if (g_rx_fifo_err_seen) flags |= (1u << 2);
    if (g_tx_fifo_err_seen) flags |= (1u << 3);

    out[0] = 2;
    out[1] = flags;
    out[2] = (uint8_t)g_desired_state;
    out[3] = (uint8_t)st.state;
    out[4] = marc;
    out[5] = (uint8_t)(rxbytes & 0x7Fu);
    out[6] = (uint8_t)(txbytes & 0x7Fu);
    out[7] = g_last_rx_chunk;

    out[8]  = (uint8_t)(rssi_dbm_x10 & 0xFF);
    out[9]  = (uint8_t)((rssi_dbm_x10 >> 8) & 0xFF);

    out[10] = (uint8_t)(g_rx_total_bytes & 0xFF);
    out[11] = (uint8_t)((g_rx_total_bytes >> 8) & 0xFF);
    out[12] = (uint8_t)((g_rx_total_bytes >> 16) & 0xFF);
    out[13] = (uint8_t)((g_rx_total_bytes >> 24) & 0xFF);

    out[14] = (uint8_t)(g_tx_total_bytes & 0xFF);
    out[15] = (uint8_t)((g_tx_total_bytes >> 8) & 0xFF);
    out[16] = (uint8_t)((g_tx_total_bytes >> 16) & 0xFF);
    out[17] = (uint8_t)((g_tx_total_bytes >> 24) & 0xFF);

    out[18] = (uint8_t)(g_rx_overflow_count & 0xFF);
    out[19] = (uint8_t)((g_rx_overflow_count >> 8) & 0xFF);

    out[20] = (uint8_t)(g_rx_fifo_err_count & 0xFF);
    out[21] = (uint8_t)((g_rx_fifo_err_count >> 8) & 0xFF);

    out[22] = (uint8_t)(g_tx_fifo_err_count & 0xFF);
    out[23] = (uint8_t)((g_tx_fifo_err_count >> 8) & 0xFF);

    uint32_t up = uptime_ms_now();
    out[24] = (uint8_t)(up & 0xFF);
    out[25] = (uint8_t)((up >> 8) & 0xFF);
    out[26] = (uint8_t)((up >> 16) & 0xFF);
    out[27] = (uint8_t)((up >> 24) & 0xFF);

    out[28] = (uint8_t)(g_last_doppler_retune_us & 0xFF);
    out[29] = (uint8_t)((g_last_doppler_retune_us >> 8) & 0xFF);
    out[30] = (uint8_t)((g_last_doppler_retune_us >> 16) & 0xFF);
    out[31] = (uint8_t)((g_last_doppler_retune_us >> 24) & 0xFF);

    out[32] = (uint8_t)(g_max_doppler_retune_us & 0xFF);
    out[33] = (uint8_t)((g_max_doppler_retune_us >> 8) & 0xFF);
    out[34] = (uint8_t)((g_max_doppler_retune_us >> 16) & 0xFF);
    out[35] = (uint8_t)((g_max_doppler_retune_us >> 24) & 0xFF);

    *out_len = 36;

    g_rx_overflow_seen = false;
    g_rx_fifo_err_seen = false;
    g_tx_fifo_err_seen = false;

    return true;
}

// -----------------------------
// Command handler
// -----------------------------
static void handle_cmd(uint8_t type, uint8_t seq, const uint8_t* body, uint16_t len)
{
    hook_usb_activity();

    const uint8_t rsp_type = (uint8_t)(type | MSG_RSP_FLAG);

    switch (type)
    {
        case MSG_PING:
        {
            const char pong[] = "PONG";
            send_frame(rsp_type, seq, (const uint8_t*)pong, (uint16_t)sizeof(pong));
        } break;

        case MSG_GET_INFO:
        {
            uint8_t part = 0, ver = 0;
            bool ok1 = cc1200_read_ext(g_radio, 0x8F, &part);
            bool ok2 = cc1200_read_ext(g_radio, 0x90, &ver);
            uint8_t b[2] = { ok1 ? part : 0x00, ok2 ? ver : 0x00 };
            send_frame(rsp_type, seq, b, (uint16_t)sizeof(b));
        } break;

        case MSG_SET_STATE:
        {
            if (len < 1) { send_error(0x01, seq); break; }

            if (body[0] == STATE_RX)
            {
                set_desired_state(STATE_RX);
                g_radio_fsm = RADIO_FSM_RX;
                (void)cc1200_strobe(g_radio, CC1200_CMD_SRX);
            }
            else if (body[0] == STATE_TX)
            {
                set_desired_state(STATE_TX);
                g_radio_fsm = RADIO_FSM_TX;
                (void)cc1200_strobe(g_radio, CC1200_CMD_STX);
            }
            else
            {
                set_desired_state(STATE_IDLE);
                g_radio_fsm = RADIO_FSM_IDLE;
                (void)cc1200_strobe(g_radio, CC1200_CMD_SIDLE);
            }

            uint8_t ok = 1;
            send_frame(rsp_type, seq, &ok, 1);
        } break;

        case MSG_SET_STREAMING:
        {
            if (len < 1) { send_error(0x01, seq); break; }

            g_streaming = (body[0] != 0);

            if (g_streaming)
            {
                set_desired_state(STATE_RX);
                g_radio_fsm = RADIO_FSM_RX;
                (void)cc1200_strobe(g_radio, CC1200_CMD_SRX);
            }

            uint8_t b[1] = { (uint8_t)(g_streaming ? 1 : 0) };
            send_frame(rsp_type, seq, b, 1);
        } break;

        case MSG_GET_METRICS:
        {
            uint8_t b[64];
            uint16_t out_len = 0;
            if (!build_metrics_v2(b, &out_len)) { send_error(0x02, seq); break; }
            send_frame(rsp_type, seq, b, out_len);
        } break;

        case MSG_READ_REG:
        {
            if (len < 1) { send_error(0x01, seq); break; }
            uint8_t v = 0;
            bool ok = cc1200_read_reg(g_radio, body[0], &v);
            uint8_t b[2] = { (uint8_t)(ok ? 1 : 0), v };
            send_frame(rsp_type, seq, b, (uint16_t)sizeof(b));
        } break;

        case MSG_WRITE_REG:
        {
            if (len < 2) { send_error(0x01, seq); break; }
            bool ok = cc1200_write_reg(g_radio, body[0], body[1]);
            uint8_t b[1] = { (uint8_t)(ok ? 1 : 0) };
            send_frame(rsp_type, seq, b, 1);
        } break;

        case MSG_READ_EXT:
        {
            if (len < 1) { send_error(0x01, seq); break; }
            uint8_t v = 0;
            bool ok = cc1200_read_ext(g_radio, body[0], &v);
            uint8_t b[2] = { (uint8_t)(ok ? 1 : 0), v };
            send_frame(rsp_type, seq, b, (uint16_t)sizeof(b));
        } break;

        case MSG_WRITE_EXT:
        {
            if (len < 2) { send_error(0x01, seq); break; }
            bool ok = cc1200_write_ext(g_radio, body[0], body[1]);
            uint8_t b[1] = { (uint8_t)(ok ? 1 : 0) };
            send_frame(rsp_type, seq, b, 1);
        } break;

        case MSG_RX_READ:
        {
            if (len < 1) { send_error(0x01, seq); break; }

            uint8_t rxbytes = 0;
            if (!cc1200_read_ext(g_radio, CC1200_EXTADDR_NUM_RXBYTES, &rxbytes))
            {
                send_error(0x02, seq);
                break;
            }

            if (rxbytes & 0x80u)
            {
                g_rx_overflow_seen = true;
                g_rx_overflow_count++;
                (void)cc1200_strobe(g_radio, CC1200_CMD_SFRX);
                uint8_t out[1] = { 0 };
                send_frame(rsp_type, seq, out, 1);
                break;
            }

            uint8_t n = (uint8_t)(rxbytes & 0x7Fu);
            uint8_t maxn = body[0];
            if (n > maxn) n = maxn;

            uint8_t out[1 + 255];
            out[0] = n;

            if (n > 0)
            {
                if (!cc1200_fifo_read(g_radio, &out[1], n))
                {
                    send_error(0x03, seq);
                    break;
                }
                g_rx_total_bytes += (uint32_t)n;
                g_last_rx_chunk = n;
            }

            send_frame(rsp_type, seq, out, (uint16_t)(1u + n));
        } break;

        case MSG_STROBE:
        {
            if (len < 1) { send_error(0x01, seq); break; }
            cc1200_status_t st = cc1200_strobe(g_radio, (cc1200_cmd_t)body[0]);
            uint8_t b[1] = { st.raw };
            send_frame(rsp_type, seq, b, 1);
        } break;

        case MSG_TX_FLUSH:
        {
            (void)cc1200_strobe(g_radio, CC1200_CMD_SFTX);
            uint8_t ok = 1;
            send_frame(rsp_type, seq, &ok, 1);
        } break;

        case MSG_TX_WRITE:
        {
            if (len < 2) { send_error(0x01, seq); break; }

            uint8_t flags = body[0];
            uint8_t count = body[1];

            if ((uint16_t)(2u + (uint16_t)count) != len) { send_error(0x02, seq); break; }

            const uint8_t* data = &body[2];

            bool ok = tx_write_fifo(flags, data, count);

            uint8_t out[2];
            out[0] = ok ? 1u : 0u;
            out[1] = ok ? count : 0u;
            send_frame(rsp_type, seq, out, (uint16_t)sizeof(out));
        } break;

        case MSG_TX_SEND:
        {
            uint8_t flags = (len >= 1) ? body[0] : 0u;
            bool ok = tx_send(flags);
            uint8_t out[1] = { ok ? 1u : 0u };
            send_frame(rsp_type, seq, out, 1);
        } break;

        case MSG_PROFILE_CLEAR:
        {
            profile_clear();
            uint8_t ok = 1;
            send_frame(rsp_type, seq, &ok, 1);
        } break;

        case MSG_PROFILE_BEGIN:
        {
            uint16_t expected = 0;
            if (len >= 2)
                expected = (uint16_t)body[0] | ((uint16_t)body[1] << 8);

            bool okb = profile_begin(expected);
            uint8_t ok = okb ? 1u : 0u;
            send_frame(rsp_type, seq, &ok, 1);
        } break;

        case MSG_PROFILE_CHUNK:
        {
            bool okc = profile_chunk_append(body, len);

            uint8_t out[3];
            out[0] = okc ? 1u : 0u;
            out[1] = (uint8_t)(g_prof_count & 0xFFu);
            out[2] = (uint8_t)((g_prof_count >> 8) & 0xFFu);
            send_frame(rsp_type, seq, out, (uint16_t)sizeof(out));
        } break;

        case MSG_PROFILE_APPLY:
        {
            uint8_t flags = (len >= 1) ? body[0] : 0u;

            bool ok = profile_apply(g_radio);

            if (ok)
            {
                if (flags & 0x04u) (void)cc1200_strobe(g_radio, CC1200_CMD_SFRX);
                if (flags & 0x08u) (void)cc1200_strobe(g_radio, CC1200_CMD_SFTX);

                if (flags & 0x02u) (void)cc1200_strobe(g_radio, CC1200_CMD_SIDLE);
                if (flags & 0x01u) (void)cc1200_strobe(g_radio, CC1200_CMD_SRX);

                if ((flags & 0x03u) == 0u)
                    ensure_radio_state();
            }

            uint8_t rsp = ok ? 1u : 0u;
            send_frame(rsp_type, seq, &rsp, 1);
        } break;

        case MSG_SET_FREQ_WORD:
        {
            // Body:
            //   [0] FREQ2
            //   [1] FREQ1
            //   [2] FREQ0
            // Optional:
            //   [3] flags
            //        bit0: request calibration for this retune
            //
            // This does not retune immediately.
            // It queues the newest requested frequency for the RX FSM.

            if (len < 3) { send_error(0x01, seq); break; }

            uint8_t f2 = body[0];
            uint8_t f1 = body[1];
            uint8_t f0 = body[2];
            uint8_t flags = (len >= 4) ? body[3] : 0u;

            uint32_t w = ((uint32_t)f2 << 16) |
                         ((uint32_t)f1 <<  8) |
                         ((uint32_t)f0 <<  0);

            g_pending_freq_word = (w & 0x00FFFFFFu);

            if (flags & 0x01u)
            {
                g_freq_retune_need_cal = true;
            }
            else
            {
                if (abs_u32_diff((w & 0x00FFFFFFu), g_last_applied_freq_word) > FREQ_RETUNE_CAL_THRESHOLD_WORDS)
                    g_freq_retune_need_cal = true;
            }

            g_freq_retune_pending = true;

            uint8_t ok = 1u;
            send_frame(rsp_type, seq, &ok, 1);
        } break;

        default:
            send_error(0xFF, seq);
            break;
    }
}

void usb_proto_init(cc1200_t* radio, const usb_proto_hooks_t* hooks)
{
    g_radio = radio;
    if (hooks) g_hooks = *hooks;

    g_streaming = false;
    g_rx_enc_len = 0;

    g_desired_state = STATE_IDLE;

    g_rx_total_bytes = 0;
    g_tx_total_bytes = 0;

    g_rx_overflow_count = 0;
    g_rx_fifo_err_count = 0;
    g_tx_fifo_err_count = 0;

    g_rx_overflow_seen = false;
    g_rx_fifo_err_seen = false;
    g_tx_fifo_err_seen = false;

    g_last_rx_chunk = 0;

    g_radio_fsm = RADIO_FSM_IDLE;
    g_freq_retune_pending = false;
    g_freq_retune_need_cal = false;
    g_pending_freq_word = 0;
    g_last_applied_freq_word = 0;
    g_freq_retune_count = 0;

    g_last_doppler_retune_us = 0;
    g_max_doppler_retune_us = 0;

    profile_clear();
}

void usb_proto_poll(void)
{
    // 1) Read bytes from stdio (USB CDC)
    while (true)
    {
        int c = getchar_timeout_us(0);
        if (c == PICO_ERROR_TIMEOUT) break;

        uint8_t b = (uint8_t)c;

        if (b == 0x00)
        {
            if (g_rx_enc_len > 0)
            {
                size_t dec_len = cobs_decode(g_rx_dec, sizeof(g_rx_dec), g_rx_enc, g_rx_enc_len);
                g_rx_enc_len = 0;

                if (dec_len >= 6)
                {
                    uint8_t type = g_rx_dec[0];
                    uint8_t seq  = g_rx_dec[1];
                    uint16_t len = (uint16_t)g_rx_dec[2] | ((uint16_t)g_rx_dec[3] << 8);

                    if ((size_t)(4 + len + 2) == dec_len)
                    {
                        uint16_t rx_crc = (uint16_t)g_rx_dec[4 + len] |
                                          ((uint16_t)g_rx_dec[4 + len + 1] << 8);
                        uint16_t calc   = crc16_ccitt_false(g_rx_dec, 4 + len);
                        if (rx_crc == calc)
                        {
                            handle_cmd(type, seq, &g_rx_dec[4], len);
                        }
                        else
                        {
                            hook_error();
                            send_error(0xEE, seq);
                        }
                    }
                    else
                    {
                        hook_error();
                        send_error(0xED, seq);
                    }
                }
            }
        }
        else
        {
            if (g_rx_enc_len < sizeof(g_rx_enc))
                g_rx_enc[g_rx_enc_len++] = b;
            else
            {
                g_rx_enc_len = 0;
                hook_error();
            }
        }
    }

    maybe_stream_rx();
}