// Protocol handler — dual CC1200 + motor control over UART
// Based on usb_proto.c from CC1200RXFrontend (Robbe Lehaen)
// Extended for: motor control commands, position events, IMU access
#include "proto.h"

#include <string.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"

#include "cobs.h"
#include "crc16.h"

// CC1200 EXT status registers (SWRU346B)
#define CC1200_EXTADDR_FREQOFF1     0x0A
#define CC1200_EXTADDR_FREQOFF0     0x0B
#define CC1200_EXTADDR_FREQ2        0x0C
#define CC1200_EXTADDR_FREQ1        0x0D
#define CC1200_EXTADDR_FREQ0        0x0E
#define CC1200_EXTADDR_RSSI1        0x71
#define CC1200_EXTADDR_RSSI0        0x72
#define CC1200_EXTADDR_MARCSTATE    0x73
#define CC1200_EXTADDR_NUM_TXBYTES  0xD6
#define CC1200_EXTADDR_NUM_RXBYTES  0xD7

// Framing buffers
#define RX_ENC_MAX   512
#define RX_DEC_MAX   512
#define TX_ENC_MAX   520  // COBS worst-case: payload + ceil(payload/254) overhead

static cc1200_t* g_radios   = NULL;
static uint8_t   g_radio_count = 0;
static uint8_t   g_active   = 0;        // currently selected radio index

static bool g_streaming = false;
static uint8_t g_streaming_channel = 0;  // channel that enabled streaming

static buzzer_cb_t g_buzzer_cb = NULL;

// Motor callbacks
static motor_set_position_cb_t   g_motor_set_position_cb = NULL;
static motor_get_position_cb_t   g_motor_get_position_cb = NULL;
static motor_park_cb_t           g_motor_park_cb = NULL;
static motor_stop_cb_t           g_motor_stop_cb = NULL;
static motor_home_cb_t           g_motor_home_cb = NULL;
static motor_get_status_cb_t     g_motor_get_status_cb = NULL;
static motor_set_gains_cb_t      g_motor_set_gains_cb = NULL;
static motor_zero_encoders_cb_t  g_motor_zero_encoders_cb = NULL;
static imu_read_cb_t             g_imu_read_cb = NULL;
static motor_recal_cb_t          g_motor_recal_cb = NULL;

static bool g_position_streaming = false;

// Radio FSM — auto-recovers from FIFO errors, keeps radio in desired state
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

// Cumulative counters
static uint32_t g_rx_total_bytes = 0;
static uint32_t g_tx_total_bytes = 0;
static uint16_t g_rx_overflow_count = 0;
static uint16_t g_rx_fifo_err_count = 0;
static uint16_t g_tx_fifo_err_count = 0;
static bool     g_rx_overflow_seen = false;
static bool     g_rx_fifo_err_seen = false;
static bool     g_tx_fifo_err_seen = false;
static uint8_t  g_last_rx_chunk = 0;

void proto_set_buzzer_cb(buzzer_cb_t cb) { g_buzzer_cb = cb; }

void proto_set_motor_set_position_cb(motor_set_position_cb_t cb)  { g_motor_set_position_cb = cb; }
void proto_set_motor_get_position_cb(motor_get_position_cb_t cb)  { g_motor_get_position_cb = cb; }
void proto_set_motor_park_cb(motor_park_cb_t cb)                  { g_motor_park_cb = cb; }
void proto_set_motor_stop_cb(motor_stop_cb_t cb)                  { g_motor_stop_cb = cb; }
void proto_set_motor_home_cb(motor_home_cb_t cb)                  { g_motor_home_cb = cb; }
void proto_set_motor_get_status_cb(motor_get_status_cb_t cb)      { g_motor_get_status_cb = cb; }
void proto_set_motor_set_gains_cb(motor_set_gains_cb_t cb)        { g_motor_set_gains_cb = cb; }
void proto_set_motor_zero_encoders_cb(motor_zero_encoders_cb_t cb){ g_motor_zero_encoders_cb = cb; }
void proto_set_imu_read_cb(imu_read_cb_t cb)                     { g_imu_read_cb = cb; }
void proto_set_motor_recal_cb(motor_recal_cb_t cb)                { g_motor_recal_cb = cb; }

void proto_set_position_streaming(bool enable) { g_position_streaming = enable; }
bool proto_is_position_streaming(void)         { return g_position_streaming; }

// Transport write functions (channel 0 = UART, channel 1 = USB)
#define PROTO_NUM_CHANNELS 2
static proto_write_fn_t g_write_fn[PROTO_NUM_CHANNELS] = { NULL, NULL };
static uint8_t g_active_channel = 0;  // which channel is currently being replied to

// RX accumulators for COBS frames — one per channel
static uint8_t g_rx_enc[PROTO_NUM_CHANNELS][RX_ENC_MAX];
static size_t  g_rx_enc_len[PROTO_NUM_CHANNELS] = { 0, 0 };

// Working buffers (shared — only one command processed at a time)
static uint8_t g_rx_dec[RX_DEC_MAX];
static uint8_t g_tx_payload[RX_DEC_MAX];
static uint8_t g_tx_enc[TX_ENC_MAX];

// Active radio shortcut
static inline cc1200_t* active_radio(void) {
    if (!g_radios || g_active >= g_radio_count) return NULL;
    return &g_radios[g_active];
}

// Helpers
static inline uint32_t uptime_ms_now(void) {
    return to_ms_since_boot(get_absolute_time());
}

static inline uint32_t abs_u32_diff(uint32_t a, uint32_t b) {
    return (a >= b) ? (a - b) : (b - a);
}

// Float LE helpers
static inline void put_f32_le(uint8_t* buf, float val) {
    union { float f; uint8_t b[4]; } u;
    u.f = val;
    buf[0] = u.b[0]; buf[1] = u.b[1]; buf[2] = u.b[2]; buf[3] = u.b[3];
}

static inline float get_f32_le(const uint8_t* buf) {
    union { float f; uint8_t b[4]; } u;
    u.b[0] = buf[0]; u.b[1] = buf[1]; u.b[2] = buf[2]; u.b[3] = buf[3];
    return u.f;
}

// Radio FSM — auto-recover from FIFO errors, keep radio in desired state
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
        // TX is explicit — do not auto-restart
    } else {
        if (st.state != CC1200_STATE_IDLE)
            (void)cc1200_strobe(radio, CC1200_CMD_SIDLE);
    }
}

// Doppler retune — IDLE -> write FREQ -> optional SCAL -> SRX
static bool perform_freq_retune(void) {
    cc1200_t* radio = active_radio();
    if (!radio) return false;

    uint32_t w = g_pending_freq_word & 0x00FFFFFFu;
    uint8_t freq_bytes[3];
    freq_bytes[0] = (uint8_t)((w >> 16) & 0xFFu);
    freq_bytes[1] = (uint8_t)((w >>  8) & 0xFFu);
    freq_bytes[2] = (uint8_t)( w        & 0xFFu);

    if (g_freq_retune_need_cal) {
        (void)cc1200_strobe(radio, CC1200_CMD_SIDLE);
        bool ok = cc1200_write_ext_burst(radio, CC1200_EXTADDR_FREQ2, freq_bytes, 3);
        if (!ok) return false;
        uint8_t zero[2] = {0, 0};
        (void)cc1200_write_ext_burst(radio, CC1200_EXTADDR_FREQOFF1, zero, 2);
        (void)cc1200_strobe(radio, CC1200_CMD_SCAL);
        g_freq_retune_need_cal = false;
        if (g_desired_state == STATE_RX)
            (void)cc1200_strobe(radio, CC1200_CMD_SRX);
        else if (g_desired_state == STATE_TX)
            (void)cc1200_strobe(radio, CC1200_CMD_STX);
        sleep_us(500);
    } else {
        int32_t delta = (int32_t)w - (int32_t)(g_last_applied_freq_word & 0x00FFFFFFu);
        int32_t freqoff = delta * 4;
        if (freqoff > 32767) freqoff = 32767;
        if (freqoff < -32768) freqoff = -32768;
        uint8_t fo[2];
        fo[0] = (uint8_t)((freqoff >> 8) & 0xFF);
        fo[1] = (uint8_t)(freqoff & 0xFF);
        bool ok = cc1200_write_ext_burst(radio, CC1200_EXTADDR_FREQOFF1, fo, 2);
        if (!ok) return false;
    }

    g_freq_retune_pending = false;
    return true;
}

// ---- Transport I/O ----
static void write_to_active_channel(const uint8_t* data, size_t len)
{
    if (g_write_fn[g_active_channel])
        g_write_fn[g_active_channel](data, len);
}

// Default UART write (used if no write_fn set for channel 0)
static void default_uart_write(const uint8_t* data, size_t len)
{
    uart_write_blocking(uart0, data, len);
}

static void send_frame(uint8_t type, uint8_t seq, const uint8_t* body, uint16_t body_len)
{
    if (body_len > (uint16_t)(sizeof(g_tx_payload) - 8)) return;

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

    write_to_active_channel(g_tx_enc, enc_len);
    uint8_t delim = 0x00;
    write_to_active_channel(&delim, 1);
}

// Send frame on a specific channel (for events that go to UART regardless)
static void send_frame_on_channel(uint8_t channel, uint8_t type, uint8_t seq,
                                  const uint8_t* body, uint16_t body_len)
{
    uint8_t saved = g_active_channel;
    g_active_channel = channel;
    send_frame(type, seq, body, body_len);
    g_active_channel = saved;
}

static void send_error(uint8_t code, uint8_t seq)
{
    uint8_t b[2] = { code, 0 };
    send_frame(EVT_ERROR, seq, b, (uint16_t)sizeof(b));
}

// ---- Profile staging (RAM) ----
typedef struct {
    uint8_t is_ext;
    uint8_t addr;
    uint8_t val;
} prof_entry_t;

#define PROFILE_MAX_ENTRIES  768

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

    bool idle_ok = false;
    for (int w = 0; w < 50; w++) {
        uint8_t marc = 0;
        if (cc1200_read_ext(dev, CC1200_EXTADDR_MARCSTATE, &marc) && (marc & 0x1F) == 0x01) {
            idle_ok = true;
            break;
        }
        sleep_us(100);
    }
    if (!idle_ok) return false;

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

// ---- Metrics v2 (28 bytes) ----
static inline int16_t sign_extend_12(uint16_t v12)
{
    v12 &= 0x0FFFu;
    if (v12 & 0x0800u) return (int16_t)(v12 - 0x1000u);
    return (int16_t)v12;
}

static bool build_metrics(uint8_t* out, uint16_t* out_len)
{
    cc1200_t* radio = active_radio();
    if (!radio) return false;

    cc1200_status_t chip_st = cc1200_strobe(radio, CC1200_CMD_SNOP);

    uint8_t marc = 0, rxbytes = 0, txbytes = 0, rssi1 = 0, rssi0 = 0;
    (void)cc1200_read_ext(radio, CC1200_EXTADDR_MARCSTATE, &marc);
    (void)cc1200_read_ext(radio, CC1200_EXTADDR_NUM_RXBYTES, &rxbytes);
    (void)cc1200_read_ext(radio, CC1200_EXTADDR_NUM_TXBYTES, &txbytes);
    (void)cc1200_read_ext(radio, CC1200_EXTADDR_RSSI1, &rssi1);
    (void)cc1200_read_ext(radio, CC1200_EXTADDR_RSSI0, &rssi0);

    int16_t rssi_dbm_x10;
    if (!(rssi0 & 0x01u)) {
        rssi_dbm_x10 = (int16_t)-32768;
    } else {
        uint16_t rssi12_u = ((uint16_t)rssi1 << 4) | (uint16_t)((rssi0 >> 4) & 0x0Fu);
        int16_t  rssi12   = sign_extend_12(rssi12_u);
        rssi_dbm_x10 = (rssi12 != (int16_t)-2048)
                        ? (int16_t)((rssi12 * 10) / 16)
                        : (int16_t)-32768;
    }

    uint8_t flags = 0;
    if (g_streaming)        flags |= 0x01;
    if (g_rx_overflow_seen) flags |= 0x02;
    if (g_rx_fifo_err_seen) flags |= 0x04;
    if (g_tx_fifo_err_seen) flags |= 0x08;
    g_rx_overflow_seen = false;
    g_rx_fifo_err_seen = false;
    g_tx_fifo_err_seen = false;

    uint32_t uptime = uptime_ms_now();

    memset(out, 0, 28);
    out[0] = 2;
    out[1] = flags;
    out[2] = (uint8_t)g_desired_state;
    out[3] = chip_st.raw;
    out[4] = marc;
    out[5] = (uint8_t)(rxbytes & 0x7Fu);
    out[6] = txbytes;
    out[7] = g_last_rx_chunk;
    out[8] = (uint8_t)(rssi_dbm_x10 & 0xFF);
    out[9] = (uint8_t)((rssi_dbm_x10 >> 8) & 0xFF);
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
    out[24] = (uint8_t)(uptime & 0xFF);
    out[25] = (uint8_t)((uptime >> 8) & 0xFF);
    out[26] = (uint8_t)((uptime >> 16) & 0xFF);
    out[27] = (uint8_t)((uptime >> 24) & 0xFF);

    *out_len = 28;
    return true;
}

// ---- Streaming RX ----
static void maybe_stream_rx(void)
{
    if (!g_streaming) return;
    cc1200_t* radio = active_radio();
    if (!radio) return;

    if (g_freq_retune_pending) {
        perform_freq_retune();
    }

    ensure_radio_state();

    uint8_t rxbytes = 0;
    if (!cc1200_read_ext(radio, CC1200_EXTADDR_NUM_RXBYTES, &rxbytes))
        return;

    if (rxbytes & 0x80u) {
        g_rx_overflow_seen = true;
        g_rx_overflow_count++;
        (void)cc1200_strobe(radio, CC1200_CMD_SFRX);
        if (g_desired_state == STATE_RX)
            (void)cc1200_strobe(radio, CC1200_CMD_SRX);
        return;
    }

    uint8_t n = (uint8_t)(rxbytes & 0x7Fu);
    if (n == 0) return;

    while (n > 0)
    {
        uint8_t chunk = (n > 64u) ? 64u : n;
        uint8_t buf[64];

        if (!cc1200_fifo_read(radio, buf, chunk))
            break;

        g_rx_total_bytes += chunk;
        g_last_rx_chunk = chunk;

        uint8_t body[1 + 64];
        body[0] = chunk;
        memcpy(&body[1], buf, chunk);

        send_frame(EVT_RX_DATA, 0, body, (uint16_t)(1u + chunk));

        if (g_freq_retune_pending)
            break;

        if (!cc1200_read_ext(radio, CC1200_EXTADDR_NUM_RXBYTES, &rxbytes))
            break;

        if (rxbytes & 0x80u) {
            g_rx_overflow_seen = true;
            g_rx_overflow_count++;
            (void)cc1200_strobe(radio, CC1200_CMD_SFRX);
            if (g_desired_state == STATE_RX)
                (void)cc1200_strobe(radio, CC1200_CMD_SRX);
            break;
        }
        n = (uint8_t)(rxbytes & 0x7Fu);
    }
}

// ---- TX helpers ----
static bool tx_write_fifo(uint8_t flags, const uint8_t* data, uint8_t count)
{
    cc1200_t* radio = active_radio();
    if (!radio) return false;

    if (flags & 0x02u)
        (void)cc1200_strobe(radio, CC1200_CMD_SIDLE);
    if (flags & 0x01u)
        (void)cc1200_strobe(radio, CC1200_CMD_SFTX);

    if (count == 0) return true;
    return cc1200_fifo_write(radio, data, (size_t)count);
}

static bool tx_send(uint8_t flags)
{
    cc1200_t* radio = active_radio();
    if (!radio) return false;

    if (flags & 0x04u)
        (void)cc1200_strobe(radio, CC1200_CMD_SFTX);

    (void)cc1200_strobe(radio, CC1200_CMD_STX);

    if (flags & 0x01u) (void)cc1200_strobe(radio, CC1200_CMD_SIDLE);
    if (flags & 0x02u) (void)cc1200_strobe(radio, CC1200_CMD_SRX);

    return true;
}

// ---- Command handler ----
static void handle_cmd(uint8_t type, uint8_t seq, const uint8_t* body, uint16_t len)
{
    const uint8_t rsp_type = (uint8_t)(type | MSG_RSP_FLAG);
    cc1200_t* radio = active_radio();

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
            if (radio) {
                cc1200_read_ext(radio, 0x8F, &part);
                cc1200_read_ext(radio, 0x90, &ver);
            }
            uint8_t b[4] = { g_active, g_radio_count, part, ver };
            send_frame(rsp_type, seq, b, (uint16_t)sizeof(b));
        } break;

        case MSG_SELECT_RADIO:
        {
            if (len < 1) { send_error(0x01, seq); break; }
            uint8_t idx = body[0];
            if (idx >= g_radio_count) { send_error(0x02, seq); break; }
            g_active = idx;
            g_streaming = false;
            g_desired_state = STATE_IDLE;
            g_radio_fsm = RADIO_FSM_IDLE;
            g_freq_retune_pending = false;
            profile_clear();
            uint8_t b[1] = { g_active };
            send_frame(rsp_type, seq, b, 1);
        } break;

        case MSG_SET_STATE:
        {
            if (len < 1 || !radio) { send_error(0x01, seq); break; }
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
            uint8_t ok = 1;
            send_frame(rsp_type, seq, &ok, 1);
        } break;

        case MSG_SET_STREAMING:
        {
            if (len < 1) { send_error(0x01, seq); break; }
            g_streaming = (body[0] != 0);
            if (g_streaming) g_streaming_channel = g_active_channel;
            if (g_streaming && radio) {
                set_desired_state(STATE_RX);
                g_radio_fsm = RADIO_FSM_RX;
                (void)cc1200_strobe(radio, CC1200_CMD_SRX);
            }
            uint8_t b[1] = { (uint8_t)(g_streaming ? 1 : 0) };
            send_frame(rsp_type, seq, b, 1);
        } break;

        case MSG_GET_METRICS:
        {
            uint8_t b[32];
            uint16_t out_len = 0;
            if (!build_metrics(b, &out_len)) { send_error(0x02, seq); break; }
            send_frame(rsp_type, seq, b, out_len);
        } break;

        case MSG_READ_REG:
        {
            if (len < 1 || !radio) { send_error(0x01, seq); break; }
            uint8_t v = 0;
            bool ok = cc1200_read_reg(radio, body[0], &v);
            uint8_t b[2] = { (uint8_t)(ok ? 1 : 0), v };
            send_frame(rsp_type, seq, b, (uint16_t)sizeof(b));
        } break;

        case MSG_WRITE_REG:
        {
            if (len < 2 || !radio) { send_error(0x01, seq); break; }
            bool ok = cc1200_write_reg(radio, body[0], body[1]);
            uint8_t b[1] = { (uint8_t)(ok ? 1 : 0) };
            send_frame(rsp_type, seq, b, 1);
        } break;

        case MSG_READ_EXT:
        {
            if (len < 1 || !radio) { send_error(0x01, seq); break; }
            uint8_t v = 0;
            bool ok = cc1200_read_ext(radio, body[0], &v);
            uint8_t b[2] = { (uint8_t)(ok ? 1 : 0), v };
            send_frame(rsp_type, seq, b, (uint16_t)sizeof(b));
        } break;

        case MSG_WRITE_EXT:
        {
            if (len < 2 || !radio) { send_error(0x01, seq); break; }
            bool ok = cc1200_write_ext(radio, body[0], body[1]);
            uint8_t b[1] = { (uint8_t)(ok ? 1 : 0) };
            send_frame(rsp_type, seq, b, 1);
        } break;

        case MSG_RX_READ:
        {
            if (len < 1 || !radio) { send_error(0x01, seq); break; }
            uint8_t rxbytes = 0;
            if (!cc1200_read_ext(radio, CC1200_EXTADDR_NUM_RXBYTES, &rxbytes)) { send_error(0x02, seq); break; }
            if (rxbytes & 0x80u) {
                g_rx_overflow_seen = true;
                g_rx_overflow_count++;
                (void)cc1200_strobe(radio, CC1200_CMD_SFRX);
                if (g_desired_state == STATE_RX)
                    (void)cc1200_strobe(radio, CC1200_CMD_SRX);
                send_error(0x04, seq);
                break;
            }
            uint8_t n = (uint8_t)(rxbytes & 0x7Fu);
            uint8_t maxn = body[0];
            if (n > maxn) n = maxn;
            uint8_t out[1 + 255];
            out[0] = n;
            if (n > 0) {
                if (!cc1200_fifo_read(radio, &out[1], n)) { send_error(0x03, seq); break; }
            }
            send_frame(rsp_type, seq, out, (uint16_t)(1u + n));
        } break;

        case MSG_STROBE:
        {
            if (len < 1 || !radio) { send_error(0x01, seq); break; }
            if (body[0] < 0x30 || body[0] > 0x3D) { send_error(0x02, seq); break; }
            cc1200_status_t st = cc1200_strobe(radio, (cc1200_cmd_t)body[0]);
            uint8_t b[1] = { st.raw };
            send_frame(rsp_type, seq, b, 1);
        } break;

        case MSG_TX_FLUSH:
        {
            if (!radio) { send_error(0x01, seq); break; }
            (void)cc1200_strobe(radio, CC1200_CMD_SFTX);
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
            if (ok) g_tx_total_bytes += count;
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
            bool ok = profile_apply(radio);
            if (ok) {
                if (flags & 0x04u) (void)cc1200_strobe(radio, CC1200_CMD_SFRX);
                if (flags & 0x08u) (void)cc1200_strobe(radio, CC1200_CMD_SFTX);
                if (flags & 0x02u) (void)cc1200_strobe(radio, CC1200_CMD_SIDLE);
                if (flags & 0x01u) (void)cc1200_strobe(radio, CC1200_CMD_SRX);
            }
            uint8_t rsp = ok ? 1u : 0u;
            send_frame(rsp_type, seq, &rsp, 1);
        } break;

        case MSG_SET_FREQ_WORD:
        {
            if (len < 3 || !radio) { send_error(0x01, seq); break; }
            uint32_t new_word = ((uint32_t)body[0] << 16) |
                                ((uint32_t)body[1] << 8)  |
                                ((uint32_t)body[2]);
            bool force_cal = (len >= 4 && (body[3] & 0x01u));

            uint32_t diff = abs_u32_diff(new_word, g_last_applied_freq_word);
            bool need_cal = force_cal || (diff > 0x0D00u);

            g_pending_freq_word = new_word;
            g_freq_retune_need_cal = need_cal;
            g_freq_retune_pending = true;

            if (!g_streaming) {
                perform_freq_retune();
            }

            uint8_t ok = 1;
            send_frame(rsp_type, seq, &ok, 1);
        } break;

        case MSG_BUZZER:
        {
            if (len < 1) { send_error(0x01, seq); break; }
            uint8_t pattern = body[0];
            if (g_buzzer_cb) g_buzzer_cb(pattern);
            uint8_t ok = 1;
            send_frame(rsp_type, seq, &ok, 1);
        } break;

        // ---- Motor control commands ----
        case MSG_SET_POSITION:
        {
            if (len < 8) { send_error(0x01, seq); break; }
            float az = get_f32_le(&body[0]);
            float el = get_f32_le(&body[4]);
            if (g_motor_set_position_cb) g_motor_set_position_cb(az, el);
            uint8_t ok = 1;
            send_frame(rsp_type, seq, &ok, 1);
        } break;

        case MSG_GET_POSITION:
        {
            if (!g_motor_get_position_cb) { send_error(0x02, seq); break; }
            float az = 0, el = 0;
            uint8_t status = 0;
            g_motor_get_position_cb(&az, &el, &status);
            uint8_t b[9];
            put_f32_le(&b[0], az);
            put_f32_le(&b[4], el);
            b[8] = status;
            send_frame(rsp_type, seq, b, 9);
        } break;

        case MSG_PARK:
        {
            if (g_motor_park_cb) g_motor_park_cb();
            uint8_t ok = 1;
            send_frame(rsp_type, seq, &ok, 1);
        } break;

        case MSG_STOP:
        {
            if (g_motor_stop_cb) g_motor_stop_cb();
            uint8_t ok = 1;
            send_frame(rsp_type, seq, &ok, 1);
        } break;

        case MSG_HOME:
        {
            if (g_motor_home_cb) g_motor_home_cb();
            uint8_t ok = 1;
            send_frame(rsp_type, seq, &ok, 1);
        } break;

        case MSG_GET_MOTOR_STATUS:
        {
            if (!g_motor_get_status_cb) { send_error(0x02, seq); break; }
            uint8_t b[32];
            uint16_t out_len = 0;
            g_motor_get_status_cb(b, &out_len);
            send_frame(rsp_type, seq, b, out_len);
        } break;

        case MSG_SET_GAINS:
        {
            if (len < 12) { send_error(0x01, seq); break; }
            float kp = get_f32_le(&body[0]);
            float ki = get_f32_le(&body[4]);
            float kd = get_f32_le(&body[8]);
            if (g_motor_set_gains_cb) g_motor_set_gains_cb(kp, ki, kd);
            uint8_t ok = 1;
            send_frame(rsp_type, seq, &ok, 1);
        } break;

        case MSG_ZERO_ENCODERS:
        {
            if (g_motor_zero_encoders_cb) g_motor_zero_encoders_cb();
            uint8_t ok = 1;
            send_frame(rsp_type, seq, &ok, 1);
        } break;

        case MSG_IMU_READ:
        {
            if (!g_imu_read_cb) { send_error(0x02, seq); break; }
            uint8_t b[16];
            uint16_t out_len = 0;
            g_imu_read_cb(b, &out_len);
            send_frame(rsp_type, seq, b, out_len);
        } break;

        case MSG_RECAL:
        {
            if (g_motor_recal_cb) g_motor_recal_cb();
            uint8_t ok = 1;
            send_frame(rsp_type, seq, &ok, 1);
        } break;

        default:
            send_error(0xFF, seq);
            break;
    }
}

void proto_set_write_fn(uint8_t channel, proto_write_fn_t fn)
{
    if (channel < PROTO_NUM_CHANNELS)
        g_write_fn[channel] = fn;
}

void proto_init(cc1200_t radios[], uint8_t radio_count)
{
    g_radios = radios;
    g_radio_count = radio_count;
    g_active = 0;
    g_streaming = false;
    g_rx_enc_len[0] = 0;
    g_rx_enc_len[1] = 0;
    g_active_channel = 0;
    g_write_fn[0] = default_uart_write;
    g_write_fn[1] = NULL;
    g_desired_state = STATE_IDLE;
    g_radio_fsm = RADIO_FSM_IDLE;
    g_freq_retune_pending = false;
    g_freq_retune_need_cal = false;
    g_pending_freq_word = 0;
    g_last_applied_freq_word = 0;
    g_rx_total_bytes = 0;
    g_tx_total_bytes = 0;
    g_rx_overflow_count = 0;
    g_rx_fifo_err_count = 0;
    g_tx_fifo_err_count = 0;
    g_rx_overflow_seen = false;
    g_rx_fifo_err_seen = false;
    g_tx_fifo_err_seen = false;
    g_last_rx_chunk = 0;
    g_position_streaming = false;
    profile_clear();
}

// Process one incoming byte on a given channel
static void process_byte(uint8_t b, uint8_t channel)
{
    if (channel >= PROTO_NUM_CHANNELS) return;

    if (b == 0x00)
    {
        if (g_rx_enc_len[channel] > 0)
        {
            size_t dec_len = cobs_decode(g_rx_dec, sizeof(g_rx_dec),
                                          g_rx_enc[channel], g_rx_enc_len[channel]);
            g_rx_enc_len[channel] = 0;

            g_active_channel = channel;

            if (dec_len >= 6)
            {
                uint8_t type = g_rx_dec[0];
                uint8_t seq  = g_rx_dec[1];
                uint16_t plen = (uint16_t)g_rx_dec[2] | ((uint16_t)g_rx_dec[3] << 8);

                if ((size_t)(4 + plen + 2) == dec_len)
                {
                    uint16_t rx_crc = (uint16_t)g_rx_dec[4 + plen] | ((uint16_t)g_rx_dec[4 + plen + 1] << 8);
                    uint16_t calc   = crc16_ccitt_false(g_rx_dec, 4 + plen);
                    if (rx_crc == calc)
                        handle_cmd(type, seq, &g_rx_dec[4], plen);
                    else
                        send_error(0xEE, seq);
                }
                else
                {
                    send_error(0xED, 0);
                }
            }
        }
    }
    else
    {
        if (g_rx_enc_len[channel] < RX_ENC_MAX)
            g_rx_enc[channel][g_rx_enc_len[channel]++] = b;
        else
            g_rx_enc_len[channel] = 0;
    }
}

void proto_feed_bytes(const uint8_t* data, size_t len, uint8_t channel)
{
    for (size_t i = 0; i < len; i++)
        process_byte(data[i], channel);
}

void proto_poll(void)
{
    // Read bytes from UART (channel 0)
    while (uart_is_readable(uart0))
        process_byte(uart_getc(uart0), 0);

    // Stream RX data — events go to whichever channel enabled streaming
    g_active_channel = g_streaming_channel;
    maybe_stream_rx();
}

bool proto_is_streaming(void) {
    return g_streaming;
}

uint8_t proto_active_radio(void) {
    return g_active;
}

void proto_send_position_event(float az_deg, float el_deg, uint8_t status) {
    uint8_t b[9];
    put_f32_le(&b[0], az_deg);
    put_f32_le(&b[4], el_deg);
    b[8] = status;
    // Send on UART channel (0)
    send_frame_on_channel(0, EVT_POSITION, 0, b, 9);
}
