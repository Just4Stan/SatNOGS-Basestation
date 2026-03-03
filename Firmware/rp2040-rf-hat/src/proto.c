// Protocol handler — dual CC1200 over UART
// Based on usb_proto.c from CC1200RXFrontend (Robbe Lehaen)
// Adapted for: UART I/O, dual radio select, PlatformIO/Arduino-Pico
#include "proto.h"

#include <string.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"

#include "cobs.h"
#include "crc16.h"

// CC1200 EXT status registers (SWRU346B)
#define CC1200_EXTADDR_RSSI1        0x71
#define CC1200_EXTADDR_RSSI0        0x72
#define CC1200_EXTADDR_MARCSTATE    0x73
#define CC1200_EXTADDR_NUM_RXBYTES  0xD7

// Framing buffers
#define RX_ENC_MAX   512
#define RX_DEC_MAX   512
#define TX_ENC_MAX   520  // COBS worst-case: payload + ceil(payload/254) overhead

static cc1200_t* g_radios   = NULL;
static uint8_t   g_radio_count = 0;
static uint8_t   g_active   = 0;        // currently selected radio index

static bool g_streaming = false;

static buzzer_cb_t g_buzzer_cb = NULL;

void proto_set_buzzer_cb(buzzer_cb_t cb) {
    g_buzzer_cb = cb;
}

// RX accumulator for COBS frames (from UART)
static uint8_t g_rx_enc[RX_ENC_MAX];
static size_t  g_rx_enc_len = 0;

// Working buffers
static uint8_t g_rx_dec[RX_DEC_MAX];
static uint8_t g_tx_payload[RX_DEC_MAX];
static uint8_t g_tx_enc[TX_ENC_MAX];

// Active radio shortcut
static inline cc1200_t* active_radio(void) {
    if (!g_radios || g_active >= g_radio_count) return NULL;
    return &g_radios[g_active];
}

// ---- UART I/O ----
static void uart_write_bytes(const uint8_t* data, size_t len)
{
    uart_write_blocking(uart0, data, len);
}

static void send_frame(uint8_t type, uint8_t seq, const uint8_t* body, uint16_t body_len)
{
    // Cap body so COBS output fits in TX_ENC_MAX (payload+6 + ~2 COBS overhead)
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

    uart_write_bytes(g_tx_enc, enc_len);
    uint8_t delim = 0x00;
    uart_write_bytes(&delim, 1);
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

    // Wait for IDLE state (poll MARCSTATE, max ~5ms)
    for (int w = 0; w < 50; w++) {
        uint8_t marc = 0;
        if (cc1200_read_ext(dev, CC1200_EXTADDR_MARCSTATE, &marc) && (marc & 0x1F) == 0x01)
            break;  // IDLE
        sleep_us(100);
    }

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

// ---- Metrics ----
static inline int16_t sign_extend_12(uint16_t v12)
{
    v12 &= 0x0FFFu;
    if (v12 & 0x0800u) return (int16_t)(v12 - 0x1000u);
    return (int16_t)v12;
}

static bool read_metrics(uint8_t* out, uint16_t* out_len)
{
    cc1200_t* radio = active_radio();
    if (!radio) return false;

    uint8_t marc = 0, rxbytes = 0, rssi1 = 0, rssi0 = 0;

    if (!cc1200_read_ext(radio, CC1200_EXTADDR_MARCSTATE, &marc)) return false;
    (void)cc1200_read_ext(radio, CC1200_EXTADDR_NUM_RXBYTES, &rxbytes);
    (void)cc1200_read_ext(radio, CC1200_EXTADDR_RSSI1, &rssi1);
    (void)cc1200_read_ext(radio, CC1200_EXTADDR_RSSI0, &rssi0);

    uint16_t rssi12_u = ((uint16_t)rssi1 << 4) | (uint16_t)(rssi0 & 0x0Fu);
    int16_t  rssi12   = sign_extend_12(rssi12_u);

    int16_t rssi_dbm_x10 = (int16_t)-32768;
    int8_t  rssi_raw_1db = 0;

    if (rssi12 != (int16_t)-2048)
    {
        rssi_dbm_x10 = (int16_t)((rssi12 * 10) / 16);
        int16_t approx_dbm = (int16_t)(rssi_dbm_x10 / 10);
        if (approx_dbm < -128) approx_dbm = -128;
        if (approx_dbm >  127) approx_dbm =  127;
        rssi_raw_1db = (int8_t)approx_dbm;
    }

    out[0] = marc;
    out[1] = (uint8_t)(rxbytes & 0x7Fu);
    out[2] = (uint8_t)rssi_raw_1db;
    out[3] = (uint8_t)(rssi_dbm_x10 & 0xFF);
    out[4] = (uint8_t)((rssi_dbm_x10 >> 8) & 0xFF);

    *out_len = 5;
    return true;
}

// ---- Streaming RX ----
static void maybe_stream_rx(void)
{
    if (!g_streaming) return;
    cc1200_t* radio = active_radio();
    if (!radio) return;

    uint8_t rxbytes = 0;
    if (!cc1200_read_ext(radio, CC1200_EXTADDR_NUM_RXBYTES, &rxbytes))
        return;

    uint8_t n = (uint8_t)(rxbytes & 0x7Fu);
    if (n == 0) return;

    while (n > 0)
    {
        uint8_t chunk = (n > 64u) ? 64u : n;
        uint8_t buf[64];

        if (!cc1200_fifo_read(radio, buf, chunk))
            break;

        uint8_t body[1 + 64];
        body[0] = chunk;
        memcpy(&body[1], buf, chunk);

        send_frame(EVT_RX_DATA, 0, body, (uint16_t)(1u + chunk));

        if (!cc1200_read_ext(radio, CC1200_EXTADDR_NUM_RXBYTES, &rxbytes))
            break;
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
            // Body: active(1) count(1) part(1) ver(1)
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
            g_streaming = false;  // reset streaming on radio switch
            profile_clear();
            uint8_t b[1] = { g_active };
            send_frame(rsp_type, seq, b, 1);
        } break;

        case MSG_SET_STATE:
        {
            if (len < 1 || !radio) { send_error(0x01, seq); break; }
            if (body[0] == STATE_RX)
                (void)cc1200_strobe(radio, CC1200_CMD_SRX);
            else if (body[0] == STATE_TX)
                (void)cc1200_strobe(radio, CC1200_CMD_STX);
            else
                (void)cc1200_strobe(radio, CC1200_CMD_SIDLE);
            uint8_t ok = 1;
            send_frame(rsp_type, seq, &ok, 1);
        } break;

        case MSG_SET_STREAMING:
        {
            if (len < 1) { send_error(0x01, seq); break; }
            g_streaming = (body[0] != 0);
            uint8_t b[1] = { (uint8_t)(g_streaming ? 1 : 0) };
            send_frame(rsp_type, seq, b, 1);
        } break;

        case MSG_GET_METRICS:
        {
            uint8_t b[16];
            uint16_t out_len = 0;
            if (!read_metrics(b, &out_len)) { send_error(0x02, seq); break; }
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
            // Validate strobe range (0x30-0x3D)
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

        case MSG_BUZZER:
        {
            if (len < 1) { send_error(0x01, seq); break; }
            uint8_t pattern = body[0];
            if (g_buzzer_cb) g_buzzer_cb(pattern);
            uint8_t ok = 1;
            send_frame(rsp_type, seq, &ok, 1);
        } break;

        default:
            send_error(0xFF, seq);
            break;
    }
}

void proto_init(cc1200_t radios[], uint8_t radio_count)
{
    g_radios = radios;
    g_radio_count = radio_count;
    g_active = 0;
    g_streaming = false;
    g_rx_enc_len = 0;
    profile_clear();
}

void proto_poll(void)
{
    // Read bytes from UART
    while (uart_is_readable(uart0))
    {
        uint8_t b = uart_getc(uart0);

        if (b == 0x00)
        {
            // end-of-frame
            if (g_rx_enc_len > 0)
            {
                size_t dec_len = cobs_decode(g_rx_dec, sizeof(g_rx_dec), g_rx_enc, g_rx_enc_len);
                g_rx_enc_len = 0;

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
                        {
                            handle_cmd(type, seq, &g_rx_dec[4], plen);
                        }
                        else
                        {
                            send_error(0xEE, seq);
                        }
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
            if (g_rx_enc_len < sizeof(g_rx_enc))
                g_rx_enc[g_rx_enc_len++] = b;
            else
                g_rx_enc_len = 0;  // overflow, reset
        }
    }

    // Stream RX data if enabled
    maybe_stream_rx();
}

bool proto_is_streaming(void) {
    return g_streaming;
}

uint8_t proto_active_radio(void) {
    return g_active;
}
