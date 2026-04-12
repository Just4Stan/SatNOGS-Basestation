#pragma once
// AX.25 G3RUH decoder — NRZ-I + G3RUH descrambler + HDLC deframer
//
// Feed demodulated bits from CC1200 serial mode. Outputs complete AX.25 frames.
// Typical use: ISS 437.800 MHz 9600 GFSK, SONATE-2, etc.

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

// Maximum AX.25 frame size (256 info + 14 addr + 1 ctrl + 1 PID + 2 FCS = 274)
// Allow extra for oversized frames
#define AX25_MAX_FRAME  330

typedef struct {
    // G3RUH descrambler: 17-bit LFSR, taps at 12 and 17
    uint32_t g3ruh_sr;

    // NRZ-I decoder
    uint8_t nrzi_prev;

    // HDLC deframer
    uint8_t  frame_buf[AX25_MAX_FRAME];
    uint16_t frame_len;      // bytes accumulated
    uint8_t  bit_buf;        // current byte being assembled
    uint8_t  bit_count;      // bits in bit_buf (0-7)
    uint8_t  ones_count;     // consecutive 1s for bit stuffing
    bool     in_frame;       // true after 0x7E flag detected
    bool     frame_ready;    // true when a complete frame is available
    uint16_t frame_out_len;  // length of completed frame

    // Statistics
    uint32_t flags_seen;
    uint32_t frames_decoded;
    uint32_t crc_errors;
} ax25_decoder_t;

// Initialize decoder state (call once)
void ax25_decoder_init(ax25_decoder_t *d);

// Feed one demodulated bit (0 or 1) from CC1200 serial output.
// Returns true if a complete frame is now available in d->frame_buf[0..d->frame_out_len-1].
bool ax25_decoder_feed_bit(ax25_decoder_t *d, uint8_t bit);

// Reset decoder (e.g., between passes)
void ax25_decoder_reset(ax25_decoder_t *d);

#ifdef __cplusplus
}
#endif
