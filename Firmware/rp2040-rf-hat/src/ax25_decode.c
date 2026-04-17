// AX.25 G3RUH decoder — NRZ-I + G3RUH descrambler + HDLC deframer
//
// Pipeline per bit:
//   1. NRZ-I decode (differential → absolute)
//   2. G3RUH descramble (17-bit LFSR, taps 12+17)
//   3. HDLC deframe (0x7E flag, bit destuffing, CRC-16/CCITT check)
//
// Reference:
//   - G3RUH scrambler: https://www.amsat.org/amsat/articles/g3ruh/109.html
//   - HDLC/AX.25: ITU-T X.25 / AX.25 Link Access Procedure

#include "ax25_decode.h"
#include <string.h>

// CRC-16/CCITT (X.25 FCS) — polynomial 0x8408 (reflected)
static uint16_t crc16_ccitt_update(uint16_t crc, uint8_t bit) {
    uint8_t feedback = (crc & 1) ^ bit;
    crc >>= 1;
    if (feedback)
        crc ^= 0x8408;
    return crc;
}

void ax25_decoder_init(ax25_decoder_t *d) {
    memset(d, 0, sizeof(*d));
    d->g3ruh_sr = 0;
    d->nrzi_prev = 0;
}

void ax25_decoder_reset(ax25_decoder_t *d) {
    d->g3ruh_sr = 0;
    d->nrzi_prev = 0;
    d->frame_len = 0;
    d->bit_buf = 0;
    d->bit_count = 0;
    d->ones_count = 0;
    d->in_frame = false;
    d->frame_ready = false;
    d->frame_out_len = 0;
}

// G3RUH descrambler: x^17 + x^12 + 1
// Input: scrambled bit from demodulator
// Output: descrambled bit
static inline uint8_t g3ruh_descramble(ax25_decoder_t *d, uint8_t bit) {
    // G3RUH descrambler: polynomial x^17 + x^12 + 1
    // Shift register: bit 0 = most recently shifted in, bit 16 = oldest
    // Tap at delay 12: sr >> 11, tap at delay 17: sr >> 16
    uint8_t tap_12 = (d->g3ruh_sr >> 11) & 1;
    uint8_t tap_17 = (d->g3ruh_sr >> 16) & 1;
    uint8_t out = bit ^ tap_12 ^ tap_17;

    // Shift input bit into register (bit enters at position 0)
    d->g3ruh_sr = ((d->g3ruh_sr << 1) | (bit & 1)) & 0x1FFFF;

    return out;
}

// NRZ-I decode: same bit = 1, transition = 0
static inline uint8_t nrzi_decode(ax25_decoder_t *d, uint8_t bit) {
    uint8_t decoded = (bit == d->nrzi_prev) ? 1 : 0;
    d->nrzi_prev = bit;
    return decoded;
}

bool ax25_decoder_feed_bit(ax25_decoder_t *d, uint8_t raw_bit) {
    d->frame_ready = false;

    // TX chain: data → NRZ-I encode → G3RUH scramble → modulate
    // RX chain: demodulate → G3RUH descramble → NRZ-I decode
    // Step 1: G3RUH descramble (operates on raw demodulated bits)
    uint8_t descrambled = g3ruh_descramble(d, raw_bit & 1);

    // Step 2: NRZ-I decode (operates on descrambled bits)
    uint8_t bit = nrzi_decode(d, descrambled);

    // Step 3: HDLC deframing
    // Track consecutive 1s
    if (bit) {
        d->ones_count++;

        // 7+ consecutive 1s = abort
        if (d->ones_count >= 7) {
            d->in_frame = false;
            d->frame_len = 0;
            d->bit_count = 0;
            d->bit_buf = 0;
            return false;
        }

        // 6 consecutive 1s might be a flag (0x7E = 01111110)
        if (d->ones_count == 6) {
            // This could be a flag — we'll confirm on the next 0 bit
            return false;
        }

        // Normal data bit (ones_count < 6)
        if (d->in_frame) {
            d->bit_buf = (d->bit_buf >> 1) | 0x80;
            d->bit_count++;
            if (d->bit_count == 8) {
                if (d->frame_len < AX25_MAX_FRAME)
                    d->frame_buf[d->frame_len++] = d->bit_buf;
                d->bit_buf = 0;
                d->bit_count = 0;
            }
        }
    } else {
        // bit == 0
        if (d->ones_count == 5) {
            // Bit stuffing: 0 after five 1s — discard this bit
            d->ones_count = 0;
            return false;
        }

        if (d->ones_count == 6) {
            // Flag detected (0x7E): 01111110
            d->flags_seen++;

            if (d->in_frame && d->frame_len >= 17) {
                // Minimum AX.25: 14 addr + 1 ctrl + 2 FCS = 17 bytes
                // Verify CRC-16/CCITT (FCS)
                // FCS is computed over all bytes except the FCS itself
                // Good FCS leaves 0xF0B8 as residual
                uint16_t crc = 0xFFFF;
                for (uint16_t i = 0; i < d->frame_len; i++) {
                    uint8_t byte = d->frame_buf[i];
                    for (int b = 0; b < 8; b++) {
                        crc = crc16_ccitt_update(crc, (byte >> b) & 1);
                    }
                }

                if (crc == 0xF0B8) {
                    // Valid frame! Strip FCS (last 2 bytes)
                    d->frame_out_len = d->frame_len - 2;
                    d->frame_ready = true;
                    d->frames_decoded++;
                } else {
                    d->crc_errors++;
                }
            }

            // Start new frame after flag
            d->in_frame = true;
            d->frame_len = 0;
            d->bit_buf = 0;
            d->bit_count = 0;
            d->ones_count = 0;
            return d->frame_ready;
        }

        d->ones_count = 0;

        if (d->in_frame) {
            d->bit_buf = (d->bit_buf >> 1);  // 0 bit — MSB stays 0
            d->bit_count++;
            if (d->bit_count == 8) {
                if (d->frame_len < AX25_MAX_FRAME)
                    d->frame_buf[d->frame_len++] = d->bit_buf;
                d->bit_buf = 0;
                d->bit_count = 0;
            }
        }
    }

    return false;
}
