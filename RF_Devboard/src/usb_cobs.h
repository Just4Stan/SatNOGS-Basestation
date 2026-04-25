#pragma once
#include <stddef.h>
#include <stdint.h>

// Returns encoded length, or 0 on error (dst too small)
size_t cobs_encode(uint8_t* dst, size_t dst_max, const uint8_t* src, size_t src_len);

// Returns decoded length, or 0 on error (malformed or dst too small)
size_t cobs_decode(uint8_t* dst, size_t dst_max, const uint8_t* src, size_t src_len);
