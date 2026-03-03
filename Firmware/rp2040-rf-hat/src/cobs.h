#pragma once
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

size_t cobs_encode(uint8_t* dst, size_t dst_max, const uint8_t* src, size_t src_len);
size_t cobs_decode(uint8_t* dst, size_t dst_max, const uint8_t* src, size_t src_len);

#ifdef __cplusplus
}
#endif
