// COBS encoder/decoder — from CC1200RXFrontend (Robbe Lehaen)
#include "cobs.h"

size_t cobs_encode(uint8_t* dst, size_t dst_max, const uint8_t* src, size_t src_len)
{
    if (!dst || !src) return 0;

    size_t read_i = 0;
    size_t write_i = 1;
    size_t code_i = 0;
    uint8_t code = 1;

    if (dst_max == 0) return 0;

    while (read_i < src_len)
    {
        if (write_i >= dst_max) return 0;

        if (src[read_i] == 0)
        {
            dst[code_i] = code;
            code_i = write_i++;
            code = 1;
            read_i++;
        }
        else
        {
            dst[write_i++] = src[read_i++];
            code++;
            if (code == 0xFF)
            {
                dst[code_i] = code;
                code_i = write_i++;
                code = 1;
                if (write_i > dst_max) return 0;
            }
        }
    }

    if (code_i >= dst_max) return 0;
    dst[code_i] = code;
    return write_i;
}

size_t cobs_decode(uint8_t* dst, size_t dst_max, const uint8_t* src, size_t src_len)
{
    if (!dst || !src) return 0;
    if (src_len == 0) return 0;

    size_t read_i = 0;
    size_t write_i = 0;

    while (read_i < src_len)
    {
        uint8_t code = src[read_i++];
        if (code == 0) return 0;

        for (uint8_t i = 1; i < code; i++)
        {
            if (read_i >= src_len) return 0;
            if (write_i >= dst_max) return 0;
            dst[write_i++] = src[read_i++];
        }

        if (code != 0xFF && read_i < src_len)
        {
            if (write_i >= dst_max) return 0;
            dst[write_i++] = 0x00;
        }
    }

    return write_i;
}
