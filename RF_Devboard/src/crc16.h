#pragma once
#include <stddef.h>
#include <stdint.h>

// CRC-16/CCITT-FALSE: poly=0x1021, init=0xFFFF, xorout=0x0000, refin=false, refout=false
uint16_t crc16_ccitt_false(const uint8_t* data, size_t len);
