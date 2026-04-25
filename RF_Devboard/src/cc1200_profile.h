#pragma once
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#include "cc1200.h"

typedef struct {
    uint8_t addr;
    uint8_t val;
    bool    is_ext; // false = normal (0x00..0x2E), true = EXT (0x2F + ext addr)
} cc1200_regpair_t;

// Apply a list of reg writes.
bool cc1200_apply_profile(cc1200_t* dev, const cc1200_regpair_t* p, size_t n);
