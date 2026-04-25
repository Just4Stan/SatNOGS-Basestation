#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "hardware/spi.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    spi_inst_t* spi;
    uint32_t    baud_hz;

    // RP2040 pins
    uint        pin_sck;
    uint        pin_mosi;
    uint        pin_miso;   // CC1200 SO
    uint        pin_csn;    // CC1200 CSn (active-low)
    uint        pin_reset;  // CC1200 RESET_N (active-low). Use UINT32_MAX if not connected.

    // Timing
    uint32_t    so_ready_timeout_us; // wait for SO low after CSn low

    // CC120x requirement: >=100 ns delay between consecutive data bytes during
    // burst WRITE access to configuration registers.
    uint32_t    cfg_burst_interbyte_delay_cycles;
} cc1200_hal_t;

bool cc1200_hal_init(cc1200_hal_t* hal);

bool cc1200_hal_select(cc1200_hal_t* hal);
void cc1200_hal_deselect(cc1200_hal_t* hal);

void cc1200_hal_reset_pulse(cc1200_hal_t* hal, uint32_t low_time_us, uint32_t post_time_us);

bool cc1200_hal_xfer(cc1200_hal_t* hal, const uint8_t* tx, uint8_t* rx, size_t len);

static inline uint8_t cc1200_hal_xfer_u8(cc1200_hal_t* hal, uint8_t v)
{
    uint8_t r = 0;
    (void)cc1200_hal_xfer(hal, &v, &r, 1);
    return r;
}

static inline void cc1200_hal_cfg_burst_interbyte_delay(const cc1200_hal_t* hal)
{
    uint32_t cycles = hal ? hal->cfg_burst_interbyte_delay_cycles : 0;
    while (cycles--)
        __asm volatile("nop");
}

#ifdef __cplusplus
}
#endif
