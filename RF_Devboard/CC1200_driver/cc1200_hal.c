#include "cc1200_hal.h"

#include "pico/stdlib.h"
#include "hardware/clocks.h"

#ifndef UINT32_MAX
#define UINT32_MAX 0xFFFFFFFFu
#endif

static inline void csn_low(const cc1200_hal_t* hal)  { gpio_put(hal->pin_csn, 0); }
static inline void csn_high(const cc1200_hal_t* hal) { gpio_put(hal->pin_csn, 1); }

bool cc1200_hal_init(cc1200_hal_t* hal)
{
    if (!hal || !hal->spi) return false;

    spi_init(hal->spi, hal->baud_hz);

    // Make SPI format explicit (CC120x typically Mode 0).
    spi_set_format(hal->spi, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);

    gpio_set_function(hal->pin_sck,  GPIO_FUNC_SPI);
    gpio_set_function(hal->pin_mosi, GPIO_FUNC_SPI);
    gpio_set_function(hal->pin_miso, GPIO_FUNC_SPI);

    gpio_init(hal->pin_csn);
    gpio_set_dir(hal->pin_csn, GPIO_OUT);
    csn_high(hal);

    if (hal->pin_reset != UINT32_MAX)
    {
        gpio_init(hal->pin_reset);
        gpio_set_dir(hal->pin_reset, GPIO_OUT);
        gpio_put(hal->pin_reset, 1);
    }

    gpio_pull_up(hal->pin_miso);

    // ~100 ns delay in cycles
    const uint32_t sys_hz = clock_get_hz(clk_sys);
    uint32_t cycles = sys_hz / 10000000u;
    if (cycles == 0) cycles = 1;
    hal->cfg_burst_interbyte_delay_cycles = cycles;

    return true;
}

bool cc1200_hal_select(cc1200_hal_t* hal)
{
    if (!hal) return false;

    csn_low(hal);

    // Wait for SO low
    const absolute_time_t t0 = get_absolute_time();
    while (gpio_get(hal->pin_miso) != 0)
    {
        if (absolute_time_diff_us(t0, get_absolute_time()) > (int64_t)hal->so_ready_timeout_us)
        {
            csn_high(hal);
            return false;
        }
        tight_loop_contents();
    }
    return true;
}

void cc1200_hal_deselect(cc1200_hal_t* hal)
{
    if (!hal) return;
    csn_high(hal);
}

void cc1200_hal_reset_pulse(cc1200_hal_t* hal, uint32_t low_time_us, uint32_t post_time_us)
{
    if (!hal || hal->pin_reset == UINT32_MAX) return;

    gpio_put(hal->pin_reset, 0);
    sleep_us(low_time_us);

    gpio_put(hal->pin_reset, 1);
    sleep_us(post_time_us);
}

bool cc1200_hal_xfer(cc1200_hal_t* hal, const uint8_t* tx, uint8_t* rx, size_t len)
{
    if (!hal || !hal->spi || len == 0) return false;

    // Pico SDK wants non-NULL buffers; support NULL by doing byte-wise.
    if (!tx || !rx)
    {
        for (size_t i = 0; i < len; i++)
        {
            uint8_t t = tx ? tx[i] : 0x00;
            uint8_t r = 0x00;
            spi_write_read_blocking(hal->spi, &t, &r, 1);
            if (rx) rx[i] = r;
        }
        return true;
    }

    spi_write_read_blocking(hal->spi, tx, rx, len);
    return true;
}
