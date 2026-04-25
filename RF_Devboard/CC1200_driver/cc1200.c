#include "cc1200.h"

#include <string.h>
#include "pico/stdlib.h"

#define CC1200_SPI_READ   0x80
#define CC1200_SPI_BURST  0x40
#define CC1200_SPI_ADDR_M 0x3F

static inline uint8_t hdr_read(uint8_t addr, bool burst)
{
    return (uint8_t)(CC1200_SPI_READ | (burst ? CC1200_SPI_BURST : 0) | (addr & CC1200_SPI_ADDR_M));
}

static inline uint8_t hdr_write(uint8_t addr, bool burst)
{
    return (uint8_t)((burst ? CC1200_SPI_BURST : 0) | (addr & CC1200_SPI_ADDR_M));
}

cc1200_status_t cc1200_status_from_u8(uint8_t s)
{
    cc1200_status_t st = {
        .chip_ready = ((s & 0x80u) == 0),
        .state      = (cc1200_state_t)((s >> 4) & 0x07u),
        .fifo_bytes = (uint8_t)(s & 0x0Fu),
        .raw        = s
    };
    return st;
}

bool cc1200_init(cc1200_t* dev, const cc1200_hal_t* hal_cfg)
{
    if (!dev || !hal_cfg) return false;
    memset(dev, 0, sizeof(*dev));
    dev->hal = *hal_cfg;
    return cc1200_hal_init(&dev->hal);
}

bool cc1200_begin(cc1200_t* dev)
{
    if (!dev) return false;

    cc1200_hal_reset_pulse(&dev->hal, 1000, 1000);

    if (!cc1200_hal_select(&dev->hal)) return false;

    (void)cc1200_hal_xfer_u8(&dev->hal, (uint8_t)CC1200_CMD_SRES);

    const absolute_time_t t0 = get_absolute_time();
    while (gpio_get(dev->hal.pin_miso) != 0)
    {
        if (absolute_time_diff_us(t0, get_absolute_time()) > (int64_t)dev->hal.so_ready_timeout_us)
        {
            cc1200_hal_deselect(&dev->hal);
            return false;
        }
        tight_loop_contents();
    }

    cc1200_hal_deselect(&dev->hal);
    return true;
}

cc1200_status_t cc1200_strobe(cc1200_t* dev, cc1200_cmd_t cmd)
{
    cc1200_status_t st = {0};
    if (!dev) return st;

    if (!cc1200_hal_select(&dev->hal)) return st;
    uint8_t status_raw = cc1200_hal_xfer_u8(&dev->hal, (uint8_t)cmd);
    cc1200_hal_deselect(&dev->hal);

    return cc1200_status_from_u8(status_raw);
}

bool cc1200_read_reg(cc1200_t* dev, uint8_t addr, uint8_t* out)
{
    if (!dev || !out) return false;
    if ((addr & 0xC0u) != 0) return false;

    if (!cc1200_hal_select(&dev->hal)) return false;

    uint8_t tx[2] = { hdr_read(addr, false), 0x00 };
    uint8_t rx[2] = { 0 };

    bool ok = cc1200_hal_xfer(&dev->hal, tx, rx, sizeof(tx));
    cc1200_hal_deselect(&dev->hal);

    if (!ok) return false;
    *out = rx[1];
    return true;
}

bool cc1200_write_reg(cc1200_t* dev, uint8_t addr, uint8_t value)
{
    if (!dev) return false;
    if ((addr & 0xC0u) != 0) return false;

    if (!cc1200_hal_select(&dev->hal)) return false;

    uint8_t tx[2] = { hdr_write(addr, false), value };
    uint8_t rx[2] = { 0 };

    bool ok = cc1200_hal_xfer(&dev->hal, tx, rx, sizeof(tx));
    cc1200_hal_deselect(&dev->hal);
    return ok;
}

bool cc1200_read_burst(cc1200_t* dev, uint8_t start_addr, uint8_t* out, size_t len)
{
    if (!dev || !out || len == 0) return false;

    if (!cc1200_hal_select(&dev->hal)) return false;

    (void)cc1200_hal_xfer_u8(&dev->hal, hdr_read(start_addr, true));
    for (size_t i = 0; i < len; i++)
        out[i] = cc1200_hal_xfer_u8(&dev->hal, 0x00);

    cc1200_hal_deselect(&dev->hal);
    return true;
}

bool cc1200_write_burst(cc1200_t* dev, uint8_t start_addr, const uint8_t* data, size_t len)
{
    if (!dev || !data || len == 0) return false;

    if (!cc1200_hal_select(&dev->hal)) return false;

    (void)cc1200_hal_xfer_u8(&dev->hal, hdr_write(start_addr, true));

    const bool is_cfg_reg_space = (start_addr <= 0x2Eu);

    for (size_t i = 0; i < len; i++)
    {
        (void)cc1200_hal_xfer_u8(&dev->hal, data[i]);
        if (is_cfg_reg_space && len > 1 && (i + 1 < len))
            cc1200_hal_cfg_burst_interbyte_delay(&dev->hal);
    }

    cc1200_hal_deselect(&dev->hal);
    return true;
}

bool cc1200_read_status(cc1200_t* dev, uint8_t status_addr, uint8_t* out)
{
    return cc1200_read_status_burst(dev, status_addr, out, 1);
}

bool cc1200_read_status_burst(cc1200_t* dev, uint8_t status_start_addr, uint8_t* out, size_t len)
{
    if (!dev || !out || len == 0) return false;

    if (!cc1200_hal_select(&dev->hal)) return false;

    // Status regs are read-only; use read header.
    // Allow burst if len > 1.
    const bool burst = (len > 1);
    (void)cc1200_hal_xfer_u8(&dev->hal, hdr_read(status_start_addr, burst));

    for (size_t i = 0; i < len; i++)
        out[i] = cc1200_hal_xfer_u8(&dev->hal, 0x00);

    cc1200_hal_deselect(&dev->hal);
    return true;
}

// Extended access: [HDR 0x2F] [ext_addr] [data...]
bool cc1200_read_ext(cc1200_t* dev, uint8_t ext_addr, uint8_t* out)
{
    return cc1200_read_ext_burst(dev, ext_addr, out, 1);
}

bool cc1200_write_ext(cc1200_t* dev, uint8_t ext_addr, uint8_t value)
{
    return cc1200_write_ext_burst(dev, ext_addr, &value, 1);
}

bool cc1200_read_ext_burst(cc1200_t* dev, uint8_t ext_start_addr, uint8_t* out, size_t len)
{
    if (!dev || !out || len == 0) return false;

    if (!cc1200_hal_select(&dev->hal)) return false;

    const bool burst = (len > 1);
    (void)cc1200_hal_xfer_u8(&dev->hal, hdr_read(CC1200_ADDR_EXTENDED, burst));
    (void)cc1200_hal_xfer_u8(&dev->hal, ext_start_addr);

    for (size_t i = 0; i < len; i++)
        out[i] = cc1200_hal_xfer_u8(&dev->hal, 0x00);

    cc1200_hal_deselect(&dev->hal);
    return true;
}

bool cc1200_write_ext_burst(cc1200_t* dev, uint8_t ext_start_addr, const uint8_t* data, size_t len)
{
    if (!dev || !data || len == 0) return false;

    if (!cc1200_hal_select(&dev->hal)) return false;

    const bool burst = (len > 1);
    (void)cc1200_hal_xfer_u8(&dev->hal, hdr_write(CC1200_ADDR_EXTENDED, burst));
    (void)cc1200_hal_xfer_u8(&dev->hal, ext_start_addr);

    for (size_t i = 0; i < len; i++)
        (void)cc1200_hal_xfer_u8(&dev->hal, data[i]);

    cc1200_hal_deselect(&dev->hal);
    return true;
}

bool cc1200_fifo_read(cc1200_t* dev, uint8_t* out, size_t len)
{
    if (!dev || !out || len == 0) return false;

    if (!cc1200_hal_select(&dev->hal)) return false;

    (void)cc1200_hal_xfer_u8(&dev->hal, hdr_read(CC1200_ADDR_FIFO, true));
    for (size_t i = 0; i < len; i++)
        out[i] = cc1200_hal_xfer_u8(&dev->hal, 0x00);

    cc1200_hal_deselect(&dev->hal);
    return true;
}

bool cc1200_fifo_write(cc1200_t* dev, const uint8_t* data, size_t len)
{
    if (!dev || !data || len == 0) return false;

    if (!cc1200_hal_select(&dev->hal)) return false;

    (void)cc1200_hal_xfer_u8(&dev->hal, hdr_write(CC1200_ADDR_FIFO, true));
    for (size_t i = 0; i < len; i++)
        (void)cc1200_hal_xfer_u8(&dev->hal, data[i]);

    cc1200_hal_deselect(&dev->hal);
    return true;
}
