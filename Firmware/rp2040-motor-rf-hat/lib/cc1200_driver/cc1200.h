#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "cc1200_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

// ---- CC1200 SPI constants ----
// Standard FIFO address
#define CC1200_ADDR_FIFO          0x3F
// Extended address command
#define CC1200_ADDR_EXTENDED      0x2F

// Strobes (CC1101-compatible, also used by CC120x family)
typedef enum
{
    CC1200_CMD_SRES   = 0x30,
    CC1200_CMD_SFSTXON= 0x31,
    CC1200_CMD_SXOFF  = 0x32,
    CC1200_CMD_SCAL   = 0x33,
    CC1200_CMD_SRX    = 0x34,
    CC1200_CMD_STX    = 0x35,
    CC1200_CMD_SIDLE  = 0x36,
    CC1200_CMD_SAFC   = 0x37,
    CC1200_CMD_SWOR   = 0x38,
    CC1200_CMD_SPWD   = 0x39,
    CC1200_CMD_SFRX   = 0x3A,
    CC1200_CMD_SFTX   = 0x3B,
    CC1200_CMD_SWORRST= 0x3C,
    CC1200_CMD_SNOP   = 0x3D
} cc1200_cmd_t;

typedef enum
{
    CC1200_STATE_IDLE         = 0,
    CC1200_STATE_RX           = 1,
    CC1200_STATE_TX           = 2,
    CC1200_STATE_FSTXON       = 3,
    CC1200_STATE_CALIBRATE    = 4,
    CC1200_STATE_SETTLING     = 5,
    CC1200_STATE_RX_FIFO_ERR  = 6,
    CC1200_STATE_TX_FIFO_ERR  = 7
} cc1200_state_t;

typedef struct
{
    bool           chip_ready; // true when CHIP_RDYn bit indicates ready
    cc1200_state_t state;
    uint8_t        fifo_bytes; // low nibble from status (meaning depends on config; still useful)
    uint8_t        raw;
} cc1200_status_t;

typedef struct
{
    cc1200_hal_t hal;
} cc1200_t;

// ---- Public API ----

bool cc1200_init(cc1200_t* dev, const cc1200_hal_t* hal_cfg);

/**
 * Hard reset (optional) + SRES + basic sanity check.
 * Returns false if SO-ready handshake fails.
 */
bool cc1200_begin(cc1200_t* dev);

/**
 * Send a command strobe and return the status byte parsed.
 */
cc1200_status_t cc1200_strobe(cc1200_t* dev, cc1200_cmd_t cmd);

// Standard register access (0x00..0x2E)
bool cc1200_read_reg(cc1200_t* dev, uint8_t addr, uint8_t* out);
bool cc1200_write_reg(cc1200_t* dev, uint8_t addr, uint8_t value);
bool cc1200_read_burst(cc1200_t* dev, uint8_t start_addr, uint8_t* out, size_t len);
bool cc1200_write_burst(cc1200_t* dev, uint8_t start_addr, const uint8_t* data, size_t len);

// Read CC1200 "status registers" (different access mechanism than extended regs)
bool cc1200_read_status(cc1200_t* dev, uint8_t status_addr, uint8_t* out);
bool cc1200_read_status_burst(cc1200_t* dev, uint8_t status_start_addr, uint8_t* out, size_t len);

// Extended register access (via 0x2F + ext_addr)
bool cc1200_read_ext(cc1200_t* dev, uint8_t ext_addr, uint8_t* out);
bool cc1200_write_ext(cc1200_t* dev, uint8_t ext_addr, uint8_t value);
bool cc1200_read_ext_burst(cc1200_t* dev, uint8_t ext_start_addr, uint8_t* out, size_t len);
bool cc1200_write_ext_burst(cc1200_t* dev, uint8_t ext_start_addr, const uint8_t* data, size_t len);

// FIFO access (0x3F)
bool cc1200_fifo_read(cc1200_t* dev, uint8_t* out, size_t len);
bool cc1200_fifo_write(cc1200_t* dev, const uint8_t* data, size_t len);

/**
 * Helper: decode status byte.
 */
cc1200_status_t cc1200_status_from_u8(uint8_t status);

#ifdef __cplusplus
}
#endif
