#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "cc1200.h"

// Message types
typedef enum {
    MSG_PING            = 0x01,
    MSG_GET_INFO        = 0x02,
    MSG_SET_STATE       = 0x03,
    MSG_SET_STREAMING   = 0x04,
    MSG_GET_METRICS     = 0x05,

    MSG_READ_REG        = 0x10,
    MSG_WRITE_REG       = 0x11,
    MSG_READ_EXT        = 0x12,
    MSG_WRITE_EXT       = 0x13,

    MSG_RX_READ         = 0x20,
    MSG_STROBE          = 0x21,

    MSG_TX_WRITE        = 0x22,
    MSG_TX_SEND         = 0x23,
    MSG_TX_FLUSH        = 0x24,

    MSG_PROFILE_CLEAR   = 0x30,
    MSG_PROFILE_BEGIN   = 0x31,
    MSG_PROFILE_CHUNK   = 0x32,
    MSG_PROFILE_APPLY   = 0x33,

    MSG_SET_FREQ_WORD   = 0x34,

    MSG_RSP_FLAG        = 0x80,

    EVT_RX_DATA         = 0xE0,
    EVT_ERROR           = 0xE1,
} msg_type_t;

typedef enum {
    STATE_IDLE = 0,
    STATE_RX   = 1,
    STATE_TX   = 2,
} proto_state_t;

typedef struct {
    void (*on_usb_activity)(void);
    void (*on_error)(void);
} usb_proto_hooks_t;

void usb_proto_init(cc1200_t* radio, const usb_proto_hooks_t* hooks);
void usb_proto_poll(void);