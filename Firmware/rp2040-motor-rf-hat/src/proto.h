#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "cc1200.h"
#include "config.h"

#ifdef __cplusplus
extern "C" {
#endif

// Message types — compatible with CC1200RXFrontend protocol + motor extensions
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

    // Dual-radio extension: select which CC1200 to talk to
    MSG_SELECT_RADIO    = 0x40,

    // Buzzer control: trigger beep patterns
    MSG_BUZZER          = 0x50,

    // Motor control (0x60-0x6F)
    MSG_SET_POSITION    = 0x60,  // body: [az_f32_le, el_f32_le]
    MSG_GET_POSITION    = 0x61,  // rsp: [az_f32_le, el_f32_le, status_u8]
    MSG_PARK            = 0x62,
    MSG_STOP            = 0x63,
    MSG_HOME            = 0x64,  // trigger IMU homing
    MSG_GET_MOTOR_STATUS= 0x65,  // rsp: [state_u8, fault_u8, homed_u8, az_f32, el_f32, az_tgt_f32, el_tgt_f32]
    MSG_SET_GAINS       = 0x66,  // body: [kp_f32, ki_f32, kd_f32]
    MSG_ZERO_ENCODERS   = 0x67,
    MSG_IMU_READ        = 0x68,  // rsp: [x_f32, y_f32, z_f32, el_deg_f32]
    MSG_RECAL           = 0x69,  // re-read IMU, re-zero EL

    MSG_RSP_FLAG        = 0x80,

    EVT_RX_DATA         = 0xE0,
    EVT_ERROR           = 0xE1,
    EVT_POSITION        = 0xE2,  // async 10Hz: [az_f32, el_f32, status_u8]
} msg_type_t;

typedef enum {
    STATE_IDLE = 0,
    STATE_RX   = 1,
    STATE_TX   = 2,
} proto_state_t;

// Write function callback — allows routing responses to UART or USB
typedef void (*proto_write_fn_t)(const uint8_t* data, size_t len);

// Initialize protocol with array of radios
void proto_init(cc1200_t radios[], uint8_t radio_count);

// Set the write function for a given transport channel
// Channel 0 = UART (default), Channel 1 = USB
void proto_set_write_fn(uint8_t channel, proto_write_fn_t fn);

// Call frequently from main loop — reads UART, handles commands, streams RX
void proto_poll(void);

// Feed raw bytes from an external source (e.g. USB Serial)
// Responses will be routed via channel 1 write function
void proto_feed_bytes(const uint8_t* data, size_t len, uint8_t channel);

// State accessors for status LED
bool proto_is_streaming(void);
uint8_t proto_active_radio(void);

// Buzzer callback — set by main.cpp, called by proto when MSG_BUZZER arrives
typedef void (*buzzer_cb_t)(uint8_t pattern);
void proto_set_buzzer_cb(buzzer_cb_t cb);

// Motor/IMU callback types — set by main.cpp for motor command handling
typedef void (*motor_set_position_cb_t)(float az_deg, float el_deg);
typedef void (*motor_get_position_cb_t)(float* az_deg, float* el_deg, uint8_t* status);
typedef void (*motor_park_cb_t)(void);
typedef void (*motor_stop_cb_t)(void);
typedef void (*motor_home_cb_t)(void);  // trigger IMU re-homing
typedef void (*motor_get_status_cb_t)(uint8_t* buf, uint16_t* len);
typedef void (*motor_set_gains_cb_t)(float kp, float ki, float kd);
typedef void (*motor_zero_encoders_cb_t)(void);
typedef void (*imu_read_cb_t)(uint8_t* buf, uint16_t* len);
typedef void (*motor_recal_cb_t)(void);  // re-read IMU, re-zero EL

void proto_set_motor_set_position_cb(motor_set_position_cb_t cb);
void proto_set_motor_get_position_cb(motor_get_position_cb_t cb);
void proto_set_motor_park_cb(motor_park_cb_t cb);
void proto_set_motor_stop_cb(motor_stop_cb_t cb);
void proto_set_motor_home_cb(motor_home_cb_t cb);
void proto_set_motor_get_status_cb(motor_get_status_cb_t cb);
void proto_set_motor_set_gains_cb(motor_set_gains_cb_t cb);
void proto_set_motor_zero_encoders_cb(motor_zero_encoders_cb_t cb);
void proto_set_imu_read_cb(imu_read_cb_t cb);
void proto_set_motor_recal_cb(motor_recal_cb_t cb);

// Send EVT_POSITION event on UART channel (called from main loop at 10 Hz)
void proto_send_position_event(float az_deg, float el_deg, uint8_t status);

// Enable/disable position event streaming
void proto_set_position_streaming(bool enable);
bool proto_is_position_streaming(void);

#ifdef __cplusplus
}
#endif
