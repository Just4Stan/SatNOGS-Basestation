#pragma once
#include <stdbool.h>
#include <stdint.h>

typedef enum {
    LEDSTATE_BOOT = 0,
    LEDSTATE_USB_READY,
    LEDSTATE_OK,
    LEDSTATE_WARN,
    LEDSTATE_ERROR
} led_state_t;

void leds_init(void);

void leds_set_rgb(bool r, bool g, bool b);
void leds_set_state(led_state_t st);

void leds_activity_blink(uint32_t ms_on);
void leds_startup_sequence(void);

const char* leds_state_name(led_state_t st);
