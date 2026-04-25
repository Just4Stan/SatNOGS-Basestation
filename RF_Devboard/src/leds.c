#include "leds.h"
#include "config.h"

#include "pico/stdlib.h"
#include "hardware/gpio.h"

void leds_init(void) {
    gpio_init(PIN_LED_BLUE);
    gpio_init(PIN_LED_GREEN);
    gpio_init(PIN_LED_RED);
    gpio_init(PIN_LED_EXTRA);

    gpio_set_dir(PIN_LED_BLUE, GPIO_OUT);
    gpio_set_dir(PIN_LED_GREEN, GPIO_OUT);
    gpio_set_dir(PIN_LED_RED, GPIO_OUT);
    gpio_set_dir(PIN_LED_EXTRA, GPIO_OUT);

    gpio_put(PIN_LED_BLUE, 0);
    gpio_put(PIN_LED_GREEN, 0);
    gpio_put(PIN_LED_RED, 0);
    gpio_put(PIN_LED_EXTRA, 0);
}

void leds_set_rgb(bool r, bool g, bool b) {
    gpio_put(PIN_LED_RED,   r ? 1 : 0);
    gpio_put(PIN_LED_GREEN, g ? 1 : 0);
    gpio_put(PIN_LED_BLUE,  b ? 1 : 0);
}

void leds_set_state(led_state_t st) {
    switch (st) {
        case LEDSTATE_BOOT:      leds_set_rgb(1, 1, 1); break; // White
        case LEDSTATE_USB_READY: leds_set_rgb(0, 0, 1); break; // Blue
        case LEDSTATE_OK:        leds_set_rgb(0, 1, 0); break; // Green
        case LEDSTATE_WARN:      leds_set_rgb(1, 1, 0); break; // Yellow
        case LEDSTATE_ERROR:     leds_set_rgb(1, 0, 0); break; // Red
        default:                 leds_set_rgb(0, 0, 0); break;
    }
}

void leds_activity_blink(uint32_t ms_on) {
    gpio_put(PIN_LED_EXTRA, 1);
    sleep_ms(ms_on);
    gpio_put(PIN_LED_EXTRA, 0);
}

void leds_startup_sequence(void) {
    sleep_ms(700);
    leds_set_rgb(1,0,0); sleep_ms(700);
    leds_set_rgb(0,1,0); sleep_ms(700);
    leds_set_rgb(0,0,1); sleep_ms(700);
    leds_set_rgb(0,0,0); sleep_ms(300);
}

const char* leds_state_name(led_state_t st) {
    switch (st) {
        case LEDSTATE_BOOT:      return "BOOT";
        case LEDSTATE_USB_READY: return "USB_READY";
        case LEDSTATE_OK:        return "OK";
        case LEDSTATE_WARN:      return "WARN";
        case LEDSTATE_ERROR:     return "ERROR";
        default:                 return "UNKNOWN";
    }
}
