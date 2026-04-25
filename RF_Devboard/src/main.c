// src/main.c
#include "pico/stdlib.h"
#include "pico/stdio_usb.h"
#include "hardware/gpio.h"

#include "config.h"
#include "leds.h"
#include "usb_proto.h"
#include "cc1200.h"
#include "cc1200_profile.h"

// CC1200 ext regs (SWRU346B)
#define CC1200_EXTADDR_MARCSTATE     0x73
#define CC1200_EXTADDR_PARTNUMBER    0x8F
#define CC1200_EXTADDR_PARTVERSION   0x90

static void heartbeat_update(bool *hb_state, absolute_time_t *t_next, uint32_t period_ms) {
    if (absolute_time_diff_us(get_absolute_time(), *t_next) <= 0) {
        *hb_state = !(*hb_state);
        gpio_put(PIN_LED_EXTRA, *hb_state ? 1 : 0);
        *t_next = delayed_by_ms(*t_next, period_ms);
    }
}

// Optional hooks: keep them minimal and NEVER printf here
static void on_usb_activity(void) {
    // could implement a short LED pulse if you want (but keep it simple for now)
}

static void on_error(void) {
    leds_set_state(LEDSTATE_ERROR);
}

static void rgb_from_chip_ok(bool cc_ok) {
    leds_set_state(cc_ok ? LEDSTATE_OK : LEDSTATE_ERROR);
}

int main(void) {
    // IMPORTANT: stdio is still used for USB CDC, but we must NOT printf anywhere
    stdio_init_all();
    stdio_set_translate_crlf(&stdio_usb, false);  // binary protocol: 0x0a must not become 0x0d 0x0a

    leds_init();
    leds_set_state(LEDSTATE_BOOT);

    // Heartbeat LED (direct GPIO control)
    gpio_put(PIN_LED_EXTRA, 0);

    // CC1200 HAL config from config.h
    cc1200_hal_t hal_cfg = {
        .spi = CC1200_SPI_INST,
        .baud_hz = CC1200_SPI_BAUD_HZ,

        .pin_sck = CC1200_PIN_SCK,
        .pin_mosi = CC1200_PIN_MOSI,
        .pin_miso = CC1200_PIN_MISO,
        .pin_csn  = CC1200_PIN_CSN,
        .pin_reset = CC1200_PIN_RESET,

        .so_ready_timeout_us = CC1200_SO_READY_TIMEOUT_US
    };

    cc1200_t radio;
    bool ok = cc1200_init(&radio, &hal_cfg);
    if (!ok) {
        leds_set_state(LEDSTATE_ERROR);
        while (true) tight_loop_contents();
    }

    ok = cc1200_begin(&radio);
    if (!ok) {
        leds_set_state(LEDSTATE_ERROR);
        while (true) {
            // keep heartbeat so you know firmware is alive
            static bool hb = false;
            static absolute_time_t t_hb;
            static bool init = false;
            if (!init) { t_hb = delayed_by_ms(get_absolute_time(), 500); init = true; }
            heartbeat_update(&hb, &t_hb, 500);
            sleep_ms(2);
        }
    }

    // Quick identity check to set LED state (no printing!)
    uint8_t part = 0, ver = 0, marc = 0;
    bool ok_part = cc1200_read_ext(&radio, CC1200_EXTADDR_PARTNUMBER, &part);
    bool ok_ver  = cc1200_read_ext(&radio, CC1200_EXTADDR_PARTVERSION, &ver);
    bool ok_marc = cc1200_read_ext(&radio, CC1200_EXTADDR_MARCSTATE, &marc);

    bool cc_ok = ok_part && ok_ver && ok_marc && (part == 0x20);
    rgb_from_chip_ok(cc_ok);

    // Start USB protocol
    usb_proto_hooks_t hooks = {
        .on_usb_activity = on_usb_activity,
        .on_error = on_error
    };
    usb_proto_init(&radio, &hooks);

    // Heartbeat: 1 Hz toggle
    bool hb = false;
    absolute_time_t t_hb = delayed_by_ms(get_absolute_time(), 500);

    // Main loop: protocol + heartbeat only
    while (true) {
        heartbeat_update(&hb, &t_hb, 500);
        usb_proto_poll();
        sleep_ms(2);
    }
}
