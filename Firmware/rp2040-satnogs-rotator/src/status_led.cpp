#include "status_led.h"

#include <Adafruit_NeoPixel.h>
#include "pins.h"
#include "pico/stdlib.h"

static Adafruit_NeoPixel pixel(1, pins::kNeoPixel, NEO_GRB + NEO_KHZ800);
static LedState g_state = LedState::kBoot;
static uint32_t g_last_toggle_ms = 0;
static bool g_blink_on = true;

// Brightness: 40/255 is plenty for a status indicator (saves power, not blinding)
static constexpr uint8_t BR = 40;

static uint32_t color_for_state(LedState s, bool blink_on) {
    switch (s) {
        case LedState::kBoot:
            // Breathing white: use blink_on for simple pulse
            return blink_on ? pixel.Color(BR, BR, BR) : pixel.Color(BR/4, BR/4, BR/4);
        case LedState::kHoming:
            return blink_on ? pixel.Color(BR, BR, 0) : pixel.Color(0, 0, 0);
        case LedState::kIdle:
            return pixel.Color(0, BR, 0);
        case LedState::kTracking:
            return pixel.Color(0, 0, BR);
        case LedState::kOnTarget:
            return pixel.Color(0, BR, BR);
        case LedState::kFault:
            return pixel.Color(BR, 0, 0);
        default:
            return pixel.Color(0, 0, 0);
    }
}

void status_led_init() {
    pixel.begin();
    pixel.setBrightness(255);  // brightness baked into color values
    pixel.setPixelColor(0, pixel.Color(BR, BR, BR));
    pixel.show();
    g_last_toggle_ms = millis();
}

void status_led_set(LedState state) {
    g_state = state;
}

void status_led_update() {
    uint32_t now = millis();

    // Blink interval depends on state
    uint32_t interval;
    switch (g_state) {
        case LedState::kBoot:    interval = 800;  break;  // slow pulse
        case LedState::kHoming:  interval = 500;  break;  // 1 Hz blink
        case LedState::kFault:   interval = 250;  break;  // fast blink
        default:                 interval = 0;     break;  // solid
    }

    if (interval > 0 && (now - g_last_toggle_ms) >= interval) {
        g_blink_on = !g_blink_on;
        g_last_toggle_ms = now;
    } else if (interval == 0) {
        g_blink_on = true;
    }

    pixel.setPixelColor(0, color_for_state(g_state, g_blink_on));
    pixel.show();
}
