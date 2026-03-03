#pragma once

#include <cstdint>

// WS2812B NeoPixel status LED for the rotator Pico.
// Colors indicate the current rotator state at a glance.

enum class LedState : uint8_t {
    kBoot,      // white pulse — initializing
    kHoming,    // yellow blink 1 Hz — homing in progress
    kIdle,      // solid green — ready, waiting for commands
    kTracking,  // solid blue — moving to target
    kOnTarget,  // solid cyan — within deadband of target
    kFault,     // solid red — motor fault or error
};

void status_led_init();
void status_led_set(LedState state);
void status_led_update();  // call from main loop
