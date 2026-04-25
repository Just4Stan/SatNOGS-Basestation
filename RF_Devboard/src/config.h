#pragma once
#include <stdint.h>

// =====================
// Board / MCU Pin Config
// =====================

// RGB LED pins
#define PIN_LED_BLUE     12
#define PIN_LED_GREEN    13
#define PIN_LED_RED      14

#define PIN_LED_EXTRA    15

//CC1200
#define CC1200_SPI_INST       spi0
#define CC1200_SPI_BAUD_HZ    (5 * 1000 * 1000) // start safe at 5MHz

#define CC1200_PIN_SCK        18
#define CC1200_PIN_MOSI       19
#define CC1200_PIN_MISO       20
#define CC1200_PIN_CSN        21
#define CC1200_PIN_GPIO0      22
#define CC1200_PIN_GPIO2      23
#define CC1200_PIN_GPIO3      24
#define CC1200_PIN_RESET      25

#define CC1200_SO_READY_TIMEOUT_US  5000
