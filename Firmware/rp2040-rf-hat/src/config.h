#pragma once
#include <stdint.h>

// =============================================================================
// RF HAT Pin Configuration — matches rf_hat_circuit.py
// =============================================================================
//
// Architecture:
//   Pi 3A+ (UART) --> Pico --> SPI1 --> CC1200 UHF (432 MHz)
//                          --> SPI0 --> CC1200 VHF (144 MHz)

// ---- UART to Raspberry Pi (UART0) ----
#define PIN_UART_TX          0   // GP0 → Pi GPIO15 (RXD)
#define PIN_UART_RX          1   // GP1 → Pi GPIO14 (TXD)
#define UART_BAUD           115200

// ---- UHF CC1200 (432 MHz) on SPI1 ----
#define UHF_SPI_INST        spi1
#define UHF_SPI_BAUD_HZ     (5 * 1000 * 1000)
#define UHF_PIN_SCK          10  // GP10
#define UHF_PIN_MOSI         11  // GP11
#define UHF_PIN_MISO         12  // GP12
#define UHF_PIN_CSN          13  // GP13
#define UHF_PIN_RESET         8  // GP8
#define UHF_PIN_GPIO0         7  // GP7
#define UHF_PIN_GPIO2         9  // GP9
#define UHF_PIN_GPIO3         6  // GP6

// ---- VHF CC1200 (144 MHz) on SPI0 ----
#define VHF_SPI_INST        spi0
#define VHF_SPI_BAUD_HZ     (5 * 1000 * 1000)
#define VHF_PIN_SCK          18  // GP18
#define VHF_PIN_MOSI         19  // GP19
#define VHF_PIN_MISO         16  // GP16
#define VHF_PIN_CSN          17  // GP17
#define VHF_PIN_RESET        20  // GP20
#define VHF_PIN_GPIO0        22  // GP22
#define VHF_PIN_GPIO2        21  // GP21
#define VHF_PIN_GPIO3        26  // GP26

// ---- Common ----
#define CC1200_SO_READY_TIMEOUT_US  5000

// ---- Onboard LED (Pico) ----
// PIN_LED already defined by Arduino-Pico variant (pin 25)

// ---- WS2812B NeoPixel status LEDs ----
#define PIN_NEOPIXEL         5   // GP5 — WS2812B data-in (chain of 4)
#define NEOPIXEL_COUNT       4   // 4 LEDs on GP5 (driven as 1 for now)

// ---- Passive buzzer (on Pico, via switching transistor) ----
#define PIN_BUZZER           4   // GP4 — PWM to NPN/MOSFET gate driving buzzer

// ---- Radio selection ----
#define RADIO_UHF   0
#define RADIO_VHF   1
#define RADIO_COUNT 2
