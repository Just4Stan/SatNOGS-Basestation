#pragma once
#include <stdint.h>

// =============================================================================
// MOTOR_RF_HAT Pin Configuration — Combined motor controller + RF HAT
// =============================================================================
//
// Architecture:
//   Pi 3A+ (UART) --> Pico --> SPI1 --> CC1200 UHF (432 MHz)
//                          --> SPI0 --> CC1200 VHF (144 MHz) + ADXL345 IMU
//                          --> A4950 x2 (AZ + EL motor drivers)
//                          --> Quadrature encoders x2

// ---- UART to Raspberry Pi (UART0) ----
#define PIN_UART_TX          0   // GP0 -> Pi GPIO15 (RXD)
#define PIN_UART_RX          1   // GP1 -> Pi GPIO14 (TXD)
#define UART_BAUD           115200

// ---- AZ Motor (A4950, 2-pin PWM mode) ----
#define PIN_AZ_IN1           2   // GP2 — PWM for forward
#define PIN_AZ_IN2           3   // GP3 — PWM for reverse

// ---- WS2812B NeoPixel status LEDs ----
#define PIN_NEOPIXEL         4   // GP4 — WS2812B data-in (chain of 4)
#define NEOPIXEL_COUNT       4

// ---- Passive buzzer (on Pico, via switching transistor) ----
#define PIN_BUZZER           5   // GP5 — PWM to NPN/MOSFET gate driving buzzer

// ---- UHF CC1200 (432 MHz) on SPI1 ----
#define UHF_SPI_INST        spi1
#define UHF_SPI_BAUD_HZ     (5 * 1000 * 1000)
#define UHF_PIN_GPIO0         7  // GP7 — UHF CC1200 GPIO0 (packet sync)
#define UHF_PIN_RESET         8  // GP8
#define UHF_PIN_SCK          10  // GP10
#define UHF_PIN_MOSI         11  // GP11
#define UHF_PIN_MISO         12  // GP12
#define UHF_PIN_CSN          13  // GP13

// ---- AZ Quadrature Encoder ----
#define PIN_AZ_ENC_A         14  // GP14
#define PIN_AZ_ENC_B         15  // GP15

// ---- VHF CC1200 (144 MHz) on SPI0 (shared with IMU) ----
#define VHF_SPI_INST        spi0
#define VHF_SPI_BAUD_HZ     (5 * 1000 * 1000)
#define VHF_PIN_MISO         16  // GP16 (shared with IMU)
#define VHF_PIN_CSN          17  // GP17
#define VHF_PIN_SCK          18  // GP18 (shared with IMU)
#define VHF_PIN_MOSI         19  // GP19 (shared with IMU)
#define VHF_PIN_RESET        20  // GP20

// ---- EL Quadrature Encoder ----
#define PIN_EL_ENC_A         21  // GP21
#define PIN_EL_ENC_B         22  // GP22

// ---- ADXL345 IMU on SPI0 (shared bus with VHF CC1200, mode switching) ----
#define PIN_IMU_CS           26  // GP26
// IMU uses VHF_PIN_SCK (GP18), VHF_PIN_MOSI (GP19), VHF_PIN_MISO (GP16)

// ---- EL Motor (A4950, 2-pin PWM mode) ----
#define PIN_EL_IN1           27  // GP27 — PWM for forward
#define PIN_EL_IN2           28  // GP28 — PWM for reverse

// ---- Common ----
#define CC1200_SO_READY_TIMEOUT_US  5000

// ---- Onboard LED (Pico) ----
// PIN_LED already defined by Arduino-Pico variant (pin 25)

// ---- Radio selection ----
#define RADIO_UHF   0
#define RADIO_VHF   1
#define RADIO_COUNT 2

// =============================================================================
// Motor/Rotator Configuration — from rp2040-satnogs-rotator config.h
// =============================================================================
// Motor: FAPG36-555-EN, 24V, 6mm shaft, 16 RPM
// Internal gearbox: 516:1 | External: AZ 40/15, EL 55/40

#ifdef __cplusplus
namespace motor_config {

// Homing via endstop switches (repeatable absolute reference at startup).
constexpr bool kUseEndstopHoming = false; // Set true once endstops are wired

// ALERT pins — A4950 has no ALERT output, so always ignore.
constexpr bool kIgnoreDriverFaults = true;

// Encoder ticks per degree of axis output rotation.
constexpr float kAzTicksPerDegree = 244.6f; // 64 ticks/rev x 516:1 gearbox x 40/15 external
constexpr float kElTicksPerDegree = 126.1f; // 64 ticks/rev x 516:1 gearbox x 55/40 external

// Direction conventions: if an axis moves the wrong way, flip the sign.
constexpr bool kAzInvert = false;
constexpr bool kElInvert = true;

// Endstop polarity: true means GPIO reads 0 when switch is pressed.
constexpr bool kEndstopActiveLow = true;

// Soft limits (degrees).
// AZ: no slip ring, +/-360 from north (720 total). Rewind (PARK) between passes.
constexpr float kAzMinDeg = -360.0f;
constexpr float kAzMaxDeg = 360.0f;
constexpr float kElMinDeg = 0.0f;
constexpr float kElMaxDeg = 180.0f;

// Park position (degrees).
constexpr float kParkAzDeg = 0.0f;
constexpr float kParkElDeg = 0.0f;

// PID control loop
constexpr float kDeadbandDeg = 0.05f;    // tight deadband for smooth continuous tracking
constexpr float kKp = 0.15f;
constexpr float kKi = 0.03f;
constexpr float kKd = 0.02f;
constexpr float kIntegralMaxDeg = 5.0f;  // anti-windup clamp (degrees*seconds)
constexpr float kDtSeconds = 0.01f;      // 100 Hz tick rate
constexpr float kDFilterAlpha = 0.15f;   // D-term low-pass filter (0..1, lower = more filtering)
constexpr float kDutyFilterAlpha = 0.15f; // duty output smoothing (0..1, lower = smoother)

// PWM
constexpr uint32_t kPwmHz = 20000;
constexpr float kMaxDuty = 0.95f; // 0..1 (linear up to 95%; 100% can trip driver overcurrent on stall)
constexpr float kMinDuty = 0.15f; // 0..1 (kept for homing reference; not used in PID)

// Homing
constexpr float kHomeDuty = 0.25f;
constexpr uint32_t kHomeTimeoutMs = 20000;
constexpr uint32_t kHomePhaseTimeoutMs = kHomeTimeoutMs / 2;
constexpr int8_t kAzHomeFirstDir = -1;
constexpr int8_t kElHomeFirstDir = -1;

} // namespace motor_config
#endif
