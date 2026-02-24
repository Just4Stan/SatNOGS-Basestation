#pragma once

// Pin mapping extracted from `MotorPCB/motorPCB.net` (U5 = PICO module).
// All numbers are RP2040 GPIO numbers (GPx).

#include "pico/types.h"

namespace pins {

// Azimuth motor driver
constexpr uint kAzIn1 = 15;   // /Azimuth/AZ_EN/IN1
constexpr uint kAzIn2 = 14;   // /Azimuth/AZ_PH/IN2
constexpr uint kAzPwm = 22;   // /Azimuth/AZ_PWM
constexpr uint kAzAlert = 19; // /Azimuth/AZ_ALERT

// Elevation motor driver (IN1/IN2 swapped in firmware to correct motor direction)
constexpr uint kElIn1 = 28;   // /Elevation/EL_PH/IN2 (was IN1, swapped for correct UP direction)
constexpr uint kElIn2 = 27;   // /Elevation/EL_EN/IN1 (was IN2, swapped for correct UP direction)
constexpr uint kElPwm = 21;   // /Elevation/EL_PWM
constexpr uint kElAlert = 20; // /Elevation/EL_ALERT

// Quadrature encoders
constexpr uint kAzEncA = 11; // /Encoders/AZ_ENC_B (swapped for positive count direction)
constexpr uint kAzEncB = 10; // /Encoders/AZ_ENC_A
constexpr uint kElEncA = 12; // /Encoders/EL_ENC_A (swapped back: motor dir reversed, so encoder dir must also reverse)
constexpr uint kElEncB = 13; // /Encoders/EL_ENC_B

// Endstops
constexpr uint kAzEndstop = 8; // /Encoders/AZ_END
constexpr uint kElEndstop = 9; // /Encoders/EL_END

// SPI0 (ADXL345 accelerometer)
constexpr uint kSpiMiso = 4;  // GP4 - TP8
constexpr uint kSpiCs   = 5;  // GP5 - TP5
constexpr uint kSpiSck  = 6;  // GP6 - TP4
constexpr uint kSpiMosi = 7;  // GP7 - TP7

// Pico on-board LED (active high)
constexpr uint kLed = 25;

} // namespace pins
