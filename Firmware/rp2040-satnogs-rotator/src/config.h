#pragma once

// Mechanical/electrical calibration values.
// Motor: FAPG36-555-EN, 24V, 6mm shaft, 16 RPM
// Internal gearbox: 516:1 | External: AZ 40/15, EL 55/40

namespace config {

// Homing via endstop switches (repeatable absolute reference at startup).
constexpr bool kUseEndstopHoming = false; // Set true once endstops are wired

// ALERT pins float high without 24V motor supply, triggering false faults.
constexpr bool kIgnoreDriverFaults = true; // Set false for production

// Encoder ticks per degree of axis output rotation.
constexpr float kAzTicksPerDegree = 244.6f; // 64 ticks/rev × 516:1 gearbox × 40/15 external
constexpr float kElTicksPerDegree = 126.1f; // 64 ticks/rev × 516:1 gearbox × 55/40 external

// Direction conventions: if an axis moves the wrong way, flip the sign.
constexpr bool kAzInvert = false;
constexpr bool kElInvert = false;

// Endstop polarity: true means GPIO reads 0 when switch is pressed.
constexpr bool kEndstopActiveLow = true;

// Soft limits (degrees).
// AZ: no slip ring, ±360° from north (720° total). Rewind (PARK) between passes.
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
constexpr float kIntegralMaxDeg = 5.0f;  // anti-windup clamp (degrees·seconds)
constexpr float kDtSeconds = 0.01f;      // 100 Hz tick rate
constexpr float kDFilterAlpha = 0.15f;   // D-term low-pass filter (0..1, lower = more filtering)
constexpr float kDutyFilterAlpha = 0.05f; // duty output smoothing (0..1, lower = smoother)

// PWM
constexpr uint32_t kPwmHz = 20000;
constexpr float kMaxDuty = 0.35f; // 0..1 (capped for smooth motion)
constexpr float kMinDuty = 0.15f; // 0..1 (kept for homing reference; not used in PID)

// Homing
constexpr float kHomeDuty = 0.25f;
constexpr uint32_t kHomeTimeoutMs = 20000;
// When only a single "home" switch exists (not a min/max limit pair), searching both directions is safer.
constexpr uint32_t kHomePhaseTimeoutMs = kHomeTimeoutMs / 2;
// First search direction for homing: -1 or +1.
constexpr int8_t kAzHomeFirstDir = -1;
constexpr int8_t kElHomeFirstDir = -1;

} // namespace config
