#include "rotator.h"

#include <algorithm>
#include <cmath>
#include <cstdio>

#include "config.h"
#include "hardware/gpio.h"
#include "pico/stdlib.h"

// Use motor_config namespace from merged config.h (no pins.h dependency)
namespace config = motor_config;

namespace {

float clamp_deg(float value, float min_deg, float max_deg) {
  if (value < min_deg) return min_deg;
  if (value > max_deg) return max_deg;
  return value;
}

float ticks_to_deg(int32_t ticks, float ticks_per_deg, bool invert) {
  const float deg = static_cast<float>(ticks) / ticks_per_deg;
  return invert ? -deg : deg;
}

float wrap_candidate(float base_deg, int k) { return base_deg + 360.0f * static_cast<float>(k); }

float choose_wrapped_target(float commanded_deg, float current_deg, float min_deg, float max_deg) {
  // Consider AZ targets that differ by +/-360, pick the nearest that stays within soft limits.
  float best = clamp_deg(commanded_deg, min_deg, max_deg);
  float best_dist = std::abs(best - current_deg);

  for (int k : {-1, 0, 1}) {
    const float candidate = wrap_candidate(commanded_deg, k);
    if (candidate < min_deg || candidate > max_deg) continue;
    const float dist = std::abs(candidate - current_deg);
    if (dist < best_dist) {
      best = candidate;
      best_dist = dist;
    }
  }
  return best;
}

} // namespace

Rotator::Rotator(MotorPwm& az_motor, MotorPwm& el_motor, Quadrature& az_enc, Quadrature& el_enc)
    : az_motor_(az_motor),
      el_motor_(el_motor),
      az_enc_(az_enc),
      el_enc_(el_enc),
      target_az_deg_(0.0f),
      target_el_deg_(0.0f),
      az_deg_(0.0f),
      el_deg_(0.0f),
      az_homed_(false),
      el_homed_(false),
      manual_mode_(true),
      kp_(config::kKp),
      ki_(config::kKi),
      kd_(config::kKd),
      az_integral_(0.0f),
      el_integral_(0.0f),
      az_prev_error_(0.0f),
      el_prev_error_(0.0f),
      az_d_filtered_(0.0f),
      el_d_filtered_(0.0f),
      az_duty_smooth_(0.0f),
      el_duty_smooth_(0.0f) {}

void Rotator::init() {
  // No endstop or alert GPIOs on the combined board (A4950 has no ALERT).
  // Homing is handled by IMU in main.cpp setup().
  begin_homing();
}

void Rotator::tick() {
  update_measurements();
  if (manual_mode_) return; // Raw duty set directly, skip control loop.
  update_control();
  recenter_az();
}

void Rotator::set_target(float az_deg, float el_deg) {
  set_target_az(az_deg);
  set_target_el(el_deg);
}

void Rotator::set_target_az(float az_deg) {
  manual_mode_ = false;
  target_az_deg_ = choose_wrapped_target(az_deg, az_deg_, config::kAzMinDeg, config::kAzMaxDeg);
}

void Rotator::set_target_el(float el_deg) {
  manual_mode_ = false;
  target_el_deg_ = clamp_deg(el_deg, config::kElMinDeg, config::kElMaxDeg);
}

void Rotator::set_raw_az(float duty) {
  manual_mode_ = true;
  az_motor_.set(duty);
}

void Rotator::set_raw_el(float duty) {
  manual_mode_ = true;
  el_motor_.set(duty);
}

void Rotator::zero_encoders() {
  az_enc_.set_ticks(0);
  el_enc_.set_ticks(0);
}

void Rotator::stop_az() {
  az_motor_.stop();
  target_az_deg_ = az_deg_;
  az_integral_ = 0.0f;
  az_prev_error_ = 0.0f;
  az_d_filtered_ = 0.0f;
  az_duty_smooth_ = 0.0f;
}

void Rotator::stop_el() {
  el_motor_.stop();
  target_el_deg_ = el_deg_;
  el_integral_ = 0.0f;
  el_prev_error_ = 0.0f;
  el_d_filtered_ = 0.0f;
  el_duty_smooth_ = 0.0f;
}

void Rotator::stop_all() {
  stop_az();
  stop_el();
}

void Rotator::halt_motors() {
  az_motor_.stop();
  el_motor_.stop();
  az_integral_ = 0.0f;
  el_integral_ = 0.0f;
  az_prev_error_ = 0.0f;
  el_prev_error_ = 0.0f;
  az_d_filtered_ = 0.0f;
  el_d_filtered_ = 0.0f;
  az_duty_smooth_ = 0.0f;
  el_duty_smooth_ = 0.0f;
}

void Rotator::park() {
  manual_mode_ = false;
  target_az_deg_ = config::kParkAzDeg;
  target_el_deg_ = config::kParkElDeg;
}

void Rotator::reset_home() { begin_homing(); }

void Rotator::calibrate_el(float el_deg) {
  const float clamped = clamp_deg(el_deg, config::kElMinDeg, config::kElMaxDeg);
  int32_t ticks = static_cast<int32_t>(clamped * config::kElTicksPerDegree);
  if (config::kElInvert) ticks = -ticks;
  el_enc_.set_ticks(ticks);
  el_deg_ = clamped;
  target_el_deg_ = clamped;
  el_homed_ = true;
}

void Rotator::calibrate_az_zero() {
  az_enc_.set_ticks(0);
  az_deg_ = 0.0f;
  target_az_deg_ = 0.0f;
  az_homed_ = true;
}

RotatorStatus Rotator::status() const {
  RotatorStatus st;
  st.az_deg = az_deg_;
  st.el_deg = el_deg_;
  st.az_homed = az_homed_;
  st.el_homed = el_homed_;

  const bool moving = (std::abs(target_az_deg_ - az_deg_) > config::kDeadbandDeg) ||
                      (std::abs(target_el_deg_ - el_deg_) > config::kDeadbandDeg);
  st.moving = moving;
  st.status_reg = moving ? 2u : 1u;

  uint32_t err = 1u;
  if (!(az_homed_ && el_homed_)) err |= 4u;
  st.error_reg = err;

  return st;
}

void Rotator::update_measurements() {
  az_deg_ = ticks_to_deg(az_enc_.ticks(), config::kAzTicksPerDegree, config::kAzInvert);
  el_deg_ = ticks_to_deg(el_enc_.ticks(), config::kElTicksPerDegree, config::kElInvert);
}

void Rotator::update_control() {
  // Runaway detection: if actual position is far outside soft limits, emergency stop.
  constexpr float kRunawayMarginDeg = 50.0f;
  if (az_deg_ < config::kAzMinDeg - kRunawayMarginDeg ||
      az_deg_ > config::kAzMaxDeg + kRunawayMarginDeg) {
    stop_all();
    std::printf("SAFETY: AZ runaway detected (%.1f deg) -- motors stopped\n", az_deg_);
    return;
  }
  if (el_deg_ < config::kElMinDeg - kRunawayMarginDeg ||
      el_deg_ > config::kElMaxDeg + kRunawayMarginDeg) {
    stop_all();
    std::printf("SAFETY: EL runaway detected (%.1f deg) -- motors stopped\n", el_deg_);
    return;
  }

  const float az_error = target_az_deg_ - az_deg_;
  const float el_error = target_el_deg_ - el_deg_;

  float az_raw = compute_pid(az_error, &az_integral_, &az_prev_error_, &az_d_filtered_);
  float el_raw = compute_pid(el_error, &el_integral_, &el_prev_error_, &el_d_filtered_);

  // The kInvert flags flip the position reading in ticks_to_deg() so that
  // increasing degrees matches the intended physical direction.  The motor
  // output must be flipped too, otherwise the PID loop has positive feedback.
  if (config::kAzInvert) az_raw = -az_raw;
  if (config::kElInvert) el_raw = -el_raw;

  // Smooth the duty output to prevent start-stop twitching during continuous tracking.
  az_duty_smooth_ += config::kDutyFilterAlpha * (az_raw - az_duty_smooth_);
  el_duty_smooth_ += config::kDutyFilterAlpha * (el_raw - el_duty_smooth_);

  az_motor_.set(az_duty_smooth_);
  el_motor_.set(el_duty_smooth_);
}

float Rotator::compute_pid(float error_deg, float* integral, float* prev_error, float* d_filtered) {
  const float abs_error = std::abs(error_deg);

  if (abs_error <= config::kDeadbandDeg) {
    *integral = 0.0f;
    *prev_error = error_deg;
    *d_filtered = 0.0f;
    return 0.0f;
  }

  // D-term: filtered derivative
  const float raw_deriv = (error_deg - *prev_error) / config::kDtSeconds;
  *d_filtered = *d_filtered + config::kDFilterAlpha * (raw_deriv - *d_filtered);
  *prev_error = error_deg;

  // Compute provisional PID output to check for saturation.
  float duty = kp_ * error_deg + ki_ * (*integral) + kd_ * (*d_filtered);

  // Saturation-based anti-windup: only accumulate integral when output is NOT
  // saturated, or when the error would reduce the integral (opposite sign).
  const bool saturated = std::abs(duty) >= config::kMaxDuty;
  const bool error_would_increase = (error_deg > 0.0f && *integral > 0.0f) ||
                                    (error_deg < 0.0f && *integral < 0.0f);
  if (!saturated || !error_would_increase) {
    *integral += error_deg * config::kDtSeconds;
    *integral = std::clamp(*integral, -config::kIntegralMaxDeg, config::kIntegralMaxDeg);
  }

  // Recompute with updated integral.
  duty = kp_ * error_deg + ki_ * (*integral) + kd_ * (*d_filtered);

  duty = std::clamp(duty, -config::kMaxDuty, config::kMaxDuty);
  return duty;
}

void Rotator::recenter_az() {
  const float az_error = std::abs(target_az_deg_ - az_deg_);
  if (az_error > config::kDeadbandDeg) return;  // still moving

  const float abs_az = std::abs(az_deg_);
  if (abs_az < 350.0f) return;  // not near the edge

  const int32_t full_turn = static_cast<int32_t>(360.0f * config::kAzTicksPerDegree);
  if (az_deg_ > 0) {
    az_enc_.set_ticks(az_enc_.ticks() - full_turn);
  } else {
    az_enc_.set_ticks(az_enc_.ticks() + full_turn);
  }
  update_measurements();
  target_az_deg_ = az_deg_;
}

void Rotator::set_gains(float kp, float ki, float kd) {
  kp_ = kp;
  ki_ = ki;
  kd_ = kd;
  az_integral_ = 0.0f;
  el_integral_ = 0.0f;
  az_prev_error_ = 0.0f;
  el_prev_error_ = 0.0f;
  az_d_filtered_ = 0.0f;
  el_d_filtered_ = 0.0f;
}

void Rotator::begin_homing() {
  // No endstop homing on combined board. IMU homing done in main.cpp setup().
  az_homed_ = false;
  el_homed_ = false;
}

int32_t Rotator::raw_az_ticks() const { return az_enc_.ticks(); }
int32_t Rotator::raw_el_ticks() const { return el_enc_.ticks(); }
