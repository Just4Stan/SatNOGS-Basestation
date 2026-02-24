#include "rotator.h"

#include <algorithm>
#include <cmath>

#include "config.h"
#include "hardware/gpio.h"
#include "pico/stdlib.h"
#include "pins.h"

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
  // Consider AZ targets that differ by ±360°, pick the nearest that stays within soft limits.
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
      homing_(false),
      az_fault_(false),
      el_fault_(false),
      manual_mode_(true),
      az_home_phase_(0),
      el_home_phase_(0),
      az_home_phase_start_ms_(0),
      el_home_phase_start_ms_(0),
      homing_start_(nil_time),
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
  gpio_init(pins::kAzEndstop);
  gpio_set_dir(pins::kAzEndstop, GPIO_IN);
  gpio_pull_up(pins::kAzEndstop);

  gpio_init(pins::kElEndstop);
  gpio_set_dir(pins::kElEndstop, GPIO_IN);
  gpio_pull_up(pins::kElEndstop);

  gpio_init(pins::kAzAlert);
  gpio_set_dir(pins::kAzAlert, GPIO_IN);
  gpio_pull_up(pins::kAzAlert);

  gpio_init(pins::kElAlert);
  gpio_set_dir(pins::kElAlert, GPIO_IN);
  gpio_pull_up(pins::kElAlert);

  begin_homing();
}

void Rotator::tick() {
  update_measurements();
  update_faults();
  if (!config::kIgnoreDriverFaults && (az_fault_ || el_fault_)) {
    stop_all();
    return;
  }
  if (manual_mode_) return; // Raw duty set directly, skip control loop.
  if (homing_) {
    run_homing();
    return;
  }
  update_control();
}

void Rotator::set_target(float az_deg, float el_deg) {
  set_target_az(az_deg);
  set_target_el(el_deg);
}

void Rotator::set_target_az(float az_deg) {
  manual_mode_ = false;
  target_az_deg_ = clamp_deg(az_deg, config::kAzMinDeg, config::kAzMaxDeg);
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

void Rotator::park() { set_target(config::kParkAzDeg, config::kParkElDeg); }

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

  // Rough status bits compatible with SatNOGS rotator firmware docs.
  const bool moving = (std::abs(target_az_deg_ - az_deg_) > config::kDeadbandDeg) ||
                      (std::abs(target_el_deg_ - el_deg_) > config::kDeadbandDeg) || homing_;
  st.moving = moving;
  st.status_reg = moving ? 2u : 1u;

  // Error register:
  // - 1: no error
  // - 4: not homed / homing error
  // - 8: motor driver fault (ALERT asserted) on either axis
  uint32_t err = 1u;
  if (!(az_homed_ && el_homed_)) err |= 4u;
  if (az_fault_ || el_fault_) err |= 8u;
  st.error_reg = err;

  return st;
}

void Rotator::update_measurements() {
  az_deg_ = ticks_to_deg(az_enc_.ticks(), config::kAzTicksPerDegree, config::kAzInvert);
  el_deg_ = ticks_to_deg(el_enc_.ticks(), config::kElTicksPerDegree, config::kElInvert);
}

void Rotator::update_faults() {
  // TB6642FG/FTG ALERT behavior (datasheet `Hardware/C2150576.pdf`):
  // - Normal operation: ALERT output is LOW (open-drain pulling low)
  // - Fault (UVLO/TSD/VSD/ISD): ALERT output becomes HIGH (open-drain off, external pull-up raises it)
  az_fault_ = driver_fault_az();
  el_fault_ = driver_fault_el();
}

void Rotator::update_control() {
  const float az_error = target_az_deg_ - az_deg_;
  const float el_error = target_el_deg_ - el_deg_;

  const float az_raw = compute_pid(az_error, &az_integral_, &az_prev_error_, &az_d_filtered_);
  const float el_raw = compute_pid(el_error, &el_integral_, &el_prev_error_, &el_d_filtered_);

  // Smooth the duty output to prevent start-stop twitching during continuous tracking.
  // At 100 Hz with alpha=0.05, time constant ~0.2s → smooth ramp-up/down.
  az_duty_smooth_ += config::kDutyFilterAlpha * (az_raw - az_duty_smooth_);
  el_duty_smooth_ += config::kDutyFilterAlpha * (el_raw - el_duty_smooth_);

  az_motor_.set(az_duty_smooth_);
  el_motor_.set(el_duty_smooth_);

  // Endstop safety: prevent driving further into a pressed endstop during normal operation.
  if (endstop_pressed_az() && (az_error < 0.0f)) az_motor_.stop();
  if (endstop_pressed_el() && (el_error < 0.0f)) el_motor_.stop();
}

float Rotator::compute_pid(float error_deg, float* integral, float* prev_error, float* d_filtered) {
  const float abs_error = std::abs(error_deg);

  if (abs_error <= config::kDeadbandDeg) {
    // Within deadband: stop output and reset PID state.
    *integral = 0.0f;
    *prev_error = error_deg;
    *d_filtered = 0.0f;
    return 0.0f;
  }

  // D-term: filtered derivative
  const float raw_deriv = (error_deg - *prev_error) / config::kDtSeconds;
  *d_filtered = *d_filtered + config::kDFilterAlpha * (raw_deriv - *d_filtered);
  *prev_error = error_deg;

  // Integrate (only outside deadband to prevent limit-cycle wind-up)
  *integral += error_deg * config::kDtSeconds;
  *integral = std::clamp(*integral, -config::kIntegralMaxDeg, config::kIntegralMaxDeg);

  // PID output
  float duty = kp_ * error_deg + ki_ * (*integral) + kd_ * (*d_filtered);

  // Clamp to max duty. No minimum floor — the I-term handles stiction naturally.
  duty = std::clamp(duty, -config::kMaxDuty, config::kMaxDuty);
  return duty;
}

void Rotator::set_gains(float kp, float ki, float kd) {
  kp_ = kp;
  ki_ = ki;
  kd_ = kd;
  // Reset PID state when gains change.
  az_integral_ = 0.0f;
  el_integral_ = 0.0f;
  az_prev_error_ = 0.0f;
  el_prev_error_ = 0.0f;
  az_d_filtered_ = 0.0f;
  el_d_filtered_ = 0.0f;
}

void Rotator::begin_homing() {
  if (!config::kUseEndstopHoming) {
    // IMU-based calibration isn't implemented yet; leave unhomed.
    az_homed_ = false;
    el_homed_ = false;
    homing_ = false;
    return;
  }

  homing_ = true;
  az_homed_ = false;
  el_homed_ = false;

  az_home_phase_ = 0;
  el_home_phase_ = 0;
  const uint32_t now_ms = to_ms_since_boot(get_absolute_time());
  az_home_phase_start_ms_ = now_ms;
  el_home_phase_start_ms_ = now_ms;

  homing_start_ = get_absolute_time();
}

void Rotator::run_homing() {
  const uint32_t elapsed_ms = to_ms_since_boot(get_absolute_time()) - to_ms_since_boot(homing_start_);
  if (elapsed_ms > config::kHomeTimeoutMs) {
    stop_all();
    homing_ = false;
    return;
  }

  // With only a single "home" switch per axis, we search in one direction and then the other.
  run_homing_axis(&az_homed_,
                  az_motor_,
                  az_enc_,
                  endstop_pressed_az(),
                  config::kAzHomeFirstDir,
                  &az_home_phase_,
                  &az_home_phase_start_ms_);
  run_homing_axis(&el_homed_,
                  el_motor_,
                  el_enc_,
                  endstop_pressed_el(),
                  config::kElHomeFirstDir,
                  &el_home_phase_,
                  &el_home_phase_start_ms_);

  if (az_homed_ && el_homed_) {
    homing_ = false;
    target_az_deg_ = 0.0f;
    target_el_deg_ = 0.0f;
  }
}

void Rotator::run_homing_axis(bool* homed,
                              MotorPwm& motor,
                              Quadrature& encoder,
                              bool endstop_pressed,
                              int8_t first_dir,
                              uint8_t* phase,
                              uint32_t* phase_start_ms) {
  if (*homed) return;

  if (endstop_pressed) {
    motor.stop();
    encoder.set_ticks(0);
    *homed = true;
    return;
  }

  const uint32_t now_ms = to_ms_since_boot(get_absolute_time());
  const uint32_t phase_elapsed = now_ms - *phase_start_ms;

  int8_t dir = first_dir;
  if (dir == 0) dir = -1;
  if (*phase >= 1) dir = static_cast<int8_t>(-dir);

  if (phase_elapsed > config::kHomePhaseTimeoutMs) {
    if (*phase == 0) {
      *phase = 1;
      *phase_start_ms = now_ms;
      motor.stop();
      return;
    }
    // Both directions failed (or the home switch is unreachable).
    motor.stop();
    return;
  }

  motor.set(static_cast<float>(dir) * config::kHomeDuty);
}

bool Rotator::endstop_pressed_az() const {
  const bool raw = gpio_get(pins::kAzEndstop) != 0;
  return config::kEndstopActiveLow ? !raw : raw;
}

bool Rotator::endstop_pressed_el() const {
  const bool raw = gpio_get(pins::kElEndstop) != 0;
  return config::kEndstopActiveLow ? !raw : raw;
}

bool Rotator::driver_fault_az() const { return gpio_get(pins::kAzAlert) != 0; }

bool Rotator::driver_fault_el() const { return gpio_get(pins::kElAlert) != 0; }

int32_t Rotator::raw_az_ticks() const { return az_enc_.ticks(); }
int32_t Rotator::raw_el_ticks() const { return el_enc_.ticks(); }
bool Rotator::endstop_az_raw() const { return gpio_get(pins::kAzEndstop) == 0; }
bool Rotator::endstop_el_raw() const { return gpio_get(pins::kElEndstop) == 0; }
bool Rotator::alert_az_raw() const { return gpio_get(pins::kAzAlert) != 0; }
bool Rotator::alert_el_raw() const { return gpio_get(pins::kElAlert) != 0; }
