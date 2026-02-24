#pragma once

#include <cstdint>

#include "motor_pwm.h"
#include "quadrature.h"

struct RotatorStatus {
  float az_deg = 0.0f;
  float el_deg = 0.0f;
  bool az_homed = false;
  bool el_homed = false;
  bool moving = false;
  uint32_t status_reg = 1; // idle
  uint32_t error_reg = 1;  // no_error
};

class Rotator {
public:
  Rotator(MotorPwm& az_motor, MotorPwm& el_motor, Quadrature& az_enc, Quadrature& el_enc);

  void init();
  void tick();

  void set_target(float az_deg, float el_deg);
  void set_target_az(float az_deg);
  void set_target_el(float el_deg);

  void stop_az();
  void stop_el();
  void stop_all();

  void park();
  void reset_home();

  // IMU-based calibration: set encoder ticks to match a known elevation angle.
  void calibrate_el(float el_deg);
  // Assume current position is AZ=0 (user pointed north before power-on).
  void calibrate_az_zero();

  // Manual (raw duty) mode for bench testing. Bypasses the P-controller.
  void set_raw_az(float duty);
  void set_raw_el(float duty);
  void zero_encoders();
  bool manual_mode() const { return manual_mode_; }

  RotatorStatus status() const;

  // Runtime PID gain adjustment (for tuning experiments).
  void set_gains(float kp, float ki, float kd);
  float kp() const { return kp_; }
  float ki() const { return ki_; }
  float kd() const { return kd_; }

  // Direct access for diagnostics.
  int32_t raw_az_ticks() const;
  int32_t raw_el_ticks() const;
  bool endstop_az_raw() const;
  bool endstop_el_raw() const;
  bool alert_az_raw() const;
  bool alert_el_raw() const;

private:
  void update_measurements();
  void update_control();
  void update_faults();
  void begin_homing();
  void run_homing();
  void run_homing_axis(bool* homed,
                       MotorPwm& motor,
                       Quadrature& encoder,
                       bool endstop_pressed,
                       int8_t first_dir,
                       uint8_t* phase,
                       uint32_t* phase_start_ms);

  float compute_pid(float error_deg, float* integral, float* prev_error, float* d_filtered);

  bool endstop_pressed_az() const;
  bool endstop_pressed_el() const;
  bool driver_fault_az() const;
  bool driver_fault_el() const;

  MotorPwm& az_motor_;
  MotorPwm& el_motor_;
  Quadrature& az_enc_;
  Quadrature& el_enc_;

  float target_az_deg_;
  float target_el_deg_;

  float az_deg_;
  float el_deg_;

  bool az_homed_;
  bool el_homed_;
  bool homing_;
  bool az_fault_;
  bool el_fault_;
  bool manual_mode_;
  uint8_t az_home_phase_;
  uint8_t el_home_phase_;
  uint32_t az_home_phase_start_ms_;
  uint32_t el_home_phase_start_ms_;
  absolute_time_t homing_start_;

  // PID state
  float kp_;
  float ki_;
  float kd_;
  float az_integral_;
  float el_integral_;
  float az_prev_error_;
  float el_prev_error_;
  float az_d_filtered_;
  float el_d_filtered_;

  // Duty output smoothing (prevents start-stop twitching during continuous tracking)
  float az_duty_smooth_;
  float el_duty_smooth_;
};
