#pragma once

#include <cstdint>

#include "pico/types.h"

// A4950 2-pin motor driver: IN1 and IN2 are both PWM outputs.
// Forward: IN1=duty, IN2=0
// Reverse: IN1=0, IN2=duty
// Coast/stop: IN1=0, IN2=0
class MotorPwm {
public:
  MotorPwm(uint in1_gpio, uint in2_gpio);

  void init(uint32_t pwm_hz);

  // duty: -1..1 (sign = direction, magnitude = duty cycle)
  void set(float duty);
  void stop();

private:
  uint in1_gpio_;
  uint in2_gpio_;

  uint in1_slice_;
  uint in1_channel_;
  uint in2_slice_;
  uint in2_channel_;
  uint16_t pwm_wrap_;
};
