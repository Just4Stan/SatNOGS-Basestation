#pragma once

#include <cstdint>

#include "pico/types.h"

class MotorPwm {
public:
  // in1/in2 are direction pins; pwm is hardware PWM pin.
  MotorPwm(uint in1_gpio, uint in2_gpio, uint pwm_gpio);

  void init(uint32_t pwm_hz);

  // duty: -1..1 (sign = direction, magnitude = duty cycle)
  void set(float duty);
  void stop();

private:
  void set_direction(bool forward);
  void set_duty(float duty_0_to_1);

  uint in1_gpio_;
  uint in2_gpio_;
  uint pwm_gpio_;

  uint pwm_slice_;
  uint pwm_channel_;
  uint16_t pwm_wrap_;
};
