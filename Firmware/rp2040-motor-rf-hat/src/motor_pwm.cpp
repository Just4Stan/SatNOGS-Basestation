#include "motor_pwm.h"

#include <algorithm>

#include "hardware/gpio.h"
#include "hardware/clocks.h"
#include "hardware/pwm.h"
#include "pico/stdlib.h"

MotorPwm::MotorPwm(uint in1_gpio, uint in2_gpio)
    : in1_gpio_(in1_gpio),
      in2_gpio_(in2_gpio),
      in1_slice_(0),
      in1_channel_(0),
      in2_slice_(0),
      in2_channel_(0),
      pwm_wrap_(0) {}

void MotorPwm::init(uint32_t pwm_hz) {
  // Both IN1 and IN2 are PWM outputs for A4950 2-pin mode.
  gpio_set_function(in1_gpio_, GPIO_FUNC_PWM);
  in1_slice_ = pwm_gpio_to_slice_num(in1_gpio_);
  in1_channel_ = pwm_gpio_to_channel(in1_gpio_);

  gpio_set_function(in2_gpio_, GPIO_FUNC_PWM);
  in2_slice_ = pwm_gpio_to_slice_num(in2_gpio_);
  in2_channel_ = pwm_gpio_to_channel(in2_gpio_);

  // Compute wrap for desired PWM frequency.
  // f_pwm = f_sys / (clkdiv * (wrap + 1))
  const uint32_t sys_hz = clock_get_hz(clk_sys);
  float clkdiv = 1.0f;
  uint32_t wrap = (sys_hz / pwm_hz) - 1u;
  if (wrap > 0xFFFF) {
    clkdiv = static_cast<float>(wrap + 1u) / 65536.0f;
    wrap = static_cast<uint32_t>((sys_hz / (pwm_hz * clkdiv)) - 1u);
    wrap = std::min<uint32_t>(wrap, 0xFFFF);
  }
  pwm_wrap_ = static_cast<uint16_t>(wrap);

  // Configure IN1 PWM slice
  pwm_set_clkdiv(in1_slice_, clkdiv);
  pwm_set_wrap(in1_slice_, pwm_wrap_);
  pwm_set_chan_level(in1_slice_, in1_channel_, 0);
  pwm_set_enabled(in1_slice_, true);

  // Configure IN2 PWM slice (may be same slice if pins are on same slice)
  if (in2_slice_ != in1_slice_) {
    pwm_set_clkdiv(in2_slice_, clkdiv);
    pwm_set_wrap(in2_slice_, pwm_wrap_);
  }
  pwm_set_chan_level(in2_slice_, in2_channel_, 0);
  if (in2_slice_ != in1_slice_) {
    pwm_set_enabled(in2_slice_, true);
  }
}

void MotorPwm::set(float duty) {
  duty = std::clamp(duty, -1.0f, 1.0f);
  if (duty == 0.0f) {
    stop();
    return;
  }

  const float abs_duty = std::abs(duty);
  const uint16_t level = static_cast<uint16_t>(abs_duty * static_cast<float>(pwm_wrap_));

  if (duty > 0.0f) {
    // Forward: IN1=duty, IN2=0
    pwm_set_chan_level(in1_slice_, in1_channel_, level);
    pwm_set_chan_level(in2_slice_, in2_channel_, 0);
  } else {
    // Reverse: IN1=0, IN2=duty
    pwm_set_chan_level(in1_slice_, in1_channel_, 0);
    pwm_set_chan_level(in2_slice_, in2_channel_, level);
  }
}

void MotorPwm::stop() {
  // Coast: both outputs low
  pwm_set_chan_level(in1_slice_, in1_channel_, 0);
  pwm_set_chan_level(in2_slice_, in2_channel_, 0);
}
