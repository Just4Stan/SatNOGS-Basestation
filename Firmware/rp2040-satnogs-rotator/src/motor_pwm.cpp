#include "motor_pwm.h"

#include <algorithm>

#include "hardware/gpio.h"
#include "hardware/clocks.h"
#include "hardware/pwm.h"
#include "pico/stdlib.h"

MotorPwm::MotorPwm(uint in1_gpio, uint in2_gpio, uint pwm_gpio)
    : in1_gpio_(in1_gpio),
      in2_gpio_(in2_gpio),
      pwm_gpio_(pwm_gpio),
      pwm_slice_(0),
      pwm_channel_(0),
      pwm_wrap_(0) {}

void MotorPwm::init(uint32_t pwm_hz) {
  gpio_init(in1_gpio_);
  gpio_set_dir(in1_gpio_, GPIO_OUT);
  gpio_put(in1_gpio_, 0);

  gpio_init(in2_gpio_);
  gpio_set_dir(in2_gpio_, GPIO_OUT);
  gpio_put(in2_gpio_, 0);

  gpio_set_function(pwm_gpio_, GPIO_FUNC_PWM);
  pwm_slice_ = pwm_gpio_to_slice_num(pwm_gpio_);
  pwm_channel_ = pwm_gpio_to_channel(pwm_gpio_);

  // Compute wrap for desired PWM frequency.
  // f_pwm = f_sys / (clkdiv * (wrap + 1))
  const uint32_t sys_hz = clock_get_hz(clk_sys);
  float clkdiv = 1.0f;
  uint32_t wrap = (sys_hz / (pwm_hz)) - 1u;
  if (wrap > 0xFFFF) {
    // Increase clkdiv to keep wrap within 16-bit.
    clkdiv = static_cast<float>(wrap + 1u) / 65536.0f;
    wrap = static_cast<uint32_t>((sys_hz / (pwm_hz * clkdiv)) - 1u);
    wrap = std::min<uint32_t>(wrap, 0xFFFF);
  }

  pwm_set_clkdiv(pwm_slice_, clkdiv);
  pwm_wrap_ = static_cast<uint16_t>(wrap);
  pwm_set_wrap(pwm_slice_, pwm_wrap_);
  pwm_set_chan_level(pwm_slice_, pwm_channel_, 0);
  pwm_set_enabled(pwm_slice_, true);
}

void MotorPwm::set(float duty) {
  duty = std::clamp(duty, -1.0f, 1.0f);
  if (duty == 0.0f) {
    stop();
    return;
  }

  const bool forward = duty > 0.0f;
  set_direction(forward);
  set_duty(std::abs(duty));
}

void MotorPwm::stop() {
  gpio_put(in1_gpio_, 0);
  gpio_put(in2_gpio_, 0);
  pwm_set_chan_level(pwm_slice_, pwm_channel_, 0);
}

void MotorPwm::set_direction(bool forward) {
  // Generic IN1/IN2 direction model:
  // forward => IN1=1, IN2=0
  // reverse => IN1=0, IN2=1
  gpio_put(in1_gpio_, forward ? 1 : 0);
  gpio_put(in2_gpio_, forward ? 0 : 1);
}

void MotorPwm::set_duty(float duty_0_to_1) {
  duty_0_to_1 = std::clamp(duty_0_to_1, 0.0f, 1.0f);
  const uint16_t level = static_cast<uint16_t>(duty_0_to_1 * static_cast<float>(pwm_wrap_));
  pwm_set_chan_level(pwm_slice_, pwm_channel_, level);
}
