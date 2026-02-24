#include "quadrature.h"

#include "hardware/gpio.h"
#include "pico/stdlib.h"

namespace {

uint8_t read_ab(uint gpio_a, uint gpio_b) {
  const uint a = gpio_get(gpio_a);
  const uint b = gpio_get(gpio_b);
  return static_cast<uint8_t>((a << 1) | b);
}

// State transition table (last_state << 2 | new_state) => delta ticks.
// Derived from Gray code quadrature sequence.
constexpr int8_t kDelta[16] = {
    0,  -1, +1, 0,  //
    +1, 0,  0,  -1, //
    -1, 0,  0,  +1, //
    0,  +1, -1, 0   //
};

} // namespace

Quadrature::Quadrature(uint gpio_a, uint gpio_b)
    : gpio_a_(gpio_a), gpio_b_(gpio_b), ticks_(0), last_state_(0) {}

void Quadrature::init() {
  gpio_init(gpio_a_);
  gpio_set_dir(gpio_a_, GPIO_IN);
  // Keep an internal pull-up enabled. This is safe for open-collector encoder outputs and also helps
  // with floating inputs during bring-up. If your encoder outputs are push-pull at 3.3 V, this is fine.
  gpio_pull_up(gpio_a_);

  gpio_init(gpio_b_);
  gpio_set_dir(gpio_b_, GPIO_IN);
  gpio_pull_up(gpio_b_);

  last_state_ = read_ab(gpio_a_, gpio_b_);
}

int32_t Quadrature::ticks() const { return ticks_; }

void Quadrature::set_ticks(int32_t value) {
  ticks_ = value;
  last_state_ = read_ab(gpio_a_, gpio_b_);
}

void Quadrature::on_edge() {
  const uint8_t new_state = read_ab(gpio_a_, gpio_b_);
  const uint8_t idx = static_cast<uint8_t>((last_state_ << 2) | new_state);
  ticks_ += kDelta[idx];
  last_state_ = new_state;
}
