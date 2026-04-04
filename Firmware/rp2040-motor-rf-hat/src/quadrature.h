#pragma once

#include <cstdint>

#include "pico/types.h"

class Quadrature {
public:
  Quadrature(uint gpio_a, uint gpio_b);

  void init();
  int32_t ticks() const;
  void set_ticks(int32_t value);

  // Called from shared GPIO IRQ handler.
  void on_edge();

private:
  uint gpio_a_;
  uint gpio_b_;
  volatile int32_t ticks_;
  uint8_t last_state_;
};
