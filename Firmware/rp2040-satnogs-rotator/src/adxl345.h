#pragma once

#include <cstdint>
#include "pico/types.h"

class ADXL345 {
public:
  ADXL345(uint cs_pin, uint sck_pin, uint mosi_pin, uint miso_pin);

  bool init();  // Returns true if DEVID reads 0xE5.
  void read_accel(float* x_g, float* y_g, float* z_g);

  // Elevation angle from gravity vector (degrees, 0 = horizontal, 90 = zenith).
  // Mounting convention: at EL=0, X-axis UP, Y-axis LEFT (rotation axis).
  float elevation_deg();

  // Averaged elevation reading for calibration (reduces noise).
  float elevation_deg_averaged(int samples = 20);

  uint8_t device_id();

private:
  uint8_t read_reg(uint8_t reg);
  void read_regs(uint8_t reg, uint8_t* buf, uint8_t len);
  void write_reg(uint8_t reg, uint8_t value);

  uint cs_;
  uint sck_;
  uint mosi_;
  uint miso_;
  bool ok_;
};
