#include "adxl345.h"

#include <cmath>

#include "hardware/gpio.h"
#include "hardware/spi.h"
#include "pico/stdlib.h"

// ADXL345 registers
static constexpr uint8_t REG_DEVID       = 0x00;
static constexpr uint8_t REG_BW_RATE     = 0x2C;
static constexpr uint8_t REG_POWER_CTL   = 0x2D;
static constexpr uint8_t REG_DATA_FORMAT = 0x31;
static constexpr uint8_t REG_DATAX0      = 0x32;

// SPI read/write bits
static constexpr uint8_t SPI_READ_BIT    = 0x80;
static constexpr uint8_t SPI_MULTI_BIT   = 0x40;

ADXL345::ADXL345(uint cs_pin, uint sck_pin, uint mosi_pin, uint miso_pin)
    : cs_(cs_pin), sck_(sck_pin), mosi_(mosi_pin), miso_(miso_pin), ok_(false) {}

bool ADXL345::init() {
  // Init SPI0 at 1 MHz, Mode 3 (CPOL=1, CPHA=1)
  spi_init(spi0, 1000000);
  spi_set_format(spi0, 8, SPI_CPOL_1, SPI_CPHA_1, SPI_MSB_FIRST);

  gpio_set_function(sck_, GPIO_FUNC_SPI);
  gpio_set_function(mosi_, GPIO_FUNC_SPI);
  gpio_set_function(miso_, GPIO_FUNC_SPI);

  // CS is manual (GPIO, active low)
  gpio_init(cs_);
  gpio_set_dir(cs_, GPIO_OUT);
  gpio_put(cs_, 1);

  sleep_ms(10);

  // Check device ID
  uint8_t id = device_id();
  if (id != 0xE5) {
    ok_ = false;
    return false;
  }

  // Configure: ±2g, full resolution, 100 Hz output rate, measurement mode
  write_reg(REG_DATA_FORMAT, 0x08); // Full resolution, ±2g
  write_reg(REG_BW_RATE, 0x0A);    // 100 Hz
  write_reg(REG_POWER_CTL, 0x08);  // Measurement mode

  sleep_ms(10);
  ok_ = true;
  return true;
}

uint8_t ADXL345::device_id() {
  return read_reg(REG_DEVID);
}

void ADXL345::read_accel(float* x_g, float* y_g, float* z_g) {
  uint8_t buf[6];
  read_regs(REG_DATAX0, buf, 6);

  int16_t raw_x = static_cast<int16_t>(buf[0] | (buf[1] << 8));
  int16_t raw_y = static_cast<int16_t>(buf[2] | (buf[3] << 8));
  int16_t raw_z = static_cast<int16_t>(buf[4] | (buf[5] << 8));

  // Full resolution mode: 3.9 mg/LSB regardless of range
  constexpr float scale = 0.0039f;
  *x_g = raw_x * scale;
  *y_g = raw_y * scale;
  *z_g = raw_z * scale;
}

float ADXL345::elevation_deg() {
  float x, y, z;
  read_accel(&x, &y, &z);

  // Mounting: at EL=0 (horizontal), X-axis UP (+1g), Y-axis LEFT (rotation axis).
  // As EL increases, gravity shifts from X toward Z.
  // EL = atan2(z, x). At EL=0: atan2(0,1)=0. At EL=90: atan2(1,0)=90.
  return std::atan2(z, x) * (180.0f / 3.14159265f);
}

float ADXL345::elevation_deg_averaged(int samples) {
  float sum = 0.0f;
  for (int i = 0; i < samples; ++i) {
    sum += elevation_deg();
    sleep_ms(10);
  }
  return sum / static_cast<float>(samples);
}

uint8_t ADXL345::read_reg(uint8_t reg) {
  uint8_t tx[2] = {static_cast<uint8_t>(reg | SPI_READ_BIT), 0x00};
  uint8_t rx[2];

  gpio_put(cs_, 0);
  spi_write_read_blocking(spi0, tx, rx, 2);
  gpio_put(cs_, 1);

  return rx[1];
}

void ADXL345::read_regs(uint8_t reg, uint8_t* buf, uint8_t len) {
  uint8_t cmd = reg | SPI_READ_BIT | SPI_MULTI_BIT;

  gpio_put(cs_, 0);
  spi_write_blocking(spi0, &cmd, 1);
  spi_read_blocking(spi0, 0x00, buf, len);
  gpio_put(cs_, 1);
}

void ADXL345::write_reg(uint8_t reg, uint8_t value) {
  uint8_t tx[2] = {reg, value};

  gpio_put(cs_, 0);
  spi_write_blocking(spi0, tx, 2);
  gpio_put(cs_, 1);
}
