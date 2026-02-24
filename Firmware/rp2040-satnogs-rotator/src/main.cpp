#include <cstdio>

#include <Arduino.h>
#include "hardware/gpio.h"
#include "pico/stdlib.h"

#include "adxl345.h"
#include "config.h"
#include "motor_pwm.h"
#include "pins.h"
#include "quadrature.h"
#include "rotator.h"
#include "satnogs_protocol.h"

namespace {

Quadrature* g_az_enc = nullptr;
Quadrature* g_el_enc = nullptr;

void gpio_irq_callback(uint gpio, uint32_t events) {
  (void)events;
  if (g_az_enc && (gpio == pins::kAzEncA || gpio == pins::kAzEncB)) g_az_enc->on_edge();
  if (g_el_enc && (gpio == pins::kElEncA || gpio == pins::kElEncB)) g_el_enc->on_edge();
}

void enable_encoder_irqs() {
  const uint32_t event_mask = GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL;

  gpio_set_irq_enabled_with_callback(pins::kAzEncA, event_mask, true, &gpio_irq_callback);
  gpio_set_irq_enabled(pins::kAzEncB, event_mask, true);

  gpio_set_irq_enabled(pins::kElEncA, event_mask, true);
  gpio_set_irq_enabled(pins::kElEncB, event_mask, true);
}

} // namespace

void setup() {
  Serial.begin(115200);

  // Heartbeat LED on Pico onboard GP25.
  gpio_init(pins::kLed);
  gpio_set_dir(pins::kLed, GPIO_OUT);
  gpio_put(pins::kLed, 1);

  sleep_ms(1500); // give USB CDC time to enumerate

  static MotorPwm az_motor(pins::kAzIn1, pins::kAzIn2, pins::kAzPwm);
  static MotorPwm el_motor(pins::kElIn1, pins::kElIn2, pins::kElPwm);
  az_motor.init(config::kPwmHz);
  el_motor.init(config::kPwmHz);

  static Quadrature az_enc(pins::kAzEncA, pins::kAzEncB);
  static Quadrature el_enc(pins::kElEncA, pins::kElEncB);
  az_enc.init();
  el_enc.init();

  g_az_enc = &az_enc;
  g_el_enc = &el_enc;
  enable_encoder_irqs();

  static Rotator rotator(az_motor, el_motor, az_enc, el_enc);
  rotator.init();

  // ADXL345 accelerometer on SPI0
  static ADXL345 imu(pins::kSpiCs, pins::kSpiSck, pins::kSpiMosi, pins::kSpiMiso);
  const bool imu_ok = imu.init();

  // Calibration: AZ assumed north, EL from IMU gravity reading.
  rotator.calibrate_az_zero();
  if (imu_ok) {
    const float imu_el = imu.elevation_deg_averaged(20);
    rotator.calibrate_el(imu_el);
    std::printf("EL calibrated from IMU: %.1f deg\n", imu_el);
  } else {
    rotator.calibrate_el(0.0f);
    std::printf("IMU not found — EL assumed 0 (place antenna horizontal)\n");
  }

  static SatnogsProtocol protocol(rotator, imu_ok ? &imu : nullptr);

  std::printf("\n== SatNOGS-RP2040-v0.2 ready ==\n");
  std::printf("IMU: %s (ID=0x%02X)\n", imu_ok ? "OK" : "NOT FOUND", imu.device_id());
  std::printf("AZ zeroed (point north before power-on)\n");
  std::printf("Type HELP for commands\n");

  char line[128];
  size_t line_len = 0;

  absolute_time_t last_tick = get_absolute_time();
  absolute_time_t last_blink = get_absolute_time();
  absolute_time_t last_monitor = get_absolute_time();
  bool led_state = true;

  for (;;) {
    // Control loop tick ~100 Hz.
    if (absolute_time_diff_us(last_tick, get_absolute_time()) >= 10000) {
      rotator.tick();
      last_tick = get_absolute_time();
    }

    // Heartbeat LED toggle ~1 Hz (500ms on, 500ms off).
    if (absolute_time_diff_us(last_blink, get_absolute_time()) >= 500000) {
      led_state = !led_state;
      gpio_put(pins::kLed, led_state ? 1 : 0);
      last_blink = get_absolute_time();
    }

    // Live monitor stream ~10 Hz.
    if (protocol.monitoring() && absolute_time_diff_us(last_monitor, get_absolute_time()) >= 100000) {
      protocol.print_monitor_line();
      last_monitor = get_absolute_time();
    }

    if (!Serial.available()) {
      tight_loop_contents();
      continue;
    }
    const int ch = Serial.read();

    if (ch == '\r' || ch == '\n') {
      if (line_len > 0) {
        protocol.handle_line(line, line_len);
        line_len = 0;
      }
      continue;
    }

    if (line_len < sizeof(line) - 1) {
      line[line_len++] = static_cast<char>(ch);
      line[line_len] = '\0';
    } else {
      // Overflow: reset line.
      line_len = 0;
    }
  }
}

void loop() {
  // Never reached: setup() runs the main loop.
}

