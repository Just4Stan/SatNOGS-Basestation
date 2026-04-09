#include <cstdio>

#include <Arduino.h>
#include "hardware/gpio.h"
#include "hardware/watchdog.h"
#include "pico/stdlib.h"

#include "adxl345.h"
#include "config.h"
#include "motor_pwm.h"
#include "pins.h"
#include "quadrature.h"
#include "rotator.h"
#include "satnogs_protocol.h"
#include "status_led.h"

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

  status_led_init();  // NeoPixel on GP16

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

  // Calibration: AZ assumed north (point antenna north before power-on).
  rotator.calibrate_az_zero();

  // EL homing: drive motor until IMU reads 0° (horizontal), then zero encoder.
  if (imu_ok) {
    const float initial_el = imu.elevation_deg_averaged(10);
    std::printf("EL homing: IMU reads %.1f deg, driving to horizontal...\n", initial_el);

    status_led_set(LedState::kTracking);  // visual feedback during homing

    constexpr float kElHomingDeadband = 1.5f;  // degrees — close enough to horizontal
    constexpr float kElHomingDuty = 0.25f;     // slow and gentle
    constexpr uint32_t kElHomingTimeout = 15000; // 15 seconds max
    const uint32_t homing_start = to_ms_since_boot(get_absolute_time());
    bool homed = false;

    while (to_ms_since_boot(get_absolute_time()) - homing_start < kElHomingTimeout) {
      const float el = imu.elevation_deg_averaged(3);

      if (std::abs(el) <= kElHomingDeadband) {
        el_motor.stop();
        rotator.calibrate_el(0.0f);
        std::printf("EL homed to horizontal (IMU: %.1f deg)\n", el);
        homed = true;
        break;
      }

      // Drive toward 0°.  Negative duty decreases physical EL.
      const float duty = (el > 0) ? -kElHomingDuty : kElHomingDuty;
      el_motor.set(duty);
      watchdog_update();
      sleep_ms(50);
    }

    if (!homed) {
      el_motor.stop();
      const float fallback = imu.elevation_deg_averaged(20);
      rotator.calibrate_el(fallback);
      std::printf("EL homing timeout — calibrated from IMU at %.1f deg\n", fallback);
    }
  } else {
    rotator.calibrate_el(0.0f);
    std::printf("IMU not found — EL assumed 0 (place antenna horizontal)\n");
  }

  static SatnogsProtocol protocol(rotator, imu_ok ? &imu : nullptr);

  status_led_set(LedState::kIdle);  // init done

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

  // Hardware watchdog — 8 second timeout
  watchdog_enable(8000, true);

  for (;;) {
    watchdog_update();

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

      // Update NeoPixel state based on rotator status (1 Hz is enough)
      auto st = rotator.status();
      if (st.error_reg != 1) {
        status_led_set(LedState::kFault);
      } else if (st.moving) {
        status_led_set(LedState::kTracking);
      } else if (st.status_reg == 1) {
        status_led_set(LedState::kIdle);
      } else {
        status_led_set(LedState::kOnTarget);
      }
    }

    // NeoPixel update (~20 Hz is fine, but we run it every loop iteration;
    // internally it only changes on blink toggles or state changes)
    status_led_update();

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

