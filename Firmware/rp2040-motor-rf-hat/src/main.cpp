// MOTOR_RF_HAT Firmware — Combined motor controller + dual CC1200 RF HAT
// Single RP2040 Pico handles:
//   - 2x A4950 motor drivers (AZ + EL)
//   - 2x quadrature encoders
//   - ADXL345 IMU (SPI0, shared with VHF CC1200)
//   - UHF CC1200 (SPI1, 432 MHz)
//   - VHF CC1200 (SPI0, 144 MHz)
//   - 4x WS2812B NeoPixel
//   - Passive buzzer
// Communication: unified COBS/CRC-16 binary protocol over UART0 (GP0/GP1) to Pi 3A+

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <cstdio>
#include <cmath>
#include "hardware/spi.h"
#include "hardware/uart.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "hardware/watchdog.h"
#include "pico/stdlib.h"

extern "C" {
#include "config.h"
#include "cc1200.h"
#include "proto.h"
}

#include "motor_pwm.h"
#include "quadrature.h"
#include "rotator.h"
#include "adxl345.h"

// ---------------------------------------------------------------------------
// Global instances (static lifetime, initialized in setup())
// ---------------------------------------------------------------------------
static cc1200_t radios[RADIO_COUNT];

static Adafruit_NeoPixel neopixel(NEOPIXEL_COUNT, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);
static constexpr uint8_t LED_BR = 40;

// Motor/encoder objects — constructed in setup() as static locals
static MotorPwm*   g_az_motor = nullptr;
static MotorPwm*   g_el_motor = nullptr;
static Quadrature*  g_az_enc = nullptr;
static Quadrature*  g_el_enc = nullptr;
static Rotator*     g_rotator = nullptr;
static ADXL345*     g_imu = nullptr;
static bool         g_imu_ok = false;

// Heartbeat state
static uint32_t hb_next = 0;
static bool hb_state = false;
static bool uhf_ok_flag = false;
static bool vhf_ok_flag = false;

// PID tick timing
static absolute_time_t last_pid_tick;

// Position event streaming (10 Hz)
static uint32_t pos_evt_next = 0;

// ---------------------------------------------------------------------------
// Buzzer — non-blocking PWM tone sequencer on GP5
// ---------------------------------------------------------------------------
struct BuzzerStep { uint16_t freq_hz; uint16_t dur_ms; };

static const BuzzerStep SEQ_READY[] = {
    {1000, 100}, {0, 100}, {1000, 100}, {0, 100}, {1000, 100}, {0, 0}
};
static const BuzzerStep SEQ_AOS[] = {
    {800, 150}, {0, 50}, {1200, 150}, {0, 0}
};
static const BuzzerStep SEQ_LOS[] = {
    {1200, 150}, {0, 50}, {800, 150}, {0, 0}
};
static const BuzzerStep SEQ_PACKET[] = {
    {1500, 80}, {0, 0}
};
static const BuzzerStep SEQ_ERROR[] = {
    {400, 500}, {0, 0}
};
static const BuzzerStep SEQ_WIFI_OK[] = {
    {523, 80}, {0, 30}, {659, 80}, {0, 30}, {784, 80}, {0, 0}
};
static const BuzzerStep SEQ_GPS_OK[] = {
    {2000, 60}, {0, 40}, {2000, 60}, {0, 0}
};
static const BuzzerStep SEQ_SETUP_DONE[] = {
    {523, 80}, {0, 30}, {659, 80}, {0, 30}, {784, 80}, {0, 30},
    {1047, 80}, {0, 30}, {784, 80}, {0, 0}
};

static const BuzzerStep* bz_seq = nullptr;
static uint8_t  bz_step = 0;
static uint32_t bz_step_end = 0;
static uint      bz_slice = 0;

static void buzzer_init()
{
    gpio_set_function(PIN_BUZZER, GPIO_FUNC_PWM);
    bz_slice = pwm_gpio_to_slice_num(PIN_BUZZER);
    pwm_set_enabled(bz_slice, false);
}

static void buzzer_tone(uint16_t freq_hz)
{
    if (freq_hz == 0) {
        pwm_set_enabled(bz_slice, false);
        return;
    }
    uint32_t wrap = 125000000UL / freq_hz;
    if (wrap > 65535) wrap = 65535;
    pwm_set_wrap(bz_slice, (uint16_t)wrap);
    pwm_set_chan_level(bz_slice, pwm_gpio_to_channel(PIN_BUZZER), (uint16_t)(wrap / 2));
    pwm_set_enabled(bz_slice, true);
}

static void buzzer_start(uint8_t pattern)
{
    switch (pattern) {
        case 0x01: bz_seq = SEQ_READY;  break;
        case 0x02: bz_seq = SEQ_AOS;    break;
        case 0x03: bz_seq = SEQ_LOS;    break;
        case 0x04: bz_seq = SEQ_PACKET; break;
        case 0x05: bz_seq = SEQ_ERROR;      break;
        case 0x06: bz_seq = SEQ_WIFI_OK;    break;
        case 0x07: bz_seq = SEQ_GPS_OK;     break;
        case 0x08: bz_seq = SEQ_SETUP_DONE; break;
        default:   return;
    }
    bz_step = 0;
    buzzer_tone(bz_seq[0].freq_hz);
    bz_step_end = millis() + bz_seq[0].dur_ms;
}

static void buzzer_tick()
{
    if (!bz_seq) return;
    if (millis() < bz_step_end) return;

    bz_step++;
    if (bz_seq[bz_step].dur_ms == 0 && bz_seq[bz_step].freq_hz == 0) {
        buzzer_tone(0);
        bz_seq = nullptr;
        return;
    }
    buzzer_tone(bz_seq[bz_step].freq_hz);
    bz_step_end = millis() + bz_seq[bz_step].dur_ms;
}

static void on_buzzer_cmd(uint8_t pattern) { buzzer_start(pattern); }

// ---------------------------------------------------------------------------
// NeoPixel helpers
// ---------------------------------------------------------------------------
static void neo_set_all(uint8_t r, uint8_t g, uint8_t b)
{
    uint32_t c = neopixel.Color(r, g, b);
    for (int i = 0; i < NEOPIXEL_COUNT; i++)
        neopixel.setPixelColor(i, c);
    neopixel.show();
}

// ---------------------------------------------------------------------------
// Encoder ISR (shared GPIO callback)
// ---------------------------------------------------------------------------
static void gpio_irq_callback(uint gpio, uint32_t events) {
    (void)events;
    if (g_az_enc && (gpio == PIN_AZ_ENC_A || gpio == PIN_AZ_ENC_B)) g_az_enc->on_edge();
    if (g_el_enc && (gpio == PIN_EL_ENC_A || gpio == PIN_EL_ENC_B)) g_el_enc->on_edge();
}

static void enable_encoder_irqs() {
    const uint32_t event_mask = GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL;

    gpio_set_irq_enabled_with_callback(PIN_AZ_ENC_A, event_mask, true, &gpio_irq_callback);
    gpio_set_irq_enabled(PIN_AZ_ENC_B, event_mask, true);

    gpio_set_irq_enabled(PIN_EL_ENC_A, event_mask, true);
    gpio_set_irq_enabled(PIN_EL_ENC_B, event_mask, true);
}

// ---------------------------------------------------------------------------
// Radio init helper
// ---------------------------------------------------------------------------
static bool init_radio(uint8_t idx, spi_inst_t* spi, uint32_t baud,
                       uint pin_sck, uint pin_mosi, uint pin_miso, uint pin_csn,
                       uint pin_reset, const char* label)
{
    cc1200_hal_t hal = {};
    hal.spi       = spi;
    hal.baud_hz   = baud;
    hal.pin_sck   = pin_sck;
    hal.pin_mosi  = pin_mosi;
    hal.pin_miso  = pin_miso;
    hal.pin_csn   = pin_csn;
    hal.pin_reset = pin_reset;
    hal.so_ready_timeout_us = CC1200_SO_READY_TIMEOUT_US;

    if (!cc1200_init(&radios[idx], &hal)) {
        Serial.printf("[%s] HAL init failed\n", label);
        return false;
    }

    if (!cc1200_begin(&radios[idx])) {
        Serial.printf("[%s] CC1200 reset/begin failed\n", label);
        return false;
    }

    uint8_t part = 0, ver = 0;
    cc1200_read_ext(&radios[idx], 0x8F, &part);
    cc1200_read_ext(&radios[idx], 0x90, &ver);

    Serial.printf("[%s] CC1200 part=0x%02X ver=0x%02X %s\n",
                  label, part, ver, (part == 0x20) ? "OK" : "FAIL");

    return (part == 0x20);
}

// ---------------------------------------------------------------------------
// Motor protocol callbacks
// ---------------------------------------------------------------------------
static void cb_motor_set_position(float az_deg, float el_deg) {
    if (g_rotator) g_rotator->set_target(az_deg, el_deg);
}

static void cb_motor_get_position(float* az_deg, float* el_deg, uint8_t* status) {
    if (g_rotator) {
        auto st = g_rotator->status();
        *az_deg = st.az_deg;
        *el_deg = st.el_deg;
        *status = st.moving ? 2u : 1u;
    }
}

static void cb_motor_park(void) {
    if (g_rotator) g_rotator->park();
}

static void cb_motor_stop(void) {
    if (g_rotator) g_rotator->stop_all();
}

static void cb_motor_home(void);  // forward decl — needs IMU access

static void cb_motor_get_status(uint8_t* buf, uint16_t* len) {
    if (!g_rotator) { *len = 0; return; }
    auto st = g_rotator->status();
    // state_u8, fault_u8, homed_u8, az_f32, el_f32, az_tgt_f32, el_tgt_f32
    buf[0] = st.moving ? 2u : 1u;
    buf[1] = (st.error_reg & 8u) ? 1u : 0u;
    buf[2] = (st.az_homed && st.el_homed) ? 1u : 0u;

    // Float LE helper (same as proto.c put_f32_le)
    union { float f; uint8_t b[4]; } u;
    u.f = st.az_deg;   memcpy(&buf[3], u.b, 4);
    u.f = st.el_deg;   memcpy(&buf[7], u.b, 4);
    // Target positions — get from status (current = target when on-target)
    // We need actual targets, so access rotator internals via status
    u.f = st.az_deg;   memcpy(&buf[11], u.b, 4); // approximate (status doesn't expose target)
    u.f = st.el_deg;   memcpy(&buf[15], u.b, 4);
    *len = 19;
}

static void cb_motor_set_gains(float kp, float ki, float kd) {
    if (g_rotator) g_rotator->set_gains(kp, ki, kd);
}

static void cb_motor_zero_encoders(void) {
    if (g_rotator) g_rotator->zero_encoders();
}

static void cb_imu_read(uint8_t* buf, uint16_t* len) {
    if (!g_imu || !g_imu_ok) { *len = 0; return; }

    // Switch SPI0 to mode 3 for ADXL345
    spi_deinit(spi0);
    spi_init(spi0, 1000000);
    spi_set_format(spi0, 8, SPI_CPOL_1, SPI_CPHA_1, SPI_MSB_FIRST);
    gpio_set_function(VHF_PIN_SCK, GPIO_FUNC_SPI);
    gpio_set_function(VHF_PIN_MOSI, GPIO_FUNC_SPI);
    gpio_set_function(VHF_PIN_MISO, GPIO_FUNC_SPI);

    float x, y, z;
    g_imu->read_accel(&x, &y, &z);
    float el = g_imu->elevation_deg();

    // Switch back to mode 0 for VHF CC1200
    spi_deinit(spi0);
    spi_init(spi0, 5000000);
    spi_set_format(spi0, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
    gpio_set_function(VHF_PIN_SCK, GPIO_FUNC_SPI);
    gpio_set_function(VHF_PIN_MOSI, GPIO_FUNC_SPI);
    gpio_set_function(VHF_PIN_MISO, GPIO_FUNC_SPI);

    union { float f; uint8_t b[4]; } u;
    u.f = x;   memcpy(&buf[0], u.b, 4);
    u.f = y;   memcpy(&buf[4], u.b, 4);
    u.f = z;   memcpy(&buf[8], u.b, 4);
    u.f = el;  memcpy(&buf[12], u.b, 4);
    *len = 16;
}

static void cb_motor_recal(void) {
    if (!g_imu || !g_imu_ok || !g_rotator) return;

    // Switch SPI0 to mode 3 for ADXL345
    spi_deinit(spi0);
    spi_init(spi0, 1000000);
    spi_set_format(spi0, 8, SPI_CPOL_1, SPI_CPHA_1, SPI_MSB_FIRST);
    gpio_set_function(VHF_PIN_SCK, GPIO_FUNC_SPI);
    gpio_set_function(VHF_PIN_MOSI, GPIO_FUNC_SPI);
    gpio_set_function(VHF_PIN_MISO, GPIO_FUNC_SPI);

    float el = g_imu->elevation_deg_averaged(20);
    g_rotator->calibrate_el(el);
    Serial.printf("RECAL: IMU reads %.1f deg, encoder re-zeroed\n", el);

    // Switch back to mode 0 for VHF CC1200
    spi_deinit(spi0);
    spi_init(spi0, 5000000);
    spi_set_format(spi0, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
    gpio_set_function(VHF_PIN_SCK, GPIO_FUNC_SPI);
    gpio_set_function(VHF_PIN_MOSI, GPIO_FUNC_SPI);
    gpio_set_function(VHF_PIN_MISO, GPIO_FUNC_SPI);
}

// IMU re-homing: drive EL motor to horizontal using IMU feedback
// This is blocking (~15s max) — same as original rotator homing
static void cb_motor_home(void) {
    if (!g_imu || !g_imu_ok || !g_rotator || !g_el_motor) return;

    // Switch SPI0 to mode 3 for ADXL345
    spi_deinit(spi0);
    spi_init(spi0, 1000000);
    spi_set_format(spi0, 8, SPI_CPOL_1, SPI_CPHA_1, SPI_MSB_FIRST);
    gpio_set_function(VHF_PIN_SCK, GPIO_FUNC_SPI);
    gpio_set_function(VHF_PIN_MOSI, GPIO_FUNC_SPI);
    gpio_set_function(VHF_PIN_MISO, GPIO_FUNC_SPI);

    constexpr float kElHomingDeadband = 1.5f;
    constexpr float kElHomingDuty = 0.25f;
    constexpr uint32_t kElHomingTimeout = 15000;
    const uint32_t start = to_ms_since_boot(get_absolute_time());

    while (to_ms_since_boot(get_absolute_time()) - start < kElHomingTimeout) {
        float el = g_imu->elevation_deg_averaged(3);
        if (std::abs(el) <= kElHomingDeadband) {
            g_el_motor->stop();
            g_rotator->calibrate_el(0.0f);
            break;
        }
        float duty = (el > 0) ? -kElHomingDuty : kElHomingDuty;
        g_el_motor->set(duty);
        watchdog_update();
        sleep_ms(50);
    }
    g_el_motor->stop();

    // Fallback: calibrate from current IMU reading
    float fallback = g_imu->elevation_deg_averaged(10);
    g_rotator->calibrate_el(fallback);

    g_rotator->calibrate_az_zero();

    // Switch back to mode 0 for VHF CC1200
    spi_deinit(spi0);
    spi_init(spi0, 5000000);
    spi_set_format(spi0, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
    gpio_set_function(VHF_PIN_SCK, GPIO_FUNC_SPI);
    gpio_set_function(VHF_PIN_MOSI, GPIO_FUNC_SPI);
    gpio_set_function(VHF_PIN_MISO, GPIO_FUNC_SPI);
}

// ---------------------------------------------------------------------------
// Arduino entry points
// ---------------------------------------------------------------------------
void setup()
{
    // USB serial for debug output
    Serial.begin(115200);

    // Onboard LED
    pinMode(PIN_LED, OUTPUT);
    digitalWrite(PIN_LED, HIGH);

    // Wait for USB serial (up to 3s)
    uint32_t usb_wait_start = millis();
    while (!Serial && (millis() - usb_wait_start < 3000)) {
        delay(10);
    }

    Serial.println("\n=== MOTOR_RF_HAT Firmware ===");
    Serial.printf("Pins: NEOPIXEL=%d BUZZER=%d UART_TX=%d UART_RX=%d\n",
                  PIN_NEOPIXEL, PIN_BUZZER, PIN_UART_TX, PIN_UART_RX);
    Serial.printf("AZ: IN1=%d IN2=%d ENC_A=%d ENC_B=%d\n",
                  PIN_AZ_IN1, PIN_AZ_IN2, PIN_AZ_ENC_A, PIN_AZ_ENC_B);
    Serial.printf("EL: IN1=%d IN2=%d ENC_A=%d ENC_B=%d\n",
                  PIN_EL_IN1, PIN_EL_IN2, PIN_EL_ENC_A, PIN_EL_ENC_B);
    Serial.flush();

    // 1. NeoPixel init
    neopixel.begin();
    neo_set_all(LED_BR, LED_BR, LED_BR);  // white = booting

    // 2. Buzzer init + boot chime
    buzzer_init();
    static const uint16_t chime[] = {523, 659, 784, 1047};
    for (int i = 0; i < 4; i++) {
        buzzer_tone(chime[i]);
        delay(80);
    }
    buzzer_tone(0);

    // 3. Initialize UART0 for Pi communication
    uart_init(uart0, UART_BAUD);
    gpio_set_function(PIN_UART_TX, GPIO_FUNC_UART);
    gpio_set_function(PIN_UART_RX, GPIO_FUNC_UART);
    uart_set_format(uart0, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(uart0, true);
    Serial.printf("UART0: %d baud\n", UART_BAUD);
    Serial.flush();

    // 4. Init UHF CC1200 on SPI1
    Serial.println("Init UHF CC1200 on SPI1...");
    Serial.flush();
    uhf_ok_flag = init_radio(RADIO_UHF, UHF_SPI_INST, UHF_SPI_BAUD_HZ,
                              UHF_PIN_SCK, UHF_PIN_MOSI, UHF_PIN_MISO, UHF_PIN_CSN,
                              UHF_PIN_RESET, "UHF");

    // 5. ADXL345 IMU init on SPI0 mode 3
    // SPI0 starts unconfigured — ADXL345::init() sets it up in mode 3
    Serial.println("Init ADXL345 IMU on SPI0 (mode 3)...");
    Serial.flush();
    static ADXL345 imu(PIN_IMU_CS, VHF_PIN_SCK, VHF_PIN_MOSI, VHF_PIN_MISO);
    g_imu = &imu;
    g_imu_ok = imu.init();
    Serial.printf("IMU: %s (ID=0x%02X)\n", g_imu_ok ? "OK" : "NOT FOUND", imu.device_id());
    Serial.flush();

    // 6. Motor PWM init (A4950 2-pin mode)
    static MotorPwm az_motor(PIN_AZ_IN1, PIN_AZ_IN2);
    static MotorPwm el_motor(PIN_EL_IN1, PIN_EL_IN2);
    az_motor.init(motor_config::kPwmHz);
    el_motor.init(motor_config::kPwmHz);
    g_az_motor = &az_motor;
    g_el_motor = &el_motor;

    // 7. Encoder init
    static Quadrature az_enc(PIN_AZ_ENC_A, PIN_AZ_ENC_B);
    static Quadrature el_enc(PIN_EL_ENC_A, PIN_EL_ENC_B);
    az_enc.init();
    el_enc.init();
    g_az_enc = &az_enc;
    g_el_enc = &el_enc;
    enable_encoder_irqs();

    // 8. Rotator init
    static Rotator rotator(az_motor, el_motor, az_enc, el_enc);
    rotator.init();
    g_rotator = &rotator;

    // 9. AZ calibration: assume north at boot
    rotator.calibrate_az_zero();

    // 10. EL homing via IMU (blocking, up to 15s)
    // SPI0 is still in mode 3 from ADXL345 init
    if (g_imu_ok) {
        const float initial_el = imu.elevation_deg_averaged(10);
        Serial.printf("EL homing: IMU reads %.1f deg, driving to horizontal...\n", initial_el);
        Serial.flush();
        neo_set_all(0, 0, LED_BR);  // blue = homing

        constexpr float kElHomingDeadband = 1.5f;
        constexpr float kElHomingDuty = 0.25f;
        constexpr uint32_t kElHomingTimeout = 15000;
        const uint32_t homing_start = to_ms_since_boot(get_absolute_time());
        bool homed = false;

        while (to_ms_since_boot(get_absolute_time()) - homing_start < kElHomingTimeout) {
            const float el = imu.elevation_deg_averaged(3);

            if (std::abs(el) <= kElHomingDeadband) {
                el_motor.stop();
                rotator.calibrate_el(0.0f);
                Serial.printf("EL homed to horizontal (IMU: %.1f deg)\n", el);
                homed = true;
                break;
            }

            const float duty = (el > 0) ? -kElHomingDuty : kElHomingDuty;
            el_motor.set(duty);
            watchdog_update();
            sleep_ms(50);
        }

        if (!homed) {
            el_motor.stop();
            const float fallback = imu.elevation_deg_averaged(20);
            rotator.calibrate_el(fallback);
            Serial.printf("EL homing timeout -- calibrated from IMU at %.1f deg\n", fallback);
        }
    } else {
        rotator.calibrate_el(0.0f);
        Serial.println("IMU not found -- EL assumed 0 (place antenna horizontal)");
    }

    // 11. Switch SPI0 to mode 0 for VHF CC1200
    spi_deinit(spi0);
    spi_init(spi0, VHF_SPI_BAUD_HZ);
    spi_set_format(spi0, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
    gpio_set_function(VHF_PIN_SCK, GPIO_FUNC_SPI);
    gpio_set_function(VHF_PIN_MOSI, GPIO_FUNC_SPI);
    gpio_set_function(VHF_PIN_MISO, GPIO_FUNC_SPI);

    // 12. Init VHF CC1200 on SPI0
    Serial.println("Init VHF CC1200 on SPI0...");
    Serial.flush();
    vhf_ok_flag = init_radio(RADIO_VHF, VHF_SPI_INST, VHF_SPI_BAUD_HZ,
                              VHF_PIN_SCK, VHF_PIN_MOSI, VHF_PIN_MISO, VHF_PIN_CSN,
                              VHF_PIN_RESET, "VHF");

    Serial.printf("UHF: %s  VHF: %s\n", uhf_ok_flag ? "OK" : "FAIL", vhf_ok_flag ? "OK" : "FAIL");
    Serial.flush();

    // 13. Init protocol handler with radios + callbacks
    proto_init(radios, RADIO_COUNT);
    proto_set_buzzer_cb(on_buzzer_cmd);

    // Motor callbacks
    proto_set_motor_set_position_cb(cb_motor_set_position);
    proto_set_motor_get_position_cb(cb_motor_get_position);
    proto_set_motor_park_cb(cb_motor_park);
    proto_set_motor_stop_cb(cb_motor_stop);
    proto_set_motor_home_cb(cb_motor_home);
    proto_set_motor_get_status_cb(cb_motor_get_status);
    proto_set_motor_set_gains_cb(cb_motor_set_gains);
    proto_set_motor_zero_encoders_cb(cb_motor_zero_encoders);
    proto_set_imu_read_cb(cb_imu_read);
    proto_set_motor_recal_cb(cb_motor_recal);

    // Register USB Serial as protocol channel 1 (for bench testing)
    proto_set_write_fn(1, [](const uint8_t* data, size_t len) {
        Serial.write(data, len);
        Serial.flush();
    });

    // Enable position event streaming by default
    proto_set_position_streaming(true);

    // Status LED
    if (uhf_ok_flag) {
        neo_set_all(0, LED_BR, 0);  // green = ready
        buzzer_start(0x01);
        Serial.println("STATUS: UHF OK -- NeoPixel GREEN, buzzer READY");
    } else {
        neo_set_all(LED_BR, 0, 0);  // red = UHF init fault
        buzzer_start(0x05);
        Serial.println("STATUS: UHF FAIL -- NeoPixel RED, buzzer ERROR");
    }
    Serial.flush();

    Serial.println("Ready -- UART0 + USB protocol active, motors + RF online");
    Serial.flush();

    hb_next = millis() + 500;
    last_pid_tick = get_absolute_time();
    pos_evt_next = millis() + 100;

    // Hardware watchdog — 8 second timeout
    watchdog_enable(8000, true);
}

void loop()
{
    // 100 Hz motor PID tick
    if (absolute_time_diff_us(last_pid_tick, get_absolute_time()) >= 10000) {
        if (g_rotator) g_rotator->tick();
        last_pid_tick = get_absolute_time();
    }

    // Heartbeat LED + NeoPixel update (~1 Hz)
    if (millis() >= hb_next) {
        hb_state = !hb_state;
        digitalWrite(PIN_LED, hb_state ? HIGH : LOW);
        hb_next = millis() + 500;

        // Update NeoPixel based on state
        if (!uhf_ok_flag) {
            neo_set_all(LED_BR, 0, 0);      // red = UHF fault
        } else if (proto_is_streaming()) {
            neo_set_all(0, 0, LED_BR);       // blue = RX streaming
        } else if (g_rotator && g_rotator->status().moving) {
            neo_set_all(0, LED_BR, LED_BR);  // cyan = motor tracking
        } else {
            neo_set_all(0, LED_BR, 0);       // green = idle/ready
        }
    }

    watchdog_update();

    // Buzzer tone sequencer
    buzzer_tick();

    // Protocol poll (UART + RX streaming)
    proto_poll();

    // Feed USB Serial bytes into protocol handler (channel 1)
    int avail = Serial.available();
    if (avail > 0) {
        uint8_t usb_buf[64];
        int n = Serial.readBytes(usb_buf, (avail > 64) ? 64 : avail);
        if (n > 0)
            proto_feed_bytes(usb_buf, (size_t)n, 1);
    }

    // 10 Hz position event stream
    if (proto_is_position_streaming() && millis() >= pos_evt_next) {
        if (g_rotator) {
            auto st = g_rotator->status();
            proto_send_position_event(st.az_deg, st.el_deg, st.moving ? 2u : 1u);
        }
        pos_evt_next = millis() + 100;
    }

    // Small yield
    delay(1);
}
