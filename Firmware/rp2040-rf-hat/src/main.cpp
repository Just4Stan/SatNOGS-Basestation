// RF HAT Firmware — Dual CC1200 controller for SatNOGS ground station
// Pico controls UHF (432 MHz) + VHF (144 MHz) CC1200 transceivers
// Communication: Binary COBS protocol over UART0 (GP0/GP1) to Pi 3A+
//
// Based on CC1200RXFrontend by Robbe Lehaen, adapted for dual-radio RF HAT.

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include "hardware/spi.h"
#include "hardware/uart.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"

extern "C" {
#include "config.h"
#include "cc1200.h"
#include "proto.h"
}

// Two CC1200 instances
static cc1200_t radios[RADIO_COUNT];

// NeoPixel status LEDs on GP3 (4 LEDs, driven as 1 colour for now)
static Adafruit_NeoPixel neopixel(NEOPIXEL_COUNT, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);
static constexpr uint8_t LED_BR = 40;  // brightness (0-255)

// Heartbeat state
static uint32_t hb_next = 0;
static bool hb_state = false;
static bool uhf_ok_flag = false;
static bool vhf_ok_flag = false;

// ---------------------------------------------------------------------------
// Buzzer — non-blocking PWM tone sequencer on GP2
// ---------------------------------------------------------------------------
// Pattern IDs (match Pi-side buzzer.py):
//   0x01 = ready    (triple beep 1000 Hz)
//   0x02 = AOS      (rising 800 → 1200 Hz)
//   0x03 = LOS      (falling 1200 → 800 Hz)
//   0x04 = packet   (short 1500 Hz)
//   0x05 = error    (low 400 Hz)

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
        case 0x05: bz_seq = SEQ_ERROR;  break;
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
        // End of sequence
        buzzer_tone(0);
        bz_seq = nullptr;
        return;
    }
    buzzer_tone(bz_seq[bz_step].freq_hz);
    bz_step_end = millis() + bz_seq[bz_step].dur_ms;
}

// Proto callback for MSG_BUZZER
static void on_buzzer_cmd(uint8_t pattern)
{
    buzzer_start(pattern);
}

// ---------------------------------------------------------------------------
// NeoPixel helpers — set all 4 LEDs to the same colour
// ---------------------------------------------------------------------------
static void neo_set_all(uint8_t r, uint8_t g, uint8_t b)
{
    uint32_t c = neopixel.Color(r, g, b);
    for (int i = 0; i < NEOPIXEL_COUNT; i++)
        neopixel.setPixelColor(i, c);
    neopixel.show();
}

// ---------------------------------------------------------------------------
// Radio init
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

    // Verify part number (CC1200 = 0x20)
    uint8_t part = 0, ver = 0;
    cc1200_read_ext(&radios[idx], 0x8F, &part);
    cc1200_read_ext(&radios[idx], 0x90, &ver);

    Serial.printf("[%s] CC1200 part=0x%02X ver=0x%02X %s\n",
                  label, part, ver, (part == 0x20) ? "OK" : "FAIL");

    return (part == 0x20);
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
    digitalWrite(PIN_LED, LOW);

    // NeoPixel — white during init
    neopixel.begin();
    neo_set_all(LED_BR, LED_BR, LED_BR);

    // Buzzer
    buzzer_init();

    // Initialize UART0 for Pi communication (GP0 TX, GP1 RX)
    uart_init(uart0, UART_BAUD);
    gpio_set_function(PIN_UART_TX, GPIO_FUNC_UART);
    gpio_set_function(PIN_UART_RX, GPIO_FUNC_UART);
    uart_set_format(uart0, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(uart0, true);

    delay(500);  // Let USB enumerate
    Serial.println("=== RF HAT Firmware ===");
    Serial.println("Dual CC1200 controller for SatNOGS");
    Serial.printf("UART0: %d baud (GP%d TX, GP%d RX)\n", UART_BAUD, PIN_UART_TX, PIN_UART_RX);
    Serial.printf("NeoPixel: %d LEDs on GP%d\n", NEOPIXEL_COUNT, PIN_NEOPIXEL);
    Serial.printf("Buzzer: GP%d (PWM)\n", PIN_BUZZER);

    // Init UHF CC1200 (SPI1)
    uhf_ok_flag = init_radio(RADIO_UHF, UHF_SPI_INST, UHF_SPI_BAUD_HZ,
                              UHF_PIN_SCK, UHF_PIN_MOSI, UHF_PIN_MISO, UHF_PIN_CSN,
                              UHF_PIN_RESET, "UHF");

    // Init VHF CC1200 (SPI0)
    vhf_ok_flag = init_radio(RADIO_VHF, VHF_SPI_INST, VHF_SPI_BAUD_HZ,
                              VHF_PIN_SCK, VHF_PIN_MOSI, VHF_PIN_MISO, VHF_PIN_CSN,
                              VHF_PIN_RESET, "VHF");

    Serial.printf("UHF: %s  VHF: %s\n", uhf_ok_flag ? "OK" : "FAIL", vhf_ok_flag ? "OK" : "FAIL");

    // Init protocol handler with both radios + buzzer callback
    proto_init(radios, RADIO_COUNT);
    proto_set_buzzer_cb(on_buzzer_cmd);

    // NeoPixel → green if both radios OK, red if either failed
    if (uhf_ok_flag && vhf_ok_flag) {
        neo_set_all(0, LED_BR, 0);  // green = ready
        buzzer_start(0x01);         // triple beep = ready
    } else {
        neo_set_all(LED_BR, 0, 0);  // red = init fault
        buzzer_start(0x05);         // error beep
    }

    Serial.println("Ready — listening on UART0 for commands");

    hb_next = millis() + 500;
}

void loop()
{
    // Heartbeat LED + NeoPixel update (~1 Hz)
    if (millis() >= hb_next) {
        hb_state = !hb_state;
        digitalWrite(PIN_LED, hb_state ? HIGH : LOW);
        hb_next = millis() + 500;

        // Update NeoPixel based on protocol state
        if (!uhf_ok_flag || !vhf_ok_flag) {
            neo_set_all(LED_BR, 0, 0);   // red = init fault
        } else if (proto_is_streaming()) {
            neo_set_all(0, 0, LED_BR);   // blue = RX streaming
        } else {
            neo_set_all(0, LED_BR, 0);   // green = idle/ready
        }
    }

    // Advance buzzer tone sequencer
    buzzer_tick();

    // Process protocol commands from Pi (UART)
    proto_poll();

    // Small yield
    delay(1);
}
