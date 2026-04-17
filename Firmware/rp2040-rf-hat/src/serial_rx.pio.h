// -------------------------------------------------- //
// CC1200 synchronous serial RX — PIO program          //
// -------------------------------------------------- //
//
// Samples CC1200 SERIAL_RX data on SERIAL_CLK rising edge.
// Uses `in pins, 1` to directly read the data pin — no jmp_pin needed.
//
// CC1200 GPIO0 → Pico GP7 (SERIAL_RX data)   — in_base pin 0
// CC1200 GPIO2 → Pico GP9 (SERIAL_CLK clock) — in_base pin 2
//
// in_base = GP7 (data pin, lower GPIO number)
// Clock is at in_base + (GP9 - GP7) = in_base + 2
//
// .program serial_rx
// .wrap_target
//     wait 1 pin 2        ; wait for SERIAL_CLK rising edge (in_base+2)
//     in pins, 1           ; shift in data bit from in_base (GP7) — bit 0 of pins
//     wait 0 pin 2         ; wait for SERIAL_CLK falling edge
// .wrap

#pragma once

#include "hardware/pio.h"

static const uint16_t serial_rx_program_instructions[] = {
    //     .wrap_target
    0x20a2, // 0: wait 1 pin 2       ; clock high
    0x4001, // 1: in pins, 1          ; read data pin (in_base = GP7)
    0x2022, // 2: wait 0 pin 2       ; clock low
    //     .wrap
};

static const struct pio_program serial_rx_program = {
    .instructions = serial_rx_program_instructions,
    .length = 3,
    .origin = -1,
};

static inline pio_sm_config serial_rx_program_get_default_config(uint offset) {
    pio_sm_config c = pio_get_default_sm_config();
    sm_config_set_wrap(&c, offset + 0, offset + 2);
    return c;
}

/**
 * Initialise PIO state machine for CC1200 synchronous serial RX.
 *
 * @param pio       PIO instance (pio0 or pio1)
 * @param sm        State machine index (0-3)
 * @param offset    Program load offset (from pio_add_program)
 * @param pin_rx    GPIO for SERIAL_RX data  (CC1200 GPIO0 → Pico GP7)
 * @param pin_clk   GPIO for SERIAL_CLK      (CC1200 GPIO2 → Pico GP9)
 */
static inline void serial_rx_program_init(PIO pio, uint sm, uint offset,
                                           uint pin_rx, uint pin_clk) {
    // Configure pins as PIO inputs
    pio_gpio_init(pio, pin_rx);
    pio_gpio_init(pio, pin_clk);
    pio_sm_set_consecutive_pindirs(pio, sm, pin_rx, 1, false);
    pio_sm_set_consecutive_pindirs(pio, sm, pin_clk, 1, false);

    pio_sm_config c = serial_rx_program_get_default_config(offset);

    // IN pin base = data pin. `in pins, 1` reads bit 0 relative to in_base.
    // `wait N pin M` also uses in_base — pin M = in_base + M.
    sm_config_set_in_pins(&c, pin_rx);

    // Shift left, autopush every 8 bits → byte in ISR[31:24]
    sm_config_set_in_shift(&c, false, true, 8);

    // No clock divider — PIO runs at sysclk, gated by wait instructions
    sm_config_set_clkdiv(&c, 1.0f);

    // Calculate clock pin offset from in_base
    // pin_clk MUST be > pin_rx for this to work with `wait N pin offset`
    uint clk_offset = pin_clk - pin_rx;

    // Patch PIO instructions with correct clock pin offset
    pio->instr_mem[offset + 0] = 0x20a0 | (clk_offset & 0x1F);  // wait 1 pin <clk_offset>
    pio->instr_mem[offset + 1] = 0x4001;                          // in pins, 1 (always pin 0 = data)
    pio->instr_mem[offset + 2] = 0x2020 | (clk_offset & 0x1F);  // wait 0 pin <clk_offset>

    // Init SM
    pio_sm_init(pio, sm, offset, &c);

    // No X register preload needed — `in pins` reads directly

    // Don't enable yet — caller does pio_sm_set_enabled() when ready
}
