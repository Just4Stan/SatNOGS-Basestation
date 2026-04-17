# rp2040-motor-rf-hat — v2 scaffold

**Status:** scaffold only. Target board (`MOTOR_RF_HAT/`) is design-complete and routed but **not yet fabricated**. This firmware is **not compiled, not flashed, and not currently under active development.**

## Purpose

When the v2 MOTOR_RF_HAT PCB is built, this tree locks in:

- The v2 pin plan (`src/config.h`) — AZ/EL motor drivers on GP2/3 (A4950, 2-pin PWM), AZ/EL encoders on GP14/15 and GP21/... , SPI0 shared between VHF CC1200 and ADXL345 IMU.
- The combined protocol surface — EasyComm rotator commands + COBS RF HAT protocol on a single UART to the Pi.
- The combined-board main loop scaffolding.

## What's missing (do on v2 board arrival)

1. Merge the current `../rp2040-rf-hat/src/` into this tree:
   - `ax25_decode.c/h` (AX.25 G3RUH software decoder)
   - `serial_rx.pio.h` (PIO synchronous bitstream capture)
   - Any post-April-12 changes to `proto.c`, `main.cpp`, `config.h`.
2. Merge the current `../rp2040-satnogs-rotator/src/` rotator code:
   - `rotator.cpp/h`, `motor_pwm.cpp/h`, `quadrature.cpp/h`, `adxl345.cpp/h`, `satnogs_protocol.cpp/h`, `status_led.cpp/h`, `pins.h`.
3. Resolve any pin conflicts against this tree's v2 `config.h`.
4. Build with `pio run`, flash via BOOTSEL, verify on the v2 board.

## Why not keep in sync continuously?

Keeping a half-merged fork in lockstep with two evolving firmwares (rotator + RF HAT) before the target hardware exists is more technical debt than leverage — v1 firmware improvements were adopted on v1 boards in the field, and re-merging once at v2 board arrival is less error-prone than continuously triple-maintaining. This directory therefore represents the **most recent frozen pin plan** for v2, not a buildable image.
