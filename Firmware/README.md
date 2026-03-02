# Firmware

RP2040 rotator controller firmware for the [SatNOGS Basestation](../README.md). Speaks EasyComm over USB serial for plug-and-play hamlib compatibility.

## Build & flash

```sh
cd rp2040-satnogs-rotator
pip install platformio
pio run                  # build → firmware.uf2
pio run -t upload        # flash via BOOTSEL
pio device monitor       # USB serial console (115200 baud)
```

The compiled `firmware.uf2` is automatically copied to the repo root after each build.

## Project structure

| Path | Description |
|------|-------------|
| [`rp2040-satnogs-rotator/`](rp2040-satnogs-rotator/) | PlatformIO project (Arduino-Pico core + raw Pico SDK) |
| [`rp2040-satnogs-rotator/src/`](rp2040-satnogs-rotator/src/) | Source files (main, PID, protocol, motor driver, encoder, IMU) |
| [`rp2040-satnogs-rotator/README.md`](rp2040-satnogs-rotator/README.md) | Full firmware documentation (boot sequence, PID details, all commands, config) |

See the [firmware README](rp2040-satnogs-rotator/README.md) for detailed documentation.
