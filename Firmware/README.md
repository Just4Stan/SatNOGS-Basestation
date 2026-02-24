# Firmware

RP2040 rotator controller firmware. See the [main README](../README.md) for system overview.

## Projects

- `rp2040-satnogs-rotator/` - PlatformIO project (Arduino-Pico core with full Pico SDK access)

## Build + flash

```sh
cd rp2040-satnogs-rotator
pio run                  # builds firmware.uf2
pio run -t upload        # flash via BOOTSEL
pio device monitor       # USB serial console
```

The compiled `firmware.uf2` is automatically copied to the repo root after each build.
