# Project Progress

SatNOGS ground station for AetherSpace CubeSat — Stan's Master's thesis, KU Leuven Campus Geel.

Last updated: 2026-04-01

---

## Mechanical Design

- [x] Rotator designed in Onshape (AZ + EL, 2-axis)
- [x] Fully 3D-printable in ASA — all parts printed and assembled
- [x] 1/4"-20 UNC camera tripod mount
- [x] FAPG36-555-EN motors installed (24V, 16 RPM, 516:1 gearbox, 6mm shaft)
- [x] External gearing: AZ 15:40 (2.67:1), EL 40:55 (1.375:1), double helical (15° helix, 1.333mm module, 8mm face)
- [x] 3D-printed deep groove ball bearings (press-in retainers, steel balls)
- [x] PVC pipe elevation axis with integrated screw clamps
- [x] Yagi antenna mounted (LPRS YAGI-434A, 434 MHz, 10 dBi)
- [x] EL motor stripped gear — repaired

## Motor Control PCB (MotorPCB/)

- [x] KiCad 9.0 schematic (13 hierarchical sub-sheets)
- [x] 4-layer PCB routed and fabricated (JLCPCB)
- [x] RP2040 Pico footprint + power (TPS560430XFDBVT 24V-to-5V buck)
- [x] 2x TB6642FG H-bridge motor drivers (UVLO/ISD/TSD protection)
- [x] Quadrature encoder inputs (single Hall sensor IC, 64 CPR)
- [x] ADXL345 accelerometer module on SPI0
- [x] Endstop switch inputs (1 per axis, active-low with pull-up)
- [x] Board assembled and tested
- [x] EL driver bodge wire fix (U2 TB6642FG missing traces to OUT1/OUT2 — patched via NC pins 6/11)
- [x] RC filter caps (C1-C4) on encoder inputs removed (destroyed signal)
- [x] Motor connector swap fixed (AZ↔EL were swapped on board)

## RF HAT PCB (RF_HAT/)

- [x] KiCad 9.0 schematic (from rf_hat_circuit.py + manual edits)
- [x] PCB layout (6-layer, JLCPCB JLC06121H-3313 stackup)
- [x] 50-ohm impedance-controlled RF traces
- [x] Dual CC1200 (UHF 432 MHz + VHF 144 MHz) with 40 MHz XTALs
- [x] UHF matching network — values verified (SWRR122 reference design)
- [x] VHF matching network — values placed (adapted from CC1120EM 169 MHz)
- [x] Pico as SPI host: SPI1→UHF (GP10-13), SPI0→VHF (GP16-19)
- [x] UART (GP0/GP1) to Pi GPIO14/15
- [x] 4x WS2812B NeoPixels on GP5
- [x] Passive buzzer on GP4 via switching transistor (NPN/MOSFET)
- [x] BOM with Farnell + LCSC part numbers
- [x] 89 components, clean netlist
- [x] Pi HAT 40-pin header footprint
- [x] PCB ordered and assembled (JLCPCB)
- [x] UHF CC1200 verified (part 0x20, v1.1)
- [x] LMR51420 buck replaced after blowout — working
- [ ] **VHF matching network needs VNA tuning**

## Rotator Firmware (Firmware/rp2040-satnogs-rotator/)

- [x] PlatformIO project with Arduino-Pico core
- [x] GPIO pin map from MotorPCB netlist
- [x] EasyComm protocol parser (hamlib model 204 compatible)
- [x] All SatNOGS commands: AZ/EL set+query, PARK, STOP, RESET, VE, GS, GE
- [x] Bench test commands: DUTY, TICKS, ZERO, MONITOR, INFO, GAINS, KP/KI/KD, IMU, RECAL, HELP
- [x] PID controller at 100 Hz (Kp=0.15, Ki=0.03, Kd=0.02)
- [x] Deadband 0.05°, max duty 95%, duty output filter (alpha=0.05)
- [x] D-term low-pass filter (alpha=0.15) to suppress encoder noise
- [x] I-term anti-windup clamp (5 deg-seconds)
- [x] Quadrature decoder (4x, ISR-driven, Gray-code lookup table)
- [x] TB6642FG PWM driver (20 kHz, IN1/IN2 direction mode)
- [x] ADXL345 accelerometer driver (SPI0, elevation from gravity vector)
- [x] IMU-based EL homing at boot (drives to horizontal, zeros encoder, 15s timeout)
- [x] RECAL command (re-read IMU and recalibrate EL mid-operation)
- [x] AZ shortest-path wrapping (`choose_wrapped_target()` — picks nearest equivalent)
- [x] AZ encoder recentering (`recenter_az()` — re-zeros after settling near ±360°)
- [x] Motor inversion flags (kAzInvert, kElInvert — flip both position AND motor output)
- [x] Runaway detection (emergency stop if position > soft limits + 50°)
- [x] Soft limits: AZ ±360°, EL 0-180°
- [x] Endstop safety gates (stop motor if driving into pressed endstop)
- [x] TB6642FG ALERT pin fault monitoring (UVLO/ISD/TSD)
- [x] GP25 heartbeat LED (1 Hz, Pico onboard)
- [x] Post-build script copies firmware.uf2 to repo root
- [x] Firmware builds clean, flashed and running
- [ ] Endstop homing routine implemented but disabled (`kUseEndstopHoming = false`) — needs hardware testing

## RF HAT Firmware (Firmware/rp2040-rf-hat/)

- [x] PlatformIO project with Arduino-Pico core
- [x] Ported from Robbe's CC1200RXFrontend (Pico SDK + CMake → PlatformIO + Arduino)
- [x] CC1200 SPI driver (lib/cc1200_driver/) — register, FIFO, strobe, extended access
- [x] Binary COBS protocol over UART0 (GP0/GP1, 115200 baud)
- [x] CRC-16/CCITT-FALSE integrity check
- [x] Dual CC1200 support: MSG_SELECT_RADIO (0x40) switches UHF (idx 0) / VHF (idx 1)
- [x] Full protocol: PING, GET_INFO, SET_STATE, SET_STREAMING, GET_METRICS
- [x] Register I/O: READ/WRITE_REG, READ/WRITE_EXT
- [x] FIFO: RX_READ, TX_WRITE, TX_SEND, TX_FLUSH
- [x] SmartRF profile system: PROFILE_CLEAR/BEGIN/CHUNK/APPLY (bulk register config)
- [x] RX streaming (auto-push EVT_RX_DATA events from FIFO)
- [x] RSSI calculation (12-bit signed, dBm conversion)
- [x] MSG_BUZZER (0x50) — non-blocking tone sequencer (5 patterns: ready/AOS/LOS/packet/error)
- [x] 4x WS2812B NeoPixels on GP5 (all driven as 1 colour)
- [x] Hardware PWM buzzer on GP4
- [x] USB serial kept for debug output (separate from UART protocol)
- [x] Bug fixes from Robbe's code: send_frame buffer overflow, profile_apply IDLE confirm, burst delay margin, strobe range validation, register address check
- [x] Post-build script copies rf_hat_firmware.uf2 to repo root
- [x] Firmware builds clean (3.7% flash, 5.3% RAM)
- [x] Tested on actual RF HAT hardware — UART comms verified, PING/PONG working

## Pi Software Stack (Pi/)

### rf_hat.py — CC1200 Protocol Library
- [x] COBS framing + CRC-16 matching firmware protocol
- [x] CC1200Link class with threaded RX loop and event queue
- [x] All protocol commands implemented
- [x] Frequency helpers: freq_to_regs(), regs_to_freq(), doppler_shift()
- [x] SmartRF profile parser (reads TI SmartRF Studio exports)
- [x] MSG_BUZZER support (CC1200Link.buzzer() method)

### capture.py — UHF Packet Capture
- [x] Standalone UHF receiver (433 MHz)
- [x] Flags: --freq, --log, --port, --duration

### transmit.py — VHF Packet Transmitter
- [x] Standalone VHF transmitter (145.9 MHz)
- [x] Flags: --ascii, --file, --freq, --repeat, --interval, --power

### station.py — Full Station Controller
- [x] Connects to rotctld (TCP 4533) for rotator control
- [x] Opens RF HAT via UART, loads SmartRF profile
- [x] TLE fetch from CelesTrak + orbit prediction (PyEphem)
- [x] AZ/EL tracking at 10 Hz during pass
- [x] UHF RX streaming during pass
- [x] Doppler correction at 2 Hz (--doppler flag)
- [x] Optional VHF TX after pass (--tx flag)
- [x] --list flag to show upcoming passes
- [x] --no-rf flag for rotator-only testing
- [x] --daemon mode (waits for ~/station.conf, loops passes continuously)
- [x] --log flag for logging everything to file
- [x] Writes ~/.station_status.json at 1 Hz (IPC with dashboard)
- [x] Station location priority: CLI flags > ~/station.conf > hardcoded defaults

### dashboard.py — Web Dashboard
- [x] HTTPS server (auto-generated self-signed cert in ~/.station_ssl/)
- [x] Phone-first responsive design (standard library only, no Flask)
- [x] Phone GPS via browser Geolocation API → POST /api/location → ~/station.conf
- [x] Live status display (AZ/EL, RF metrics, pass progress, system health)
- [x] Park Antenna button (→ rotctld)
- [x] Shutdown Pi button (park + sudo shutdown)
- [x] --no-ssl flag for HTTP-only mode
- [x] Auto-refresh every 1 second

### buzzer.py — Audio Feedback
- [x] Sends pattern IDs to RF HAT Pico over UART (via CC1200Link)
- [x] No RPi.GPIO dependency — Pico handles PWM locally
- [x] Graceful fallback when no link available (--no-rf mode)
- [x] Patterns: ready, AOS, LOS, packet, error

### SmartRF Configs (configs/)
- [x] UHF profile (smartrf_uhf_435.txt) — 435 MHz, 2-GFSK, ~2.4 kbps, verified from SmartRF Studio
- [x] VHF profile (smartrf_vhf_145.txt) — 145.9 MHz, adapted from UHF (frequency regs only)
- [x] CC1200 register name→address mapping (cc1200_regmap_profile.json)
- [ ] **VHF SmartRF profile needs tuning** (run SmartRF Studio for 145 MHz or verify on VNA)

### Services (services/)
- [x] dashboard.service — systemd auto-start
- [x] station.service — systemd auto-start (daemon mode)
- [x] install.sh — one-command installer (pip deps + enable services)
- [x] rotctld.service exists on the Pi already

### requirements.txt
- [x] pyserial + ephem (no RPi.GPIO needed)

### PC/Pi Test Tools (Firmware/rp2040-rf-hat/tools/)
- [x] cc1200_gui.py — CustomTkinter GUI for interactive CC1200 control
- [x] cc1200_diag.py — CLI diagnostic tool
- [x] generate_regmap.py — register map generator from TI header
- [x] smartrf_config.txt — SmartRF Studio register export

## Raspberry Pi Setup

- [x] Pi 3 Model A+ running Bookworm arm64
- [x] WiFi configured via NetworkManager (nmcli)
- [x] SSH access (pi/satnogs123)
- [x] libhamlib-utils installed (rotctld, rotctl)
- [x] rotctld.service running (model 204, /dev/ttyACM0, 115200, port 4533)
- [x] Python 3 + pip installed
- [x] SatNOGS client framework installed (Docker + Ansible)
- [x] UART enabled (enable_uart=1 + dtoverlay=disable-bt)
- [x] RF HAT tested over /dev/serial0 — COBS/CRC-16 verified
- [x] SatNOGS station registered (station ID 4712, "Aether-Basestation")
- [x] SiDS telemetry submission verified (201 OK)
- [ ] **udev rules not yet created** (persistent /dev/satnogs-rotator symlink)

## Python Tracking Scripts (repo root)

- [x] track_satellite.py — live satellite tracker (direct USB serial or via rotctld)
- [x] demo_tracking.py — thesis demo (axis showcase + simulated passes, --passes-only flag)

## Satellite Tracking — Verified Working

- [x] NOAA 11 pass tracked with <0.3° AZ/EL error throughout 15-min pass
- [x] AZ shortest-path wrapping: north-crossing (350°→10°) takes short 20° path
- [x] PARK from any position: takes shortest path back to 0°
- [x] EL IMU homing: drives antenna to horizontal at boot, zeros encoder
- [x] Runaway detection: emergency stop tested and verified
- [x] All 10 edge-case tests passed
- [x] Speed calibration: 35°/s at 95% duty, linear scaling

## Critical Bugs Found & Fixed

- [x] EL runaway — kElInvert only inverted position, not motor output → positive feedback. Fixed: invert PID output too.
- [x] AZ full-rotation on north crossing — Python sent az%360, firmware took long path. Fixed: choose_wrapped_target().
- [x] PARK unwinding wrong direction — from AZ -310° went 310° instead of 50°. Fixed: choose_wrapped_target() for park.
- [x] EL homing direction wrong — drove EL away from horizontal. Fixed: flipped duty sign.
- [x] EL direction inverted — antenna pointed down when EL increased. Fixed: kElInvert=true.
- [x] Serial port auto-detect — hardcoded list broke when port changed. Fixed: glob.glob().
- [x] Pi undervoltage from bad micro-USB cable — caused USB brownouts. Fixed: use quality cable.

## Hardware Bring-up Notes

- [x] Motor connector pinout reversed — fixed
- [x] Motor connectors physically swapped (AZ↔EL) — swapped back
- [x] RC filter caps on encoder inputs destroyed signal — removed
- [x] EL driver missing PCB traces — bodge wires applied
- [x] Encoder pins A/B swapped in firmware for correct count direction
- [x] IMU ~10° offset vs encoder at boot — normal (mounting angle), RECAL corrects

## Documentation

- [x] README.md — project overview, system diagram, quick start, SatNOGS integration
- [x] CLAUDE.md — concise project reference for AI assistant sessions
- [x] Firmware/README.md — full system architecture, data flow, build instructions
- [x] Firmware/rp2040-satnogs-rotator/README.md — detailed rotator firmware docs (PID, protocol, GPIO, config)
- [x] Firmware/rp2040-rf-hat/README.md — detailed RF HAT firmware docs (COBS protocol, CC1200 driver, message types)
- [x] Pi/README.md — complete Pi setup guide (UART, rotctld, Python tools, troubleshooting)
- [x] Pi/user_guide.md — field deployment guide for AetherSpace team
- [x] RF_HAT/README.md — RF HAT hardware docs (matching networks, stackup, impedance, BOM sources)
- [x] Hardware/README.md — motor wiring, datasheets
- [x] ThesisPaper/README.md — LaTeX build instructions, file structure, writing workflow

## Thesis Paper (ThesisPaper/)

- [x] KU Leuven thesis.cls template set up
- [x] Makefile (tectonic / latexmk / pdflatex auto-detect)
- [x] bibliography.bib with 80 references
- [x] draft.md — full content draft in Markdown
- [x] All 12 chapter files created with section headings and TODO comments
- [x] 5 appendices created (pinout, protocol, PID, SmartRF, speed test)
- [x] Custom LaTeX commands (\EUR, \degree, \code, \file, \SI, \degC)
- [x] Samenvatting (Dutch) + Abstract (English) + Abbreviations
- [x] Chapter 1 (Introduction) written in LaTeX
- [x] Chapter 2 (Background and Literature Review) written in LaTeX
- [ ] **Chapters 3-12 prose not yet written** — headings + TODO comments exist
- [ ] **Figures not yet added** (images/figures/ and images/photos/ directories exist but empty)
- [ ] **Thesis not yet compiled to PDF**

## Field Deployment UX

- [x] Zero-SSH workflow designed and documented
- [x] Systemd services for auto-start on boot (rotctld + dashboard + station daemon)
- [x] Phone GPS → ~/station.conf → automatic station location
- [x] Dashboard HTTPS for browser Geolocation API
- [x] Park + Shutdown buttons on dashboard
- [x] NeoPixel status on RF HAT Pico (4x WS2812B)
- [x] Buzzer audio cues (ready, AOS, LOS, packet, error)
- [x] First outdoor field test completed (2026-04-01) — EL tracking <0.5° error, AZ issues at zenith
- [ ] **Full field test with packet reception pending** (per-sat freq/mod config needed)

---

## Summary of Remaining Work

### Hardware
1. [x] ~~Order RF HAT PCBs~~ — assembled and working
2. [x] ~~Assemble RF HAT PCB~~ — UHF verified, LMR51420 replaced
3. [ ] VHF matching network VNA tuning

### Pi Integration
4. [x] ~~Enable UART on Pi~~ — done
5. [x] ~~Test RF HAT Pico over /dev/serial0~~ — COBS/CRC-16 verified
6. [ ] Create udev rules for persistent device names
7. [ ] Test endstop homing (kUseEndstopHoming → true)

### SatNOGS Network
8. [x] ~~Register station~~ — ID 4712 "Aether-Basestation"
9. [x] ~~SiDS telemetry submission~~ — verified (201 OK)
10. [ ] RTL-SDR integration for network heartbeat (optional)

### RF & Tracking (field test blockers — FIXED 2026-04-01)
11. [x] ~~AZ timeout resets target~~ — halt_motors() preserves targets
12. [x] ~~PID integral anti-windup~~ — saturation-based clamping
13. [x] ~~Duty smoothing too sluggish~~ — alpha 0.05 → 0.15
14. [x] ~~RotctlClient socket leak~~ — close() before reconnect
15. [x] ~~Packets list unbounded growth~~ — clear between passes

### Remaining
16. [ ] VHF SmartRF profile tuning (SmartRF Studio or VNA)
17. [ ] Full field test with packet reception (per-sat CC1200 reconfig)
18. [ ] Pointing accuracy measurements (step response, backlash, wind load)
19. [ ] Capture thesis plots/logs (encoder ticks, duty, AZ/EL error vs time)
20. [ ] Write thesis chapter prose (ch03-12 + 5 appendices)
21. [ ] Add thesis figures (photos, plots, screenshots, diagrams)
22. [ ] Compile thesis PDF
