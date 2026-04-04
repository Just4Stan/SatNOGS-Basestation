# MOTOR_RF_HAT Changelist

Combined Motor Control + RF HAT board. Merges MotorPCB functionality onto
the RF HAT Pi HAT, single RP2040 Pico, single UART to Pi.

## GPIO Pin Map (new combined board)

| GPIO | Function | Peripheral | Notes |
|------|----------|------------|-------|
| GP0 | UART TX | UART0 | → Pi GPIO15 (RXD) |
| GP1 | UART RX | UART0 | ← Pi GPIO14 (TXD) |
| GP2 | AZ Motor IN1 | PWM1A | A4950 — PWM this pin for forward |
| GP3 | AZ Motor IN2 | PWM1B | A4950 — PWM this pin for reverse |
| GP4 | NeoPixel data | PIO | WS2812B chain (4x or 2x) |
| GP5 | Buzzer | PWM2B | Passive buzzer via MOSFET |
| GP6 | UHF GPIO3 | GPIO | CC1200 CCA (optional, can cut) |
| GP7 | UHF GPIO0 | GPIO | CC1200 PKT_SYNC_RXTX |
| GP8 | UHF RESET | GPIO out | CC1200 active-low reset |
| GP9 | UHF GPIO2 | GPIO | CC1200 MARC status (optional, can cut) |
| GP10 | UHF SPI SCK | SPI1 | CC1200 UHF 5 MHz |
| GP11 | UHF SPI MOSI | SPI1 | |
| GP12 | UHF SPI MISO | SPI1 | |
| GP13 | UHF SPI CSN | SPI1 | |
| GP14 | AZ Encoder A | GPIO IRQ | Quadrature, ISR on rise/fall |
| GP15 | AZ Encoder B | GPIO IRQ | |
| GP16 | VHF SPI MISO | SPI0 | Shared bus with IMU (mode switch) |
| GP17 | VHF SPI CSN | SPI0 CS | |
| GP18 | VHF SPI SCK | SPI0 | Shared bus with IMU |
| GP19 | VHF SPI MOSI | SPI0 | Shared bus with IMU |
| GP20 | VHF RESET | GPIO out | |
| GP21 | EL Encoder A | GPIO IRQ | Quadrature, ISR on rise/fall |
| GP22 | EL Encoder B | GPIO IRQ | |
| GP23 | EL Motor IN1 | PWM3B | A4950 — PWM this pin for forward |
| GP24 | EL Motor IN2 | PWM4A | A4950 — PWM this pin for reverse |
| GP25 | Onboard LED | GPIO out | Pico module LED (heartbeat) |
| GP26 | IMU SPI CS | GPIO out | ADXL345, manual CS, shared SPI0 bus |
| GP27 | VHF GPIO0 | GPIO | (optional, can cut) |
| GP28 | VHF GPIO2 | GPIO | (optional, can cut) |
| GP29 | SPARE | — | Available for debug/future |

**Total: 29 used + 1 spare (GP29)**

### SPI0 bus sharing (GP16/18/19)

SPI0 is shared between the ADXL345 IMU and VHF CC1200. They use different
SPI modes (IMU=mode 3, CC1200=mode 0) but never operate simultaneously:

1. Boot: SPI0 configured mode 3 @ 1 MHz → ADXL345 homing
2. After homing: reconfigure SPI0 to mode 0 @ 5 MHz → VHF CC1200 active
3. RECAL command: temporarily switch to mode 3, read IMU, switch back

Both devices have separate CS pins (GP26 for IMU, GP17 for VHF).

### Optional GPIO cuts (if more pins needed)

These CC1200 GPIOx pins are NOT used in current firmware (polling-based):
- GP6 (UHF GPIO3): save 1 pin
- GP9 (UHF GPIO2): save 1 pin
- GP27 (VHF GPIO0): save 1 pin
- GP28 (VHF GPIO2): save 1 pin

Cutting all 4 frees GP6, GP9, GP27, GP28, GP29 = **5 spare**.

---

## Schematic Changes (rf_hat_circuit.py)

### ADD: Motor driver section (2x A4950ELJTR-T)

Each A4950 needs:
- VBB (pin 5): 24V motor supply, 100nF + 10µF bypass
- GND (pin 1): ground
- IN1 (pin 3): RP2040 GPIO (PWM-capable)
- IN2 (pin 2): RP2040 GPIO (PWM-capable)
- OUT1 (pin 6): motor terminal A
- OUT2 (pin 8): motor terminal B
- VREF (pin 4): tie to VBB via 10k divider for ~3A limit, or tie to VBB to disable current limiting
- LSS (pin 7): tie to GND (no sense resistor)
- PAD: solder to ground pour (thermal)

Bypass caps per driver: 100nF ceramic + 10µF electrolytic on VBB pin.

### ADD: Motor/encoder connectors

- J_AZ_MOTOR: 2-pin JST-XH (motor A+/A-)
- J_EL_MOTOR: 2-pin JST-XH (motor B+/B-)
- J_AZ_ENC: 4-pin JST-XH (VCC, GND, A, B)
- J_EL_ENC: 4-pin JST-XH (VCC, GND, A, B)

Encoder power: 3.3V from Pico (encoders are 3.3V compatible on FAPG36-555-EN).

### ADD: ADXL345 IMU (SPI, SOIC-14 or LGA-14)

- SPI0 shared bus: SCK=GP18, MOSI=GP19, MISO=GP16
- CS: GP26 (manual, active-low, 10k pull-up)
- VCC: 3.3V, bypass 100nF + 10µF
- INT1/INT2: not connected (polling only)
- SDO/ALT ADDRESS: tie to VCC for SPI mode (vs I2C)

### MODIFY: Power architecture

**Remove**: Second LMR51420 buck (only need one)

**Keep**: One LMR51420 (24V→5V, 2A) for Pico + CC1200s + NeoPixels + buzzer

**Add**: 24V direct rail to A4950 VBB pins with:
- LC filter between 24V input and CC1200 AVDD domain
- 100µF bulk electrolytic on 24V motor rail
- Ferrite bead (600Ω @ 100MHz) isolating motor 24V from buck 24V input
- 2x 100nF + 10µF per A4950 VBB

**Keep**: XT30PW-M 24V input connector (single, shared)

### MODIFY: NeoPixel

Change from WS2812B-2020 to **WS2812B-5050** (larger, easier to solder).
Reduce to 2 LEDs (motor status + RF status) or keep 4 with distinct roles:
- LED1: Motor/tracking status (green=idle, blue=tracking, cyan=on-target)
- LED2: RF status (green=UHF OK, red=fault)
- LED3: Pass indicator (off=no pass, blue=tracking, yellow=AOS imminent)
- LED4: System (white blink = heartbeat)

Move data pin from GP5 to **GP4**.

### MODIFY: Buzzer

Move from GP5 to **GP5** (unchanged). Keep BSS138 MOSFET driver circuit.

### REMOVE (optional)

Nothing mandatory to remove. VHF CC1200 stays. Second LMR51420 removed.

---

## Firmware Changes

### New unified firmware: `Firmware/rp2040-motor-rf-hat/`

Fork from `Firmware/rp2040-rf-hat/`, add motor control modules from
`Firmware/rp2040-satnogs-rotator/`.

### Files to copy from rp2040-satnogs-rotator → rp2040-motor-rf-hat

| File | Changes needed |
|------|----------------|
| `motor_pwm.h/cpp` | Refactor from 3-pin (IN1,IN2,PWM) to 2-pin (IN1,IN2). PWM whichever IN pin is active. Remove dedicated PWM GPIO. |
| `quadrature.h/cpp` | Pin numbers change only (GP14/15 for AZ, GP21/22 for EL) |
| `rotator.h/cpp` | No changes — uses MotorPwm and Quadrature interfaces |
| `adxl345.h/cpp` | Add SPI mode switching before/after access |
| `satnogs_protocol.h/cpp` | **DELETE** — replaced by COBS proto.c extensions |
| `status_led.h/cpp` | Merge into NeoPixel code in main.cpp |

### motor_pwm.cpp refactor (2-pin A4950 mode)

```cpp
// OLD (TB6642FG with separate PWM pin):
//   IN1=direction, IN2=direction, PWM=speed
//
// NEW (A4950 — PWM on the active IN pin):
//   Forward: IN1=PWM(duty), IN2=LOW
//   Reverse: IN1=LOW, IN2=PWM(duty)
//   Coast:   IN1=LOW, IN2=LOW

class MotorPwm {
public:
  MotorPwm(uint in1_gpio, uint in2_gpio);  // no pwm_gpio param
  void init(uint32_t pwm_hz);
  void set(float duty);  // -1..1, sign=direction
  void stop();
private:
  uint in1_gpio_, in2_gpio_;
  uint in1_slice_, in1_chan_;
  uint in2_slice_, in2_chan_;
  uint16_t pwm_wrap_;
};
```

Both IN1 and IN2 are configured as PWM outputs at init. To drive forward,
set IN1 duty to desired level and IN2 duty to 0. Vice versa for reverse.
No GPIO function switching needed — just set PWM levels.

### config.h changes

Replace pin defines with combined map. Add motor + encoder + IMU pins:

```c
// ---- Motor drivers (A4950) ----
#define AZ_PIN_IN1       2
#define AZ_PIN_IN2       3
#define EL_PIN_IN1      23
#define EL_PIN_IN2      24

// ---- Encoders ----
#define AZ_PIN_ENC_A    14
#define AZ_PIN_ENC_B    15
#define EL_PIN_ENC_A    21
#define EL_PIN_ENC_B    22

// ---- ADXL345 IMU (shared SPI0 bus) ----
#define IMU_SPI_INST    spi0
#define IMU_PIN_CS      26
// SCK/MOSI/MISO shared with VHF: GP18/19/16

// ---- Motor config ----
#define MOTOR_PWM_HZ    20000
#define PID_HZ          100
```

### proto.h / proto.c extensions (new motor messages)

Add to msg_type_t enum:

```c
// Motor control extension (0x60-0x6F)
MSG_SET_POSITION    = 0x60,  // body: [az_f32_le, el_f32_le]
MSG_GET_POSITION    = 0x61,  // body: empty → rsp: [az_f32, el_f32]
MSG_PARK            = 0x62,  // body: empty
MSG_STOP            = 0x63,  // body: empty
MSG_HOME            = 0x64,  // body: empty → trigger IMU homing
MSG_GET_MOTOR_STATUS= 0x65,  // rsp: [state, faults, homed, az_f32, el_f32]
MSG_SET_GAINS       = 0x66,  // body: [kp_f32, ki_f32, kd_f32]
MSG_ZERO_ENCODERS   = 0x67,  // body: empty
MSG_IMU_READ        = 0x68,  // rsp: [x_f32, y_f32, z_f32, el_deg_f32]
MSG_RECAL           = 0x69,  // re-read IMU, re-zero EL

// Motor events
EVT_POSITION        = 0xE2,  // async 10Hz: [az_f32, el_f32, status_u8]
```

### main.cpp changes

```
setup():
  1. NeoPixel + buzzer init (unchanged)
  2. UART0 init (unchanged)
  3. UHF CC1200 init on SPI1 (unchanged)
  4. ADXL345 init on SPI0 (mode 3, 1 MHz)
  5. EL IMU homing loop (blocking, ~15s max)
  6. Reconfigure SPI0 → mode 0, 5 MHz
  7. VHF CC1200 init on SPI0 (now in correct mode)
  8. Motor PWM init (A4950 IN1/IN2 on new pins)
  9. Encoder ISR init (quadrature on GP14/15, GP21/22)
  10. Proto init with motor + radio

loop():
  // 100 Hz motor tick (10ms period)
  if (time_since_last_tick >= 10000us) {
    rotator.tick();  // PID + motor update
    last_tick = now;
  }

  // Existing RF HAT loop body:
  watchdog_update();
  buzzer_tick();
  proto_poll();      // handles both CC1200 + motor commands
  usb_serial_feed();

  // 10 Hz position event stream (if enabled)
  if (position_streaming && time_since_last_pos >= 100ms) {
    send EVT_POSITION with current az/el
  }

  delay(1);  // yield
```

### SPI0 mode switch helper

```c
static void spi0_set_mode(uint8_t cpol, uint8_t cpha, uint32_t baud) {
    spi_deinit(spi0);
    spi_init(spi0, baud);
    spi_set_format(spi0, 8, (spi_cpol_t)cpol, (spi_cpha_t)cpha, SPI_MSB_FIRST);
    // Re-set pin functions (they get cleared on deinit)
    gpio_set_function(VHF_PIN_SCK,  GPIO_FUNC_SPI);
    gpio_set_function(VHF_PIN_MOSI, GPIO_FUNC_SPI);
    gpio_set_function(VHF_PIN_MISO, GPIO_FUNC_SPI);
}

// Usage:
spi0_set_mode(1, 1, 1000000);  // mode 3, 1 MHz → ADXL345
spi0_set_mode(0, 0, 5000000);  // mode 0, 5 MHz → VHF CC1200
```

---

## Pi-Side Software Changes

### rf_hat.py → pico_link.py

Extend CC1200Link class with motor control methods:

```python
class PicoLink(CC1200Link):
    # Existing CC1200 methods stay unchanged

    def set_position(self, az_deg, el_deg):
        body = struct.pack('<ff', az_deg, el_deg)
        return self._command(MSG_SET_POSITION, body)

    def get_position(self):
        rsp = self._command(MSG_GET_POSITION)
        az, el = struct.unpack('<ff', rsp.body)
        return az, el

    def park(self):
        return self._command(MSG_PARK)

    def stop(self):
        return self._command(MSG_STOP)

    def home(self):
        return self._command(MSG_HOME)

    def zero_encoders(self):
        return self._command(MSG_ZERO_ENCODERS)
```

### station.py changes

Replace RotctlClient (TCP to rotctld) with PicoLink (direct UART):

```python
# OLD:
rotctl = RotctlClient('localhost', 4533)
rotctl.set_position(az, el)

# NEW:
link = PicoLink('/dev/serial0', 115200)
link.set_position(az, el)
link.set_frequency(doppler_hz)  # same link, same UART
```

### Eliminate rotctld.service

Remove from Pi/services/. The Pico handles motor control directly.
Optional: keep a lightweight TCP bridge in station.py for hamlib compat.

### dashboard.py

No changes — already reads ~/.station_status.json for status display.

---

## PCB Layout Considerations

### EMI zoning (critical)

```
 ┌─────────────────────────────────────────────────┐
 │  MOTOR ZONE              │   RF ZONE            │
 │                          │                      │
 │  A4950 (AZ)    ENCODER   │   CC1200    SMA_UHF  │
 │  A4950 (EL)    CONNS     │   (UHF)              │
 │                          │                      │
 │  24V bulk      MOTOR     │   CC1200    SMA_VHF  │
 │  caps          CONNS     │   (VHF)              │
 │                          │                      │
 │  ─ ─ ─ ─ PICO MODULE ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ │
 │                                                 │
 │            PI 40-PIN HEADER                     │
 └─────────────────────────────────────────────────┘
```

- Solid ground plane (In1.Cu) under entire RF zone — NO motor traces crossing
- Motor 24V traces on opposite side from RF matching networks
- Ferrite bead + LC filter between motor 24V and buck 24V input
- Motor connector placement: board edge, far from SMA connectors
- A4950 thermal pads need ground pour vias for heat dissipation

### 6-layer stackup (JLCPCB, unchanged)

F.Cu / prepreg / In1.Cu (GND) / core / In2.Cu (PWR) / prepreg /
In3.Cu (PWR) / core / In4.Cu (GND) / prepreg / B.Cu

### Board size

Pi HAT form factor (85×56mm). Tight but feasible:
- Pico module: 51×21mm (dominant component)
- 2x A4950 SOIC-8: 2× (5×4mm) = small
- ADXL345 LGA-14: 5×3mm = tiny
- Motor/encoder connectors: 4x JST-XH on board edge

---

## BOM Delta vs Current RF_HAT

| Change | Part | Qty | LCSC |
|--------|------|-----|------|
| ADD | A4950ELJTR-T (motor driver) | 2 | C82404 |
| ADD | ADXL345BCCZ (IMU) | 1 | C12563 |
| ADD | 100nF 50V 0402 (A4950 bypass) | 4 | basic |
| ADD | 10µF 50V 0805 (A4950 bulk) | 2 | basic |
| ADD | 100µF 50V electrolytic (24V bulk) | 1 | — |
| ADD | Ferrite 600Ω@100MHz 0805 (motor isolation) | 1 | basic |
| ADD | JST-XH 2-pin (motor conn) | 2 | — |
| ADD | JST-XH 4-pin (encoder conn) | 2 | — |
| ADD | 10k 0402 (IMU CS pull-up) | 1 | basic |
| ADD | 100nF + 10µF (IMU bypass) | 2 | basic |
| REMOVE | LMR51420 (second buck) | -1 | — |
| REMOVE | Buck passives (2nd) | -~8 | — |
| KEEP | Everything else from RF_HAT | — | — |

**Net add: ~15 components. Net remove: ~9. Delta: +6 components.**

---

## Repo Structure

New firmware directory: `Firmware/rp2040-motor-rf-hat/`

```
Firmware/rp2040-motor-rf-hat/
├── platformio.ini
├── src/
│   ├── main.cpp          (merged: RF HAT main + rotator init/loop)
│   ├── config.h          (combined pin map)
│   ├── proto.h/c         (extended with motor messages 0x60-0x69)
│   ├── cc1200.h/c        (unchanged from rf-hat)
│   ├── cobs.h/c          (unchanged)
│   ├── crc16.h/c         (unchanged)
│   ├── motor_pwm.h/cpp   (refactored: 2-pin A4950)
│   ├── quadrature.h/cpp  (from rotator, new pin numbers)
│   ├── rotator.h/cpp     (from rotator, unchanged logic)
│   └── adxl345.h/cpp     (from rotator, SPI mode switch added)
└── copy_uf2.py
```

KiCad project stays in `RF_HAT/` — update `rf_hat_circuit.py` to add
motor driver section. Output still goes to `RF_HAT_out/`.

Old firmwares (`rp2040-satnogs-rotator/`, `rp2040-rf-hat/`) remain for
reference and fallback to 2-board architecture.
