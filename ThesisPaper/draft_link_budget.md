# Link Budget Analysis for the SatNOGS Ground Station

## Introduction

A link budget is a systematic accounting of all gains and losses in a radio communication path from transmitter to receiver. It determines whether a received signal will exceed the receiver sensitivity threshold with sufficient margin for reliable demodulation. For satellite communications, the link budget is particularly critical because the path losses are extreme --- signals travel hundreds to thousands of kilometres through free space --- and the transmit powers are constrained by the limited electrical power available on spacecraft.

This analysis was motivated by a concrete observation: after extensive field testing of the ground station, no satellite signal was ever detected above the CC1200 noise floor. An overnight 12-hour monitoring session captured 55 raw data dumps across multiple satellite passes, yielding zero decoded packets. All captured data was demodulated noise. The link budget provides the quantitative framework to explain this result and identify what system modifications are required.

## System Description

The receive chain consists of the following elements:

```
                                    3 m RG58 coax
  Satellite ~~~> [Yagi Antenna] ---N-type--SMA--- [CC1200 RX on Pi HAT]
   (LEO)         Siretta Oscar 44                   TI CC1200 transceiver
                 9 dBi, 400-480 MHz                 NF = 7 dB (confirmed TI)
                 linear polarisation                2.4 kbps 2-GFSK config
```

**Ground station components:**

| Component | Specification | Source |
|---|---|---|
| Antenna | Siretta Oscar 44, 5-element Yagi, 9 dBi peak, 400--480 MHz, linear polarisation, VSWR < 1.5, N-type female | Siretta datasheet Rev 1.0 |
| Feedline | RG58 coaxial cable, 3 m, N-type male to SMA male | Standard RG58/U |
| Receiver | TI CC1200 on custom RP2040-based Pi HAT, SMA input, no external LNA | Custom PCB |
| Tracking | 2-axis AZ/EL rotator, < 0.3 deg pointing accuracy | Verified in field |

**Receiver configuration** (from SmartRF Studio export `smartrf_uhf_435.txt`):

| Register | Value | Meaning |
|---|---|---|
| MODCFG\_DEV\_E | 0x09 | 2-GFSK, deviation exponent = 1 |
| DEVIATION\_M | 0x06 | Frequency deviation ~ 5.2 kHz |
| SYMBOL\_RATE2/1/0 | 0x4F/0x75/0x10 | Symbol rate = 2.4 ksps (2.4 kbps for 2-FSK) |
| CHAN\_BW | 0x8E | RX channel filter bandwidth ~ 10 kHz |
| AGC\_GAIN\_ADJUST | 0x9D | RSSI offset = -99 dB (signed: 0x9D = -99) |
| AGC\_CFG2 | 0x20 | AGC hysteresis, normal operation |
| AGC\_CFG1 | 0x11 | AGC settling, min LNA gain = 0 |
| AGC\_CFG0 | 0x90 | AGC loop gain |
| LNA | 0x01 | Internal LNA in normal (not high-gain) mode |
| IFAMP | 0x05 | IF amplifier normal bias |
| FS\_CFG | 0x14 | LO divider = 8 (410--480 MHz band) |
| FREQ2/1/0 | 0x57/0x00/0x00 | Centre frequency = 435.0 MHz |
| PA\_CFG1 | 0x7F | TX power = +14 dBm (not relevant for RX) |

## Free Space Path Loss

The free space path loss (FSPL) for a direct line-of-sight link is:

$$\text{FSPL (dB)} = 20 \log_{10}(d) + 20 \log_{10}(f) + 20 \log_{10}\left(\frac{4\pi}{c}\right)$$

Simplified at 437 MHz:

$$\text{FSPL (dB)} = 20 \log_{10}(d_\text{km}) + 20 \log_{10}(437 \times 10^6) + 32.45$$

The slant range depends on the satellite altitude $h$ and the ground station elevation angle $\theta$. For a spherical Earth with radius $R_E = 6371$ km:

$$d = \sqrt{(R_E + h)^2 - (R_E \cos\theta)^2} - R_E \sin\theta$$

| Elevation | Altitude (km) | Slant Range (km) | FSPL (dB) |
|---|---|---|---|
| 10 deg | 400 | 1340 | 149.0 |
| 10 deg | 600 | 1830 | 151.6 |
| 30 deg | 400 | 620 | 142.2 |
| 30 deg | 600 | 920 | 145.6 |
| 45 deg | 400 | 480 | 139.9 |
| 60 deg | 400 | 420 | 138.8 |
| 90 deg (zenith) | 400 | 400 | 138.4 |
| 90 deg (zenith) | 600 | 600 | 141.9 |

At the best case (overhead pass at 400 km), FSPL = 138.4 dB. At a typical 30 deg elevation pass at 600 km, FSPL = 145.6 dB. At the horizon (10 deg), FSPL exceeds 149 dB.

## Additional Losses

### Cable Loss

RG58 coaxial cable has a specified attenuation of approximately 6.2 dB per 100 feet (0.20 dB/m) at 450 MHz. For a 3 m cable run:

$$L_\text{cable} = 3 \times 0.20 = 0.6\ \text{dB}$$

Additionally, two connectors (N-type to SMA adapter) contribute approximately 0.1--0.2 dB each. Total feedline loss:

$$L_\text{feedline} = 0.6 + 0.3 = 0.9\ \text{dB}$$

### Polarisation Mismatch

Most CubeSat antennas are circularly polarised (turnstile or canted turnstile configuration), while the Siretta Oscar 44 Yagi is linearly polarised. The theoretical polarisation mismatch between a circularly polarised transmitter and a linearly polarised receiver is exactly 3 dB. In practice, this can vary from 0 dB (when the linear antenna aligns with the instantaneous polarisation vector) to total null, but the statistical average is:

$$L_\text{pol} = 3.0\ \text{dB}$$

For satellites with linear antennas (e.g., simple dipole or monopole), the polarisation loss depends on the relative orientation and can range from 0 dB (aligned) to very large values (cross-polarised), with a statistical average of approximately 3 dB over a full pass due to satellite tumbling.

### Atmospheric Losses

At 437 MHz, atmospheric absorption through the troposphere is negligible (< 0.1 dB) for elevation angles above 10 deg. Ionospheric effects include Faraday rotation (rotation of the polarisation plane, already captured in the polarisation loss budget) and scintillation (typically < 1 dB at UHF). We budget:

$$L_\text{atm} = 0.5\ \text{dB}$$

### Pointing Loss

The Siretta Oscar 44 has a 3 dB beamwidth of 54 deg (H-plane) by 48 deg (E-plane). With the rotator achieving < 0.3 deg pointing accuracy and the satellite subtending a negligible angle, pointing loss is minimal:

$$L_\text{point} = 0.5\ \text{dB}$$

## CC1200 Receiver Sensitivity

### Datasheet Specifications

The CC1200 datasheet (SWRS123D) specifies the following sensitivities at 433 MHz in High Performance Mode:

| Data Rate | Modulation | Deviation | Filter BW | Sensitivity |
|---|---|---|---|---|
| 1.2 kbps | 2-FSK | 4 kHz | 11 kHz | -123 dBm |
| 38.4 kbps | 2-GFSK | 20 kHz | 104 kHz | -111 dBm |
| 50 kbps | 2-GFSK | 25 kHz | 104 kHz | -109 dBm |

For the ground station configuration of 2.4 kbps 2-GFSK with ~5 kHz deviation and ~10 kHz channel filter, the sensitivity can be estimated by interpolation or calculated from first principles.

### Theoretical Sensitivity Calculation

Receiver sensitivity is given by:

$$P_\text{sens} = -174\ \text{dBm/Hz} + \text{NF} + 10\log_{10}(\text{BW}_\text{noise}) + \text{SNR}_\text{req}$$

where:
- $-174$ dBm/Hz is the thermal noise floor at 290 K
- NF = 7 dB is the CC1200 total noise figure (confirmed by TI engineers on the E2E forum)
- $\text{BW}_\text{noise}$ is the noise bandwidth of the channel filter
- $\text{SNR}_\text{req}$ is the required SNR for the modulation scheme

For 2-GFSK at BER = $10^{-3}$, the required $E_b/N_0 \approx 10$ dB. The relationship between SNR and $E_b/N_0$ is:

$$\text{SNR} = \frac{E_b}{N_0} \cdot \frac{R_b}{B} = 10\ \text{dB} + 10\log_{10}\left(\frac{2400}{10000}\right) = 10 - 6.2 = 3.8\ \text{dB}$$

Therefore:

$$P_\text{sens} = -174 + 7 + 10\log_{10}(10000) + 3.8 = -174 + 7 + 40 + 3.8 = -123.2\ \text{dBm}$$

This aligns well with the datasheet specification of -123 dBm at 1.2 kbps with 11 kHz filter. At 2.4 kbps the required $E_b/N_0$ remains similar but the bit rate doubles, yielding:

$$P_\text{sens,2.4k} \approx -123 + 3 = -120\ \text{dBm}$$

This is our theoretical best-case CC1200 sensitivity at the configured data rate. We will use **-120 dBm** as the CC1200 sensitivity for the link budget.

### Measured Noise Floor vs. Theoretical

The CC1200 RSSI readings, after the AGC\_GAIN\_ADJUST offset correction (-99 dB), showed a noise floor of:
- **-100 to -105 dBm** when the antenna pointed at the horizon (ground thermal noise)
- **-105 to -107 dBm** when the antenna pointed at cold sky (high elevation)

The theoretical thermal noise floor at the CC1200 input, for a 10 kHz noise bandwidth and 7 dB NF, is:

$$N_\text{floor} = -174 + 7 + 40 = -127\ \text{dBm}$$

The measured noise floor of -100 to -107 dBm is **20--27 dB above** the theoretical thermal floor. This discrepancy has multiple contributing factors:

1. **RSSI measurement granularity**: The CC1200 RSSI has a resolution of 0.5 dB (11-bit) but an accuracy of +/- 3 dB (datasheet). The AGC\_GAIN\_ADJUST value of -99 dB is a factory/SmartRF-derived offset that may not be perfectly calibrated for this specific hardware configuration.

2. **External noise environment**: At 435 MHz, the sky noise temperature is not 290 K. Galactic background noise at UHF contributes an antenna temperature of 100--1000 K depending on pointing direction. Ground noise from thermal radiation at the horizon adds significant noise. The 5--7 dB difference between sky-pointing and horizon-pointing RSSI confirms the antenna is indeed seeing real external noise variation.

3. **Board-level interference**: The CC1200 sits on a Pi HAT PCB alongside an RP2040 microcontroller (125 MHz clock with harmonics), a WS2812B NeoPixel driver, an LMR51420 switching buck converter (2.1 MHz fundamental), and the Raspberry Pi 3A+ (1.2 GHz ARM with broadband digital noise on GPIO and HDMI). Any of these can couple into the CC1200 input through the PCB ground plane, SMA connector, or trace routing.

4. **LMR51420 switching noise**: The buck converter operates at 2.1 MHz with harmonics extending well into the UHF band. At the 207th harmonic (2.1 MHz x 207 = 434.7 MHz), switching noise could fall directly on the receive frequency. Even with the LC output filter, the ripple current creates magnetic field coupling through the PCB.

5. **AGC behaviour**: The CC1200 AGC adjusts the front-end gain to maintain a target signal level. In the absence of a real signal, the AGC may settle at a gain that amplifies board-level noise, presenting a higher apparent noise floor than thermal noise alone.

6. **Quantisation and processing noise**: The CC1200 RSSI is derived from the digital signal processing chain after the ADC. Quantisation noise and digital processing artifacts contribute to the apparent noise floor, particularly when the input signal is below the ADC noise floor.

The practical effect is that the **effective sensitivity of this receiver implementation is approximately -105 dBm**, not the theoretical -120 dBm. This 15 dB gap is the central problem.

## Complete Link Budget

### Budget Table --- Nominal Case (30 deg elevation, 600 km altitude)

| Parameter | Value | Unit | Notes |
|---|---|---|---|
| **Transmitter** | | | |
| TX power (CubeSat) | +14.0 | dBm | 25 mW, typical CC1200-based 3U CubeSat |
| TX antenna gain | +2.0 | dBi | Turnstile/dipole on CubeSat |
| EIRP | +16.0 | dBm | |
| **Path** | | | |
| Free space path loss | -145.6 | dB | 437 MHz, 920 km slant range |
| Atmospheric loss | -0.5 | dB | Troposphere + ionosphere |
| Polarisation mismatch | -3.0 | dB | Circular TX vs linear RX |
| **Receiver** | | | |
| RX antenna gain | +9.0 | dBi | Siretta Oscar 44 peak |
| Feedline loss | -0.9 | dB | 3 m RG58 + connectors |
| Pointing loss | -0.5 | dB | Rotator accuracy |
| **Signal at CC1200 input** | **-125.5** | **dBm** | |
| | | | |
| CC1200 sensitivity (theoretical) | -120.0 | dBm | 2.4 kbps 2-GFSK, BER 10^-3 |
| **Link margin (theoretical)** | **-5.5** | **dB** | NEGATIVE --- link fails |
| | | | |
| CC1200 sensitivity (measured) | -105.0 | dBm | Effective, from RSSI data |
| **Link margin (measured)** | **-20.5** | **dB** | NEGATIVE --- deeply in the noise |

### Budget Table --- Best Case (90 deg elevation, 400 km, strong satellite)

| Parameter | Value | Unit | Notes |
|---|---|---|---|
| TX power (ISS UHF digipeater) | +40.0 | dBm | 10 W |
| TX antenna gain | +2.0 | dBi | ISS UHF antenna |
| EIRP | +42.0 | dBm | |
| Free space path loss | -138.4 | dB | 437 MHz, 400 km zenith |
| Atmospheric loss | -0.3 | dB | |
| Polarisation mismatch | -3.0 | dB | |
| RX antenna gain | +9.0 | dBi | |
| Feedline loss | -0.9 | dB | |
| Pointing loss | -0.2 | dB | |
| **Signal at CC1200 input** | **-91.8** | **dBm** | |
| CC1200 sensitivity (theoretical) | -120.0 | dBm | |
| **Link margin (theoretical)** | **+28.2** | **dB** | Positive --- should work |
| CC1200 sensitivity (measured) | -105.0 | dBm | |
| **Link margin (measured)** | **+13.2** | **dB** | Positive --- should work |

### Budget Table --- Multiple Satellite Scenarios

| Scenario | TX Power | Altitude | Elev | EIRP | FSPL | Received | Margin (theory) | Margin (measured) |
|---|---|---|---|---|---|---|---|---|
| AetherSpace (CC1200, design) | +14 dBm | 600 km | 30 deg | +16 dBm | 145.6 dB | -125.5 dBm | **-5.5 dB** | **-20.5 dB** |
| AetherSpace (CC1200, zenith) | +14 dBm | 600 km | 90 deg | +16 dBm | 141.9 dB | -121.8 dBm | **-1.8 dB** | **-16.8 dB** |
| ISS digipeater (10 W) | +40 dBm | 408 km | 30 deg | +42 dBm | 142.3 dB | -97.1 dBm | **+22.9 dB** | **+7.9 dB** |
| ISS digipeater (zenith) | +40 dBm | 408 km | 90 deg | +42 dBm | 138.5 dB | -93.3 dBm | **+26.7 dB** | **+11.7 dB** |
| ISS beacon (0.3 W day) | +25 dBm | 408 km | 39 deg | +27 dBm | 140.7 dB | -109.6 dBm | **+10.4 dB** | **-4.6 dB** |
| Typical CubeSat (0.5 W) | +27 dBm | 500 km | 30 deg | +29 dBm | 144.1 dB | -111.0 dBm | **+9.0 dB** | **-6.0 dB** |
| Weak CubeSat (25 mW) | +14 dBm | 500 km | 30 deg | +16 dBm | 144.1 dB | -124.0 dBm | **-4.0 dB** | **-19.0 dB** |
| Weak CubeSat (25 mW, zenith) | +14 dBm | 500 km | 90 deg | +16 dBm | 140.3 dB | -120.2 dBm | **-0.2 dB** | **-15.2 dB** |

**Key finding**: Even with the theoretical CC1200 sensitivity of -120 dBm, the AetherSpace link closes with negative margin. With the measured noise floor, nothing weaker than the ISS 10 W digipeater is detectable. The ISS 300 mW beacon and typical 0.5 W CubeSats fall in the grey zone --- theoretically receivable but not with the actual measured noise floor.

### ISS Pass at 39 deg: Why No Signal Was Detected

During field testing, an ISS pass at 39 deg maximum elevation produced no detectable signal above the noise floor. The ISS operates multiple transmitters on UHF:

- **APRS digipeater**: 437.550 MHz, 10 W (when active) --- this would have been detectable (+7.9 dB measured margin at 30 deg) **if** the CC1200 was tuned to 437.550 MHz and configured for 1200 baud AFSK. The ground station was configured for 435.0 MHz, 2.4 kbps 2-GFSK, which is the wrong frequency and wrong modulation for ISS APRS.
- **Telemetry beacon**: various frequencies, 300 mW --- with -4.6 dB measured margin, not detectable.
- **FM repeater**: 437.800 MHz, varies --- again wrong frequency for a 435 MHz, 10 kHz BW receiver.

This explains the null result: the CC1200 was not tuned to any active ISS transmitter frequency, and its narrow 10 kHz channel filter would reject any signal even a few kHz away.

## Impact of an External LNA

### Friis Equation for Cascaded Noise Figure

When an external LNA is placed between the antenna and the CC1200, the system noise figure is determined by the Friis equation:

$$F_\text{sys} = F_\text{LNA} + \frac{F_\text{cable} - 1}{G_\text{LNA}} + \frac{F_\text{CC1200} - 1}{G_\text{LNA} \cdot G_\text{cable}}$$

where all values are in linear (not dB) scale.

### Case 1: LNA at antenna feed (before cable)

Using a typical mast-mounted LNA (e.g., Minicircuits ZX60-P33ULN+ or SPF5189-based board):
- LNA noise figure: $F_\text{LNA}$ = 0.5 dB (linear: 1.122)
- LNA gain: $G_\text{LNA}$ = 20 dB (linear: 100)
- Cable loss: $L_\text{cable}$ = 0.9 dB (linear: 1.230, so $F_\text{cable}$ = 1.230)
- CC1200 noise figure: $F_\text{CC1200}$ = 7 dB (linear: 5.012)

$$F_\text{sys} = 1.122 + \frac{1.230 - 1}{100} + \frac{5.012 - 1}{100 \times 0.813} = 1.122 + 0.0023 + 0.0494 = 1.174$$

$$\text{NF}_\text{sys} = 10\log_{10}(1.174) = 0.70\ \text{dB}$$

The system noise figure drops from **7.0 dB to 0.70 dB** --- an improvement of **6.3 dB**.

New theoretical sensitivity at 2.4 kbps, 10 kHz BW:

$$P_\text{sens,LNA} = -174 + 0.70 + 40 + 3.8 = -129.5\ \text{dBm}$$

### Case 2: LNA after cable (at CC1200 HAT input)

If the LNA is placed at the CC1200 HAT (after the cable):
- Cable noise figure: $F_\text{cable}$ = 0.9 dB (linear: 1.230)

$$F_\text{sys} = F_\text{cable} + \frac{F_\text{LNA} - 1}{G_\text{cable}} + \frac{F_\text{CC1200} - 1}{G_\text{cable} \cdot G_\text{LNA}}$$

$$F_\text{sys} = 1.230 + \frac{1.122 - 1}{0.813} + \frac{5.012 - 1}{0.813 \times 100} = 1.230 + 0.150 + 0.0494 = 1.429$$

$$\text{NF}_\text{sys} = 10\log_{10}(1.429) = 1.55\ \text{dB}$$

This is still a large improvement (7.0 dB down to 1.55 dB = **5.45 dB improvement**), but 0.85 dB worse than mounting the LNA at the antenna.

### Practical Impact on Noise Floor

The LNA does not merely improve theoretical sensitivity; it also **dominates the front-end gain**, which means the CC1200's AGC sees a clean, amplified version of the antenna signal rather than board-level noise. With 20 dB of gain before the CC1200, the antenna signal and sky noise are elevated well above the CC1200's internal noise and PCB interference. The effective noise floor should drop to approximately:

$$N_\text{floor,LNA} \approx -174 + 0.70 + 40 = -133.3\ \text{dBm}$$

Even if board-level noise adds 5 dB of degradation (rather than the current 20 dB), the practical noise floor would be approximately -128 dBm, giving a measured sensitivity of perhaps **-125 dBm** --- a 20 dB improvement over the current situation.

### Revised Link Budget with LNA

| Scenario | Received | Margin (no LNA, measured) | Margin (with LNA, estimated) |
|---|---|---|---|
| AetherSpace, 30 deg | -125.5 dBm | **-20.5 dB** | **+0.5 dB** (marginal) |
| AetherSpace, zenith | -121.8 dBm | **-16.8 dB** | **+3.2 dB** (viable) |
| ISS digipeater, 30 deg | -97.1 dBm | **+7.9 dB** | **+27.9 dB** (strong) |
| Typical CubeSat (0.5 W), 30 deg | -111.0 dBm | **-6.0 dB** | **+14.0 dB** (comfortable) |
| Weak CubeSat (25 mW), 30 deg | -124.0 dBm | **-19.0 dB** | **+1.0 dB** (marginal) |

An LNA transforms the system from one where essentially nothing is receivable to one where most satellites become accessible, and even the weak AetherSpace link becomes marginally viable at high elevations.

## CC1200 Hardware Modem vs. RTL-SDR + Software Approach

### CC1200 Hardware Modem

The CC1200 performs all RF front-end, filtering, demodulation, and bit synchronisation in dedicated hardware. It outputs **hard-decision bits** --- each received bit is either 0 or 1 with no confidence information. This has implications:

**Advantages:**
- Zero latency, real-time demodulation
- Deterministic timing (no OS jitter)
- Very low power consumption (< 20 mA RX current)
- Self-contained: no SDR software stack, no DSP expertise required
- Direct packet output via SPI FIFO

**Disadvantages:**
- **Sensitivity ceiling**: The CC1200 cannot demodulate signals below its sensitivity threshold. There is no way to integrate longer to extract weak signals.
- **Fixed demodulator**: The on-chip 2-GFSK demodulator is optimised for its supported modulation formats. It cannot perform coherent integration, matched filtering on custom waveforms, or spectral accumulation.
- **No post-processing**: Once a bit decision is made, the raw IQ data is lost. There is no opportunity for offline processing with better algorithms.
- **Noise figure penalty**: At 7 dB NF, the CC1200 discards 5--6 dB of theoretical sensitivity compared to a purpose-built satellite receiver front end with 1--2 dB NF.

### RTL-SDR + GNU Radio

The RTL-SDR V3 dongle (R820T2 tuner, RTL2832U ADC) captures raw IQ samples at up to 2.4 Msps with 8-bit resolution. These are processed in software using GNU Radio or custom Python scripts.

**Advantages:**
- **Lower noise figure**: The R820T2 achieves NF $\approx$ 3.5 dB at 435 MHz (at maximum gain setting). With an external LNA, system NF drops to < 1 dB.
- **Sub-noise-floor signal extraction**: Software can perform coherent integration over many symbols, FFT-based detection, and matched filtering to extract signals 10--15 dB below the noise floor. This is the standard approach in the SatNOGS network.
- **Flexible demodulation**: Any modulation scheme can be decoded: FSK, AFSK, GMSK, BPSK, or entirely custom formats. New satellites can be supported by adding a GNU Radio flowgraph.
- **Waterfall and spectrum analysis**: Raw IQ data enables visual signal identification, even when automatic demodulation fails.
- **Community ecosystem**: The SatNOGS network (2000+ stations) is built entirely on RTL-SDR + GNU Radio. Decoder plugins exist for hundreds of satellite protocols.

**Disadvantages:**
- Requires a host computer with sufficient CPU (Pi 3A+ with 512 MB RAM is marginal)
- 8-bit ADC dynamic range limits performance in strong-signal environments
- Software complexity: GNU Radio flowgraphs, sample rate management, buffer overflows
- Higher power consumption than CC1200 hardware modem

### Quantitative Comparison

| Parameter | CC1200 | RTL-SDR V3 | RTL-SDR + LNA |
|---|---|---|---|
| Noise figure | 7 dB | 3.5 dB | ~1.0 dB |
| Sensitivity (2.4 kbps FSK) | -120 dBm | -123.5 dBm | -126 dBm |
| Processing gain (coherent integration) | 0 dB | 10--15 dB | 10--15 dB |
| Effective sensitivity | -120 dBm | -135 dBm | -140 dBm |
| Channel bandwidth | Fixed per config | Software-defined | Software-defined |
| Modulation formats | 2-(G)FSK, MSK, ASK, OOK | Any (software) | Any (software) |
| SatNOGS compatibility | None (custom protocol) | Full (standard stack) | Full (standard stack) |

The processing gain available in software is the decisive factor. A properly configured RTL-SDR station with GNU Radio can achieve an effective sensitivity 15--20 dB better than the CC1200 hardware modem, because it can:
1. Accumulate energy over many symbol periods (coherent integration)
2. Use optimal matched filtering adapted to the exact waveform
3. Apply forward error correction (FEC) decoding with soft decisions
4. Perform spectral detection to find signals that are invisible in the time domain

## Summary and Recommendations

### Why No Satellites Have Been Received

The link budget reveals three compounding problems:

1. **Marginal theoretical link**: Even with perfect receiver performance (-120 dBm sensitivity), the AetherSpace 25 mW CubeSat link closes with -5.5 dB margin at typical elevations. There is no margin for any real-world degradation.

2. **15--20 dB noise floor elevation**: The measured CC1200 noise floor (-100 to -107 dBm) is 15--20 dB above the theoretical thermal noise floor (-127 dBm). This is caused by a combination of PCB-level interference from the buck converter, RP2040 digital noise, Raspberry Pi broadband emissions, and possibly imperfect RSSI calibration.

3. **Wrong frequency/modulation**: The CC1200 was configured for 435.0 MHz, 2.4 kbps 2-GFSK with a 10 kHz channel filter. Most active amateur satellites transmit on different frequencies (e.g., ISS APRS at 437.550 MHz) with different modulations (1200 baud AFSK, 9600 baud GMSK). Even if a satellite signal arrived at -90 dBm, it would be rejected by the narrow channel filter if off-frequency by more than 5 kHz.

### Required System Modifications

In order of impact:

1. **Add an external LNA** at the antenna feed point. A mast-mounted LNA with NF < 1 dB and gain of 15--20 dB is the single most impactful modification. This recovers approximately 6 dB from noise figure improvement and an estimated 10--15 dB from suppressing the board-level noise floor. Cost: EUR 15--30 for an SPF5189-based LNA module. An LNA with a SAW bandpass filter (e.g., 430--440 MHz) is preferable to reject out-of-band interference.

2. **Correct the frequency and modulation** per satellite. Each satellite requires configuration of the exact downlink frequency and modulation parameters. The sat\_profiles.json database and per-pass reconfiguration in station.py address this, but many profiles need verification against actual satellite documentation.

3. **Consider an RTL-SDR parallel path**. For the broadest satellite compatibility and best sensitivity, adding an RTL-SDR dongle alongside the CC1200 provides software-defined reception with 15--20 dB better effective sensitivity. The CC1200 remains the primary receiver for the AetherSpace mission (native CC1200 packet format), while the RTL-SDR handles the wider amateur satellite ecosystem.

4. **Reduce PCB-level noise**. Future revisions of the RF HAT should separate the CC1200 RF input from digital noise sources: use a solid ground plane under the RF traces, add filtering on the LMR51420 buck converter output, and consider a shielding can over the CC1200 and matching network.

### Minimum Viable Configuration for AetherSpace

For the thesis target --- receiving AetherSpace at 433 MHz with CC1200 native packets:

- **With LNA**: Link margin = +0.5 dB at 30 deg elevation, +3.2 dB at zenith. Marginal but potentially viable.
- **Without LNA**: Link margin = -20.5 dB. Not viable under any conditions.

The LNA is not optional. It is a hard requirement for the AetherSpace link to close.
