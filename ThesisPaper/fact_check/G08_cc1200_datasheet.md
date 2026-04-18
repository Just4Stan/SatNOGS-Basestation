# G8 — CC1200 Datasheet Claims

Reference: TI SWRS123D (CC1200 datasheet, July 2013, revised October 2014) — local copy at `MOTOR_RF_HAT/datasheets/cc1200.pdf`. Companion CC120x User Guide SWRU346 for register fields.

## Claim 1 — Sensitivity at 2.4 kbps 2-GFSK
**Location:** ch02_background.tex:66
**Quote:** "CC1200 sensitivity (datasheet, 2.4 kbps 2-GFSK) −120 dBm"
**Verdict:** PARTIAL
**Computed/Evidence:** SWRS123D §4.10.3 (433 MHz band, High-Performance Mode, p.12) only tabulates two points: **−123 dBm @ 1.2 kbps 2-FSK** and **−111 dBm @ 38.4 kbps 2-GFSK**. There is NO 2.4 kbps 2-GFSK entry at 433 MHz. The 868/915/920 MHz table (§4.10.2, p.11) also lacks 2.4 kbps. −120 dBm is a reasonable interpolation between the two datasheet points but is not a literal datasheet number.
**Note:** Wording "datasheet ... 2.4 kbps 2-GFSK = −120 dBm" implies a direct quotation. Either change to "interpolated from datasheet 1.2 kbps / 38.4 kbps points" or pick one of the tabulated rates.

## Claim 2 — TI-supported frequency bands
**Location:** ch04_rf_frontend.tex:257-263
**Quote:** "164–190 MHz, 410–475 MHz, 820–950 MHz"
**Verdict:** OK
**Computed/Evidence:** SWRS123D §4.5 RF Characteristics table (p.8). Exact match for the three primary bands. Datasheet §1.3 description and §4.6 Regulatory Standards confirm the same ranges.
**Note:** Additional secondary bands (137–158.3, 205–237.5, 274–316.6 MHz) are listed in §4.5 as "contact TI" — the thesis correctly limits itself to the three primary bands.

## Claim 3 — Receiver differential LNA_P / LNA_N, DC path to ground
**Location:** ch04_rf_frontend.tex:269
**Quote:** "Receiver is differential LNA with LNA_P and LNA_N pins; requires DC path to ground"
**Verdict:** OK
**Computed/Evidence:** SWRS123D §3.2 Pin Configuration table (p.6). Pin 19 LNA_P / Pin 20 LNA_N both "Differential RX input (requires DC path to ground)". Exact match.

## Claim 4 — Single-ended PA, DC path to supply
**Location:** ch04_rf_frontend.tex:271
**Quote:** "Transmitter output single-ended at PA pin, requires DC path to supply"
**Verdict:** OK
**Computed/Evidence:** SWRS123D §3.2, p.6. Pin 17 PA "Single-ended TX output (requires DC path to VDD)". Exact match.

## Claim 5 — Optimum RX source impedance
**Location:** ch04_rf_frontend.tex:279-287
**Quote:** "433 MHz: differential 100+j60 Ω, single-ended 50+j30 Ω; 169 MHz: differential 140+j40 Ω, single-ended 70+j20 Ω"
**Verdict:** OK
**Computed/Evidence:** SWRS123D §4.10.1 General Receive Parameters (p.10). Table entry "Optimum source impedance": 433 MHz band = 100+j60 / 50+j30 Ω, 169 MHz band = 140+j40 / 70+j20 Ω. Exact match (the column header confirms differential/single-ended pairing).

## Claim 6 — Optimum TX load impedance
**Location:** ch04_rf_frontend.tex:297-298
**Quote:** "433 MHz = 55+j25 Ω; 169 MHz = 80+j0 Ω"
**Verdict:** OK
**Computed/Evidence:** SWRS123D §4.11 Transmit Parameters (p.13). "Optimum load impedance": 433 MHz = 55+j25 Ω, 169 MHz = 80+j0 Ω. Exact match.

## Claim 7 — Sensitivity at 1.2 kbps in 433 MHz band
**Location:** ch04_rf_frontend.tex:308
**Quote:** "Up to −123 dBm sensitivity at 1.2 kbps in 433 MHz band"
**Verdict:** OK
**Computed/Evidence:** SWRS123D §4.10.3 (p.12): "1.2 kbps 2-FSK, DEV=4 kHz, CHF=11 kHz → −123 dBm". Exact match.
**Note:** Technically 2-FSK (not 2-GFSK) per the condition column; ch04 does not specify modulation for this number so still OK.

## Claim 8 — Up to +15 dBm at 433 MHz, 0.4 dB step
**Location:** ch04_rf_frontend.tex:318-319
**Quote:** "Up to +15 dBm at 433 MHz; programmable output power with 0.4 dB step size"
**Verdict:** OK
**Computed/Evidence:** SWRS123D §4.11 (p.13). "Max output power" table: +15 dBm at 433 MHz (at VDD=3.0 V); +16 dBm at VDD=3.6 V. "Output power step size" = 0.4 dB within fine step range. Exact match.
**Note:** The headline feature list (§1.1, p.1) claims "up to +16 dBm with 0.4 dB step size" — the thesis understates this, but "up to +15 dBm" is the 3.0 V number and is strictly correct.

## Claim 9 — 46–49 mA high-performance mode at 433 MHz
**Location:** ch04_rf_frontend.tex:325
**Quote:** "46–49 mA current consumption in high-performance mode at 433 MHz"
**Verdict:** OK
**Computed/Evidence:** SWRS123D §4.8.2 (p.9). 433 MHz High-Performance Mode TX current: +14 dBm = 46 mA, +15 dBm = 49 mA, +10 dBm = 35 mA. The 46–49 mA range matches the +14/+15 dBm pair.
**Note:** Strictly this is TX current, not total "current consumption". Wording could be tightened to "TX current consumption".

## Claim 10 — Bands correspond to ISM/SRD
**Location:** ch04_rf_frontend.tex:265
**Quote:** "These bands correspond to typical ISM and SRD applications"
**Verdict:** OK
**Computed/Evidence:** SWRS123D §1.3 description (p.2): "The device is mainly intended for the ISM (Industrial, Scientific, and Medical) and SRD (Short Range Device) frequency bands at 164–190 MHz, 410–475 MHz, and 820–950 MHz." Exact phrasing backs the thesis claim.

## Claim 11 — Part ID 0x20, version 0x11
**Location:** ch06_results.tex:114
**Quote:** "SPI readback confirms part ID 0x20 and version 0x11"
**Verdict:** PARTIAL
**Computed/Evidence:** Part ID = 0x20 is corroborated by firmware (`Firmware/rp2040-rf-hat/src/main.cpp:178,186` — expected PARTNUMBER value at ext register 0x8F is 0x20). This is consistent with CC120x user guide SWRU346 convention (PARTNUMBER register distinguishes CC1200 from CC1120/CC1121/CC1125). **Version 0x11 is not tabulated in SWRS123D** — the datasheet does not document a specific PARTVERSION value; this is a silicon-revision-specific readback that depends on the chip lot. 0x11 is plausible for a CC1200 rev D but cannot be cross-checked against the datasheet itself. Treat 0x11 as empirical-only.
**Note:** Rephrase as "part ID 0x20 (matches CC1200) and silicon version 0x11" to make clear that only the part ID is a datasheet-specified value.

## Claim 12 — "Low-power, high-performance RF transceiver"
**Location:** ch05 §RF HAT (and ch02 background)
**Verdict:** OK
**Computed/Evidence:** SWRS123D title (p.1): "CC1200 Low-Power, High-Performance RF Transceiver". Verbatim match.

## Claim 13 — Register states PKT_FORMAT=0x01, FIFO_EN=0, SYNC_MODE=0x00
**Location:** ch05_firmware_software.tex:121
**Quote:** "PKT_FORMAT=0x01 (synchronous serial), FIFO_EN=0, SYNC_MODE=0x00"
**Verdict:** OK
**Computed/Evidence:** Firmware ground-truth in `Firmware/rp2040-rf-hat/src/proto.c:687-716`:
- `PKT_CFG2` bits[1:0] = 01 → PKT_FORMAT = synchronous serial mode (register address 0x26)
- `MDMCFG1` bit 6 cleared → FIFO_EN = 0
- `SYNC_CFG1` bits[7:5] = 000 → SYNC_MODE = 0 (blind/no-sync)
All three field names, bit positions, and the "synchronous serial" interpretation of PKT_FORMAT=01 are consistent with CC120x User Guide SWRU346 register map conventions. All are valid CC1200 register states — synchronous serial with no sync-word detection is exactly the documented configuration for external demod/descrambler use (which is what the G3RUH pipeline requires).
**Note:** The thesis says "SYNC_MODE=0x00" which is the bit-field value, not the full register value of SYNC_CFG1 — the wording is fine but a reader may misread it as "write 0x00 to SYNC_CFG1", which would also zero other fields in that register. Consider clarifying as "SYNC_MODE field = 0 in SYNC_CFG1".

## Summary

- OK: 2, 3, 4, 5, 6, 7, 8, 9, 10, 12, 13 (11 claims)
- PARTIAL: 1 (sensitivity at 2.4 kbps is interpolated, not quoted), 11 (version 0x11 is empirical, not datasheet-specified)
- WRONG: none
- UNVERIFIABLE: none

Only two actionable fixes: (a) rephrase ch02:66 to note that the −120 dBm @ 2.4 kbps figure is interpolated between the 1.2 kbps and 38.4 kbps datasheet points, not a direct datasheet quotation; (b) rephrase ch06:114 to make clear that only part ID 0x20 is datasheet-specified, version 0x11 is an empirical readback.
