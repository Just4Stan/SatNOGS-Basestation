# Overnight Test Results — 2026-04-12

## CRITICAL BUGS FOUND & FIXED

### 1. PIO Serial RX all-zeros (FIXED — firmware)
- **Root cause**: `word >> 24` reading wrong byte from PIO FIFO. Shift-left autopush puts data in ISR[7:0], not [31:24].
- **Fix**: `word & 0xFF` in `maybe_stream_serial()` (proto.c line 503)
- **Also**: Rewrote PIO program from 6-instruction `jmp pin` to 3-instruction `in pins, 1`
- **Also**: Removed `gpio_pull_down` on serial data pin (replaced with `gpio_disable_pulls`)
- **Verified**: OBJECT AP pass captured 95,256 data chunks, 3MB raw

### 2. CC1200 serial blind mode oversampling (~8x)
- **Root cause**: SYNC_MODE=0 (blind) outputs at internal demodulator sample rate, not symbol rate
- **Measured**: 7.5x oversampling (4800 baud configured → 36000 bps actual)
- **Fix attempt 1**: SYNC_MODE=010 with loose threshold — BROKE serial clock (never starts without sync)
- **Fix attempt 2**: Runtime measurement + majority-vote decimation in firmware
- **Current status**: Decimation implemented, rate reduced from 8.5 KB/s to 1.2 KB/s

### 3. False packet counting in serial mode (FIXED — Python)
- **Root cause**: `poll_packets()` counted every non-zero 32-byte raw chunk as a "packet"
- **Symptoms**: Dashboard showed "95,256 packets" when zero real frames decoded
- **Fix**: In serial mode, only count decoded AX.25 frames (body[1]==0x01) as packets
- **Raw data still captured to .raw dump file for post-processing

### 4. TRANSPARENT_MODE_EN theory (DISPROVED)
- Setting MDMCFG0 bit 6 = 1 did NOT fix the all-zeros issue
- The real fix was the byte extraction bug

## ISSUES STILL OPEN

### Serial mode AX.25 decoding not yet verified
- Firmware decimates and runs G3RUH + HDLC decoder
- No CRC-valid frames received yet (need confirmed active G3RUH sat)
- Best candidate: SONATE-2 (437.025 MHz, 9600 GMSK G3RUH, pass ~00:00 UTC)
- Offline decoder (decode_serial.py) also found zero CRC-valid frames

### Satellite filtering issues
- Dashboard CC1200 filter too restrictive — shows mostly "OBJECT" entries
- Many real sats filtered out because SatNOGS DB enrichment fails
- station.py skips many sats with "no UHF transmitter in SatNOGS DB"
- Protocol inference sometimes wrong (e.g., Geoscan framing misidentified as G3RUH)

### Dashboard UX issues noted
- JS error: "Identifier 'rc' has already been declared" (stale, from older session)
- "Unknown Satellite" appears when TLE name doesn't match
- Recent passes show inflated packet counts from before fix
- VIZARD-ION Geoscan framing should use packet mode, not serial mode

## METRICS (as of 21:11 UTC)
- Total passes tracked: ~20
- Passes with real non-zero data: 5 (BLUEWALKER_3, MIMAN, OBJECT AP, CubeBel-2, SMDC ONE)
- CRC-verified AX.25 frames: 0 (pending SONATE-2 pass)
- Firmware reflashes: 4
- Pi service restarts: ~15
- No service crashes (watchdog healthy)
- Memory stable at 41-45% on 512MB Pi 3A+
- Disk: 51GB free
- Temp: 24-31C

## STATION CONFIG (overnight)
- Mode: SCAN
- Dwell: 60s per satellite
- Min elevation: 10 deg
- Scan window: 12 hours
- Doppler: enabled
- RF: CC1200 auto
