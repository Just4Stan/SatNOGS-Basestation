# Overnight Bug Tracker
# Checked every 2 minutes, fixes pushed immediately

## KNOWN BUGS (to fix tonight)

### BUG-01: AFSK 1.2k marked as CC1200 compatible [OPEN]
- S-NET B/C shows "YES — CC1200 hardware modem" but AFSK 1200 is Bell 202 audio
- station.py already filters this in sat_library.py line 280 but dashboard enrichment doesn't

### BUG-02: "assumed" G3RUH sats didn't use serial mode [FIXED 21:35]
- use_serial_mode excluded "assumed" protocol — now fixed

### BUG-03: Raw serial chunks counted as packets [FIXED 21:17]
- poll_packets now skips raw chunks in serial_ax25 mode

### BUG-04: Fallback serial mode doesn't set _active_rx_mode [OPEN]
- When packet mode auto-falls back to serial after 30s, _active_rx_mode stays "packet"
- Raw chunks then pass the serial filter and get counted as packets
- Fix: set _active_rx_mode = "serial_ax25" in the fallback path

### BUG-05: VIZARD-ION uses wrong protocol (G3RUH instead of Geoscan) [FIXED 21:17]
- Added to sat_profiles.json with correct Geoscan config

### BUG-06: Dashboard recent passes show inflated pkt counts [COSMETIC]
- Pre-fix passes show noise chunk counts (95256, 53716 etc)
- Will age out naturally

### BUG-07: station.log is binary (grep fails) [OPEN]
- Station log has binary data mixed in from raw hex dumps
- Fix: filter non-printable chars from log lines

### BUG-08: Duplicate MDMCFG0 save in firmware [WON'T FIX - can't reflash]

### BUG-09: FORESAIL-1 PRIME link margin shows -127.3 dB initially [COSMETIC]
- Before any RSSI reading, link margin calculation produces garbage

## FIXES DEPLOYED TONIGHT
1. word & 0xFF (firmware) — PIO byte extraction
2. PIO rewrite (firmware) — in pins instead of jmp pin
3. gpio_disable_pulls (firmware) — was pull_down
4. G3RUH/NRZ-I order swap (firmware) — descramble first, then NRZ-I
5. Edge-triggered clock recovery (firmware) — replaces majority vote
6. Serial mode for all G3RUH sats (Python) — removed "assumed" exclusion
7. Raw chunk filter (Python) — only count decoded AX.25 as packets
8. Zero chunk skip (Python) — don't dump all-zero PIO data
9. VIZARD-ION Geoscan profile (JSON) — correct packet mode config
