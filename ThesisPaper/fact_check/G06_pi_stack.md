# G6 — Pi Software Stack

## Claim 1 — TLE from CelesTrak + PyEphem trajectory
**Location:** ch05_firmware_software.tex:L168
**Quote:** "Fetch TLEs from CelesTrak, compute trajectory with PyEphem."
**Verdict:** PARTIAL
**Computed/Evidence:** station.py:L58 imports `ephem` (PyEphem). station.py:L222-229 `TLE_URLS` — primary source is AMSAT (`amsat.org/tle/current/nasabare.txt`); CelesTrak is an explicit fallback. station.py:L354-378 fetches from dashboard's cached TLE file first, then AMSAT. CelesTrak may 403 under rate limiting.
**Note:** "CelesTrak" is misleading as the primary source. AMSAT is primary, CelesTrak is fallback. Reword to "Fetch TLEs from AMSAT/CelesTrak" or "Fetch TLEs from online sources".

## Claim 2 — AZ/EL to rotctld at 10 Hz
**Location:** ch05_firmware_software.tex:L170
**Quote:** "Stream AZ/EL to rotctld at 10 Hz."
**Verdict:** OK
**Computed/Evidence:** station.py:L213 `UPDATE_HZ = 10`. station.py:L1463 `dt = 1.0 / UPDATE_HZ`. Tracking loop at L1500-1627 runs one set_position per tick.

## Claim 3 — Doppler at 2 Hz, FREQOFF normal, full FREQ for large jumps
**Location:** ch05_firmware_software.tex:L172
**Quote:** "Apply Doppler correction at 2 Hz --- FREQOFF register normal, full FREQ rewrite for large jumps."
**Verdict:** WRONG
**Computed/Evidence:** station.py:L214 `DOPPLER_HZ = 2` — rate is correct. But `update_uhf_doppler()` at station.py:L969-983 uses `set_frequency_word()` (full FREQ2/FREQ1/FREQ0 rewrite) for EVERY update. FREQOFF register is never written as a Doppler mechanism — it is only READ as a receiver-side error estimate (`EXT_FREQOFF_EST1/EST0` at station.py:L727-736, rf_hat.py:L82-83). rf_hat.py:L743-759 shows `set_frequency()` writes FS_CFG + FREQ word; `set_frequency_word()` does an atomic FREQ-word write via `MSG_SET_FREQ_WORD`. No FREQOFF/FREQ split path exists.
**Note:** Remove the "FREQOFF register normal, full FREQ rewrite for large jumps" phrase. Also contradicts CLAUDE.md/MEMORY.md, which claimed the FREQOFF path. Rewrite as: "Doppler correction at 2 Hz via atomic FREQ-register rewrite (`MSG_SET_FREQ_WORD`)."

## Claim 4 — Telemetry CSV at 5 Hz
**Location:** ch05_firmware_software.tex:L173
**Quote:** "Drain RX packets and log telemetry ... to CSV at 5 Hz."
**Verdict:** OK
**Computed/Evidence:** Loop runs at 10 Hz (station.py:L1623 `tick += 1`, L1624 `next_tick = time.time() + dt` with dt=0.1). station.py:L1580 `if tick % 2 == 0` writes status and CSV → every 2nd tick → 5 Hz. CSV append at L1608-1619.

## Claim 5 — .station_status.json at ~5 Hz
**Location:** ch05_firmware_software.tex:L174
**Quote:** "Write ~/.station_status.json at ~5 Hz for the dashboard."
**Verdict:** OK (contradicts CLAUDE.md)
**Computed/Evidence:** station.py:L1579-1605 writes `write_dashboard_status(status_data)` inside the `if tick % 2 == 0` branch → 5 Hz during tracking. CLAUDE.md says "1 Hz" — that's wrong. In pre-AOS wait (station.py:L1871-1872), status is written every 2 s (0.5 Hz), not 5 Hz — so 5 Hz is only during active tracking, lower during wait.
**Note:** "~5 Hz" is accurate for the tracking path. Update CLAUDE.md and MEMORY.md.

## Claim 6 — Scan mode: 30 s dwell near peak elevation
**Location:** ch05_firmware_software.tex:L177
**Quote:** "Scan mode cycles through visible satellites near peak elevation with a 30 s dwell each."
**Verdict:** OK
**Computed/Evidence:** station.py:L1750 `build_schedule(..., dwell_s=30)`. L1763-1769 in scan mode centres a `dwell_s`-second window at each pass's `max_time` (±15 s around peak). Greedy non-overlapping schedule.

## Claim 7 — 136--960 MHz frequency calculation
**Location:** ch05_firmware_software.tex:L181
**Quote:** "frequency calculation across 136--960 MHz"
**Verdict:** OK
**Computed/Evidence:** rf_hat.py:L127 band 0x02 covers 820-960 MHz, L131 band 0x0B covers 136-160 MHz; intermediate LO-divider bands fill the rest of the range (410-480 MHz etc.).

## Claim 8 — Modulation config 2-FSK, 2-GFSK, 4-FSK, OOK
**Location:** ch05_firmware_software.tex:L181
**Quote:** "modulation config (2-FSK, 2-GFSK, 4-FSK, OOK)"
**Verdict:** PARTIAL
**Computed/Evidence:** rf_hat.py:L117-121 modulation table lists 2-FSK, 2-GFSK, 4-FSK, OOK — all four claimed. Also supports 4-GFSK and ASK (not mentioned). rf_hat.py:L791 comment: `"2-GFSK", "2-FSK", "4-GFSK", "4-FSK", "ASK", "OOK"`.
**Note:** Accurate as far as it goes; could extend list or say "e.g." — minor.

## Claim 9 — SPA at https://<pi-ip>:5000
**Location:** ch05_firmware_software.tex:L185
**Quote:** "serves a single-page app at https://<pi-ip>:5000"
**Verdict:** OK
**Computed/Evidence:** dashboard.py:L2004-2005 `--port` default 5000. L2026-2034 SSL setup. L5 header also shows `https://<pi-ip>:5000`.

## Claim 10 — Self-signed cert in ~/.station_ssl/
**Location:** ch05_firmware_software.tex:L187
**Quote:** "A self-signed TLS certificate is auto-generated on first run (~/.station_ssl/)"
**Verdict:** OK
**Computed/Evidence:** dashboard.py:L1977 `cert_dir = os.path.expanduser("~/.station_ssl")`. L1985-1988 openssl req -x509 -newkey rsa:2048 generates the self-signed cert on first run.

## Claim 11 — Five-step setup wizard
**Location:** ch05_firmware_software.tex:L196
**Quote:** "The setup wizard has five steps"
**Verdict:** OK
**Computed/Evidence:** dashboard.html:L1832-1917 defines step-dot 1-5 and `onboard-step-1` through `onboard-step-5`: GPS, compass calibration, attach antenna, choose receiver, ready.

## Claim 12 — rotctld model 204 TCP 4533
**Location:** ch05_firmware_software.tex:L219
**Quote:** "rotctld.service --- hamlib rotator daemon, model 204, listening on TCP 4533"
**Verdict:** OK
**Computed/Evidence:** rotctld.service:L8 `ExecStart=/usr/bin/rotctld -m 204 -r /dev/ttyACM0 -s 115200 -T 0.0.0.0`. Port 4533 is rotctld's default when no `-t` flag is passed; station.py:L205 `ROTCTLD_PORT = 4533` confirms the client side.
**Note:** Port 4533 is implicit (hamlib default), not explicit in the ExecStart.

## Claim 13 — station.service waits for station.conf
**Location:** ch05_firmware_software.tex:L221
**Quote:** "station.service --- station controller in daemon mode. Waits for station.conf"
**Verdict:** OK
**Computed/Evidence:** station.service:L9 invokes `station.py --daemon`. station.py:L2179-2225 daemon mode waits for `~/station.conf` (L2224-2225 log: "Waiting for station location (set GPS via dashboard)...").

## Claim 14 — setup-pi.sh idempotent, enable_uart, disable-bt, rfkill
**Location:** ch05_firmware_software.tex:L224
**Quote:** "configures a fresh Raspberry Pi with all dependencies, UART configuration (enable_uart=1, dtoverlay=disable-bt), a WiFi rfkill workaround ..., and service installation. The script is idempotent."
**Verdict:** OK
**Computed/Evidence:** setup-pi.sh:L152-158 `grep -q "^enable_uart=1"` then append-if-missing (idempotent); same pattern for `dtoverlay=disable-bt`. L28-86 WiFi rfkill workaround (masks systemd-rfkill, installs rfkill-unblock.service, NetworkManager no-rfkill config).

## Claim 15 — Station 4712 "Aether-Basestation"
**Location:** ch05_firmware_software.tex:L228
**Quote:** "station 4712 (``Aether-Basestation'') on network.satnogs.org"
**Verdict:** OK
**Computed/Evidence:** README.md:L188, docs/system_diagram.html:L70, Pi/docs/satnogs_integration.md:L90-95,222 all give ID 4712 / name "Aether-Basestation".

## Claim 16 — SiDS submission verified HTTP 201
**Location:** ch05_firmware_software.tex:L228
**Quote:** "SiDS telemetry submission to db.satnogs.org is verified (HTTP 201 OK)"
**Verdict:** OK
**Computed/Evidence:** satnogs.py:L26 `POST https://db.satnogs.org/api/telemetry/`. L50 `SATNOGS_DB_URL`. L234,L257-259 `if code == 201:` logged as "→ 201 OK". MEMORY.md notes "SiDS submission verified (HTTP 201)".

## Claim 17 — ~7400 LOC Python
**Location:** ch01_introduction.tex:L45
**Quote:** "The Python stack described in §\ref{sec:pi-stack} (~7400 LOC across the Pi packages listed in Appendix~\ref{app:bom})"
**Verdict:** PARTIAL
**Computed/Evidence:** `wc -l Pi/*.py` sums to **7922** lines across 10 files (buzzer 88, capture 191, dashboard 2064, rf_backend 69, rf_hat 975, sat_library 487, satnogs 354, station 2947, transmit 185, wifi_provision 562). Under-reports by 522 lines (~7%).
**Note:** Update to "~7900 LOC" or "~8 kLOC". MEMORY.md also says ~7400 and is stale.

## Claim 18 — satnogs_protocol.cpp ~250 lines
**Location:** ch05_firmware_software.tex:L92
**Quote:** "The full EasyComm + extensions parser is implemented in satnogs_protocol.cpp (~250 lines of C++)."
**Verdict:** OK
**Computed/Evidence:** `wc -l Firmware/rp2040-satnogs-rotator/src/satnogs_protocol.cpp` = **250** lines (header .h adds 32). Exactly matches.

## Claim 19 — Rotator socket timeout 0.5 s, 15 s keepalive during AOS wait
**Location:** ch06_results.tex:L287
**Quote:** "Reduced to 0.5 s with fire-and-forget sends, plus a 15 s keepalive ping during AOS wait"
**Verdict:** OK
**Computed/Evidence:** station.py:L571 `self.sock.settimeout(0.5)` on Rotctld connect. L1348-1359 AOS wait loop: `_last_keepalive = time.monotonic()`; every 15 s re-sends `rotctl.set_position(rise_az, 0)`. "Fire-and-forget": `_cmd()` at L582-589 sends then tries `recv` with 0.5 s timeout that falls through to empty string — not strictly fire-and-forget (still waits 0.5 s) but effectively so at the protocol level.
**Note:** Minor nuance — "fire-and-forget" is generous; the recv still blocks up to 0.5 s. Optional reword: "reduced the blocking window to 0.5 s".
