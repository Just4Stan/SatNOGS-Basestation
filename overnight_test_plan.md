# Overnight Ground Station Test Plan
# Started: 2026-04-12 ~21:20 CEST
# Target end: 2026-04-13 ~12:00 CEST (15 hours)
# Agent: Claude Opus 4.6

## Pi Access
- IP: 192.168.0.161 (satnogs.local)
- SSH: sshpass -p 'satnogs123' ssh pi@192.168.0.161
- Dashboard: https://192.168.0.161:5000

## PHASE 1: SETUP & BASELINE (21:20-22:00)
- [ ] 1.1 Set up persistent logging on Pi (tee all service logs)
- [ ] 1.2 Open dashboard in browser, screenshot initial state
- [ ] 1.3 Check all 3 services healthy (dashboard, station, rotctld)
- [ ] 1.4 Record baseline RSSI, packet counts, error counts
- [ ] 1.5 Check disk space, memory, CPU — establish resource baseline
- [ ] 1.6 Verify station.conf is correct (lat/lon/elev)
- [ ] 1.7 Count total satellites in TLE cache
- [ ] 1.8 List upcoming passes for next 4 hours
- [ ] 1.9 Check CC1200 health via dashboard debug panel
- [ ] 1.10 Set up watchdog script on Pi that pings services every 60s

## PHASE 2: MODE 1 — TRACK MODE STRESS TEST (22:00-01:00)
- [ ] 2.1 Ensure auto_mode=track in station.conf
- [ ] 2.2 Let station auto-pick passes, monitor first full pass
- [ ] 2.3 Check dashboard shows live AZ/EL, RSSI, sky plot
- [ ] 2.4 Verify Doppler correction is updating (freq changes in status)
- [ ] 2.5 Check pass CSV logging is working
- [ ] 2.6 Check raw binary dumps are being created
- [ ] 2.7 Monitor for "SKIP" messages — count how many sats get skipped vs tracked
- [ ] 2.8 Verify FIFO error recovery (check rx_overflow_count, rx_fifo_err_count)
- [ ] 2.9 Test manual track override via dashboard "Track" button
- [ ] 2.10 Verify park between passes works
- [ ] 2.11 Check rotctld connection stability (socket reconnect)
- [ ] 2.12 Monitor memory usage trend (512MB Pi — watch for leaks)
- [ ] 2.13 Check if serial mode fallback triggers (30s no-data threshold)
- [ ] 2.14 Validate any received packets — check entropy, structure
- [ ] 2.15 Screenshot dashboard during active tracking (sky plot, RSSI graph)

## PHASE 3: MODE 2 — SCAN MODE STRESS TEST (01:00-04:00)
- [ ] 3.1 Switch to scan mode via dashboard Settings
- [ ] 3.2 Verify station.py picks up mode change without restart
- [ ] 3.3 Monitor scan schedule generation (how many visits scheduled)
- [ ] 3.4 Check dwell time per satellite (should be ~30-45s)
- [ ] 3.5 Verify per-satellite CC1200 reconfiguration (freq + modulation changes)
- [ ] 3.6 Monitor scan_pos counter in status (x/N progression)
- [ ] 3.7 Check that scan properly parks between visits
- [ ] 3.8 Verify abort mechanism works (press Stop during scan)
- [ ] 3.9 Time the slew between satellites — is 8s enough?
- [ ] 3.10 Count total satellites visited vs packets received
- [ ] 3.11 Check for protocol inference errors (wrong modulation guessed)
- [ ] 3.12 Monitor CC1200 MARC state transitions during reconfig
- [ ] 3.13 Verify raw dump files created per visit
- [ ] 3.14 Stress test: change scan_dwell to 15s — does it break?
- [ ] 3.15 Screenshot scan mode dashboard (scan_pos indicator)

## PHASE 4: MODE 3 — OFF MODE + MANUAL CONTROL (04:00-05:00)
- [ ] 4.1 Switch to auto_mode=off via dashboard
- [ ] 4.2 Verify station.py stops tracking and idles
- [ ] 4.3 Test manual jog controls (AZ+, AZ-, EL+, EL-)
- [ ] 4.4 Test manual position entry (specific AZ/EL)
- [ ] 4.5 Test Park button from Off mode
- [ ] 4.6 Switch back to Track — verify it resumes within 1 cycle
- [ ] 4.7 Switch to Scan — verify it starts scan cycle
- [ ] 4.8 Rapid mode switching: Off→Track→Scan→Off→Track (5x)
- [ ] 4.9 Check for race conditions in mode switching
- [ ] 4.10 Verify dashboard shows correct mode indicator

## PHASE 5: DASHBOARD UX AUDIT (05:00-07:00)
- [ ] 5.1 Test all 3 pages: Track, Control, Settings
- [ ] 5.2 Check responsive layout (resize window to phone size)
- [ ] 5.3 Verify sky plot renders correctly during pass
- [ ] 5.4 Check RSSI display updates in real-time
- [ ] 5.5 Verify pass list loads and filters work (UHF/VHF/CC1200/SDR/elevation)
- [ ] 5.6 Test satellite search/filter in pass list
- [ ] 5.7 Check CC1200 debug panel updates (MARC, FIFO, overflow, uptime)
- [ ] 5.8 Verify "Track" button on individual satellites works
- [ ] 5.9 Test GPS location button (simulated — check API call)
- [ ] 5.10 Check Settings page: all toggles work, persist to station.conf
- [ ] 5.11 Verify shutdown button triggers confirmation
- [ ] 5.12 Check for JavaScript console errors
- [ ] 5.13 Test page refresh during active tracking — state preserved?
- [ ] 5.14 Check pass history display (recent passes)
- [ ] 5.15 Monitor network requests for failed API calls
- [ ] 5.16 Check for memory leaks in JS (long-running page)
- [ ] 5.17 Verify WiFi provisioning page accessible if needed
- [ ] 5.18 Check certificate warning handling (self-signed SSL)
- [ ] 5.19 Test Calibrate North button in setup wizard
- [ ] 5.20 Verify hardware detection API returns correct info

## PHASE 6: DATA VALIDATION & ANALYSIS (07:00-09:00)
- [ ] 6.1 Download all raw captures from Pi
- [ ] 6.2 Analyze .json metadata files for each pass
- [ ] 6.3 Check packet entropy distribution — real data vs noise
- [ ] 6.4 Validate AX.25 frame structure on any decoded packets
- [ ] 6.5 Check RSSI profiles across passes (should rise/fall with elevation)
- [ ] 6.6 Verify Doppler curves match expected profiles
- [ ] 6.7 Cross-reference received frequencies with SatNOGS DB
- [ ] 6.8 Check for duplicate packets in logs
- [ ] 6.9 Analyze pass CSV files — AZ/EL tracking accuracy
- [ ] 6.10 Compare CC1200 FREQOFF_EST readings to computed Doppler

## PHASE 7: RELIABILITY & EDGE CASES (09:00-11:00)
- [ ] 7.1 Kill station.py and verify systemd restarts it
- [ ] 7.2 Kill dashboard.py and verify restart
- [ ] 7.3 Simulate rotctld crash — does station.py reconnect?
- [ ] 7.4 Check UART reliability — any framing errors overnight?
- [ ] 7.5 Check if TLE cache refreshes properly
- [ ] 7.6 Verify pass prediction accuracy (compare predicted vs actual AOS)
- [ ] 7.7 Monitor for Python exceptions in station.log
- [ ] 7.8 Check for file descriptor leaks (lsof count)
- [ ] 7.9 Verify atomic status file writes (no truncated JSON reads)
- [ ] 7.10 Test what happens when disk gets full (won't actually fill)
- [ ] 7.11 Check if SatNOGS DB API calls succeed or timeout gracefully
- [ ] 7.12 Verify watchdog timer doesn't false-trigger

## PHASE 8: BUG FIXES & IMPROVEMENTS (11:00-12:00)
- [ ] 8.1 Compile all bugs found overnight
- [ ] 8.2 Implement fixes for critical issues
- [ ] 8.3 Deploy fixes to Pi via sshpass/scp
- [ ] 8.4 Restart services, verify fixes work
- [ ] 8.5 Write overnight test report
- [ ] 8.6 Update MEMORY.md with findings

## METRICS TO TRACK CONTINUOUSLY
- Total passes tracked
- Total packets received (valid vs noise)
- Total bytes captured
- Peak/min/avg RSSI per pass
- Service uptime (no crashes)
- Memory usage trend
- Disk usage trend
- CC1200 overflow/error counts
- Dashboard response time
- Rotator tracking accuracy
