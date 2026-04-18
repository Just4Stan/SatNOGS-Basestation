# G11 — Field Reception Fact-Check (ch06)

**Sources consulted:**
- `/Users/stan/Documents/GitHub/SatNOGS-Basestation/captures/evidence/signal_evidence.md` (exists)
- `/Users/stan/Documents/GitHub/SatNOGS-Basestation/captures/evidence/miman_decode_report.json` (exists)
- `/Users/stan/Documents/GitHub/SatNOGS-Basestation/captures/evidence/raw_analysis_report.json` (exists)
- `/Users/stan/Documents/GitHub/SatNOGS-Basestation/captures/20260409/pass_logs/` (present, 204 CSVs)
- `/Users/stan/Documents/GitHub/SatNOGS-Basestation/captures/20260412/` (present, 10 CSVs)
- `/Users/stan/Documents/GitHub/SatNOGS-Basestation/Pi/configs/smartrf_uhf_435.txt`
- `/Users/stan/Documents/GitHub/SatNOGS-Basestation/Pi/sat_profiles.json`
- `/Users/stan/Documents/GitHub/SatNOGS-Basestation/Pi/station.py`
- `/Users/stan/Documents/GitHub/SatNOGS-Basestation/Firmware/rp2040-rf-hat/src/proto.c`
- `/Users/stan/Documents/GitHub/SatNOGS-Basestation/Pi/capture.py`, `transmit.py`

---

| # | Claim (ch06 line) | Verdict | Notes |
|---|---|---|---|
| 1 | ch06 L155: "~210 passes total — 16 Apr1 / 184 Apr9 / 10 Apr12" | **VERIFIED** | `pass_logs/` contains 16×`20260401_*`, 184×`20260409_*`, +4×`20260402_*` (not cited). `captures/20260412/` contains 10 CSVs. 16+184+10 = 210 holds. Note: 4 extra Apr 2 logs exist and are unacknowledged. |
| 2 | ch06 L198: "+4.2 dB rise, 0-20° → 60-90° bin, 62 passes" | **VERIFIED** | signal_evidence.md §1: −91.7→−87.5 dBm = +4.2 dB. "62 tracked passes" matches. N-per-bin still "TBD" in source; Std Dev also TBD. |
| 3 | ch06 L216: "up to 81 bytes, 4 independent captures" | **VERIFIED** | signal_evidence.md §2 states 4 SNUGLITE-II captures; raw_analysis_report.json lists SNUGLITE-II_a/b/c/d. "81 bytes" appears verbatim in evidence. Match-count histogram by pair documented (275/365/706/372/655/716). |
| 4 | ch06 L220: "17-29 dB RSSI variation AOS→peak→LOS" | **UNVERIFIABLE from source** | Neither signal_evidence.md nor raw_analysis_report.json states 17 dB or 29 dB numerically. Figure `09_rssi_vs_range.png` referenced for "six highest-dynamic-range passes" but explicit range numbers not in the source doc. Likely derived from underlying CSV data but number not surfaced in evidence files. |
| 5 | ch06 L236/L141: "±9 kHz sweep at 432 MHz" | **CONSISTENT, minor drift** | signal_evidence.md §4 cites "~18 kHz at 437 MHz" (= ±9 kHz) from LEO 400-600 km. Physics check: max radial Doppler at 432 MHz with v=7.8 km/s → ±11.2 kHz. ±9 kHz is a typical (non-overhead) pass value, plausible but claim is stated at 432 MHz whereas source doc figure is at 437 MHz. Close enough. |
| 6 | ch06 L245: "entropy 6.95-7.91 bits/byte" | **VERIFIED** | signal_evidence.md §6: GO-32=6.95, BLUEWALKER_3=7.91. Range matches exactly. |
| 7 | ch06 L247: "MIMAN Golay rate 73% Apr9 / 66% Apr12" | **VERIFIED** | miman_decode_report.json: apr09=73.3%, apr12=66.1%. signal_evidence.md §7 rounds identically. |
| 8 | ch06 L247: "Random false-positive baseline ≈57% Golay(24,12) 3-error" | **VERIFIED (as stated in source)** | signal_evidence.md §7: "~57% for Golay(24,12) with 3-error correction". Parroted to ch06 faithfully. Underlying math not independently recomputed here. |
| 9 | ch06 L123-130: "Indoors −92 to −105 dBm, outdoors −77 to −80 dBm" | **UNVERIFIABLE** | No indoor/outdoor RSSI table found in evidence files. signal_evidence.md only gives system noise floor ≈−95.8 dBm. Claim is consistent with general noise-floor-above-thermal pattern but specific min/max values not in audited sources. |
| 10 | ch06 L128: "AGC_GAIN_ADJUST = −99 dB" | **VERIFIED** | `Pi/configs/smartrf_uhf_435.txt:27`: `0x9D, // AGC_GAIN_ADJUST RSSI Offset Configuration (-99 dB per CC120X UG Table 8)`. VHF profile has 0x00; UHF uses −99 dB. |
| 11 | ch06 L272: "Measured effective noise floor −100 to −107 dBm" | **UNVERIFIABLE** | signal_evidence.md §5 states noise floor ≈−95.8 dBm, not −100 to −107. ch06's figure is 5-12 dB lower (more noise) than the evidence document. No session-level min/max noise-floor-vs-pointing data in the audited files. Numbers match CLAUDE.md memory but not the checked-in evidence artifact. |
| 12 | ch06 L272: "−127 dBm thermal baseline" | **CONSISTENT** (physics, not in source) | Not stated in signal_evidence.md. Matches standard kTB calculation for typical receiver bandwidth (≈25 kHz CC1200) at 290 K: kTB = −174 + 10log(25e3) ≈ −130 dBm. −127 dBm plausible for wider IF BW. Not a source-citable number. |
| 13 | ch06 L173: "AX.100 ASM sync word 0x930B51DE" | **VERIFIED** | `Pi/sat_profiles.json` references 0x930B51DE at 8 locations; `Pi/station.py:126,148,163` uses `"930B51DE"` as AX.100 Mode 5/6 sync. |
| 14 | ch06 L173: "Fixed-length 255-byte chunks in FIFO mode" | **VERIFIED** | `Firmware/rp2040-rf-hat/src/proto.c:555,583,601`: `g_last_rx_chunk = (uint8_t)(flen > 255 ? 255 : flen);` and `uint8_t out[1 + 255]` at :1198 confirms 255-byte payload cap. |
| 15 | ch05 L150: "Serial mode for ≥4800 bps FSK" | **VERIFIED** | `Pi/station.py:133,159` uses `baud >= 4800` condition with "2-FSK"/"2-GFSK" check; ch05 L150 matches. |
| 16 | ch07 L60: "approximately 210 tracked passes" | **VERIFIED consistent** | Matches ch06 L155 count 16+184+10=210. Self-consistent across chapters. |
| 17 | ch06 L277: "Total: 16+184+10 = 210 files" | **VERIFIED** | Arithmetic correct; file counts confirmed in §1 above. |
| 18 | Reception-chain validated via "short-range ground-to-ground CC1200-to-CC1200" (ch06 L164) | **PARTIALLY VERIFIED** | `Pi/capture.py` (UHF RX standalone) and `Pi/transmit.py` (VHF TX standalone) exist and are documented as Rx/Tx tools. No grep hit for the literal phrase "short-range", "ground-to-ground", "loopback" or explicit test-framework evidence of cross-CC1200 ground decode in the repo. The code paths support the claim (both TX and RX standalone utilities exist) but no captured test log or report demonstrates a successful end-to-end decode on the ground. Claim relies on unlogged bench work. |

---

## Cross-chapter consistency check

- **Pass count 210** is self-consistent between ch06 L155, ch06 L277 arithmetic, and ch07 L60. 
- **Zero protocol-valid frames** asserted identically in ch06 L155-162 and ch07 L28. 
- **LNA root-cause** mentioned in both ch06 L273 and ch07 L60 — consistent. 
- **"Measured effective noise floor −100 to −107 dBm"** in ch06 L272 conflicts with `signal_evidence.md §5` value of −95.8 dBm. ch06 matches CLAUDE.md memory but not the checked-in evidence doc — this is a **source-of-truth mismatch** between ch06 and signal_evidence.md.
- **Antenna claim drift (external to claim list):** ch06 L273 discusses LNA as dominant gap and states the Yagi is already in use (via link-budget cross-ref). signal_evidence.md §9 still recommends "use directional antenna (10-element Yagi)" as a future recommendation, implying turnstile in use at time of writing. ch06 is now current; signal_evidence.md is stale on antenna — not a ch06 error but worth noting that the cited evidence doc is dated 2026-04-13 and partially outdated.

---

## Issues requiring author action

1. **Claim 4 (17-29 dB RSSI variation):** add a source reference or annotate as "derived from CSV logs, not in evidence summary". If taken from `09_rssi_vs_range.png`, state the computation.
2. **Claim 9 (indoor/outdoor RSSI min/max):** no evidence file documents these specific bounds. Either add measurement provenance (which bench-log / session) or soften to "typical" values.
3. **Claim 11 (−100 to −107 dBm noise floor):** conflicts with signal_evidence.md §5 (−95.8 dBm). Reconcile — either update signal_evidence.md with updated measurements or weaken ch06 claim to match the cited source.
4. **Claim 18 (ground-to-ground validation):** no artefact (log/capture/photo/report) was found documenting a successful end-to-end CC1200↔CC1200 bench decode. Add a verification log file or a paragraph describing the test protocol and result.
5. **Unacknowledged Apr 2 logs:** 4 CSVs under `20260409/pass_logs/` named `20260402_*.csv`. Either acknowledge (three-session claim is loose) or exclude from pass_logs folder.
