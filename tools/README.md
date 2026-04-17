# tools/

Offline analysis utilities. These do **not** run on the station — they process captured RF data after a pass.

| Tool | Input | Output |
|------|-------|--------|
| `analyze_passes.py` | `captures/**/*.raw` + pass CSV logs | Statistical reception report + figures (used to generate `captures/evidence/signal_evidence.md`) |
| `ax100_decoder.py` | Raw bitstream from CC1200 serial mode | Decoded GOMspace AX.100 Mode 5 frames |
| `decode_ax100.py` | CC1200 FIFO dumps (`*.raw`) | AX.100 Mode 5 ASM + Golay(24,12) + RS(255,223) decode |
| `decode_serial.py` | Oversampled CC1200 blind-mode serial captures | Clock-recovered, resampled bitstream |
| `usp_decoder.py` | Raw bitstream | Unified Space Protocol (GMSK) frames |

All scripts are standalone CLIs. Run with `--help` for usage. None are imported by the station stack in `Pi/` — they exist to post-process the data the station produces.
