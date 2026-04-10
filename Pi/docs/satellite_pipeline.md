# Satellite Data Pipeline — How Sats Get On Screen

## Overview

```
CelesTrak/AMSAT TLE → PyEphem pass prediction → SatNOGS DB transmitter lookup → filtering → dashboard
```

## Step 1: TLE Acquisition

**Sources (priority order):**
1. Dashboard cache (`~/.station_tles.json`) — shared, <2h old
2. AMSAT (`amsat.org/tle/current/nasabare.txt`) — primary, reliable
3. CelesTrak — fallback (amateur, weather, NOAA, stations)

**station.py** checks dashboard cache first (avoids API thrashing), falls back to AMSAT + CelesTrak if stale (>30 min).

## Step 2: Pass Prediction

- `find_passes(tles, hours=12)` uses PyEphem
- Filter: minimum 5deg elevation, >10s duration
- Returns: `rise_time`, `set_time`, `max_el`, `rise_az`, `set_az`, `duration`
- Dashboard computes trajectory (10 points for sky plot)

## Step 3: Transmitter Enrichment

### SatNOGS DB API (`dashboard.py`)
- Endpoint: `https://db.satnogs.org/api/transmitters/?format=json&status=active`
- **Server-side filters:**
  - `type` in {`Transmitter`, `Transceiver`} — excludes Receiver (uplink-only)
  - Frequency: 420-450 MHz (UHF) or 130-175 MHz (VHF)
  - `alive=true`
- **Cache:** 12 hours
- **CRITICAL: If no transmitter found for a satellite, it is DROPPED from the pass list entirely**

### Per-satellite transmitter data:
```json
{
  "freq_mhz": 437.250,
  "mode": "GMSK",
  "baud": 9600,
  "band": "UHF",
  "cc1200_ok": true,
  "rtlsdr_ok": true,
  "description": "Mode U - 9k6 GMSK"
}
```

### Recently Heard Activity (background fetch)
- Endpoint: `network.satnogs.org/api/observations/?vetted_status=good&...`
- Provides: `last_heard_days`, `obs_count_30d`, `activity_score`
- Cache: disk (`~/.satnogs_cache/recently_heard.json`) + RAM (1h TTL)

### Local Profiles (`sat_profiles.json`)
- 10 known satellites with hand-tuned modulation params
- **Checked FIRST** — overrides SatNOGS DB data
- Contains: freq, modulation format, symbol rate, deviation, bandwidth, sync word

## Step 4: CC1200 Compatibility Check

**CC1200 hardware can decode:**
- `2-FSK`, `2-GFSK`, `4-FSK`, `4-GFSK`, `ASK`, `OOK`

**SatNOGS mode mapping:**
| SatNOGS Mode | CC1200 Mode | cc1200_ok |
|---|---|---|
| FSK, FFSK | 2-FSK | YES |
| GFSK, GMSK, MSK | 2-GFSK | YES |
| 4FSK | 4-FSK | YES |
| OOK, ASK | native | YES |
| AFSK <=1200 baud | — | NO (Bell 202 audio tones) |
| BPSK | — | NO |
| LRPT, HRPT | — | NO |

**Note:** Even "CC1200 compatible" modes like GMSK may use USP or AX.100 framing that the CC1200 packet engine cannot decode. The `cc1200_ok` flag only indicates modulation compatibility, NOT protocol compatibility.

## Step 5: Filtering

### Server-side (dashboard.py)
- Passes without transmitter data are **silently dropped**
- No indication to the user that satellites were removed

### Client-side (dashboard.html)
Default active filter: **CC1200** (only `cc1200_ok == true`)

| Filter | Condition |
|---|---|
| All | clears all filters |
| UHF | 420-450 MHz |
| VHF | 130-175 MHz |
| CC1200 | `cc1200_ok == true` **(DEFAULT)** |
| SDR | `rtlsdr_ok == true` (always true) |
| >45deg | `max_el >= 45` |
| >20deg | `max_el >= 20` |
| Heard <30d | `recently_heard == true` |
| Heard <7d | `last_heard_days <= 7` |

## Step 6: Radio Configuration (when Track is pressed)

### Profile Lookup Chain (`station.py`)
1. Check local `sat_profiles.json` by NORAD ID or name
2. If not found, query SatNOGS DB API for transmitters
3. Pick best: CC1200-compatible first, then closest to 435 MHz

### Configuration Tiers
| Tier | Trigger | What happens | Speed |
|---|---|---|---|
| C | `profile.smartrf_profile` set | Full register dump from SmartRF export | ~500ms |
| B | `profile.modulation` set | Write ~15 critical registers (sym rate, dev, BW, sync) | ~100ms |
| A | freq only (default) | Just `set_frequency()`, keep existing modulation | ~20ms |

### Modulation Parameter Derivation (Tier B, from SatNOGS baud rate)
```
GMSK/MSK:  deviation = baud / 4          (h = 0.5)
FSK:       deviation = baud * 0.35       (h ~ 0.7)
RX BW:     max(12.5, (baud + 2*dev) * 1.2 / 1000) kHz   (Carson's rule + 20%)
```

Example — 9600 baud GMSK:
```
deviation = 9600 / 4 = 2400 Hz
rx_bw = (9600 + 2*2400) * 1.2 / 1000 = 17.28 kHz
```

### Base SmartRF Profile (loaded at init)
- File: `configs/smartrf_uhf_435.txt`
- Default: 435 MHz, 2-GFSK, 2400 bps, 5 kHz deviation, 25 kHz RX BW
- Tier A satellites inherit these settings (only freq changes)

## Step 7: Doppler Correction

- Runs at 2 Hz during pass (`--doppler` flag, default on)
- Uses PyEphem `range_velocity` for range rate
- `uhf_base_freq_hz` stored separately from Doppler-shifted value
- Small shifts (<200 kHz): FREQOFF register (no packet loss)
- Large shifts: SIDLE → FREQ write → SCAL → SRX

## Data Flow Diagram

```
TLE Sources (AMSAT, CelesTrak)
         |
    [~/.station_tles.json cache]
         |
    PyEphem Pass Prediction (5deg min, >10s)
         |
    SatNOGS DB API (transmitter lookup)
    |                    |
    | freq/mode/baud     | recently heard score
    |                    |
    [Transmitter cache 12h]
         |
    Server-side filter: drop passes with no transmitter
         |
    Dashboard /api/passes response
         |
    Client-side filter: CC1200 (default), freq band, elevation, activity
         |
    User clicks Track
         |
    station.py: lookup_with_satnogs_fallback()
    |                              |
    | Local sat_profiles.json      | SatNOGS DB API
    | (10 sats, hand-tuned)        | (fallback, auto-derived params)
         |
    configure_for_satellite() → Tier C/B/A
         |
    CC1200 register writes + set_frequency()
         |
    Pass tracking loop (rotator + Doppler + RX streaming)
```

## Known Issues

- **Most satellites show 0 packets** because they use USP or AX.100 framing that the CC1200 packet engine cannot decode, even though the modulation is compatible
- **Satellites without SatNOGS DB transmitter data are silently dropped** — no indication to user
- **CC1200 filter is default-active** — hides BPSK, AFSK, and other non-CC1200 satellites
- **Tier A config is a guess** — satellite gets default 2.4k GFSK modulation, likely wrong
- **No sync word info from SatNOGS DB** — CC1200 needs exact sync word to trigger packet decode
