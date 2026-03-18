# SatNOGS Integration Guide

How the Aether-Basestation integrates with the SatNOGS ecosystem and why
our approach differs from a standard SatNOGS station.

## The SatNOGS Ecosystem

SatNOGS has four distinct components. Understanding which ones we use (and
which we don't) is critical.

### 1. SatNOGS Network (network.satnogs.org)

The central coordination hub. Manages station registration, observation
scheduling, and result storage.

| Feature | Description |
|---|---|
| Station registry | Every ground station gets an ID, location, antenna info |
| Observation scheduling | Community members schedule satellite passes on "Online" stations |
| Result storage | Audio recordings, waterfall images, decoded data from observations |
| Quality vetting | Observations rated good/bad/failed by the community |

**API**: REST at `network.satnogs.org/api/`
- `GET /stations/` — list stations (public)
- `GET /observations/` — list observations (public)
- `GET /jobs/` — pull scheduled work (authenticated, station-specific)

**How "Online" works**: The official satnogs-client polls `GET /api/jobs/`
every 60 seconds. The server updates `last_seen` on each poll. If
`last_seen` goes stale, the station drops to "Offline". There is no
dedicated heartbeat endpoint — job polling IS the heartbeat.

### 2. SatNOGS DB (db.satnogs.org)

A crowd-sourced database of satellite information and received telemetry.
Separate service, separate accounts, separate API tokens.

| Feature | Description |
|---|---|
| Satellite catalog | NORAD IDs, names, operational status |
| Transmitter database | Frequencies, modulation modes, baud rates per satellite |
| Telemetry frames | Raw decoded frames submitted via SiDS API |
| TLE data | Orbital elements for pass prediction |

**API**: REST at `db.satnogs.org/api/`
- `GET /satellites/` — satellite catalog
- `GET /transmitters/` — transmitter info (used by our `sat_library.py`)
- `POST /telemetry/` — submit decoded frames (SiDS protocol)

### 3. SatNOGS Client (satnogs-client)

The official ground station software. **We do NOT run this.**

It is designed for SDR-based stations (GNU Radio + SoapySDR). It:
- Polls `/api/jobs/` for scheduled observations (this is the heartbeat)
- Controls SDR + rotator during observations
- Records audio + waterfall images
- Uploads results back to the Network
- Requires significant RAM for GNU Radio flowgraphs

### 4. SiDS Telemetry API

The Satellite Information Distribution System. A simple POST endpoint for
submitting decoded frames to the DB. **This is what we use.**

```
POST https://db.satnogs.org/api/telemetry/
Content-Type: application/x-www-form-urlencoded
Authorization: Token <DB_API_TOKEN>

noradID=99999&source=ON4xxx&timestamp=2026-03-18T14:30:00.000Z
&frame=3C594D...&locator=longLat&longitude=4.97&latitude=51.1
&version=satnogs-cc1200-1.0
```

Returns: 201 (success), 400 (bad request), 401 (unauthorized)

## API Tokens

Two completely separate token systems:

| Token | Source | Purpose | We use it? |
|---|---|---|---|
| Network API Token | network.satnogs.org → Dashboard → API Key | Authenticate job polling (heartbeat) | No* |
| DB API Token | db.satnogs.org → Settings → API Key | Authenticate telemetry frame submission | Yes |

*We store the Network token in `station.conf` for potential future use, but
currently only the DB token is actively used for SiDS submission.

## Our Station: Aether-Basestation (#4712)

| Field | Value |
|---|---|
| Station ID | 4712 |
| Name | Aether-Basestation |
| QTH Locator | JO21lc |
| Coordinates | 51.100 N, 4.970 E |
| Altitude | 25 m |
| Min Horizon | 8 deg |
| Antennas | Yagi (VHF, UHF) |
| Owner | stancoene |

## Why We're Different

### Hardware modem vs SDR

Standard SatNOGS stations use an SDR (RTL-SDR, LimeSDR, etc.) that
produces raw IQ samples. GNU Radio flowgraphs process these into audio,
waterfall images, and decoded data.

Our station uses a **TI CC1200 hardware modem**. The CC1200 handles
modulation/demodulation internally and outputs decoded packets directly.
There are no IQ samples, no audio stream, no waterfall. This means:

- We cannot run satnogs-client flowgraphs (no IQ input)
- We cannot produce waterfall images (no spectrum data)
- We CAN submit decoded frames to SatNOGS DB via SiDS
- We CAN track satellites with the rotator via hamlib

### Mobile station

Standard SatNOGS stations are fixed installations with known GPS
coordinates, static IP, and always-on internet. Our station is **mobile**:

- Deployed in the field on a tripod
- GPS coordinates change every deployment
- WiFi network changes (phone hotspot, campus WiFi, etc.)
- No permanent internet — frames are queued offline and submitted later
- AZ calibration requires pointing north at power-on
- EL calibration is automatic via ADXL345 IMU

This fundamentally changes the user experience (see Field Deployment UX
below).

### Station status: permanently "Offline"

Because we don't run satnogs-client and don't poll `/api/jobs/`, our
station will always show as "Offline" on network.satnogs.org. This is
expected and acceptable for a thesis project. The station still contributes
telemetry to the global database via SiDS.

To make the station appear "Online", we would need to add a minimal job
poller (not recommended — community members could schedule SDR observations
we can't fulfill) or add an RTL-SDR alongside the CC1200 and run the full
satnogs-client (Pi 3A+ RAM constraint makes this impractical).

## Integration Architecture

```
                    ┌─────────────────────────────┐
                    │     SatNOGS DB              │
                    │   db.satnogs.org            │
                    │                             │
                    │  POST /api/telemetry/       │◄──── SiDS frames
                    │  GET  /api/transmitters/    │────► sat profiles
                    └─────────────────────────────┘
                              ▲           │
                              │           │
                    ┌─────────┴───────────┴───────┐
                    │     Pi 3A+ (station.py)     │
                    │                             │
                    │  satnogs.py ── SiDS client  │
                    │  sat_library.py ── profiles │
                    │  rf_hat.py ── CC1200 link   │
                    │  dashboard.py ── HTTPS UI   │
                    └──────┬──────────┬───────────┘
                           │          │
                    /dev/ttyACM0  /dev/serial0
                           │          │
                    ┌──────┴──┐  ┌────┴────┐
                    │ Rotator │  │ RF HAT  │
                    │  Pico   │  │  Pico   │
                    │ (USB)   │  │ (UART)  │
                    └─────────┘  └─────────┘
```

What we use from SatNOGS:
- **DB transmitter API** — `sat_library.py` fetches satellite radio
  profiles (freq, modulation, baud rate) for per-pass CC1200 reconfiguration
- **DB SiDS API** — `satnogs.py` submits decoded CC1200 frames
- **Station registration** — station 4712 on network.satnogs.org (metadata only)

What we don't use:
- satnogs-client (SDR-only, too heavy for Pi 3A+)
- Network job scheduling (we do our own pass prediction via PyEphem)
- Audio/waterfall artifacts (CC1200 doesn't produce IQ samples)

## Field Deployment UX

Our mobile station has a unique first-time-setup flow each deployment:

```
1. Place station on tripod, point antenna NORTH
2. Power on Pi (EL auto-homes via IMU)
3. Phone connects to Pi WiFi (or shared network)
4. Open https://<pi-ip>:5000 on phone browser
5. Dashboard loads → tap "Use My GPS"
6. station.py detects location → fetches TLEs → begins tracking
7. CC1200 captures packets during passes → queued for SiDS submission
8. When internet is available, queued frames auto-submit to DB
9. Dashboard shows live rotator position, pass info, RF status
10. Tap "Shutdown" on dashboard when done → Pi parks + powers down
```

Key UX considerations for mobile operation:
- **No SSH needed** — everything via phone browser dashboard
- **GPS every deployment** — location changes, must be set each time
- **Offline-first** — frames queued locally, submitted when internet available
- **North calibration** — AZ encoder zeros on boot, user must point north
- **Self-signed HTTPS** — required for browser Geolocation API, phone warns once

## Configuration

All SatNOGS credentials are stored in `~/station.conf` on the Pi:

```json
{
    "lat": 51.1,
    "lon": 4.97,
    "elev": 25,
    "callsign": "ON4xxx",
    "satnogs_station_id": 4712,
    "satnogs_db_token": "<DB_API_TOKEN>",
    "satnogs_network_token": "<NETWORK_API_TOKEN>"
}
```

The `lat`/`lon`/`elev` fields are updated by the dashboard when the user
taps "Use My GPS". The tokens and station ID are configured once during
initial setup.

## Files

| File | Purpose |
|---|---|
| `Pi/satnogs.py` | SiDS telemetry submission client with offline queue |
| `Pi/sat_library.py` | Satellite profile DB + SatNOGS DB API fetcher |
| `Pi/sat_profiles.json` | Local satellite radio profiles (10 sats) |
| `Pi/station.py` | Pass controller — calls satnogs.py for frame submission |
| `Pi/dashboard.py` | Web dashboard — shows SatNOGS submission stats |
