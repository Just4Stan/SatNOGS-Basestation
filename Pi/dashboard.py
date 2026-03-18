#!/usr/bin/env python3
"""
Ground Station Web Dashboard — phone-friendly control center.

Open https://<pi-ip>:5000 on your phone (same WiFi/hotspot network).
No external dependencies beyond standard library (ephem optional for passes).

Field workflow:
    1. Pi boots, dashboard starts automatically
    2. Open phone browser → dashboard
    3. Tap "Set Location" → phone GPS sends lat/lon to Pi
    4. Station auto-tracks passes (station.py reads ~/station.conf)
    5. Dashboard shows live AZ/EL, satellite, RSSI, packets
    6. Tap "Park" or "Shutdown" when done

Architecture:
    - GET  /                    → dashboard.html from same directory
    - GET  /api/status          → JSON status blob (polled by JS)
    - GET  /api/passes          → upcoming satellite passes (PyEphem)
    - POST /api/location        → save lat/lon/elev to ~/station.conf
    - POST /api/position        → send position command to rotctld
    - POST /api/park            → send park command to rotctld
    - POST /api/shutdown        → park rotator + sudo shutdown

Flags:
    --simulate                  → run in simulation mode (no hardware needed)
    --no-ssl                    → disable HTTPS (GPS location won't work on phone)
    --port <n>                  → HTTPS port (default 5000)
    --bind <addr>               → bind address (default 0.0.0.0)
"""

import os
import sys
import json
import math
import time
import ssl
import random
import socket
import signal
import argparse
import datetime
import threading
import subprocess
from pathlib import Path
from http.server import HTTPServer, BaseHTTPRequestHandler

# Optional: ephem for real pass prediction
try:
    import ephem
    EPHEM_AVAILABLE = True
except ImportError:
    EPHEM_AVAILABLE = False

# ---------------------------------------------------------------------------
# Config
# ---------------------------------------------------------------------------
ROTCTLD_HOST = "localhost"
ROTCTLD_PORT = 4533
STATUS_FILE = os.path.expanduser("~/.station_status.json")
STATION_CONF = os.path.expanduser("~/station.conf")
DASHBOARD_DIR = os.path.dirname(os.path.abspath(__file__))

# Default station location (Geel, Belgium) — used when ~/station.conf not set
DEFAULT_LAT = 51.1
DEFAULT_LON = 4.97
DEFAULT_ELEV = 25.0

# TLE cache
_tle_cache = {}           # name -> (line1, line2)
_tle_cache_time = 0.0     # unix timestamp of last fetch
TLE_CACHE_SECONDS = 7200  # 2 hours

# Content-Type map for static file serving
MIME_TYPES = {
    ".html": "text/html",
    ".js": "application/javascript",
    ".css": "text/css",
    ".svg": "image/svg+xml",
    ".json": "application/json",
    ".ico": "image/x-icon",
    ".png": "image/png",
    ".jpg": "image/jpeg",
    ".woff2": "font/woff2",
    ".woff": "font/woff",
}

# Simulation mode flag (set by main() before threads start)
SIMULATE = False

# ---------------------------------------------------------------------------
# Status data (shared between polling thread and HTTP handler)
# ---------------------------------------------------------------------------
status = {
    "rotator": {"az": None, "el": None, "connected": False, "state": "disconnected"},
    "rf": {"streaming": False, "rssi": None, "packets": 0, "freq_mhz": 0,
           "satellite": "", "pass_progress": 0},
    "system": {"cpu_temp": None, "ram_used_mb": 0, "ram_total_mb": 0, "uptime": ""},
    "location": {"lat": None, "lon": None, "elev": None, "set": False},
    "satnogs": {"enabled": False, "station_id": None, "submitted": 0,
                "queued": 0, "failed": 0, "last_submit": None},
    "last_update": "",
    "simulation": False,
}
status_lock = threading.Lock()


# ---------------------------------------------------------------------------
# Simulator — generates realistic fake data for development
# ---------------------------------------------------------------------------
class Simulator:
    """Generates realistic fake data for dashboard development."""

    def __init__(self):
        self.start_time = time.time()
        self.pkt_count = 0
        self.in_pass = False
        self.pass_start = 0
        self.pass_duration = 90
        self.pass_gap = 120  # seconds between simulated passes
        self.sat_names = ["ISS (ZARYA)", "NOAA 19", "METEOR-M2 3", "AetherSpace-1", "SO-50"]
        self.current_sat_idx = 0
        # Simulated position (can be overridden by /api/position)
        self._parked = False
        # Simulated SatNOGS counters
        self._satnogs_submitted = 7
        self._satnogs_queued    = 0
        self._satnogs_last_sub  = None

    def get_status(self):
        elapsed = time.time() - self.start_time

        # Determine if we're in a pass
        cycle = elapsed % (self.pass_duration + self.pass_gap)
        self.in_pass = cycle < self.pass_duration

        if self._parked:
            az = 0.0
            el = 0.0
            rssi = None
            sat_name = ""
            state = "parked"
            progress = 0.0
        elif self.in_pass:
            progress = cycle / self.pass_duration
            # Elevation: sine curve peaking at mid-pass
            el = 72 * math.sin(math.pi * progress)
            # Azimuth: sweep from rise_az to set_az
            az = (200 + 150 * progress) % 360
            rssi = -60 - 30 * (1 - math.sin(math.pi * progress))  # stronger at peak

            # Random packets near peak elevation
            if random.random() < 0.1 * math.sin(math.pi * progress):
                self.pkt_count += 1
                # Occasionally a simulated frame gets submitted to SatNOGS
                if random.random() < 0.5:
                    self._satnogs_submitted += 1
                    self._satnogs_last_sub = datetime.datetime.now().strftime("%H:%M:%S")
                else:
                    self._satnogs_queued = max(0, self._satnogs_queued + 1)

            sat_name = self.sat_names[self.current_sat_idx % len(self.sat_names)]
            state = "tracking"
        else:
            az = (elapsed * 0.5) % 360  # slow idle sweep
            el = 0.0
            rssi = None
            sat_name = ""
            state = "idle"
            if cycle < 5:  # just finished a pass — flush simulated queue
                self.current_sat_idx += 1
                if self._satnogs_queued > 0:
                    self._satnogs_submitted += self._satnogs_queued
                    self._satnogs_queued = 0
                    self._satnogs_last_sub = datetime.datetime.now().strftime("%H:%M:%S")
            progress = 0.0

        return {
            "rotator": {
                "az": round(az, 1),
                "el": round(max(0.0, el), 1),
                "connected": True,
                "state": state,
            },
            "rf": {
                "streaming": self.in_pass and not self._parked,
                "rssi": round(rssi, 1) if rssi is not None else None,
                "packets": self.pkt_count,
                "freq_mhz": 433.0,
                "satellite": sat_name,
                "pass_progress": round(progress * 100, 1) if self.in_pass else 0,
                "rf_backend": "cc1200",
                "radio_config": {
                    "freq_mhz": 433.0,
                    "modulation": "2-GFSK",
                    "symbol_rate_bps": 9600 if self.in_pass else 2400,
                    "profile": f"{sat_name} — 9600 GFSK" if self.in_pass else "default",
                } if self.in_pass and not self._parked else None,
            },
            "system": {
                "cpu_temp": round(48 + 5 * math.sin(elapsed / 30), 1),
                "ram_used_mb": 180 + int(20 * math.sin(elapsed / 60)),
                "ram_total_mb": 512,
                "uptime": f"{int(elapsed // 3600)}h{int((elapsed % 3600) // 60):02d}m",
            },
            "satnogs": {
                "enabled":     True,
                "station_id":  9999,
                "submitted":   self._satnogs_submitted,
                "queued":      self._satnogs_queued,
                "failed":      0,
                "last_submit": self._satnogs_last_sub,
            },
            "simulation": True,
        }

    def get_passes(self):
        """Generate fake upcoming passes with full trajectory data."""
        passes = []
        now = datetime.datetime.now(datetime.timezone.utc)

        configs = [
            ("ISS (ZARYA)",   15,  72.3, 600, 215, 320),
            ("NOAA 19",       45,  45.1, 900,  10, 170),
            ("METEOR-M2 3",   80,  28.5, 480, 300,  45),
            ("AetherSpace-1", 120, 55.0, 720, 180, 350),
            ("SO-50",         180, 82.1, 540,  90, 270),
        ]

        for name, mins_away, max_el, duration, rise_az, set_az in configs:
            rise = now + datetime.timedelta(minutes=mins_away)
            set_t = rise + datetime.timedelta(seconds=duration)

            # Generate trajectory points (21 points → 20 intervals)
            trajectory = []
            n_points = 20
            az_span = (set_az - rise_az + 360) % 360
            for i in range(n_points + 1):
                t_frac = i / n_points
                t_sec = t_frac * duration
                el = max_el * math.sin(math.pi * t_frac)
                az = (rise_az + az_span * t_frac) % 360
                trajectory.append({
                    "az": round(az, 1),
                    "el": round(max(0.0, el), 1),
                    "t": round(t_sec),
                })

            passes.append({
                "name": name,
                "rise_time": rise.strftime("%Y-%m-%dT%H:%M:%SZ"),
                "set_time": set_t.strftime("%Y-%m-%dT%H:%M:%SZ"),
                "max_el": max_el,
                "duration": duration,
                "rise_az": rise_az,
                "set_az": set_az,
                "trajectory": trajectory,
            })

        return {"passes": passes}

    def park(self):
        self._parked = True

    def set_position(self, az, el):
        # Cancel park mode when a position is commanded
        self._parked = False


# Singleton simulator — only used when SIMULATE is True
_simulator = None


def get_simulator():
    global _simulator
    if _simulator is None:
        _simulator = Simulator()
    return _simulator


# ---------------------------------------------------------------------------
# Persistent rotctld connection
# ---------------------------------------------------------------------------
_rotctl_sock = None
_rotctl_lock = threading.Lock()


def _rotctl_connect():
    """Get or create a persistent connection to rotctld."""
    global _rotctl_sock
    if _rotctl_sock is not None:
        return _rotctl_sock
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(2)
        s.connect((ROTCTLD_HOST, ROTCTLD_PORT))
        _rotctl_sock = s
        return s
    except Exception:
        _rotctl_sock = None
        return None


def _rotctl_disconnect():
    global _rotctl_sock
    if _rotctl_sock:
        try:
            _rotctl_sock.close()
        except Exception:
            pass
        _rotctl_sock = None


def _rotctl_cmd(cmd: str) -> str:
    """Send a command to rotctld over persistent connection. Returns response."""
    global _rotctl_sock
    with _rotctl_lock:
        s = _rotctl_connect()
        if not s:
            return ""
        try:
            s.sendall((cmd + "\n").encode())
            resp = s.recv(256).decode().strip()
            return resp
        except Exception:
            _rotctl_disconnect()
            return ""


# ---------------------------------------------------------------------------
# Polling functions (real hardware)
# ---------------------------------------------------------------------------
def poll_rotctld():
    """Query rotctld for current AZ/EL. Returns (connected, az, el)."""
    resp = _rotctl_cmd("p")
    if resp:
        lines = resp.split("\n")
        if len(lines) >= 2:
            try:
                return True, float(lines[0]), float(lines[1])
            except ValueError:
                pass
    return False, 0.0, 0.0


def read_station_status():
    """Read ~/.station_status.json written by station.py (max 30s stale)."""
    try:
        if os.path.exists(STATUS_FILE):
            mtime = os.path.getmtime(STATUS_FILE)
            if time.time() - mtime < 30:
                with open(STATUS_FILE, "r") as f:
                    return json.load(f)
    except Exception:
        pass
    return None


def read_station_conf():
    """Read ~/station.conf. Returns (lat, lon, elev) or (None, None, None)."""
    try:
        if os.path.exists(STATION_CONF):
            with open(STATION_CONF, "r") as f:
                cfg = json.load(f)
            return cfg.get("lat"), cfg.get("lon"), cfg.get("elev")
    except Exception:
        pass
    return None, None, None


def get_system_stats():
    """Read CPU temp, RAM usage, and uptime from /proc and /sys."""
    stats = {"cpu_temp": None, "ram_used_mb": 0, "ram_total_mb": 0, "uptime": ""}
    try:
        with open("/sys/class/thermal/thermal_zone0/temp", "r") as f:
            stats["cpu_temp"] = int(f.read().strip()) / 1000.0
    except Exception:
        pass
    try:
        with open("/proc/meminfo", "r") as f:
            lines = f.readlines()
        mem = {}
        for line in lines:
            parts = line.split()
            if len(parts) >= 2:
                mem[parts[0].rstrip(":")] = int(parts[1])
        total = mem.get("MemTotal", 0) // 1024
        avail = mem.get("MemAvailable", mem.get("MemFree", 0)) // 1024
        stats["ram_total_mb"] = total
        stats["ram_used_mb"] = total - avail
    except Exception:
        pass
    try:
        with open("/proc/uptime", "r") as f:
            secs = float(f.read().split()[0])
        h = int(secs // 3600)
        m = int((secs % 3600) // 60)
        stats["uptime"] = f"{h}h{m:02d}m"
    except Exception:
        pass
    return stats


def _rotator_state(connected, rf_data):
    """Derive rotator state string from connection status and RF data."""
    if not connected:
        return "disconnected"
    if rf_data and rf_data.get("satellite"):
        return "tracking"
    return "idle"


def polling_loop():
    """Background thread: updates the shared status dict every 1 second."""
    while True:
        try:
            if SIMULATE:
                sim_status = get_simulator().get_status()
                with status_lock:
                    status.update(sim_status)
                    lat, lon, elev = read_station_conf()
                    status["location"] = {
                        "lat": lat, "lon": lon, "elev": elev,
                        "set": lat is not None,
                    }
                    status["last_update"] = datetime.datetime.now().strftime("%H:%M:%S")
            else:
                connected, az, el = poll_rotctld()

                rf_data = read_station_status()
                if rf_data:
                    rf_status = rf_data
                else:
                    rf_status = {"streaming": False, "rssi": None, "packets": 0,
                                 "freq_mhz": 0, "satellite": "", "pass_progress": 0}

                sys_stats = get_system_stats()
                lat, lon, elev = read_station_conf()

                # SatNOGS data is written into the station_status.json by station.py
                satnogs_status = {"enabled": False, "station_id": None,
                                  "submitted": 0, "queued": 0, "failed": 0,
                                  "last_submit": None}
                if rf_data and "satnogs" in rf_data:
                    satnogs_status = rf_data["satnogs"]

                # Read current rf_mode and auto_track from station.conf
                rf_mode = "auto"
                auto_track = True
                try:
                    if os.path.exists(STATION_CONF):
                        with open(STATION_CONF, "r") as f:
                            _conf = json.load(f)
                        rf_mode = _conf.get("rf_mode", "auto")
                        auto_track = _conf.get("auto_track", True)
                except Exception:
                    pass

                with status_lock:
                    status["rotator"] = {
                        "az": az, "el": el, "connected": connected,
                        "state": _rotator_state(connected, rf_status),
                    }
                    status["rf"] = rf_status
                    status["rf_mode"] = rf_mode
                    status["auto_track"] = auto_track
                    status["system"] = sys_stats
                    status["location"] = {
                        "lat": lat, "lon": lon, "elev": elev,
                        "set": lat is not None,
                    }
                    status["satnogs"] = satnogs_status
                    status["last_update"] = datetime.datetime.now().strftime("%H:%M:%S")
                    status["simulation"] = False
        except Exception:
            pass

        # Recompute passes in the background (every 2 min, non-blocking)
        try:
            if not SIMULATE:
                _recompute_passes_if_stale()
                _fetch_transmitters()  # enrich with freq/modulation data
        except Exception:
            pass

        time.sleep(1)


# ---------------------------------------------------------------------------
# Pass prediction (real, via PyEphem + CelesTrak)
# ---------------------------------------------------------------------------
SATELLITE_GROUPS = [
    # Primary: AMSAT (reliable, no rate limiting)
    "https://www.amsat.org/tle/current/nasabare.txt",
    # Fallback: CelesTrak (may 403 under rate limiting)
    "https://celestrak.org/NORAD/elements/gp.php?GROUP=stations&FORMAT=TLE",
    "https://celestrak.org/NORAD/elements/gp.php?GROUP=amateur&FORMAT=TLE",
    "https://celestrak.org/NORAD/elements/gp.php?GROUP=weather&FORMAT=TLE",
    "https://celestrak.org/NORAD/elements/gp.php?GROUP=noaa&FORMAT=TLE",
]


def _fetch_tles_from_url(url):
    """Fetch TLE data from a URL. Returns dict of name -> (line1, line2)."""
    import urllib.request
    try:
        req = urllib.request.urlopen(
            urllib.request.Request(url, headers={"User-Agent": "SatNOGS-Basestation/1.0"}),
            timeout=10)
        raw = req.read().decode("utf-8", errors="replace")
        lines = [l.strip() for l in raw.splitlines() if l.strip()]
        tles = {}
        i = 0
        while i + 2 < len(lines):
            name = lines[i]
            l1 = lines[i + 1]
            l2 = lines[i + 2]
            if l1.startswith("1 ") and l2.startswith("2 "):
                tles[name] = (l1, l2)
                i += 3
            else:
                i += 1
        return tles
    except Exception:
        return {}


def _fetch_tles_satnogs_db():
    """Fetch TLEs from SatNOGS DB API (JSON). Returns dict of name -> (line1, line2)."""
    import urllib.request
    try:
        url = "https://db.satnogs.org/api/tle/?format=json"
        req = urllib.request.urlopen(
            urllib.request.Request(url, headers={"User-Agent": "SatNOGS-Basestation/1.0"}),
            timeout=20)
        data = json.loads(req.read().decode())
        tles = {}
        for sat in data:
            name = sat.get("tle0", "").strip()
            l1 = sat.get("tle1", "").strip()
            l2 = sat.get("tle2", "").strip()
            if name and l1.startswith("1 ") and l2.startswith("2 "):
                tles[name] = (l1, l2)
        return tles
    except Exception:
        return {}


def fetch_tles():
    """Fetch TLEs from multiple sources. Cached for 2 hours."""
    global _tle_cache, _tle_cache_time

    now = time.time()
    if _tle_cache and (now - _tle_cache_time) < TLE_CACHE_SECONDS:
        return _tle_cache

    combined = {}

    # Primary: SatNOGS DB (1500+ satellites)
    satnogs_tles = _fetch_tles_satnogs_db()
    combined.update(satnogs_tles)

    # Secondary: AMSAT + CelesTrak (may add some not in SatNOGS DB)
    for url in SATELLITE_GROUPS:
        tles = _fetch_tles_from_url(url)
        combined.update(tles)

    if combined:
        _tle_cache = combined
        _tle_cache_time = now
    return _tle_cache


# ---------------------------------------------------------------------------
# SatNOGS DB transmitter cache — enriches passes with freq/modulation info
# ---------------------------------------------------------------------------
_transmitter_cache = {}       # norad_id -> [{freq, mode, baud, description, ...}]
_transmitter_cache_time = 0.0
TRANSMITTER_CACHE_SECONDS = 7200  # 2 hours

# CC1200-compatible modulations
CC1200_MODES = {'GFSK', 'GMSK', 'FSK', 'MSK', 'OOK', 'ASK', '2FSK', '2GFSK', '4FSK', '4GFSK',
                'AFSK', 'FM'}  # AFSK/FM are borderline but listed for display
# Modulations that need RTL-SDR
RTLSDR_ONLY_MODES = {'BPSK', 'DBPSK', 'QPSK', 'OQPSK', 'DQPSK', 'LoRa', 'SSTV', 'CW', 'USB', 'LSB'}


def _fetch_transmitters():
    """Fetch satellite transmitter data from SatNOGS DB API."""
    global _transmitter_cache, _transmitter_cache_time

    now = time.time()
    if _transmitter_cache and (now - _transmitter_cache_time) < TRANSMITTER_CACHE_SECONDS:
        return

    import urllib.request
    try:
        url = "https://db.satnogs.org/api/transmitters/?format=json&status=active"
        req = urllib.request.Request(url, headers={"User-Agent": "SatNOGS-Basestation/1.0"})
        resp = urllib.request.urlopen(req, timeout=15)
        data = json.loads(resp.read().decode())

        tx_map = {}
        for tx in data:
            norad = tx.get("norad_cat_id")
            if not norad:
                continue
            if norad not in tx_map:
                tx_map[norad] = []
            freq = tx.get("downlink_low") or tx.get("uplink_low") or 0
            mode = tx.get("mode") or ""
            baud = tx.get("baud") or 0
            tx_map[norad].append({
                "freq_hz": freq,
                "freq_mhz": round(freq / 1e6, 3) if freq else 0,
                "mode": mode,
                "baud": baud,
                "description": tx.get("description", ""),
                "type": tx.get("type", ""),
                "band": _freq_to_band(freq),
                "cc1200_ok": mode.upper().replace("-", "").replace(" ", "") in
                             {m.upper().replace("-", "") for m in CC1200_MODES},
                "rtlsdr_ok": True,  # RTL-SDR can capture anything
            })

        _transmitter_cache = tx_map
        _transmitter_cache_time = now
    except Exception:
        pass  # silently fail — passes work without enrichment


def _freq_to_band(freq_hz):
    """Classify frequency into band name."""
    if not freq_hz:
        return ""
    f = freq_hz / 1e6
    if f < 30:
        return "HF"
    elif f < 300:
        return "VHF"
    elif f < 1000:
        return "UHF"
    elif f < 3000:
        return "L-band"
    elif f < 10000:
        return "S-band"
    return "other"


def get_transmitter_info(norad_id):
    """Get transmitter info for a satellite. Returns list of transmitters."""
    return _transmitter_cache.get(norad_id, [])


# ---------------------------------------------------------------------------
# Pass computation cache — runs in background thread, never blocks HTTP
# ---------------------------------------------------------------------------
_pass_cache = []           # cached computed passes
_pass_cache_time = 0.0     # unix timestamp of last computation
_pass_computing = False    # True while background computation is running
_pass_cache_lock = threading.Lock()
PASS_CACHE_SECONDS = 60    # recompute every minute (removes passed sats, adds new ones)


def _recompute_passes_if_stale():
    """Background recompute of passes — called from polling thread."""
    global _pass_cache, _pass_cache_time, _pass_computing

    now = time.time()
    if (now - _pass_cache_time) < PASS_CACHE_SECONDS:
        return

    _pass_computing = True
    lat, lon, elev = read_station_conf()
    if lat is None:
        lat, lon, elev = DEFAULT_LAT, DEFAULT_LON, DEFAULT_ELEV

    passes = compute_passes(lat, lon, elev)
    with _pass_cache_lock:
        _pass_cache = passes
        _pass_cache_time = now
    _pass_computing = False


def get_cached_passes():
    """Get the most recent pass computation, enriched with transmitter data."""
    with _pass_cache_lock:
        passes = list(_pass_cache)

    # Enrich with transmitter data if available (may have been fetched after passes)
    if _transmitter_cache:
        for p in passes:
            norad = p.get("norad_id", 0)
            if norad and "freq_mhz" not in p:
                tx_list = get_transmitter_info(norad)
                best_tx = None
                for tx in tx_list:
                    if 400e6 <= (tx.get("freq_hz") or 0) <= 470e6:
                        if best_tx is None or abs(tx["freq_hz"] - 433e6) < abs(best_tx["freq_hz"] - 433e6):
                            best_tx = tx
                if not best_tx and tx_list:
                    best_tx = tx_list[0]
                if best_tx:
                    p["freq_mhz"] = best_tx.get("freq_mhz", 0)
                    p["mode"] = best_tx.get("mode", "")
                    p["baud"] = best_tx.get("baud", 0)
                    p["band"] = best_tx.get("band", "")
                    p["cc1200_ok"] = best_tx.get("cc1200_ok", False)
                    p["rtlsdr_ok"] = True

    return passes


def is_pass_computing():
    return _pass_computing


def compute_passes(lat, lon, elev, hours=12, max_passes=50):
    """Compute upcoming passes using PyEphem. Returns list of pass dicts."""
    if not EPHEM_AVAILABLE:
        return []

    tles = fetch_tles()
    if not tles:
        return []

    observer = ephem.Observer()
    observer.lat = str(lat)
    observer.lon = str(lon)
    observer.elevation = float(elev)
    observer.horizon = "0"
    observer.pressure = 0  # skip atmospheric refraction

    now_utc = datetime.datetime.now(datetime.timezone.utc)
    start_ephem = ephem.Date(
        now_utc.strftime("%Y/%m/%d %H:%M:%S")
    )
    end_ephem = start_ephem + hours / 24.0

    passes = []

    for name, (l1, l2) in tles.items():
        try:
            sat = ephem.readtle(name, l1, l2)
            observer.date = start_ephem

            while observer.date < end_ephem:
                try:
                    rise_t, rise_az, max_t, max_el, set_t, set_az = observer.next_pass(sat)
                except Exception:
                    break

                if rise_t is None or set_t is None:
                    break
                if rise_t > end_ephem:
                    break

                max_el_deg = math.degrees(float(max_el))
                if max_el_deg < 5.0:
                    # Skip passes with max elevation below 5° (not useful)
                    observer.date = set_t + ephem.minute
                    continue

                rise_dt = ephem.Date(rise_t).datetime().replace(tzinfo=datetime.timezone.utc)
                set_dt = ephem.Date(set_t).datetime().replace(tzinfo=datetime.timezone.utc)
                duration = int((set_dt - rise_dt).total_seconds())

                # Compute trajectory: ~20 points along the pass
                trajectory = []
                n_points = 20
                for i in range(n_points + 1):
                    t_frac = i / n_points
                    t_ephem = rise_t + (set_t - rise_t) * t_frac
                    observer.date = t_ephem
                    sat.compute(observer)
                    traj_az = math.degrees(float(sat.az))
                    traj_el = math.degrees(float(sat.alt))
                    trajectory.append({
                        "az": round(traj_az, 1),
                        "el": round(max(0.0, traj_el), 1),
                        "t": round(t_frac * duration),
                    })

                # Extract NORAD ID from TLE line 1
                norad_id = 0
                try:
                    norad_id = int(l1[2:7].strip())
                except Exception:
                    pass

                # Enrich with transmitter info from SatNOGS DB
                tx_list = get_transmitter_info(norad_id)
                # Pick the best UHF transmitter (closest to 433 MHz)
                best_tx = None
                for tx in tx_list:
                    if 400e6 <= (tx.get("freq_hz") or 0) <= 470e6:
                        if best_tx is None or abs(tx["freq_hz"] - 433e6) < abs(best_tx["freq_hz"] - 433e6):
                            best_tx = tx
                if not best_tx and tx_list:
                    best_tx = tx_list[0]  # fallback to first transmitter

                pass_entry = {
                    "name": name,
                    "norad_id": norad_id,
                    "rise_time": rise_dt.strftime("%Y-%m-%dT%H:%M:%SZ"),
                    "set_time": set_dt.strftime("%Y-%m-%dT%H:%M:%SZ"),
                    "max_el": round(max_el_deg, 1),
                    "duration": duration,
                    "rise_az": round(math.degrees(float(rise_az)), 1),
                    "set_az": round(math.degrees(float(set_az)), 1),
                    "trajectory": trajectory,
                }
                if best_tx:
                    pass_entry["freq_mhz"] = best_tx.get("freq_mhz", 0)
                    pass_entry["mode"] = best_tx.get("mode", "")
                    pass_entry["baud"] = best_tx.get("baud", 0)
                    pass_entry["band"] = best_tx.get("band", "")
                    pass_entry["cc1200_ok"] = best_tx.get("cc1200_ok", False)
                    pass_entry["rtlsdr_ok"] = True

                passes.append(pass_entry)

                # Advance past this pass to find the next one for this satellite
                observer.date = set_t + ephem.minute

        except Exception:
            continue

    # Sort by rise time, return top N
    passes.sort(key=lambda p: p["rise_time"])
    return passes[:max_passes]


# ---------------------------------------------------------------------------
# Actions
# ---------------------------------------------------------------------------
def save_location(lat, lon, elev):
    """Save GPS coordinates to ~/station.conf, preserving existing fields."""
    # Read any existing conf so we don't overwrite tokens/callsign/etc.
    existing = {}
    try:
        if os.path.exists(STATION_CONF):
            with open(STATION_CONF, "r") as f:
                existing = json.load(f)
    except Exception:
        pass
    existing["lat"]  = round(lat, 6)
    existing["lon"]  = round(lon, 6)
    existing["elev"] = round(elev, 1)
    with open(STATION_CONF, "w") as f:
        json.dump(existing, f, indent=4)


def send_park():
    """Send P 0.0 0.0 to rotctld. Returns True on success."""
    resp = _rotctl_cmd("P 0.0 0.0")
    return resp != ""


def send_position(az, el):
    """Send P <az> <el> to rotctld. Returns True on success."""
    resp = _rotctl_cmd(f"P {az:.1f} {el:.1f}")
    return resp != ""


def do_shutdown():
    send_park()
    time.sleep(3)
    subprocess.Popen(["sudo", "shutdown", "-h", "now"])


# ---------------------------------------------------------------------------
# Static file serving helpers
# ---------------------------------------------------------------------------
def read_static_file(rel_path):
    """Read a file from the dashboard directory. Returns (bytes, mime) or None."""
    # Sanitise: strip leading slashes and prevent path traversal
    safe = rel_path.lstrip("/")
    if ".." in safe:
        return None, None
    full = os.path.join(DASHBOARD_DIR, safe)
    if not os.path.isfile(full):
        return None, None
    ext = os.path.splitext(full)[1].lower()
    mime = MIME_TYPES.get(ext, "application/octet-stream")
    try:
        with open(full, "rb") as f:
            return f.read(), mime
    except Exception:
        return None, None


# ---------------------------------------------------------------------------
# HTTP server
# ---------------------------------------------------------------------------
class DashboardHandler(BaseHTTPRequestHandler):

    def do_GET(self):
        path = self.path.split("?")[0]  # strip query string

        if path == "/api/status":
            with status_lock:
                data = json.dumps(status)
            self._json_response(200, data)

        elif path == "/api/passes":
            self._handle_passes()

        elif path.startswith("/api/observations/"):
            self._serve_observation(path)

        elif path == "/" or path == "/index.html":
            self._serve_dashboard()

        else:
            # Try to serve any static file from the dashboard directory
            data, mime = read_static_file(path)
            if data is not None:
                self.send_response(200)
                self.send_header("Content-Type", mime)
                self.send_header("Content-Length", str(len(data)))
                self.end_headers()
                self.wfile.write(data)
            else:
                self.send_response(404)
                self.end_headers()
                self.wfile.write(b"404 Not Found")

    def do_POST(self):
        path = self.path.split("?")[0]

        if path == "/api/location":
            try:
                length = int(self.headers.get("Content-Length", 0))
                body = json.loads(self.rfile.read(length))
                lat = float(body["lat"])
                lon = float(body["lon"])
                elev = float(body.get("elev", 0))
                save_location(lat, lon, elev)
                self._json_response(200, json.dumps({"ok": True}))
            except Exception as e:
                self._json_response(400, json.dumps({"ok": False, "error": str(e)}))

        elif path == "/api/position":
            try:
                length = int(self.headers.get("Content-Length", 0))
                body = json.loads(self.rfile.read(length))
                az = float(body["az"])
                el = float(body["el"])
                if SIMULATE:
                    get_simulator().set_position(az, el)
                    self._json_response(200, json.dumps({"ok": True}))
                else:
                    ok = send_position(az, el)
                    self._json_response(200, json.dumps({"ok": ok}))
            except Exception as e:
                self._json_response(400, json.dumps({"ok": False, "error": str(e)}))

        elif path == "/api/park":
            if SIMULATE:
                get_simulator().park()
                self._json_response(200, json.dumps({"ok": True}))
            else:
                ok = send_park()
                self._json_response(200, json.dumps({"ok": ok}))

        elif path == "/api/stop":
            # Write a stop flag that station.py checks in its tracking loop
            stop_file = os.path.expanduser("~/.station_stop")
            try:
                with open(stop_file, "w") as f:
                    f.write("stop")
                self._json_response(200, json.dumps({"ok": True}))
            except Exception as e:
                self._json_response(500, json.dumps({"ok": False, "error": str(e)}))

        elif path == "/api/auto-track":
            try:
                length = int(self.headers.get("Content-Length", 0))
                body = json.loads(self.rfile.read(length))
                enabled = body.get("enabled", True)
                existing = {}
                try:
                    if os.path.exists(STATION_CONF):
                        with open(STATION_CONF, "r") as f:
                            existing = json.load(f)
                except Exception:
                    pass
                existing["auto_track"] = bool(enabled)
                if enabled:
                    existing["track_mode"] = "auto"
                    existing["track_satellite"] = ""
                with open(STATION_CONF, "w") as f:
                    json.dump(existing, f, indent=4)
                self._json_response(200, json.dumps({"ok": True, "auto_track": bool(enabled)}))
            except Exception as e:
                self._json_response(400, json.dumps({"ok": False, "error": str(e)}))

        elif path == "/api/track":
            try:
                length = int(self.headers.get("Content-Length", 0))
                body = json.loads(self.rfile.read(length))
                sat_name = body.get("satellite", "")
                # Write track file — station.py picks this up within 3s
                track_file = os.path.expanduser("~/.station_track")
                with open(track_file, "w") as f:
                    f.write(sat_name)
                # Also remove stop/pause flags
                for flag in ["~/.station_stop", "~/.station_paused"]:
                    try:
                        os.remove(os.path.expanduser(flag))
                    except FileNotFoundError:
                        pass
                self._json_response(200, json.dumps({"ok": True, "satellite": sat_name}))
            except Exception as e:
                self._json_response(400, json.dumps({"ok": False, "error": str(e)}))

        elif path == "/api/rf-mode":
            try:
                length = int(self.headers.get("Content-Length", 0))
                body = json.loads(self.rfile.read(length))
                mode = body.get("mode", "auto")
                if mode not in ("none", "cc1200", "rtlsdr", "auto"):
                    self._json_response(400, json.dumps(
                        {"ok": False, "error": f"Invalid rf_mode: {mode}"}))
                    return
                # Save to station.conf so station.py picks it up next cycle
                existing = {}
                try:
                    if os.path.exists(STATION_CONF):
                        with open(STATION_CONF, "r") as f:
                            existing = json.load(f)
                except Exception:
                    pass
                existing["rf_mode"] = mode
                with open(STATION_CONF, "w") as f:
                    json.dump(existing, f, indent=4)
                self._json_response(200, json.dumps({"ok": True, "mode": mode}))
            except Exception as e:
                self._json_response(400, json.dumps({"ok": False, "error": str(e)}))

        elif path == "/api/shutdown":
            if SIMULATE:
                self._json_response(200, json.dumps(
                    {"ok": False, "error": "Shutdown not available in simulation mode"}
                ))
            else:
                self._json_response(200, json.dumps({"ok": True}))
                threading.Thread(target=do_shutdown, daemon=True).start()

        else:
            self.send_response(404)
            self.end_headers()

    def _serve_dashboard(self):
        """Serve dashboard.html from the same directory, or an error page."""
        dashboard_path = os.path.join(DASHBOARD_DIR, "dashboard.html")
        if os.path.isfile(dashboard_path):
            try:
                with open(dashboard_path, "rb") as f:
                    content = f.read()
                self.send_response(200)
                self.send_header("Content-Type", "text/html")
                self.send_header("Content-Length", str(len(content)))
                self.end_headers()
                self.wfile.write(content)
                return
            except Exception:
                pass

        # Fallback error page if dashboard.html is missing
        error_html = (
            b"<!DOCTYPE html><html><head><meta charset='utf-8'>"
            b"<title>Dashboard Error</title></head><body style='font-family:monospace;"
            b"background:#0a0a0a;color:#ff4444;padding:40px;'>"
            b"<h1>dashboard.html not found</h1>"
            b"<p>Place dashboard.html in the same directory as dashboard.py ("
            + DASHBOARD_DIR.encode()
            + b")</p></body></html>"
        )
        self.send_response(503)
        self.send_header("Content-Type", "text/html")
        self.send_header("Content-Length", str(len(error_html)))
        self.end_headers()
        self.wfile.write(error_html)

    def _handle_passes(self):
        """Compute and return upcoming satellite passes."""
        if SIMULATE:
            result = get_simulator().get_passes()
            self._json_response(200, json.dumps(result))
            return

        if not EPHEM_AVAILABLE:
            self._json_response(503, json.dumps({
                "passes": [],
                "error": "ephem not installed — run: pip install ephem",
            }))
            return

        # Return cached passes (computed in background polling thread)
        passes = get_cached_passes()
        self._json_response(200, json.dumps({
            "passes": passes,
            "computing": is_pass_computing(),
        }))

    def _serve_observation(self, path):
        """Serve files from ~/observations/ (waterfall PNGs, etc.)."""
        # /api/observations/20260318_120000/waterfall.png → ~/observations/20260318_120000/waterfall.png
        rel = path[len("/api/observations/"):]
        if ".." in rel or rel.startswith("/"):
            self.send_response(403)
            self.end_headers()
            return
        full = os.path.join(os.path.expanduser("~/observations"), rel)
        if not os.path.isfile(full):
            self.send_response(404)
            self.end_headers()
            self.wfile.write(b"404 Not Found")
            return
        ext = os.path.splitext(full)[1].lower()
        mime = MIME_TYPES.get(ext, "application/octet-stream")
        try:
            with open(full, "rb") as f:
                data = f.read()
            self.send_response(200)
            self.send_header("Content-Type", mime)
            self.send_header("Content-Length", str(len(data)))
            self.send_header("Cache-Control", "public, max-age=3600")
            self.end_headers()
            self.wfile.write(data)
        except Exception:
            self.send_response(500)
            self.end_headers()

    def _json_response(self, code, data):
        body = data.encode() if isinstance(data, str) else data
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Access-Control-Allow-Origin", "*")
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, format, *args):
        pass  # Suppress request logging


# ---------------------------------------------------------------------------
# SSL certificate
# ---------------------------------------------------------------------------
def ensure_ssl_cert():
    """Generate a self-signed certificate if none exists.

    The browser Geolocation API requires HTTPS (won't work on plain HTTP to a
    Pi IP address).
    """
    cert_dir = os.path.expanduser("~/.station_ssl")
    cert_file = os.path.join(cert_dir, "cert.pem")
    key_file = os.path.join(cert_dir, "key.pem")

    if os.path.exists(cert_file) and os.path.exists(key_file):
        return cert_file, key_file

    os.makedirs(cert_dir, exist_ok=True)
    print("Generating self-signed SSL certificate for HTTPS...")
    print("(Your phone browser will show a security warning — tap Advanced → Proceed)")
    subprocess.run([
        "openssl", "req", "-x509", "-newkey", "rsa:2048",
        "-keyout", key_file, "-out", cert_file,
        "-days", "3650", "-nodes",
        "-subj", "/CN=SatNOGS Ground Station",
    ], check=True, capture_output=True)
    print(f"Certificate saved to {cert_dir}/")
    return cert_file, key_file


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------
def main():
    global SIMULATE

    parser = argparse.ArgumentParser(description="Ground Station Web Dashboard")
    parser.add_argument("--port", type=int, default=5000,
                        help="HTTPS port (default: 5000)")
    parser.add_argument("--bind", default="0.0.0.0",
                        help="Bind address (default: 0.0.0.0)")
    parser.add_argument("--no-ssl", action="store_true",
                        help="Disable HTTPS (GPS location won't work on phone)")
    parser.add_argument("--simulate", action="store_true",
                        help="Run in simulation mode — no hardware required")
    args = parser.parse_args()

    SIMULATE = args.simulate

    if SIMULATE:
        print("Running in SIMULATION mode — no hardware connections will be made")
        _ = get_simulator()  # initialise singleton

    poller = threading.Thread(target=polling_loop, daemon=True)
    poller.start()

    server = HTTPServer((args.bind, args.port), DashboardHandler)

    if not args.no_ssl:
        try:
            cert_file, key_file = ensure_ssl_cert()
            ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
            ctx.load_cert_chain(cert_file, key_file)
            server.socket = ctx.wrap_socket(server.socket, server_side=True)
            proto = "https"
        except Exception as e:
            print(f"WARNING: SSL setup failed ({e}) — falling back to HTTP")
            print("Phone GPS location will NOT work without HTTPS")
            proto = "http"
    else:
        proto = "http"

    sim_tag = " [SIMULATE]" if SIMULATE else ""
    print(f"Dashboard{sim_tag} running at {proto}://{args.bind}:{args.port}")
    print("Open this URL on your phone (same WiFi/hotspot)")
    if proto == "https":
        print("First visit: tap Advanced → Proceed to accept the self-signed certificate")
    if not EPHEM_AVAILABLE and not SIMULATE:
        print("NOTE: ephem not installed — /api/passes will return empty list")
        print("      Install with: pip install ephem")

    def on_signal(sig, frame):
        server.shutdown()

    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)

    server.serve_forever()


if __name__ == "__main__":
    main()
