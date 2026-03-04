#!/usr/bin/env python3
"""
SatNOGS DB Integration — submit decoded frames via the SiDS telemetry API.

Reads credentials from ~/station.conf (extended format):
    {
        "lat": 51.1, "lon": 4.97, "elev": 25,
        "callsign": "ON4xxx",
        "satnogs_station_id": 1234,
        "satnogs_db_token": "abc123...",
        "satnogs_network_token": "def456..."
    }

Usage:
    from satnogs import SatNOGSClient

    client = SatNOGSClient()          # reads config from ~/station.conf
    client.submit_frame(
        norad_id=99999,
        frame_hex="3C594D3256...",
    )
    client.flush_queue()              # retry offline-queued frames
    stats = client.get_stats()        # {"submitted": 7, "queued": 2, "failed": 0}

API:
    POST https://db.satnogs.org/api/telemetry/
    Content-Type: application/x-www-form-urlencoded
    Authorization: Token <DB_API_TOKEN>

    noradID=99999&source=ON4xxx&timestamp=2026-03-03T14:30:00.000Z
    &frame=3C594D...&locator=longLat&longitude=4.97&latitude=51.1
    &version=satnogs-cc1200-1.0

    201 = success, 400 = bad request, 401 = unauthorized
"""

import os
import json
import time
import datetime
import threading
import urllib.request
import urllib.parse
import urllib.error
from typing import Optional

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------
SATNOGS_DB_URL    = "https://db.satnogs.org/api/telemetry/"
STATION_CONF      = os.path.expanduser("~/station.conf")
QUEUE_FILE        = os.path.expanduser("~/.satnogs_queue.json")
VERSION_STRING    = "satnogs-cc1200-1.0"
REQUEST_TIMEOUT_S = 10
MAX_QUEUE_SIZE    = 500   # don't grow the queue unboundedly in the field


# ---------------------------------------------------------------------------
# SatNOGSClient
# ---------------------------------------------------------------------------
class SatNOGSClient:
    """
    Thread-safe SatNOGS DB client with offline queue.

    All public methods are safe to call from any thread. Internal state is
    protected by a single lock. Network operations are performed under the
    lock only for queue manipulation; the actual HTTP call happens without
    holding the lock to avoid blocking the station main loop.
    """

    def __init__(self, conf_path: str = STATION_CONF,
                 queue_path: str = QUEUE_FILE):
        self._conf_path  = conf_path
        self._queue_path = queue_path
        self._lock       = threading.Lock()

        # Credentials (loaded lazily from station.conf on each call so that
        # the user can add tokens to the conf file without restarting station.py)
        self._token:      Optional[str] = None
        self._callsign:   Optional[str] = None
        self._station_id: Optional[int] = None
        self._lat:        Optional[float] = None
        self._lon:        Optional[float] = None

        # Stats
        self._submitted  = 0
        self._queued     = 0    # current length of the on-disk queue
        self._failed     = 0    # permanent failures (bad request / 401)
        self._last_submit_ts: Optional[str] = None   # HH:MM:SS

        # Load the offline queue from disk
        self._queue: list = []   # list of frame dicts ready to retry
        self._load_queue()

    # -----------------------------------------------------------------------
    # Public API
    # -----------------------------------------------------------------------

    def is_configured(self) -> bool:
        """Return True if a DB API token is present in station.conf."""
        self._reload_conf()
        return bool(self._token)

    def submit_frame(self, norad_id: int, frame_hex: str) -> bool:
        """
        Submit a decoded frame to SatNOGS DB.

        If the submission fails (network error / no internet in the field),
        the frame is saved to ~/.satnogs_queue.json for later retry via
        flush_queue().

        Returns True on immediate success, False on failure (queued or dropped).
        """
        self._reload_conf()

        if not self._token:
            # Not configured — silently skip (station works without tokens)
            return False

        timestamp = datetime.datetime.now(datetime.timezone.utc).strftime(
            "%Y-%m-%dT%H:%M:%S.000Z"
        )

        frame_dict = {
            "norad_id":  norad_id,
            "frame_hex": frame_hex.replace(" ", "").upper(),
            "timestamp": timestamp,
            "callsign":  self._callsign or "",
            "lat":       self._lat,
            "lon":       self._lon,
        }

        ok = self._post_frame(frame_dict)
        if ok:
            with self._lock:
                self._submitted += 1
                self._last_submit_ts = datetime.datetime.now().strftime("%H:%M:%S")
            return True
        else:
            self._enqueue(frame_dict)
            return False

    def flush_queue(self) -> int:
        """
        Retry all queued frames.

        Called between passes (or periodically) to submit frames that were
        captured while the Pi hotspot had no internet connectivity.

        Returns the number of frames successfully submitted in this call.
        """
        self._reload_conf()

        if not self._token:
            return 0

        with self._lock:
            if not self._queue:
                return 0
            to_retry = list(self._queue)

        sent = 0
        still_pending = []

        for frame_dict in to_retry:
            ok = self._post_frame(frame_dict)
            if ok:
                sent += 1
                with self._lock:
                    self._submitted += 1
                    self._last_submit_ts = datetime.datetime.now().strftime("%H:%M:%S")
            else:
                still_pending.append(frame_dict)

        with self._lock:
            self._queue = still_pending
            self._queued = len(still_pending)
        self._save_queue()

        return sent

    def get_stats(self) -> dict:
        """Return submission statistics dict for the dashboard status file."""
        with self._lock:
            return {
                "enabled":     bool(self._token),
                "station_id":  self._station_id,
                "submitted":   self._submitted,
                "queued":      self._queued,
                "failed":      self._failed,
                "last_submit": self._last_submit_ts,
            }

    # -----------------------------------------------------------------------
    # Internal helpers
    # -----------------------------------------------------------------------

    def _reload_conf(self):
        """Re-read ~/station.conf to pick up tokens added after station start."""
        try:
            if not os.path.exists(self._conf_path):
                return
            with open(self._conf_path, "r") as f:
                cfg = json.load(f)
            with self._lock:
                self._token      = cfg.get("satnogs_db_token") or None
                self._callsign   = cfg.get("callsign") or None
                self._station_id = cfg.get("satnogs_station_id") or None
                self._lat        = cfg.get("lat")
                self._lon        = cfg.get("lon")
        except Exception:
            pass

    def _build_post_body(self, frame_dict: dict) -> bytes:
        """Encode a frame dict as application/x-www-form-urlencoded bytes."""
        params = {
            "noradID":   str(frame_dict["norad_id"]),
            "frame":     frame_dict["frame_hex"],
            "timestamp": frame_dict["timestamp"],
            "version":   VERSION_STRING,
        }
        if frame_dict.get("callsign"):
            params["source"] = frame_dict["callsign"]
        if frame_dict.get("lat") is not None and frame_dict.get("lon") is not None:
            params["locator"]   = "longLat"
            params["latitude"]  = f"{frame_dict['lat']:.6f}"
            params["longitude"] = f"{frame_dict['lon']:.6f}"
        return urllib.parse.urlencode(params).encode("ascii")

    def _post_frame(self, frame_dict: dict) -> bool:
        """
        POST one frame to SatNOGS DB.

        Returns True on HTTP 201 (created).
        Returns False on network error (should retry).
        Increments _failed and returns False on HTTP 400 / 401 (permanent error).
        Does NOT hold self._lock during the network call.
        """
        with self._lock:
            token = self._token
        if not token:
            return False

        body = self._build_post_body(frame_dict)
        req  = urllib.request.Request(
            SATNOGS_DB_URL,
            data=body,
            method="POST",
        )
        req.add_header("Content-Type", "application/x-www-form-urlencoded")
        req.add_header("Authorization", f"Token {token}")
        req.add_header("User-Agent", "SatNOGS-Basestation/1.0")

        try:
            resp = urllib.request.urlopen(req, timeout=REQUEST_TIMEOUT_S)
            code = resp.getcode()
            if code == 201:
                _log(f"SatNOGS DB: submitted NORAD {frame_dict['norad_id']} "
                     f"({len(frame_dict['frame_hex'])//2} bytes) → 201 OK")
                return True
            else:
                _log(f"SatNOGS DB: unexpected HTTP {code}")
                return False

        except urllib.error.HTTPError as e:
            if e.code in (400, 401, 422):
                # Permanent client-side error — don't queue for retry
                with self._lock:
                    self._failed += 1
                reason = e.read().decode("utf-8", errors="replace")[:200]
                _log(f"SatNOGS DB: permanent error HTTP {e.code}: {reason}")
                return True   # return True so caller doesn't re-queue it
            else:
                _log(f"SatNOGS DB: server error HTTP {e.code} — will retry")
                return False

        except Exception as e:
            _log(f"SatNOGS DB: network error — {e} — will retry")
            return False

    def _enqueue(self, frame_dict: dict):
        """Add a frame to the offline queue and persist to disk."""
        with self._lock:
            if len(self._queue) >= MAX_QUEUE_SIZE:
                # Drop oldest frame to bound disk usage
                self._queue.pop(0)
                self._failed += 1
                _log("SatNOGS DB: queue full — oldest frame dropped")
            self._queue.append(frame_dict)
            self._queued = len(self._queue)
        self._save_queue()
        _log(f"SatNOGS DB: frame queued offline (queue depth={self._queued})")

    def _save_queue(self):
        """Persist the offline queue to ~/.satnogs_queue.json."""
        try:
            with self._lock:
                data = list(self._queue)
            with open(self._queue_path, "w") as f:
                json.dump(data, f)
        except Exception as e:
            _log(f"SatNOGS DB: failed to save queue: {e}")

    def _load_queue(self):
        """Load the offline queue from disk (called once at startup)."""
        try:
            if os.path.exists(self._queue_path):
                with open(self._queue_path, "r") as f:
                    data = json.load(f)
                if isinstance(data, list):
                    with self._lock:
                        self._queue  = data
                        self._queued = len(data)
                    if data:
                        _log(f"SatNOGS DB: loaded {len(data)} queued frames from disk")
        except Exception as e:
            _log(f"SatNOGS DB: failed to load queue: {e}")


# ---------------------------------------------------------------------------
# Logging helper (avoids circular import — station.py also imports us)
# ---------------------------------------------------------------------------
def _log(msg: str):
    ts = datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]
    print(f"[{ts}] {msg}", flush=True)


# ---------------------------------------------------------------------------
# CLI smoke-test
# ---------------------------------------------------------------------------
if __name__ == "__main__":
    import sys

    print("SatNOGS DB client — smoke test")
    client = SatNOGSClient()

    if not client.is_configured():
        print("No satnogs_db_token in ~/station.conf — running in offline mode")
        print("Add the following fields to ~/station.conf to enable submission:")
        print('  "callsign": "ON4xxx"')
        print('  "satnogs_station_id": 1234')
        print('  "satnogs_db_token": "your_token_here"')
        # Demonstrate the queue mechanism
        client.submit_frame(99999, "DEADBEEF")
        stats = client.get_stats()
        print(f"Stats (expect queued=0 because no token): {stats}")
        sys.exit(0)

    print(f"Configured: callsign={client._callsign} station_id={client._station_id}")
    print("Submitting test frame (NORAD 99999, dummy hex)...")
    ok = client.submit_frame(99999, "DEADBEEF01020304")
    print(f"Result: {'OK' if ok else 'QUEUED/FAILED'}")
    stats = client.get_stats()
    print(f"Stats: {stats}")
