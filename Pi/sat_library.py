#!/usr/bin/env python3
"""
Satellite Profile Library — maps satellites to CC1200 radio configurations.

Provides per-satellite radio profiles so the ground station can reconfigure
the CC1200 between passes (frequency, modulation, symbol rate, deviation, etc.).

Three tiers of override:
  A) Frequency-only — just retune, keep base SmartRF profile
  B) Full modulation — compute and write critical registers (~15 writes)
  C) Custom SmartRF — load an entirely different register profile

Usage:
    from sat_library import SatLibrary

    lib = SatLibrary()
    profile = lib.lookup(norad_id=25544, name="ISS (ZARYA)")
    if profile:
        print(f"{profile.freq_mhz} MHz — {profile.description}")
"""

import json
import os
import time
import urllib.request
from dataclasses import dataclass, field, asdict
from pathlib import Path
from typing import Optional, List, Dict

# CC1200-compatible modulation formats
CC1200_MODES = {"2-FSK", "2-GFSK", "4-FSK", "4-GFSK", "ASK", "OOK"}

# SatNOGS DB mode strings that map to CC1200 formats
SATNOGS_MODE_MAP = {
    "FSK":                "2-FSK",
    "AFSK":               "2-FSK",     # 9600+ baud only; 1200 baud Bell 202 not compatible
    "GFSK":               "2-GFSK",
    "GMSK":               "2-GFSK",    # GMSK = GFSK with BT=0.5, h=0.5
    "MSK":                "2-GFSK",    # MSK = FSK with h=0.5
    "FFSK":               "2-FSK",
    "4FSK":               "4-FSK",
    "4GFSK":              "4-GFSK",
    "OOK":                "OOK",
    "ASK":                "ASK",
    "FSK AX.25 G3RUH":   "2-FSK",     # needs SW deframing (HDLC + G3RUH descrambler)
    "FSK AX.100 Mode 5": "2-GFSK",    # GOMspace/Syrlinks AX.100 protocol
    "FSK AX.100 Mode 6": "2-GFSK",
    "MSK AX.100 Mode 5": "2-GFSK",
    "MSK AX.100 Mode 6": "2-GFSK",
    "GMSK USP":           "2-GFSK",    # Uppsala Space Protocol framing
    "DOKA":               "2-GFSK",    # Russian DOKA protocol
    "AFSK TUBiX10":       "2-FSK",
    "GFSK Rktr":          "2-GFSK",
    "GFSK Pkst":          "2-GFSK",
}


def pick_best_uhf_transmitter(tx_list: list) -> Optional[dict]:
    """Pick the best CC1200-compatible UHF transmitter from a list.

    Selection priority:
      1. CC1200-compatible modes preferred
      2. Entries with specific descriptions over generic "Transmitter"
      3. Entries with IARU citation over uncited
      4. Most recently updated entry wins ties
    Returns None if list is empty.
    """
    if not tx_list:
        return None

    # Prefer CC1200-compatible entries
    cc_ok = [tx for tx in tx_list if tx.get("cc1200_format")]
    pool = cc_ok if cc_ok else tx_list

    def score(tx):
        """Lower score = better. Tuple sorts lexicographically."""
        desc = (tx.get("description") or "").strip()
        # Penalize generic/empty descriptions
        generic = desc.lower() in ("", "transmitter", "unknown")
        # Prefer entries with real citations
        cite = tx.get("citation", "")
        has_cite = bool(cite) and "CITATION NEEDED" not in cite
        # Prefer more recently updated (negate ISO date for min sorting)
        updated = tx.get("updated", "0")
        return (
            0 if not generic else 1,     # specific desc first
            0 if has_cite else 1,         # cited first
        )

    # Among equal scores, pick the most recently updated
    pool_sorted = sorted(pool, key=lambda tx: tx.get("updated", ""), reverse=True)
    return min(pool_sorted, key=score)


@dataclass
class ModulationConfig:
    """CC1200 modulation parameters for a satellite."""
    format: str = "2-GFSK"
    symbol_rate_bps: int = 2400
    deviation_hz: int = 5000
    rx_bw_khz: float = 25.0
    sync_word: Optional[str] = None  # hex string, e.g. "7A0E"


@dataclass
class SatProfile:
    """Radio configuration for a specific satellite."""
    freq_mhz: float
    modulation: Optional[ModulationConfig] = None
    smartrf_profile: Optional[str] = None
    description: str = ""
    cc1200_compatible: bool = True
    protocol: str = ""  # "ax25_g3ruh", "ax100_mode5", "native_cc1200", "usp", etc.
    use_serial_mode: bool = False  # True for protocols needing raw bit stream (G3RUH)


# SatNOGS DB transmitter API cache
_API_CACHE_DIR = os.path.expanduser("~/.satnogs_cache")
_API_CACHE_MAX_AGE = 86400  # 24 hours


class SatLibrary:
    """Satellite radio profile database with SatNOGS DB API integration."""

    def __init__(self, profiles_path: Optional[str] = None):
        if profiles_path is None:
            profiles_path = str(Path(__file__).parent / "sat_profiles.json")
        self.profiles_path = profiles_path
        self.default_profile: Optional[str] = None
        self.satellites: List[dict] = []
        self._load()

    def _load(self):
        """Load the local satellite profile database."""
        try:
            with open(self.profiles_path, "r", encoding="utf-8") as f:
                data = json.load(f)
            self.default_profile = data.get("default_profile")
            self.satellites = data.get("satellites", [])
        except FileNotFoundError:
            self.satellites = []
        except Exception as e:
            print(f"sat_library: failed to load {self.profiles_path}: {e}")
            self.satellites = []

    def lookup(self, norad_id: int = 0, name: str = "") -> Optional[SatProfile]:
        """Look up a satellite profile by NORAD ID (preferred) or name substring.

        Returns None if no match or if the satellite is marked as incompatible.
        """
        # First pass: match by NORAD ID
        if norad_id:
            for sat in self.satellites:
                if sat.get("norad_id") == norad_id:
                    return self._sat_to_profile(sat)

        # Second pass: match by name substring (case-insensitive)
        if name:
            name_upper = name.upper()
            for sat in self.satellites:
                match_str = sat.get("name_match", "").upper()
                if match_str and match_str in name_upper:
                    return self._sat_to_profile(sat)

        return None

    def _sat_to_profile(self, sat: dict) -> Optional[SatProfile]:
        """Convert a satellite dict entry to a SatProfile."""
        if not sat.get("cc1200_compatible", True):
            return None

        mod = None
        if "modulation" in sat:
            m = sat["modulation"]
            mod = ModulationConfig(
                format=m.get("format", "2-GFSK"),
                symbol_rate_bps=m.get("symbol_rate_bps", 2400),
                deviation_hz=m.get("deviation_hz", 5000),
                rx_bw_khz=m.get("rx_bw_khz", 25.0),
                sync_word=m.get("sync_word"),
            )

        proto = sat.get("protocol", "")
        return SatProfile(
            freq_mhz=sat.get("freq_mhz", 433.0),
            modulation=mod,
            smartrf_profile=sat.get("smartrf_profile"),
            description=sat.get("description", ""),
            cc1200_compatible=sat.get("cc1200_compatible", True),
            protocol=proto,
            use_serial_mode=proto in ("ax25_g3ruh", "usp"),
        )

    def fetch_satnogs_transmitters(self, norad_id: int) -> list:
        """Fetch transmitter info from SatNOGS DB API for a given NORAD ID.

        GET https://db.satnogs.org/api/transmitters/?satellite__norad_cat_id={id}

        Returns list of dicts with CC1200-compatible transmitters.
        Caches results locally for 24 hours.
        """
        # Check cache first
        cached = self._read_cache(norad_id)
        if cached is not None:
            return cached

        url = (
            f"https://db.satnogs.org/api/transmitters/"
            f"?satellite__norad_cat_id={norad_id}&format=json"
        )

        try:
            req = urllib.request.Request(
                url, headers={"User-Agent": "SatNOGS-Basestation/1.0"}
            )
            resp = urllib.request.urlopen(req, timeout=15)
            data = json.loads(resp.read().decode("utf-8"))
        except Exception as e:
            print(f"sat_library: SatNOGS API fetch failed for NORAD {norad_id}: {e}")
            return []

        # Filter to CC1200-compatible downlink transmitters in UHF band
        compatible = []
        for tx in data:
            if not tx.get("alive", True):
                continue
            if tx.get("type") not in ("Transceiver", "Transmitter"):
                continue

            freq = tx.get("downlink_low")
            if not freq or not (420e6 <= freq <= 450e6):
                continue

            mode = tx.get("mode", "")
            cc_fmt = SATNOGS_MODE_MAP.get(mode.upper())
            if cc_fmt is None:
                continue

            entry = {
                "norad_id": norad_id,
                "freq_mhz": freq / 1e6,
                "mode": mode,
                "cc1200_format": cc_fmt,
                "baud": tx.get("baud"),
                "description": tx.get("description", ""),
                "uuid": tx.get("uuid", ""),
            }
            compatible.append(entry)

        self._write_cache(norad_id, compatible)
        return compatible

    def update_from_satnogs(self) -> int:
        """Bulk-fetch CC1200-compatible UHF transmitters from SatNOGS DB.

        Queries all transmitters, filters to CC1200-compatible UHF modes,
        and merges new satellites into sat_profiles.json.

        Returns number of new profiles added.
        """
        url = (
            "https://db.satnogs.org/api/transmitters/"
            "?format=json&type=Transmitter"
        )

        try:
            req = urllib.request.Request(
                url, headers={"User-Agent": "SatNOGS-Basestation/1.0"}
            )
            resp = urllib.request.urlopen(req, timeout=30)
            all_tx = json.loads(resp.read().decode("utf-8"))
        except Exception as e:
            print(f"sat_library: bulk SatNOGS fetch failed: {e}")
            return 0

        # Collect CC1200-compatible UHF transmitters
        new_sats: Dict[int, dict] = {}
        existing_norad_ids = {s.get("norad_id") for s in self.satellites if s.get("norad_id")}

        for tx in all_tx:
            if not tx.get("alive", True):
                continue

            norad_id = tx.get("norad_cat_id")
            if not norad_id or norad_id in existing_norad_ids:
                continue

            freq = tx.get("downlink_low")
            if not freq or not (420e6 <= freq <= 450e6):
                continue

            mode = tx.get("mode", "")
            cc_fmt = SATNOGS_MODE_MAP.get(mode)
            if cc_fmt is None:
                # Try uppercase match
                cc_fmt = SATNOGS_MODE_MAP.get(mode.upper())
            if cc_fmt is None:
                continue

            # Skip 1200 baud AFSK — Bell 202 audio tones, not baseband FSK
            baud = tx.get("baud") or 0
            if mode.upper() == "AFSK" and baud <= 1200:
                continue

            if norad_id not in new_sats:
                entry = {
                    "name_match": "",
                    "norad_id": norad_id,
                    "freq_mhz": round(freq / 1e6, 3),
                    "description": f"SatNOGS DB: {tx.get('description', mode)}",
                }
                if baud and baud > 0:
                    # Deviation estimation based on modulation type
                    if mode.upper() in ("GMSK", "MSK", "MSK AX.100 MODE 5",
                                         "MSK AX.100 MODE 6"):
                        deviation = int(baud / 4)  # h=0.5
                    else:
                        deviation = int(baud * 0.35)  # h~0.7 typical amateur FSK
                    # Carson's rule + 20% margin
                    rx_bw = max(12.5, (baud + 2 * deviation) * 1.2 / 1000)
                    entry["modulation"] = {
                        "format": cc_fmt,
                        "symbol_rate_bps": int(baud),
                        "deviation_hz": deviation,
                        "rx_bw_khz": round(rx_bw, 1),
                    }
                new_sats[norad_id] = entry

        if not new_sats:
            return 0

        # Merge into local database
        self.satellites.extend(new_sats.values())
        self._save()
        return len(new_sats)

    def _save(self):
        """Write the current satellite database back to disk."""
        data = {
            "default_profile": self.default_profile,
            "satellites": self.satellites,
        }
        try:
            with open(self.profiles_path, "w", encoding="utf-8") as f:
                json.dump(data, f, indent=4, ensure_ascii=False)
        except Exception as e:
            print(f"sat_library: failed to save {self.profiles_path}: {e}")

    def _cache_path(self, norad_id: int) -> str:
        os.makedirs(_API_CACHE_DIR, exist_ok=True)
        return os.path.join(_API_CACHE_DIR, f"tx_{norad_id}.json")

    def _read_cache(self, norad_id: int) -> Optional[list]:
        path = self._cache_path(norad_id)
        try:
            if os.path.exists(path):
                age = time.time() - os.path.getmtime(path)
                if age < _API_CACHE_MAX_AGE:
                    with open(path, "r") as f:
                        return json.load(f)
        except Exception:
            pass
        return None

    def _write_cache(self, norad_id: int, data: list):
        try:
            with open(self._cache_path(norad_id), "w") as f:
                json.dump(data, f)
        except Exception:
            pass


# ---------------------------------------------------------------------------
# CLI: smoke test + SatNOGS API query
# ---------------------------------------------------------------------------
if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="Satellite Profile Library")
    parser.add_argument("--fetch", type=int, metavar="NORAD_ID",
                        help="Fetch transmitters from SatNOGS DB for a NORAD ID")
    parser.add_argument("--update", action="store_true",
                        help="Bulk-fetch CC1200-compatible satellites from SatNOGS DB")
    parser.add_argument("--lookup", metavar="NAME",
                        help="Look up a satellite by name")
    args = parser.parse_args()

    lib = SatLibrary()
    print(f"Loaded {len(lib.satellites)} satellite profiles")
    print(f"Default profile: {lib.default_profile}")

    if args.lookup:
        p = lib.lookup(name=args.lookup)
        if p:
            print(f"\nMatch: {p.freq_mhz} MHz — {p.description}")
            if p.modulation:
                m = p.modulation
                print(f"  Modulation: {m.format}, {m.symbol_rate_bps} bps, "
                      f"dev={m.deviation_hz} Hz, BW={m.rx_bw_khz} kHz")
                if m.sync_word:
                    print(f"  Sync word: 0x{m.sync_word}")
            if p.smartrf_profile:
                print(f"  SmartRF profile: {p.smartrf_profile}")
        else:
            print(f"\nNo profile for '{args.lookup}'")

    if args.fetch:
        print(f"\nFetching SatNOGS transmitters for NORAD {args.fetch}...")
        txs = lib.fetch_satnogs_transmitters(args.fetch)
        if txs:
            for tx in txs:
                print(f"  {tx['freq_mhz']:.3f} MHz {tx['mode']} "
                      f"({tx['cc1200_format']}) {tx.get('baud', '?')} baud "
                      f"— {tx['description']}")
        else:
            print("  No CC1200-compatible UHF transmitters found")

    if args.update:
        print("\nBulk-fetching CC1200-compatible satellites from SatNOGS DB...")
        added = lib.update_from_satnogs()
        print(f"Added {added} new satellite profiles")

    # Smoke test: look up known satellites
    if not args.fetch and not args.update and not args.lookup:
        print("\n--- Smoke test ---")
        tests = [
            (25544, "ISS (ZARYA)"),
            (43017, "FOX-1CLIFF"),
            (0, "AETHERSPACE-1"),
            (99999, "UNKNOWN-SAT"),
        ]
        for nid, name in tests:
            p = lib.lookup(norad_id=nid, name=name)
            if p:
                mod_str = ""
                if p.modulation:
                    mod_str = f" [{p.modulation.format} {p.modulation.symbol_rate_bps} bps]"
                print(f"  {name:20s} -> {p.freq_mhz:.3f} MHz{mod_str} — {p.description}")
            else:
                print(f"  {name:20s} -> no profile (or incompatible)")
