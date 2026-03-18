"""
RF Backend abstraction — common interface for CC1200 and RTL-SDR.

Both RF backends (CC1200 HAT and RTL-SDR) implement this ABC so that
station.py can switch between them without conditional logic everywhere.
"""

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import Optional, List, Tuple
import datetime


@dataclass
class RfMetrics:
    """Snapshot of RF backend metrics for status reporting."""
    signal_dbm: Optional[float] = None    # RSSI (CC1200) or dBFS (RTL-SDR)
    signal_label: str = "RSSI"            # "RSSI" or "Signal"
    signal_unit: str = "dBm"              # "dBm" or "dBFS"
    freq_hz: float = 0.0
    streaming: bool = False
    backend_name: str = ""
    packets: int = 0
    total_bytes: int = 0
    waterfall_path: Optional[str] = None  # path to last waterfall PNG


class RfBackend(ABC):
    """Abstract base class for RF reception backends."""

    @abstractmethod
    def name(self) -> str:
        """Short name for this backend (e.g. 'CC1200', 'RTL-SDR')."""

    @abstractmethod
    def open(self) -> bool:
        """Open hardware connection. Returns True on success."""

    @abstractmethod
    def close(self):
        """Release hardware resources."""

    @abstractmethod
    def start_rx(self) -> bool:
        """Begin receiving. Returns True on success."""

    @abstractmethod
    def stop_rx(self):
        """Stop receiving."""

    @abstractmethod
    def set_frequency(self, freq_hz: float) -> bool:
        """Set the receive frequency in Hz. Returns True on success."""

    @abstractmethod
    def poll_packets(self, logfile=None) -> int:
        """Process pending RX data. Returns number of new decoded packets."""

    @abstractmethod
    def get_metrics(self) -> RfMetrics:
        """Return current metrics snapshot."""

    @abstractmethod
    def get_status_dict(self) -> dict:
        """Return status dict for dashboard JSON (written to station_status.json)."""

    @abstractmethod
    def configure_for_satellite(self, profile, default_profile: str = "") -> bool:
        """Reconfigure backend for a specific satellite. Returns True on success."""
