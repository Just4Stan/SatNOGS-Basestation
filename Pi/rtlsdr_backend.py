"""
RTL-SDR RF backend — wideband IQ capture using pyrtlsdr.

Captures raw IQ samples to disk during satellite passes. Does NOT decode
packets (that's the CC1200's job). Provides:
  - Wideband IQ recording for post-pass analysis
  - Signal power estimation (dBFS)
  - Post-pass waterfall generation (via waterfall.py)

The RTL-SDR and CC1200 serve different purposes:
  - CC1200: hardware packet demodulation for AetherSpace
  - RTL-SDR: wideband capture, waterfall, FM audio, signal survey

Usage:
    from rtlsdr_backend import RtlSdrBackend

    rtl = RtlSdrBackend(device_index=0, gain='auto', sample_rate=240000)
    rtl.open()
    rtl.set_frequency(433e6)
    rtl.start_rx()
    # ... during pass ...
    rtl.stop_rx()  # triggers waterfall generation
    rtl.close()
"""

import os
import time
import struct
import threading
import datetime
from pathlib import Path
from typing import Optional

from rf_backend import RfBackend, RfMetrics

# Lazy import — pyrtlsdr may not be installed
RtlSdr = None


def _ensure_rtlsdr():
    global RtlSdr
    if RtlSdr is None:
        from rtlsdr import RtlSdr as _RtlSdr
        RtlSdr = _RtlSdr


class RtlSdrBackend(RfBackend):
    """RTL-SDR IQ capture backend."""

    def __init__(self, device_index: int = 0, gain='auto',
                 sample_rate: int = 240000):
        self.device_index = device_index
        self.gain_setting = gain
        self.sample_rate = sample_rate

        self.sdr = None
        self.freq_hz: float = 0.0
        self.streaming = False
        self.signal_dbfs: Optional[float] = None

        # IQ capture state
        self._capture_thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()
        self._iq_file = None
        self._iq_path: Optional[str] = None
        self._iq_bytes_written = 0
        self._obs_dir: Optional[str] = None

        # Post-pass waterfall
        self._waterfall_path: Optional[str] = None
        self._waterfall_thread: Optional[threading.Thread] = None

    def name(self) -> str:
        return "RTL-SDR"

    def open(self) -> bool:
        try:
            _ensure_rtlsdr()
            self.sdr = RtlSdr(self.device_index)
            self.sdr.sample_rate = self.sample_rate
            if self.gain_setting == 'auto':
                self.sdr.gain = 'auto'
            else:
                self.sdr.gain = float(self.gain_setting)
            return True
        except Exception as e:
            self.sdr = None
            return False

    def close(self):
        self._stop_capture()
        if self.sdr:
            try:
                self.sdr.close()
            except Exception:
                pass
            self.sdr = None

    def start_rx(self) -> bool:
        if not self.sdr:
            return False

        # Create observation directory
        ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        self._obs_dir = os.path.expanduser(f"~/observations/{ts}")
        os.makedirs(self._obs_dir, exist_ok=True)

        self._iq_path = os.path.join(self._obs_dir, "iq.raw")
        self._iq_bytes_written = 0
        self._waterfall_path = None

        # Start threaded capture
        self._stop_event.clear()
        self._capture_thread = threading.Thread(
            target=self._capture_loop, daemon=True)
        self._capture_thread.start()
        self.streaming = True
        return True

    def stop_rx(self):
        self._stop_capture()
        self.streaming = False

        # Trigger waterfall generation in background
        if self._iq_path and os.path.exists(self._iq_path) and self._iq_bytes_written > 0:
            wf_path = os.path.join(self._obs_dir, "waterfall.png")
            self._waterfall_thread = threading.Thread(
                target=self._generate_waterfall,
                args=(self._iq_path, wf_path, self.sample_rate, self.freq_hz),
                daemon=True,
            )
            self._waterfall_thread.start()

    def set_frequency(self, freq_hz: float) -> bool:
        self.freq_hz = freq_hz
        if self.sdr:
            try:
                self.sdr.center_freq = freq_hz
                return True
            except Exception:
                return False
        return False

    def poll_packets(self, logfile=None) -> int:
        # RTL-SDR doesn't decode packets
        return 0

    def get_metrics(self) -> RfMetrics:
        return RfMetrics(
            signal_dbm=self.signal_dbfs,
            signal_label="Signal",
            signal_unit="dBFS",
            freq_hz=self.freq_hz,
            streaming=self.streaming,
            backend_name="RTL-SDR",
            packets=0,
            total_bytes=self._iq_bytes_written,
            waterfall_path=self._waterfall_path,
        )

    def get_status_dict(self) -> dict:
        return {
            "streaming": self.streaming,
            "rssi": self.signal_dbfs,
            "rssi_unit": "dBFS",
            "rssi_label": "Signal",
            "packets": 0,
            "freq_mhz": self.freq_hz / 1e6 if self.freq_hz else 0,
            "radio_config": {
                "freq_mhz": self.freq_hz / 1e6 if self.freq_hz else 0,
                "modulation": "IQ capture",
                "symbol_rate_bps": 0,
                "profile": f"RTL-SDR {self.sample_rate/1000:.0f} ksps",
            },
            "rf_backend": "rtlsdr",
            "iq_bytes": self._iq_bytes_written,
            "waterfall": self._waterfall_path,
            "obs_dir": self._obs_dir,
        }

    def configure_for_satellite(self, profile, default_profile: str = "") -> bool:
        """For RTL-SDR, just retune to the satellite's frequency."""
        freq_hz = profile.freq_mhz * 1e6
        return self.set_frequency(freq_hz)

    # ------------------------------------------------------------------
    # Internal
    # ------------------------------------------------------------------
    def _capture_loop(self):
        """Threaded IQ capture — reads samples and writes raw interleaved
        uint8 I/Q to disk (same format as rtl_sdr CLI tool)."""
        try:
            self._iq_file = open(self._iq_path, "wb")
            # Read in 16K sample chunks (~67ms at 240 ksps)
            num_samples = 16384

            while not self._stop_event.is_set():
                try:
                    samples = self.sdr.read_samples(num_samples)
                except Exception:
                    break

                # Estimate signal power from IQ magnitude
                # samples is complex64 array, magnitude squared -> dBFS
                if len(samples) > 0:
                    import numpy as np
                    power = np.mean(np.abs(samples) ** 2)
                    if power > 0:
                        self.signal_dbfs = round(10 * np.log10(power), 1)

                # Write raw interleaved uint8 (rtl_sdr format)
                # pyrtlsdr returns complex float [-1,1], convert to uint8 [0,255]
                import numpy as np
                iq_interleaved = np.empty(len(samples) * 2, dtype=np.uint8)
                iq_interleaved[0::2] = np.clip(
                    (samples.real + 1.0) * 127.5, 0, 255).astype(np.uint8)
                iq_interleaved[1::2] = np.clip(
                    (samples.imag + 1.0) * 127.5, 0, 255).astype(np.uint8)

                raw = iq_interleaved.tobytes()
                self._iq_file.write(raw)
                self._iq_bytes_written += len(raw)

        except Exception:
            pass
        finally:
            if self._iq_file:
                self._iq_file.close()
                self._iq_file = None

    def _stop_capture(self):
        """Stop the capture thread."""
        self._stop_event.set()
        if self._capture_thread and self._capture_thread.is_alive():
            # Cancel any pending read by closing the device briefly
            try:
                self.sdr.cancel_read_async()
            except Exception:
                pass
            self._capture_thread.join(timeout=5)
        self._capture_thread = None

    @staticmethod
    def _generate_waterfall(iq_path: str, output_path: str,
                            sample_rate: int, center_freq: float):
        """Generate waterfall PNG from IQ file (runs in background thread)."""
        try:
            from waterfall import generate_waterfall
            generate_waterfall(iq_path, output_path,
                               sample_rate=sample_rate,
                               center_freq=center_freq)
        except Exception:
            pass
