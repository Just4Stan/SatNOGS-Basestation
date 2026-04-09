"""
Buzzer driver — sends beep commands to the RF HAT Pico over UART.

The passive buzzer is on Pico GP2 (driven via switching transistor).
The Pico firmware handles tone generation locally — the Pi just sends
a pattern ID and the Pico plays it non-blocking.

Usage:
    from rf_hat import CC1200Link
    from buzzer import Buzzer

    link = CC1200Link()
    link.open("/dev/serial0")
    bz = Buzzer(link)
    bz.beep_ready()      # triple beep
    bz.beep_aos()        # satellite acquired
    bz.beep_packet()     # packet received

    # Without a link (e.g., --no-rf mode): all beeps are silently skipped
    bz = Buzzer()
    bz.beep_ready()      # no-op
"""

from typing import Optional


class Buzzer:
    # Pattern IDs — must match firmware (main.cpp buzzer_start)
    READY      = 0x01
    AOS        = 0x02
    LOS        = 0x03
    PACKET     = 0x04
    ERROR      = 0x05
    WIFI_OK    = 0x06
    GPS_OK     = 0x07
    SETUP_DONE = 0x08

    def __init__(self, link=None):
        """Create a buzzer driver.

        Args:
            link: CC1200Link instance with an open UART connection.
                  If None, all beep calls are silently skipped.
        """
        self.link = link

    def _beep(self, pattern: int):
        if self.link and self.link.is_open():
            try:
                self.link.buzzer(pattern)
            except Exception:
                pass

    def beep_ready(self):
        """Station ready — triple beep (1000 Hz)."""
        self._beep(self.READY)

    def beep_aos(self):
        """AOS — rising two-tone (800 -> 1200 Hz)."""
        self._beep(self.AOS)

    def beep_los(self):
        """LOS — falling two-tone (1200 -> 800 Hz)."""
        self._beep(self.LOS)

    def beep_packet(self):
        """Packet received — short high beep (1500 Hz)."""
        self._beep(self.PACKET)

    def beep_error(self):
        """Error — low tone 0.5s (400 Hz)."""
        self._beep(self.ERROR)

    def beep_wifi_ok(self):
        """WiFi connected — ascending C5→E5→G5."""
        self._beep(self.WIFI_OK)

    def beep_gps_ok(self):
        """GPS location set — 2 high beeps (2000 Hz)."""
        self._beep(self.GPS_OK)

    def beep_setup_done(self):
        """Setup wizard complete — triumphant C5→E5→G5→C6→G5."""
        self._beep(self.SETUP_DONE)

    def cleanup(self):
        """No-op — kept for API compatibility."""
        pass
