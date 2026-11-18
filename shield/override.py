"""Physical manual override: AUTO/MANUAL toggle + OPEN/CLOSE buttons, active-low, debounced.
Polled from the control loop, so a time-based debounce keeps everything on one thread."""
import time
from dataclasses import dataclass

from . import motor as m


@dataclass(frozen=True)
class OverrideStatus:
    manual: bool = False
    open_pressed: bool = False
    close_pressed: bool = False


class Override:
    def __init__(self, chip: int, cfg):
        p = cfg.pins
        self.chip = chip
        self.pins = (p.mode_switch, p.btn_open, p.btn_close)
        self.hold_s = cfg.override.debounce_ms / 1000.0
        self.manual_when_closed = cfg.override.manual_when_switch_closed
        for pin in self.pins:
            m.claim(m.lgpio.gpio_claim_input, chip, pin, m.lgpio.SET_PULL_UP)
        now = time.monotonic()
        self._stable = [self._closed(pin) for pin in self.pins]
        self._raw = list(self._stable)
        self._since = [now] * len(self.pins)

    def _closed(self, pin: int) -> bool:
        return m.lgpio.gpio_read(self.chip, pin) == 0  # contact closed pulls the pin to GND

    def read(self) -> OverrideStatus:
        now = time.monotonic()
        for i, pin in enumerate(self.pins):
            raw = self._closed(pin)
            if raw != self._raw[i]:
                self._raw[i], self._since[i] = raw, now
            elif raw != self._stable[i] and now - self._since[i] >= self.hold_s:
                self._stable[i] = raw
        switch, open_pressed, close_pressed = self._stable
        return OverrideStatus(switch if self.manual_when_closed else not switch,
                              open_pressed, close_pressed)

    def close(self) -> None:
        for pin in self.pins:
            m.lgpio.gpio_free(self.chip, pin)
