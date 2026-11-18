"""BTS7960 motor driver, HD Hex quadrature encoder and optional limit switches (Pi 5).

BTS7960 truth table (R_EN and L_EN always driven together):
    EN low                  coast
    EN high, both PWM 0     brake
    EN high, RPWM on        forward
    EN high, LPWM on        reverse

Pi 5 notes: pigpio does not work, so GPIO is lgpio. Hardware PWM is sysfs on the RP1 PWM0
block, where GPIO18/19 are channels 2/3 and the pwmchipN number varies by kernel. Sysfs
PWM keeps running if Python dies, which is what safe_stop.sh is for.
"""
import collections
import errno
import logging
import math
import time
from pathlib import Path

try:
    import lgpio
except ImportError:  # lets the pure-logic tests run off the Pi
    lgpio = None

log = logging.getLogger(__name__)

RP1_PWM_DEVICE = "1f00098000.pwm"
RP1_PWM_CHANNEL = {12: 0, 13: 1, 18: 2, 19: 3}


def split_duty(duty: float, max_duty: float = 1.0) -> tuple[float, float]:
    """Signed duty -> (RPWM, LPWM). At most one is ever non-zero; NaN/inf gives (0, 0)."""
    if not math.isfinite(duty):
        return 0.0, 0.0
    duty = max(-max_duty, min(max_duty, duty))
    return (duty, 0.0) if duty > 0 else (0.0, -duty if duty < 0 else 0.0)


# ---- GPIO ---------------------------------------------------------------------------

def open_gpiochip() -> int:
    """The RP1 header chip is gpiochip0 on newer kernels, gpiochip4 on older: find it by label."""
    if lgpio is None:
        raise RuntimeError("lgpio missing: sudo apt install python3-lgpio (then re-run install.sh)")
    for dev in sorted(Path("/dev").glob("gpiochip*")):
        try:
            h = lgpio.gpiochip_open(int(dev.name[8:]))
        except (lgpio.error, ValueError):
            continue
        if lgpio.gpio_get_chip_info(h)[3] == "pinctrl-rp1":
            return h
        lgpio.gpiochip_close(h)
    raise RuntimeError("no 'pinctrl-rp1' GPIO chip found. Is this a Raspberry Pi 5?")


def claim(fn, chip: int, pin: int, *args) -> None:
    try:
        fn(chip, pin, *args)
    except lgpio.error as exc:
        hint = (" -- another shield process holds it (VS Code run still going? "
                "`sudo systemctl stop shield`?)") if "busy" in str(exc).lower() else ""
        raise RuntimeError(f"GPIO{pin}: {exc}{hint}") from exc


# ---- PWM ----------------------------------------------------------------------------

def find_pwm_chip() -> Path:
    root = Path("/sys/class/pwm")
    for chip in sorted(root.glob("pwmchip*")):
        if RP1_PWM_DEVICE in str(chip.resolve()):
            return chip
    raise RuntimeError("hardware PWM not found. Add 'dtoverlay=pwm-2chan,pin=18,func=2,pin2=19,"
                       "func2=2' under [all] in /boot/firmware/config.txt and reboot")


class HardwarePwm:
    hardware = True
    max_duty = 1.0

    def __init__(self, pin: int, freq_hz: int):
        if pin not in RP1_PWM_CHANNEL:
            raise RuntimeError(f"GPIO{pin} has no RP1 hardware PWM channel")
        chip = find_pwm_chip()
        self.channel = RP1_PWM_CHANNEL[pin]
        self.path = chip / f"pwm{self.channel}"
        if not self.path.exists():
            (chip / "export").write_text(str(self.channel))
        deadline = time.monotonic() + 2.0
        while True:  # a fresh export takes a moment to appear and get its udev permissions
            try:
                self._w("enable", "0")
                break
            except OSError as exc:
                if exc.errno == errno.EINVAL:
                    break
                if time.monotonic() > deadline:
                    raise RuntimeError(f"{self.path} not writable ({exc}). Run install.sh "
                                       "(udev rule) and log out/in, or use sudo") from exc
                time.sleep(0.02)
        self.period_ns = round(1e9 / freq_hz)
        self._w("duty_cycle", "0")  # duty must never exceed the period
        self._w("period", str(self.period_ns))
        if (self.path / "polarity").exists():
            self._w("polarity", "normal")  # inverted would make duty 0 = full on
        self._w("enable", "1")
        self.duty = 0.0

    def _w(self, name: str, value: str) -> None:
        (self.path / name).write_text(value)

    def set(self, duty: float) -> None:
        duty = min(1.0, max(0.0, duty))
        if duty != self.duty:
            self._w("duty_cycle", str(round(duty * self.period_ns)))
            self.duty = duty

    def close(self) -> None:
        try:
            self.set(0.0)
        finally:
            self._w("enable", "0")


class SoftPwm:
    """lgpio software PWM: fallback only (<= 10 kHz, jittery, duty capped)."""
    hardware = False

    def __init__(self, chip: int, pin: int, freq_hz: int, max_duty: float):
        self.chip, self.pin, self.freq_hz, self.max_duty = chip, pin, freq_hz, max_duty
        claim(lgpio.gpio_claim_output, chip, pin, 0)
        self.duty = 0.0

    def set(self, duty: float) -> None:
        duty = min(self.max_duty, max(0.0, duty))
        if duty == self.duty:
            return
        if duty == 0.0:
            lgpio.tx_pwm(self.chip, self.pin, 0, 0)
            lgpio.gpio_write(self.chip, self.pin, 0)
        else:
            lgpio.tx_pwm(self.chip, self.pin, self.freq_hz, duty * 100.0)
        self.duty = duty

    def close(self) -> None:
        try:
            self.set(0.0)
        finally:
            lgpio.gpio_free(self.chip, self.pin)


def make_pwm(chip: int, cfg):
    p = cfg.pins
    opened = []
    try:
        for pin in (p.rpwm, p.lpwm):
            opened.append(HardwarePwm(pin, cfg.pwm.frequency_hz))
        return opened
    except (RuntimeError, OSError) as exc:
        for pwm in opened:
            pwm.close()
        if not cfg.pwm.software_fallback:
            raise
        log.warning("hardware PWM unavailable (%s). FALLING BACK to lgpio software PWM: "
                    "%d Hz, duty capped at %.0f%%", exc, cfg.pwm.software_max_hz,
                    cfg.pwm.software_max_duty * 100)
    freq = min(cfg.pwm.frequency_hz, cfg.pwm.software_max_hz)
    rpwm = SoftPwm(chip, p.rpwm, freq, cfg.pwm.software_max_duty)
    try:
        return [rpwm, SoftPwm(chip, p.lpwm, freq, cfg.pwm.software_max_duty)]
    except Exception:
        rpwm.close()
        raise


# ---- motor driver -------------------------------------------------------------------

class Bts7960:
    """Enforces: RPWM and LPWM never both on (the inactive channel is zeroed and read back
    first), EN high before any PWM, and a brake hold before every direction reversal.
    Any write failure drops to coast before the error propagates."""

    def __init__(self, chip: int, cfg):
        self.chip = chip
        self.en_pins = (cfg.pins.r_en, cfg.pins.l_en)
        for pin in self.en_pins:  # EN low before PWM exists: the motor coasts from here on
            claim(lgpio.gpio_claim_output, chip, pin, 0)
        try:
            self.rpwm, self.lpwm = make_pwm(chip, cfg)
        except Exception:
            for pin in self.en_pins:
                lgpio.gpio_free(chip, pin)
            raise
        self.hardware_pwm = self.rpwm.hardware
        self.max_duty = min(cfg.motor.max_duty, self.rpwm.max_duty)
        self.reverse_brake_s = cfg.motor.reverse_brake_ms / 1000.0
        self.duty = 0.0      # signed duty actually applied (+ = forward/RPWM)
        self.mode = "coast"  # coast | brake | forward | reverse
        self._dir = 0
        self._stopped_at = -math.inf

    def set_duty(self, duty: float) -> float:
        """Drive at signed duty. Non-blocking: returns the duty applied, which is 0 while
        the pre-reversal brake is held, so call it every control tick."""
        r, l = split_duty(duty, self.max_duty)
        if r == l == 0.0:
            self.brake()
            return 0.0
        direction = 1 if r else -1
        if direction != self._dir:
            self.brake()
            if time.monotonic() - self._stopped_at < self.reverse_brake_s:
                return 0.0
            self._dir = direction
        self._apply(r, l)
        self.duty = r - l
        self.mode = "forward" if r else "reverse"
        return self.duty

    def brake(self) -> None:
        """Both low sides on. Slows the motor; friction holds the shell afterwards."""
        if self.duty:
            self._stopped_at = time.monotonic()
        self._apply(0.0, 0.0)
        self.duty, self.mode = 0.0, "brake"

    def coast(self) -> None:
        """Both half-bridges off. Always attempts EN low, even if PWM writes fail."""
        if self.duty:
            self._stopped_at = time.monotonic()
        try:
            self.rpwm.set(0.0)
            self.lpwm.set(0.0)
        finally:
            self._enable(False)
            self.duty, self.mode = 0.0, "coast"

    def close(self) -> None:
        try:
            self.coast()
        except Exception:
            log.exception("coast failed during close")
            self._emergency_coast()
        finally:
            for res in (self.rpwm, self.lpwm):
                try:
                    res.close()
                except Exception:
                    log.exception("closing PWM failed")
            for pin in self.en_pins:
                lgpio.gpio_free(self.chip, pin)

    def _apply(self, r: float, l: float) -> None:
        if r and l:
            self._emergency_coast()
            raise RuntimeError(f"PWM interlock: refused RPWM={r} with LPWM={l}")
        off, on, value = (self.lpwm, self.rpwm, r) if r else (self.rpwm, self.lpwm, l)
        try:
            off.set(0.0)
            if off.duty != 0.0:
                raise RuntimeError("PWM interlock: inactive channel did not read back 0")
            self._enable(True)
            on.set(value)
        except Exception:
            log.exception("motor write failed; coasting")
            self._emergency_coast()
            raise

    def _enable(self, on: bool) -> None:
        for pin in self.en_pins:
            lgpio.gpio_write(self.chip, pin, int(on))

    def _emergency_coast(self) -> None:
        """Best effort, never raises."""
        for step in (lambda: self.rpwm.set(0.0), lambda: self.lpwm.set(0.0),
                     lambda: self._enable(False)):
            try:
                step()
            except Exception:
                log.exception("emergency coast step failed")
        self.duty, self.mode = 0.0, "coast"


# ---- encoder ------------------------------------------------------------------------

# (previous AB, new AB) -> step. Forward Gray sequence: 00 -> 01 -> 11 -> 10 -> 00.
_QUAD = {(0, 1): 1, (1, 3): 1, (3, 2): 1, (2, 0): 1, (1, 0): -1, (3, 1): -1, (2, 3): -1, (0, 2): -1}


class Encoder:
    """x4 quadrature from lgpio edge callbacks. Callbacks only queue events; they are decoded
    in kernel-timestamp order when `count` is read, so A/B ordering is always right."""

    def __init__(self, chip: int, cfg):
        e = cfg.encoder
        self.chip = chip
        self.pins = (cfg.pins.enc_a, cfg.pins.enc_b)
        self.sign = -1 if e.invert else 1
        self.counts_per_output_rev = e.counts_per_motor_rev * e.gear_ratio
        for pin in self.pins:
            claim(lgpio.gpio_claim_alert, chip, pin, lgpio.BOTH_EDGES, lgpio.SET_PULL_UP)
        a, b = self.levels()
        self._state = (a << 1) | b
        self._count = 0
        self.errors = 0  # missed/illegal transitions: noise or a loose wire
        self._events = collections.deque()
        self._callbacks = [lgpio.callback(chip, pin, lgpio.BOTH_EDGES, self._edge)
                           for pin in self.pins]

    def _edge(self, chip, gpio, level, tick):
        if level < 2:  # 2 = watchdog report, not an edge
            self._events.append((tick, gpio, level))

    @property
    def count(self) -> int:
        """Signed motor-shaft counts since startup."""
        batch = []
        while self._events:
            batch.append(self._events.popleft())
        for _, gpio, level in sorted(batch):
            s = self._state
            new = (level << 1) | (s & 1) if gpio == self.pins[0] else (s & 2) | level
            step = _QUAD.get((s, new))
            if step is None:
                self.errors += 1
            else:
                self._count += step
            self._state = new
        return self.sign * self._count

    def output_revs(self, counts: int) -> float:
        """Motor counts -> revolutions of the driven gear (after gear_ratio)."""
        return counts / self.counts_per_output_rev

    def levels(self) -> tuple[int, int]:
        return tuple(lgpio.gpio_read(self.chip, pin) for pin in self.pins)

    def close(self) -> None:
        for cb in self._callbacks:
            cb.cancel()
        for pin in self.pins:
            lgpio.gpio_free(self.chip, pin)


class LimitSwitches:
    """Optional active-low end switches (limits.enabled)."""

    def __init__(self, chip: int, cfg):
        self.chip = chip
        self.pins = (cfg.pins.limit_open, cfg.pins.limit_closed)
        for pin in self.pins:
            claim(lgpio.gpio_claim_input, chip, pin, lgpio.SET_PULL_UP)

    def read(self) -> tuple[bool, bool]:
        """(at_open, at_closed)"""
        return tuple(lgpio.gpio_read(self.chip, pin) == 0 for pin in self.pins)

    def close(self) -> None:
        for pin in self.pins:
            lgpio.gpio_free(self.chip, pin)
