"""Loads config.yaml into attribute-style objects: cfg.pins.rpwm, cfg.motor.max_duty, ..."""
import os
import re
from pathlib import Path
from types import SimpleNamespace

import yaml

REPO = Path(__file__).resolve().parents[1]
DEFAULT_PATH = REPO / "config.yaml"


class ConfigError(ValueError):
    pass


def _ns(value):
    if isinstance(value, dict):
        return SimpleNamespace(**{k: _ns(v) for k, v in value.items()})
    return value


def load(path: str | Path | None = None) -> SimpleNamespace:
    path = Path(path or os.environ.get("SHIELD_CONFIG") or DEFAULT_PATH)
    try:
        cfg = _ns(yaml.safe_load(path.read_text(encoding="utf-8")))
    except (OSError, yaml.YAMLError, TypeError) as exc:
        raise ConfigError(f"cannot read {path}: {exc}") from exc
    cfg.path = path
    validate(cfg)
    return cfg


def validate(cfg) -> None:
    try:
        pins = dict(vars(cfg.pins))
        if not cfg.limits.enabled:
            pins.pop("limit_open"), pins.pop("limit_closed")
        for name, pin in pins.items():
            if not (isinstance(pin, int) and 0 <= pin <= 27):
                raise ConfigError(f"pins.{name}: {pin!r} is not a header GPIO (0-27)")
        dupes = {p for p in pins.values() if list(pins.values()).count(p) > 1}
        if dupes:
            raise ConfigError(f"pins: GPIO {sorted(dupes)} used more than once")
        if not 1 <= cfg.pwm.frequency_hz <= 25000:
            raise ConfigError("pwm.frequency_hz must be 1..25000 (BTS7960 limit)")
        if cfg.motor.close_direction not in ("forward", "reverse"):
            raise ConfigError("motor.close_direction must be forward or reverse")
        if not 0 < cfg.motor.max_duty <= 1:
            raise ConfigError("motor.max_duty must be in (0, 1]")
        if cfg.encoder.counts_per_motor_rev <= 0 or cfg.encoder.gear_ratio <= 0:
            raise ConfigError("encoder.counts_per_motor_rev and gear_ratio must be > 0")
        mo = cfg.motion
        if mo.travel_counts is not None and mo.travel_counts <= 2 * mo.soft_margin_counts:
            raise ConfigError("motion.travel_counts must exceed 2 * soft_margin_counts")
        if mo.hold not in ("brake", "pwm"):
            raise ConfigError("motion.hold must be brake or pwm")
        if min(mo.max_speed, mo.accel, mo.free_speed) <= 0:
            raise ConfigError("motion.max_speed, accel and free_speed must be > 0")
        if not 50 <= cfg.control.loop_hz <= 1000:
            raise ConfigError("control.loop_hz must be 50..1000")
        if cfg.control.watchdog_ms < 3 * 1000 / cfg.control.loop_hz:
            raise ConfigError("control.watchdog_ms must be at least 3 loop periods")
    except AttributeError as exc:
        raise ConfigError(f"missing config key: {exc}") from exc


def save(cfg, updates: dict) -> None:
    """Write {"section.key": value} into the config file in place, keeping comments."""
    lines = cfg.path.read_text(encoding="utf-8").splitlines(keepends=True)
    for dotted, value in updates.items():
        section, key = dotted.split(".")
        text = "null" if value is None else str(value)
        inside = found = False
        for i, line in enumerate(lines):
            if re.match(rf"{section}:\s*(#.*)?$", line):
                inside = True
            elif inside and re.match(r"\S", line):
                break  # left the section
            elif inside and (m := re.match(rf"(\s+{key}:\s*)[^#\n]*?(\s*#.*)?$", line.rstrip("\n"))):
                lines[i] = f"{m[1]}{text}{m[2] or ''}\n"
                setattr(getattr(cfg, section), key, value)
                found = True
                break
        if not found:
            raise ConfigError(f"{dotted} not found in {cfg.path}")
    cfg.path.write_text("".join(lines), encoding="utf-8")
