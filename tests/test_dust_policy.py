"""Dust detector on synthetic frames, and policy priority."""
import math
from types import SimpleNamespace

import numpy as np
import pytest

cv2 = pytest.importorskip("cv2")

from shield import config  # noqa: E402
from shield.policy import Policy  # noqa: E402
from shield.vision import Detection, DustDetector, DustReading  # noqa: E402

W, H, DT = 320, 240, 1 / 58
rng = np.random.default_rng(1)
yy, xx = np.mgrid[0:H, 0:W]
# A sky with some structure (clouds / the test rig edge) so there is detail to lose.
SCENE = (150 + 40 * np.sin(xx / 9.0) * np.cos(yy / 13.0) + (xx > 250) * -60).astype(np.float32)


def frame(haze=0.0, specks=0):
    img = SCENE * (1 - haze) + 200 * haze + rng.normal(0, 1.5, (H, W))
    if specks:
        ys, xs = rng.integers(0, H - 2, specks), rng.integers(0, W - 2, specks)
        for y, x in zip(ys, xs):
            img[y:y + 2, x:x + 2] = 255 if (y + x) % 2 else 30
    return np.clip(img, 0, 255).astype(np.uint8)


@pytest.fixture
def cfg():
    c = config.load()
    c.dust.warmup_s = 0.5
    return c


def feed(det, frames, t=0.0):
    r = None
    for f in frames:
        r = det.update(f, t)
        t += DT
    return r, t


def test_clear_air_stays_clear(cfg):
    r, _ = feed(DustDetector(cfg), [frame() for _ in range(200)])
    assert r.level == "none" and r.severity < 0.1


def test_dust_storm_escalates(cfg):
    det = DustDetector(cfg)
    _, t = feed(det, [frame() for _ in range(60)])  # baseline
    r, t = feed(det, [frame(haze=0.35, specks=40) for _ in range(40)], t)
    assert r.level in ("warning", "severe")
    r, t = feed(det, [frame(haze=0.8, specks=150) for _ in range(150)], t)
    assert r.level == "severe" and r.duration > 1.0
    assert r.contrast > 0.5 and r.haze > 0.5 and r.particles > 0.5


def test_hysteresis_then_clears(cfg):
    det = DustDetector(cfg)
    _, t = feed(det, [frame() for _ in range(60)])
    _, t = feed(det, [frame(haze=0.8, specks=150) for _ in range(100)], t)
    r, t = feed(det, [frame() for _ in range(5)], t)
    assert r.level != "none"  # smoothed: one clear moment doesn't reset it
    r, _ = feed(det, [frame() for _ in range(200)], t)
    assert r.level == "none" and r.duration == 0.0


# ---- policy ----------------------------------------------------------------------------

def vision(detection=Detection(), dust=DustReading(), problem="", armed=True, ready=True):
    return SimpleNamespace(armed=armed, ready=ready, detection=detection, dust=dust,
                           detection_times=(1.0, 1.01), camera_problem=lambda: problem)


CRIT = Detection("critical", ttc=0.4, area=50)


def test_disarmed_never_acts():
    assert Policy(config.load()).decide(vision(CRIT, armed=False)) is None


def test_impact_is_emergency_and_beats_everything():
    h = Policy(config.load()).decide(vision(CRIT, DustReading("severe", 0.9, duration=5),
                                            problem="low fps (20)"))
    assert h.coverage == 1.0 and h.emergency and h.ttc_s == 0.4 and h.t_capture == 1.0


def test_camera_failure_fail_safe():
    cfg = config.load()
    h = Policy(cfg).decide(vision(problem="frozen frames"))
    assert h.coverage == 1.0 and not h.emergency and "fail-safe" in h.reason
    h = Policy(cfg).decide(vision(problem="no frames for 2.0 s", ready=False))
    assert h is not None  # even before the thread re-armed
    cfg.camera.fail_safe = "none"
    assert Policy(cfg).decide(vision(problem="frozen frames")) is None


def test_dust_levels():
    cfg = config.load()
    p = Policy(cfg)
    assert p.decide(vision(dust=DustReading("warning", 0.4, duration=2))) is None
    assert p.decide(vision(dust=DustReading("severe", 0.7, duration=0.5))) is None  # not yet
    assert p.decide(vision(dust=DustReading("severe", 0.7, duration=1.5))).coverage == 1.0
    assert p.decide(vision(dust=DustReading("warning", 0.4, duration=11))).coverage == 1.0
    cfg.policy.dust_partial_close = True
    assert Policy(cfg).decide(vision(dust=DustReading("warning", 0.42, duration=2))).coverage == 0.4


def test_approaching_object_warning_is_log_only():
    assert Policy(config.load()).decide(vision(Detection("warning", ttc=2.5))) is None
