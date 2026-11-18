"""Impact TTC and camera health on synthetic frames (stationary camera, sky background)."""
import math

import numpy as np
import pytest

cv2 = pytest.importorskip("cv2")

from shield import config  # noqa: E402
from shield.vision import CameraHealth, ImpactDetector  # noqa: E402

W, H, DT = 320, 240, 1 / 58
_rng = np.random.default_rng(0)
SKY = np.clip(150 + np.linspace(-20, 20, W)[None, :] + np.zeros((H, 1)), 0, 255).astype(np.uint8)


def frame(r=0.0, cx=W / 2, cy=H / 2, textured=True, r0=10.0):
    """Sky plus a disc of radius r. Texture scales with the disc, like a real approaching ball."""
    img = SKY.astype(np.int16) + _rng.integers(-3, 4, (H, W))
    if r > 0:
        yy, xx = np.mgrid[0:H, 0:W]
        inside = np.hypot(xx - cx, yy - cy) < r
        if textured:
            u, v = (xx - cx) * r0 / r, (yy - cy) * r0 / r
            disc = np.where((np.floor(u / 3) + np.floor(v / 3)) % 2 == 0, 40, 100)
        else:
            disc = np.full((H, W), 50)
        img = np.where(inside, disc, img)
    return np.clip(img, 0, 255).astype(np.uint8)


@pytest.fixture
def cfg():
    c = config.load()
    c.impact.warmup_s = 0.1
    c.impact.ttc_threshold_s = 1.0
    return c


def run(det, frames):
    t, out = 0.0, None
    for f in [frame() for _ in range(12)] + frames:  # background first
        out = det.update(f, t)
        t += DT
    return out


@pytest.mark.parametrize("textured,tol", [(True, 0.25), (False, 0.35)])
def test_looming_object_ttc(cfg, textured, tol):
    k = 1.08  # scale growth per frame
    true_ttc = DT / math.log(k)
    d = run(ImpactDetector(cfg), [frame(10 * k ** i, textured=textured) for i in range(8)])
    assert d.level == "critical"
    assert d.ttc == pytest.approx(true_ttc, rel=tol)
    if textured:
        assert d.points >= cfg.impact.min_points  # LK flow, not the area fallback


def test_crossing_object_is_not_an_impact(cfg):
    d = run(ImpactDetector(cfg), [frame(12, cx=60 + 4 * i) for i in range(10)])
    assert d.area > 0 and d.level != "critical" and d.ttc > 2.0


def test_receding_object_is_not_an_impact(cfg):
    d = run(ImpactDetector(cfg), [frame(30 / 1.08 ** i) for i in range(8)])
    assert d.level == "none" and d.ttc == math.inf


def test_empty_sky(cfg):
    d = run(ImpactDetector(cfg), [frame() for _ in range(10)])
    assert d.level == "none" and d.area == 0


def test_health_frozen_and_black(cfg):
    h = CameraHealth(cfg)
    f = frame()
    results = [h.update(f, i * DT) for i in range(20)]
    assert results[-1] == "frozen frames"
    h.reset(0.0)
    black = np.zeros((H, W), np.uint8)
    results = [h.update(black + (i % 2), i * DT) for i in range(60)]  # 1 s of near-black
    assert results[-1] == "black frames"


def test_health_ok_and_low_fps(cfg):
    h = CameraHealth(cfg)
    assert all(h.update(frame(), i * DT) == "" for i in range(200))
    h.reset(0.0)
    results = [h.update(frame(), i * 0.05) for i in range(80)]  # 20 fps
    assert results[-1].startswith("low fps")
