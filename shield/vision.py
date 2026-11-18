"""Camera, falling-object (impact) detection, camera health and latency logging.

Everything runs on one background thread. The control loop only reads `Vision.hazard()`,
which never blocks. The camera is the ONLY hazard sensor, so a sick camera while the shield
is open triggers the fail-safe (default: close).

Detection only runs while the shield is fully OPEN in AUTO ("armed"). The camera sits
under the shell, so when the shield moves or is closed the view is the shell, not the sky.

Pi 5 notes: the Pi 5 has 22-pin mini CSI connectors, so a 15-pin camera needs the 22-to-15
adapter cable. picamera2 and OpenCV must come from apt.
"""
import collections
import csv
import logging
import math
import threading
import time
from dataclasses import dataclass
from pathlib import Path

import numpy as np

try:
    import cv2
except ImportError:  # check.py can still test the camera without OpenCV
    cv2 = None

from .state import Hazard

log = logging.getLogger(__name__)


class Camera:
    """Grayscale frames: the Y plane of YUV420, so no colour conversion is needed."""

    def __init__(self, cfg):
        from picamera2 import Picamera2
        c = cfg.camera
        info = Picamera2.global_camera_info()
        if not info:
            raise RuntimeError("no camera detected: check the 22-pin ribbon cable, then run "
                               "`rpicam-hello --list-cameras`")
        self.model = info[0].get("Model", "?")
        self.timeout = c.read_timeout_s
        self.cam = Picamera2()
        frame_us = int(1e6 / c.fps)
        config = self.cam.create_video_configuration(
            main={"size": (c.width, c.height), "format": "YUV420"},
            sensor={"output_size": tuple(c.sensor_size)}, buffer_count=4, queue=False,
            controls={"FrameDurationLimits": (frame_us, frame_us)})
        self.cam.configure(config)
        self.width, self.height = config["main"]["size"]
        self.cam.start()

    def read(self):
        """Next frame as (gray uint8 (h, w), t_capture). t_capture is the monotonic time
        the sensor exposed it (from libcamera's SensorTimestamp), for latency measurement.
        Raises on timeout."""
        request = self.cam.capture_request(wait=False).get_result(timeout=self.timeout)
        try:
            yuv = request.make_array("main")
            md = request.get_metadata()
        finally:
            request.release()
        t_rx = time.monotonic()
        t_capture = t_rx
        ts = md.get("SensorTimestamp")  # CLOCK_BOOTTIME ns
        if ts:
            age = (time.clock_gettime_ns(time.CLOCK_BOOTTIME) - ts) / 1e9
            if 0.0 <= age < 1.0:
                t_capture = t_rx - age
        return yuv[:self.height, :self.width].copy(), t_capture

    def close(self) -> None:
        try:
            self.cam.stop()
        finally:
            self.cam.close()


# ---- impact detection -----------------------------------------------------------------

@dataclass(frozen=True)
class Detection:
    level: str = "none"     # none | warning | critical
    ttc: float = math.inf   # seconds to contact (constant-velocity estimate)
    area: int = 0           # blob area, px
    points: int = 0         # LK points used (0 = blob-area fallback)
    box: tuple | None = None


def ttc_threshold(cfg) -> float:
    i = cfg.impact
    if i.ttc_threshold_s:
        return i.ttc_threshold_s
    if cfg.motion.close_time_s:
        return cfg.motion.close_time_s + i.ttc_margin_s
    return i.ttc_fallback_s


class ImpactDetector:
    """Pure frame logic: feed it (gray, t) frames, get a Detection back.

    1. Background subtraction (stationary camera) gives foreground blobs; frame differencing
       keeps only blobs that are actually moving.
    2. Sparse Lucas-Kanade flow tracks corners on the biggest moving blob from the previous
       frame. Their spread about their centroid grows by k: expansion rate = ln(k) / dt.
       (Too few corners, e.g. a plain ball: use blob-area growth, ln(A1/A0) / 2.)
    3. TTC = 1 / mean expansion rate over the last few frames. Under the threshold for
       confirm_frames in a row = CRITICAL; under twice the threshold = WARNING.

    TTC assumes constant approach speed; a ball accelerating under gravity arrives a bit
    sooner, which ttc_margin_s has to cover.
    """

    def __init__(self, cfg):
        self.c = cfg.impact
        self.threshold = ttc_threshold(cfg)
        self._kernel = np.ones((3, 3), np.uint8)
        self.reset(0.0)

    def reset(self, t: float) -> None:
        self.bg = None
        self.prev = None
        self.prev_t = t
        self.t0 = t
        self._lost()

    def _lost(self) -> None:
        self.prev_box, self.prev_area = None, 0
        self.window = collections.deque(maxlen=self.c.window_frames)
        self.hits = 0

    def update(self, gray, t: float) -> Detection:
        c = self.c
        if self.bg is None:
            self.bg, self.prev, self.prev_t, self.t0 = gray.astype(np.float32), gray, t, t
            return Detection()
        dt = t - self.prev_t
        if dt <= 0:
            return Detection()
        if t - self.t0 < c.warmup_s:  # learn the background quickly
            cv2.accumulateWeighted(gray, self.bg, 0.2)
            self.prev, self.prev_t = gray, t
            return Detection()

        fg = (cv2.absdiff(gray, cv2.convertScaleAbs(self.bg)) > c.diff_threshold).astype(np.uint8)
        fg = cv2.morphologyEx(fg, cv2.MORPH_OPEN, self._kernel)
        moving = cv2.absdiff(gray, self.prev) > c.diff_threshold
        n, _, stats, _ = cv2.connectedComponentsWithStats(fg, connectivity=8)
        # Frame differencing starts a track; after that, overlap with last frame's box keeps
        # it. A distant ball heading straight at the camera barely changes per frame.
        best = None
        for x, y, w, h, area in stats[1:]:
            if area < c.min_area_px:
                continue
            if moving[y:y + h, x:x + w].any() or self._overlaps_prev(x, y, w, h):
                if best is None or area > best[4]:
                    best = (x, y, w, h, area)

        det = Detection()
        if best is None:
            self._lost()
        else:
            box, area = best[:4], int(best[4])
            incr, points = self._expansion(gray, area)
            if incr is not None:
                self.window.append((dt, incr))
            ttc = math.inf
            if len(self.window) >= 2:
                rate = sum(i for _, i in self.window) / sum(d for d, _ in self.window)
                if rate > 1e-6:
                    ttc = 1.0 / rate
            self.hits = self.hits + 1 if ttc < self.threshold else 0
            level = ("critical" if self.hits >= c.confirm_frames else
                     "warning" if ttc < 2 * self.threshold else "none")
            det = Detection(level, ttc, area, points, tuple(int(v) for v in box))
            self.prev_box, self.prev_area = box, area

        # Update the background everywhere except the foreground, so objects don't fade in.
        cv2.accumulateWeighted(gray, self.bg, c.background_rate, mask=1 - fg)
        self.prev, self.prev_t = gray, t
        return det

    def _overlaps_prev(self, x, y, w, h) -> bool:
        if self.prev_box is None:
            return False
        px, py, pw, ph = self.prev_box
        return x < px + pw and px < x + w and y < py + ph and py < y + h

    def _expansion(self, gray, area: int):
        """Per-frame log scale change of the tracked blob, and how many LK points gave it."""
        if self.prev_box is None:
            return None, 0
        x, y, w, h = self.prev_box
        pad = 4
        mask = np.zeros_like(self.prev)
        mask[max(0, y - pad):y + h + pad, max(0, x - pad):x + w + pad] = 255
        p0 = cv2.goodFeaturesToTrack(self.prev, maxCorners=self.c.max_features,
                                     qualityLevel=0.01, minDistance=3, mask=mask)
        if p0 is not None and len(p0) >= self.c.min_points:
            p1, status, _ = cv2.calcOpticalFlowPyrLK(self.prev, gray, p0, None,
                                                     winSize=(15, 15), maxLevel=2)
            ok = status.ravel() == 1
            a, b = p0[ok].reshape(-1, 2), p1[ok].reshape(-1, 2)
            if len(a) >= self.c.min_points:
                d0 = np.linalg.norm(a - a.mean(0), axis=1).mean()
                d1 = np.linalg.norm(b - b.mean(0), axis=1).mean()
                if d0 > 1.0 and d1 > 0:
                    return math.log(d1 / d0), len(a)
        if self.prev_area:
            return 0.5 * math.log(area / self.prev_area), 0
        return None, 0


# ---- dust ------------------------------------------------------------------------------

@dataclass(frozen=True)
class DustReading:
    level: str = "none"     # none | warning | severe
    severity: float = 0.0   # smoothed 0..1
    contrast: float = 0.0   # the three scores, 0..1
    haze: float = 0.0
    particles: float = 0.0
    duration: float = 0.0   # seconds at warning or worse


def _ramp(x: float, lo_hi) -> float:
    lo, hi = lo_hi
    return min(1.0, max(0.0, (x - lo) / (hi - lo)))


class DustDetector:
    """Pure frame logic, at half resolution. See the dust section of config.yaml."""

    def __init__(self, cfg):
        self.c = cfg.dust
        self._kernel = np.ones((7, 7), np.uint8)
        self.reset(0.0)

    def reset(self, t: float) -> None:
        self.base_lap = self.base_dark = None
        self.samples = []
        self.prev = None
        self.t0 = t
        self.severity = 0.0
        self.level = "none"
        self.since = None

    def update(self, gray, t: float) -> DustReading:
        c = self.c
        small = cv2.resize(gray, (gray.shape[1] // 2, gray.shape[0] // 2),
                           interpolation=cv2.INTER_AREA)
        # Median first, so flying specks (scored separately) don't count as scene detail.
        lap = float(cv2.Laplacian(cv2.medianBlur(small, 5), cv2.CV_32F).var())
        dark = float(cv2.erode(small, self._kernel).mean())  # grayscale dark channel
        specks = 0
        if self.prev is not None:
            moving = (cv2.absdiff(small, self.prev) > c.diff_threshold).astype(np.uint8)
            _, _, stats, _ = cv2.connectedComponentsWithStats(moving, connectivity=8)
            specks = int((stats[1:, 4] <= c.max_particle_px).sum())
        self.prev = small
        if t - self.t0 < c.warmup_s:
            self.samples.append((lap, dark))
            return DustReading()
        if self.base_lap is None:
            laps, darks = zip(*self.samples) if self.samples else ([lap], [dark])
            self.base_lap = max(float(np.median(laps)), c.min_contrast)
            self.base_dark = float(np.median(darks))

        s_c = _ramp(1.0 - lap / self.base_lap, c.contrast_drop)
        s_h = _ramp((dark - self.base_dark) / max(255.0 - self.base_dark, 1.0), c.haze_rise)
        s_p = _ramp(specks, c.particles)
        wc, wh, wp = c.weights
        raw = (wc * s_c + wh * s_h + wp * s_p) / (wc + wh + wp)
        self.severity += c.smoothing * (raw - self.severity)

        s, lvl = self.severity, self.level  # hysteresis between levels
        if lvl == "none" and s >= c.warn_on:
            lvl = "warning"
        if lvl != "none":
            if s < c.warn_off:
                lvl = "none"
            elif s >= c.severe_on:
                lvl = "severe"
            elif lvl == "severe" and s < c.severe_off:
                lvl = "warning"
        self.level = lvl
        if lvl == "none":
            self.since = None
            r = c.baseline_rate  # follow slow sky changes only while the air is clear
            self.base_lap = max(self.base_lap + r * (lap - self.base_lap), c.min_contrast)
            self.base_dark += r * (dark - self.base_dark)
        elif self.since is None:
            self.since = t
        return DustReading(lvl, round(s, 3), round(s_c, 3), round(s_h, 3), round(s_p, 3),
                           0.0 if self.since is None else t - self.since)


# ---- camera health ----------------------------------------------------------------------

class CameraHealth:
    """Frozen, black, saturated, low-FPS and dropped frames. update() returns '' if healthy."""

    def __init__(self, cfg):
        self.c = cfg.camera
        self.reset(0.0)

    def reset(self, t: float) -> None:
        self.times = collections.deque()
        self.prev = None
        self.same = 0
        self.dropped = 0
        self.bad_since = None
        self.t0 = t

    @property
    def fps(self) -> float:
        if len(self.times) < 2:
            return 0.0
        return (len(self.times) - 1) / max(self.times[-1] - self.times[0], 1e-6)

    def update(self, gray, t: float) -> str:
        c = self.c
        if self.times and t - self.times[-1] > 1.8 / c.fps:
            self.dropped += 1
        self.times.append(t)
        while self.times and t - self.times[0] > 2.0:
            self.times.popleft()
        self.same = self.same + 1 if self.prev is not None and np.array_equal(gray, self.prev) else 0
        self.prev = gray
        if self.same >= c.frozen_frames:
            return "frozen frames"
        mean = float(gray.mean())
        bad = "black frames" if mean < c.dark_level else "saturated frames" if mean > c.bright_level else ""
        if bad:
            self.bad_since = self.bad_since or t
            if t - self.bad_since >= c.bad_hold_s:
                return bad
        else:
            self.bad_since = None
        if t - self.t0 > 2.0 and self.fps < c.min_fps:
            return f"low fps ({self.fps:.0f})"
        return ""


# ---- the vision thread -----------------------------------------------------------------

class Vision:
    def __init__(self, cfg):
        if cv2 is None:
            raise RuntimeError("OpenCV missing: sudo apt install python3-opencv")
        self.cfg = cfg
        self.detector = ImpactDetector(cfg)
        self.dust_detector = DustDetector(cfg)
        self.health = CameraHealth(cfg)
        self.armed = False
        self.detection = Detection()
        self.dust = DustReading()
        self.detection_times = (0.0, 0.0)  # (t_capture, t_detect) of self.detection
        self.process_ms = 0.0               # detection cost per frame
        self.fps = 0.0                      # measured camera rate, armed or not
        self._arrivals = collections.deque(maxlen=60)
        self.problem = ""                   # current camera health problem
        self.error = ""                     # last camera open/read error
        self.frame = None                   # latest gray frame (dashboard preview)
        self.last_frame = time.monotonic()
        self._rearm = False
        self._stop = False
        self._thread = threading.Thread(target=self._run, name="vision", daemon=True)
        self._thread.start()

    def arm(self, on: bool) -> None:
        """Detect only while the shield is fully open in AUTO. Re-arming starts fresh."""
        if on and not self.armed:
            self._rearm = True
            self.detection, self.dust = Detection(), DustReading()
        self.armed = on

    @property
    def ready(self) -> bool:
        """Armed and the thread has started fresh on a new frame (readings are current)."""
        return self.armed and not self._rearm

    def camera_problem(self) -> str:
        """'' if the camera is healthy. Never blocks."""
        silent = time.monotonic() - self.last_frame
        if silent > 2 * self.cfg.camera.read_timeout_s:
            return f"no frames for {silent:.1f} s ({self.error or 'camera stalled'})"
        return "" if self._rearm else self.problem

    def stop(self) -> None:
        self._stop = True
        self._thread.join(timeout=2)

    def _run(self) -> None:
        cam = None
        while not self._stop:
            if cam is None:
                try:
                    cam = Camera(self.cfg)
                    log.info("camera started: %s %dx%d", cam.model, cam.width, cam.height)
                    self.error = ""
                except Exception as exc:
                    self._camera_error(f"open failed: {exc}")
                    time.sleep(1.0)
                    continue
            try:
                gray, t_capture = cam.read()
            except Exception as exc:
                self._camera_error(f"read failed: {exc!r}")
                try:
                    cam.close()
                except Exception:
                    pass
                cam = None
                time.sleep(0.5)
                continue
            self.frame, self.last_frame = gray, time.monotonic()
            self._arrivals.append(t_capture)
            if len(self._arrivals) > 1:
                span = self._arrivals[-1] - self._arrivals[0]
                self.fps = (len(self._arrivals) - 1) / span if span > 0 else 0.0
            if not self.armed:
                continue
            if self._rearm:
                for part in (self.detector, self.dust_detector, self.health):
                    part.reset(t_capture)
                self.problem, self._rearm = "", False
            t0 = time.monotonic()
            try:
                self.problem = self.health.update(gray, t_capture)
                det = self.detector.update(gray, t_capture)
                dust = self.dust_detector.update(gray, t_capture)
            except Exception:
                log.exception("vision processing failed")
                continue
            t_detect = time.monotonic()
            self.process_ms = (t_detect - t0) * 1000
            self.detection_times = (t_capture, t_detect)
            self.detection, self.dust = det, dust
        if cam is not None:
            cam.close()

    def _camera_error(self, msg: str) -> None:
        if msg != self.error:
            log.error("camera %s (retrying)", msg)
        self.error = msg


# ---- latency log --------------------------------------------------------------------------

class LatencyLog:
    """One row per hazard close in logs/latency.csv:
    capture -> detect -> command -> closed, against the object's TTC.
    margin_s = ttc - (capture->command + close time); negative = closed too late."""

    FIELDS = ["time", "reason", "ttc_s", "capture_to_detect_ms", "detect_to_command_ms",
              "close_s", "margin_s"]

    def __init__(self, path: Path):
        self.path = path
        self.pending = None
        self.last = None  # the most recent row, for the dashboard

    def start(self, hazard: Hazard, t_command: float) -> None:
        self.pending = (hazard, t_command)

    def finish(self, t_closed: float) -> None:
        if self.pending is None:
            return
        h, t_cmd = self.pending
        self.pending = None
        close_s = t_closed - t_cmd
        row = dict(time=time.strftime("%Y-%m-%d %H:%M:%S"), reason=h.reason, ttc_s="",
                   capture_to_detect_ms="", detect_to_command_ms="", close_s=f"{close_s:.3f}",
                   margin_s="")
        if h.t_capture is not None:
            row["capture_to_detect_ms"] = f"{(h.t_detect - h.t_capture) * 1000:.1f}"
            row["detect_to_command_ms"] = f"{(t_cmd - h.t_detect) * 1000:.1f}"
        if h.ttc_s is not None and h.t_capture is not None:
            margin = h.ttc_s - (t_cmd - h.t_capture) - close_s
            row.update(ttc_s=f"{h.ttc_s:.3f}", margin_s=f"{margin:.3f}")
        self.last = row
        log.warning("hazard close: %s | capture->detect %s ms, detect->command %s ms, "
                    "close %.2f s, margin %s s", h.reason, row["capture_to_detect_ms"] or "-",
                    row["detect_to_command_ms"] or "-", close_s, row["margin_s"] or "-")
        try:
            new = not self.path.exists()
            self.path.parent.mkdir(parents=True, exist_ok=True)
            with open(self.path, "a", newline="") as f:
                w = csv.DictWriter(f, self.FIELDS)
                if new:
                    w.writeheader()
                w.writerow(row)
        except OSError:
            log.exception("latency log write failed")
