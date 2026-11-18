"""Hazard fusion: turns vision readings into one Hazard for the arbiter, and logs every change.

    falling object CRITICAL        -> emergency full close (max speed, no profile)
    camera unhealthy (fail-safe)   -> full close
    dust SEVERE for severe_hold_s,
    or WARNING+ for sustained_s    -> full close
    dust WARNING                   -> log only; or coverage = severity if dust_partial_close
    falling object WARNING         -> log only

Every close latches (state.Arbiter) until a person retracts the shell.
"""
import logging

from .state import Hazard

log = logging.getLogger(__name__)


class Policy:
    def __init__(self, cfg):
        self.cfg = cfg
        self._last = None

    def decide(self, vision) -> Hazard | None:
        """vision: anything with ready, camera_problem(), detection, detection_times, dust."""
        h, note = self._decide(vision)
        key = (h.reason if h else "", note)
        if key != self._last:
            if h:
                log.warning("policy: %s -> coverage %.0f%%%s", h.reason, h.coverage * 100,
                            " (EMERGENCY)" if h.emergency else "")
            elif note:
                log.info("policy: %s (no action)", note)
            elif self._last and any(self._last):
                log.info("policy: clear")
            self._last = key
        return h

    def _decide(self, v):
        if not v.armed:
            return None, ""
        cam = self.cfg.camera
        d = v.detection if v.ready else None
        if d is not None and d.level == "critical":
            t_capture, t_detect = v.detection_times
            return Hazard(1.0, True, f"falling object, TTC {d.ttc:.2f} s", d.ttc,
                          t_capture, t_detect), ""
        problem = v.camera_problem()
        if problem and cam.fail_safe == "close":
            return Hazard(1.0, False, f"camera fail-safe: {problem}"), ""
        if not v.ready:
            return None, f"camera: {problem}" if problem else ""
        dc, dust = self.cfg.dust, v.dust
        if dust.level == "severe" and dust.duration >= dc.severe_hold_s:
            return Hazard(1.0, False, f"dust storm (severity {dust.severity:.2f})"), ""
        if dust.level != "none" and dust.duration >= dc.sustained_s:
            return Hazard(1.0, False, f"sustained dust (severity {dust.severity:.2f}, "
                                      f"{dust.duration:.0f} s)"), ""
        if dust.level != "none":
            if self.cfg.policy.dust_partial_close:
                cov = max(0.1, round(dust.severity, 1))
                return Hazard(cov, False, f"dust {dust.level}"), ""
            return None, f"dust {dust.level}"
        if problem:
            return None, f"camera: {problem}"
        if d is not None and d.level == "warning":
            return None, "object approaching"
        return None, ""
