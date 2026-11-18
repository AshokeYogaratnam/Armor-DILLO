"""Shield motion control.

`set_coverage(x, speed_limit)` is the one command. Everything else (open, close,
emergency_close, the override, hazards, the dashboard) goes through it. `tick()` is the
~200 Hz loop body that tracks it:

    trapezoidal profile -> p_ref, v_ref
    u = v_ref/free_speed + kp*e + ki*integral(e) + kd*(v_ref - v)      e = p_ref - position
    -> soft limits -> BTS7960 -> encoder -> stall / power-loss check

Positions are motor-shaft encoder counts, 0 at the open hard stop, increasing toward closed.
"""
import logging
import math
import threading
import time

from . import motor as m

log = logging.getLogger(__name__)


def profile_step(p: float, v: float, goal: float, vmax: float, accel: float, dt: float):
    """One step of a trapezoidal velocity profile toward goal. Returns the new (p, v)."""
    d = goal - p
    if abs(d) < 0.5 and abs(v) <= accel * dt:
        return goal, 0.0
    v_want = math.copysign(min(vmax, math.sqrt(2.0 * accel * abs(d))), d)
    dv = accel * dt
    v += max(-dv, min(dv, v_want - v))
    p_new = p + v * dt
    if (goal - p_new) * d <= 0:  # would cross the goal: arrive
        return goal, 0.0
    return p_new, v


class Controller:
    def __init__(self, cfg):
        self.cfg = cfg
        self.chip = m.open_gpiochip()
        self.motor = self.encoder = self.limits = None
        try:
            self.motor = m.Bts7960(self.chip, cfg)  # EN low first: coasting from here on
            self.encoder = m.Encoder(self.chip, cfg)
            self.limits = m.LimitSwitches(self.chip, cfg) if cfg.limits.enabled else None
        except Exception:
            self.shutdown()
            raise
        if not self.motor.hardware_pwm:
            log.warning("running on SOFTWARE PWM: duty capped at %.0f%%", self.motor.max_duty * 100)
        self.sign = 1 if cfg.motor.close_direction == "forward" else -1
        self.zero = None          # raw count at the open hard stop, set by homing
        self.mode = "idle"        # idle | homing | seeking | moving | holding
        self.fault = ""
        self.goal = None
        self.direction = 0        # of the current move: + = closing
        self.speed = cfg.motion.max_speed
        self.smooth = True
        self.p_ref = self.v_ref = self.integral = 0.0
        self.position = 0         # counts from the open stop (raw before homing)
        self.velocity = 0.0       # counts/s, filtered
        self.duty = 0.0           # applied, + = closing
        self.seek_result = None
        self.watchdog_trips = 0
        self.overruns = 0
        self.last_tick = time.monotonic()
        self._lock = threading.RLock()
        self._running = False
        self._closed = False
        self._stall_pos, self._stall_t, self._moved = 0, self.last_tick, False
        self._seek_t0 = 0.0
        self.set_travel(cfg.motion.travel_counts)
        threading.Thread(target=self._watchdog, name="watchdog", daemon=True).start()

    # ---- commands -------------------------------------------------------------------

    def set_coverage(self, x: float, speed_limit: float | None = None, smooth: bool = True) -> None:
        """THE command: move to coverage x (0 = open, 1 = closed) at up to speed_limit
        counts/s. smooth=False bypasses the profile: the full error goes straight to the
        PID, so the motor runs flat out until it nears the goal. Re-sending the same command
        is a no-op, so callers can send it every tick."""
        if self.zero is None or self.travel is None:
            raise RuntimeError("not homed/calibrated: run `shield home` / `shield calibrate`")
        goal = self.goal_for(x)
        speed = min(speed_limit or self.cfg.motion.max_speed, self.cfg.motion.max_speed)
        with self._lock:
            if self.fault or self.mode in ("homing", "seeking"):
                return
            self.speed = speed
            if goal == self.goal and smooth == self.smooth and self.mode in ("moving", "holding"):
                return
            # New command: restart the profile from where the shell actually is.
            self.p_ref, self.v_ref = float(self.position), self.velocity
            if not smooth:
                self.p_ref, self.v_ref = float(goal), 0.0
            self.goal, self.smooth, self.integral = goal, smooth, 0.0
            self.direction = 1 if goal >= self.position else -1  # + = closing
            self.mode = "moving"
            self._reset_stall()

    def goal_for(self, x: float) -> int:
        """Coverage 0..1 -> target position in counts (inside the soft limits)."""
        return round(self.lo + min(1.0, max(0.0, float(x))) * (self.hi - self.lo))

    def arrived(self, x: float) -> bool:
        return self.mode == "holding" and self.goal == self.goal_for(x)

    def open(self, speed_limit=None):
        self.set_coverage(0.0, speed_limit)

    def close(self, speed_limit=None) -> None:
        self.set_coverage(1.0, speed_limit)

    def emergency_close(self) -> None:
        self.set_coverage(1.0, self.cfg.motion.max_speed, smooth=False)

    def stop(self) -> None:
        """Stop where it is and hold (brake)."""
        with self._lock:
            if self.mode in ("moving", "holding"):
                self.mode, self.goal = "idle", None

    def home(self) -> None:
        """Drive gently into the open stop (or open limit switch) and zero there."""
        with self._lock:
            self.fault, self.zero, self.goal = "", None, None
            self._start_seek("homing")

    def seek_closed(self) -> None:
        """Drive gently into the closed stop; the position lands in seek_result (calibrate)."""
        with self._lock:
            self.seek_result = None
            self._start_seek("seeking")

    def clear_fault(self) -> None:
        with self._lock:
            if self.fault:
                log.info("fault cleared: %s", self.fault)
            self.fault, self.mode, self.goal = "", "idle", None

    def set_travel(self, travel) -> None:
        self.travel = travel
        margin = self.cfg.motion.soft_margin_counts
        self.lo, self.hi = (margin, travel - margin) if travel else (None, None)

    # ---- status ---------------------------------------------------------------------

    @property
    def coverage(self) -> float | None:
        if self.zero is None or self.travel is None:
            return None
        return min(1.0, max(0.0, (self.position - self.lo) / (self.hi - self.lo)))

    def status(self) -> dict:
        enc = self.encoder
        out = dict(mode=self.mode, fault=self.fault, homed=self.zero is not None,
                   position=self.position, coverage=self.coverage, goal=self.goal,
                   goal_coverage=None if self.goal is None or self.lo is None else
                   round(min(1.0, max(0.0, (self.goal - self.lo) / (self.hi - self.lo))), 3),
                   velocity=round(self.velocity), duty=round(self.duty, 3),
                   output_revs=round(enc.output_revs(self.position), 3),
                   encoder_errors=enc.errors, watchdog_trips=self.watchdog_trips,
                   overruns=self.overruns, hardware_pwm=self.motor.hardware_pwm)
        d = self.cfg.motion.spool_diameter_mm
        if d:
            out["travel_mm"] = round(out["output_revs"] * math.pi * d, 1)
        return out

    # ---- the loop -------------------------------------------------------------------

    def loop(self, on_tick=None, until=None) -> None:
        """Run tick() at control.loop_hz. on_tick() runs first each tick (the supervisor:
        override, hazards, requests). Returns when until() is true; brakes on the way out."""
        period = 1.0 / self.cfg.control.loop_hz
        next_t = self.last_tick = time.monotonic()
        self._running = True
        try:
            while not (until and until()):
                if on_tick:
                    on_tick()
                self.tick()
                next_t += period
                delay = next_t - time.monotonic()
                if delay > 0:
                    time.sleep(delay)
                else:
                    self.overruns += 1
                    next_t = time.monotonic()
        finally:
            self._running = False
            self.motor.brake()

    def tick(self) -> None:
        with self._lock:
            now = time.monotonic()
            dt = max(now - self.last_tick, 1e-4)
            self.last_tick = now
            pos = self.sign * self.encoder.count - (self.zero or 0)
            self.velocity += 0.3 * ((pos - self.position) / dt - self.velocity)
            self.position = pos
            u = self._limit(pos, self._control(pos, now, dt))
            self.duty = self.sign * self.motor.set_duty(self.sign * u)
            self._check_stall(pos, now)

    def _control(self, pos: int, now: float, dt: float) -> float:
        c = self.cfg
        if self.fault:
            return 0.0
        if self.mode in ("homing", "seeking"):
            if now - self._seek_t0 > c.homing.timeout_s:
                self._set_fault(f"{self.mode} timed out after {c.homing.timeout_s} s")
                return 0.0
            if self.limits:
                at_open, at_closed = self.limits.read()
                if at_open if self.mode == "homing" else at_closed:
                    self._seek_done(pos)
                    return 0.0
            return (-1 if self.mode == "homing" else 1) * c.homing.duty
        if self.mode == "holding":
            if abs(self.goal - pos) <= c.motion.drift_counts:
                return self._hold(pos)
            log.info("pushed %d counts off target while holding; correcting", self.goal - pos)
            self.p_ref, self.v_ref, self.mode = float(pos), 0.0, "moving"
            self._reset_stall()
        if self.mode != "moving":
            return 0.0

        if self.smooth:
            self.p_ref, self.v_ref = profile_step(self.p_ref, self.v_ref, self.goal, self.speed,
                                                  c.motion.accel, dt)
        remaining = self.goal - pos
        if self.p_ref == self.goal and self.v_ref == 0.0 and abs(remaining) <= c.motion.tolerance_counts:
            self.mode = "holding"
            return self._hold(pos)
        p = c.pid
        e = self.p_ref - pos
        self.integral = max(-p.integral_limit, min(p.integral_limit, self.integral + e * dt))
        u = (self.v_ref / c.motion.free_speed + p.kp * e + p.ki * self.integral
             + p.kd * (self.v_ref - self.velocity))
        if abs(u) < p.min_duty and abs(remaining) > c.motion.tolerance_counts:
            u = math.copysign(p.min_duty, u or remaining)
        return max(-1.0, min(1.0, u))

    def _hold(self, pos: int) -> float:
        mo = self.cfg.motion
        if mo.hold == "pwm":
            return max(-mo.hold_duty, min(mo.hold_duty, self.cfg.pid.kp * (self.goal - pos)))
        return 0.0  # brake; friction holds the shell

    def _limit(self, pos: int, u: float) -> float:
        """Never drive further past a soft limit or into a tripped limit switch."""
        if self.limits:
            at_open, at_closed = self.limits.read()
            if (u > 0 and at_closed and self.mode != "homing") or (u < 0 and at_open and self.mode != "seeking"):
                return 0.0
        if self.mode in ("homing", "seeking") or self.lo is None or self.zero is None:
            return u
        if (u > 0 and pos >= self.hi) or (u < 0 and pos <= self.lo):
            return 0.0
        return u

    def _check_stall(self, pos: int, now: float) -> None:
        s = self.cfg.stall
        if abs(self.duty) < s.min_duty:
            self._stall_pos, self._stall_t, self._moved = pos, now, False
            return
        if abs(pos - self._stall_pos) >= s.min_counts:
            self._stall_pos, self._stall_t = pos, now
            # "Really driven" = moving the commanded way at a decent fraction of what this duty
            # should give; a few counts of coasting left over from the last move don't count.
            if self.velocity * self.duty > 0 and \
                    abs(self.velocity) > 0.2 * self.cfg.motion.free_speed * abs(self.duty):
                self._moved = True
            return
        if now - self._stall_t < s.timeout_ms / 1000.0:
            return
        if self.mode in ("homing", "seeking"):
            self._seek_done(pos)
        elif self._moved:
            self._set_fault(f"stalled at position {pos} (obstruction or jam?)")
        else:
            self._set_fault(f"no encoder motion at {abs(self.duty):.0%} duty: motor power lost "
                            "(E-stop, fuse, main switch) or jammed")

    def _reset_stall(self) -> None:
        """Every new motion gets a fresh stall timer."""
        self._stall_pos, self._stall_t, self._moved = self.position, time.monotonic(), False

    def _start_seek(self, mode: str) -> None:
        self.mode, self._seek_t0 = mode, time.monotonic()
        self._reset_stall()

    def _seek_done(self, pos: int) -> None:
        self.motor.brake()
        if self.mode == "homing":
            self.zero = pos  # zero is None while homing, so pos is the raw count
            self.position, self.velocity = 0, 0.0
            log.info("homed at the open stop%s", "" if self._moved else
                     " (no motion seen: it was already there, or motor power is off)")
        else:
            self.seek_result = pos
            log.info("closed stop at %d counts", pos)
        self.mode = "idle"

    def _set_fault(self, reason: str) -> None:
        if not self.fault:
            log.error("FAULT: %s", reason)
        self.fault, self.mode, self.goal = reason, "idle", None
        self.motor.brake()

    def _watchdog(self) -> None:
        limit = self.cfg.control.watchdog_ms / 1000.0
        while not self._closed:
            time.sleep(limit / 4)
            late = time.monotonic() - self.last_tick
            if not (self._running and late > limit):
                continue
            self.watchdog_trips += 1
            log.error("WATCHDOG: control loop silent for %.0f ms; braking", late * 1000)
            if self._lock.acquire(timeout=0.005):
                try:
                    self.motor.brake()
                finally:
                    self._lock.release()
            else:  # the loop is stuck holding the lock (hung write?): force coast
                self.motor._emergency_coast()
            while self._running and not self._closed and time.monotonic() - self.last_tick > limit:
                time.sleep(limit / 4)
            if self._running:
                log.warning("control loop resumed")

    def shutdown(self) -> None:
        """Coast and release everything. Idempotent."""
        self._closed = True
        for res in (self.motor, self.encoder, self.limits):
            if res is not None:
                try:
                    res.close()
                except Exception:
                    log.exception("closing %r failed", res)
        self.motor = self.encoder = self.limits = None
        if self.chip is not None:
            m.lgpio.gpiochip_close(self.chip)
            self.chip = None
