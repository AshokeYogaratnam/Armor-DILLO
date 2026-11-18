"""`shield` command line. From VS Code use Run and Debug (.vscode/launch.json); in a terminal:

    shield check          bring-up, shell disconnected
    shield calibrate      measure travel + close time, write them to config.yaml
    shield home           find the open stop
    shield set 0.5        home, then move to 50 % coverage
    shield run            the autonomous controller (what the systemd service runs)
"""
import argparse
import logging
import logging.handlers
import os
import signal
import socket
import sys
import time

from . import config

log = logging.getLogger("shield")


def setup_logging(cfg, verbose: bool = False) -> None:
    root = logging.getLogger()
    root.setLevel(logging.DEBUG if verbose else cfg.logging.level)
    fmt = logging.Formatter("%(asctime)s.%(msecs)03d %(levelname)-7s %(name)s: %(message)s",
                            "%H:%M:%S")
    handlers = [logging.StreamHandler()]
    path = config.REPO / cfg.logging.file
    try:
        path.parent.mkdir(parents=True, exist_ok=True)
        handlers.append(logging.handlers.RotatingFileHandler(path, maxBytes=5_000_000,
                                                             backupCount=5))
    except OSError as exc:
        print(f"file logging disabled: {exc}", file=sys.stderr)
    for h in handlers:
        h.setFormatter(fmt)
        root.addHandler(h)


def sd_notify(msg: str) -> None:
    """Tell systemd we're alive (Type=notify + WatchdogSec). No-op outside systemd."""
    addr = os.environ.get("NOTIFY_SOCKET")
    if not addr:
        return
    if addr.startswith("@"):
        addr = "\0" + addr[1:]
    try:
        with socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM) as s:
            s.connect(addr)
            s.sendall(msg.encode())
    except OSError:
        pass


def need_travel(cfg) -> None:
    if not cfg.motion.travel_counts:
        sys.exit("motion.travel_counts is not set: run `shield calibrate` first")


def home(ctrl) -> bool:
    ctrl.home()
    ctrl.loop(until=lambda: ctrl.mode != "homing")
    if ctrl.fault:
        print(f"homing failed: {ctrl.fault}")
    return not ctrl.fault


def cmd_home(cfg, args) -> int:
    from .controller import Controller
    ctrl = Controller(cfg)
    try:
        return 0 if home(ctrl) else 1
    finally:
        ctrl.shutdown()


def cmd_set(cfg, args) -> int:
    from .controller import Controller
    need_travel(cfg)
    ctrl = Controller(cfg)
    try:
        if not home(ctrl):
            return 1
        ctrl.set_coverage(args.coverage, args.speed)
        ctrl.loop(until=lambda: ctrl.mode != "moving")
        s = ctrl.status()
        print(f"coverage {s['coverage']:.0%} at {s['position']} counts"
              + (f"  FAULT: {s['fault']}" if s["fault"] else ""))
        return 1 if s["fault"] else 0
    finally:
        ctrl.shutdown()


def cmd_run(cfg, args) -> int:
    from .controller import Controller
    from .override import Override
    from .policy import Policy
    from .state import CLOSED, OPEN, Arbiter, state_of
    from .vision import LatencyLog, Vision, ttc_threshold
    need_travel(cfg)
    ctrl = Controller(cfg)
    try:
        override = Override(ctrl.chip, cfg)
    except Exception:
        ctrl.shutdown()
        raise
    vision = Vision(cfg)  # own thread; camera failures are reported, never fatal
    latency = LatencyLog(config.REPO / "logs" / "latency.csv")
    log.info("impact TTC threshold %.2f s", ttc_threshold(cfg))
    policy = Policy(cfg)
    arbiter = Arbiter()
    hub = None
    if cfg.dashboard.enabled:
        from . import dashboard
        hub = dashboard.Hub()
        dashboard.start(cfg, hub, vision)  # own thread; if it dies, control carries on
    last = {"state": None, "notify": 0.0, "source": None, "publish": 0.0}
    shed = []  # pending dust-shed wiggle targets before a manual retract

    def supervise():
        now = time.monotonic()
        if now - last["notify"] > 0.5:
            sd_notify("WATCHDOG=1")
            last["notify"] = now
        ov = override.read()
        if ctrl.mode != "homing":
            was_dust = arbiter.latched is not None and "dust" in arbiter.latched.reason
            hazard = policy.decide(vision)
            if hub is not None:
                extra = hub.take_hazard()  # dashboard EMERGENCY CLOSE / external report
                if extra is not None and (hazard is None or not hazard.emergency):
                    hazard = extra
            cmd = arbiter.decide(ov, hazard=hazard, request=hub.request if hub else None,
                                 faulted=bool(ctrl.fault))
            if cmd.source == "hazard" and last["source"] != "hazard":
                log.warning("HAZARD: %s -> closing", arbiter.latched.reason)
                latency.start(arbiter.latched, now)
            last["source"] = cmd.source
            retracting = cmd.source == "manual" and cmd.coverage == 0.0
            if retracting and was_dust and arbiter.latched is None and cfg.policy.shed_cycles:
                a = cfg.policy.shed_amplitude
                shed[:] = [1.0 - a, 1.0] * cfg.policy.shed_cycles
                log.info("shedding dust: %d wiggles before opening", cfg.policy.shed_cycles)
            if not retracting:
                shed.clear()
            if cmd.clear_fault and ctrl.fault:
                ctrl.clear_fault()
            if ctrl.zero is None and not ctrl.fault:
                ctrl.home()  # never homed (e.g. homing faulted, then a manual press cleared it)
            elif shed:
                if ctrl.arrived(shed[0]):
                    shed.pop(0)
                if shed:
                    ctrl.set_coverage(shed[0])
                else:
                    ctrl.set_coverage(0.0)
            elif cmd.coverage is None:
                ctrl.stop()
            elif cmd.emergency:
                ctrl.emergency_close()
            else:
                ctrl.set_coverage(cmd.coverage)
            source = cmd.source
        else:
            source = "homing"
        state = state_of(ctrl, source)
        if state != last["state"]:
            log.info("state %s%s  (coverage %s, source %s)", state[0],
                     f" [{state[1]}]" if state[1] else "",
                     "?" if ctrl.coverage is None else f"{ctrl.coverage:.0%}", source)
            last["state"] = state
        if latency.pending and (state[0] == CLOSED or source != "hazard"):
            latency.finish(now)
        # Watch the sky only when the shield is fully open in AUTO.
        vision.arm(state[0] == OPEN and source in ("default", "request"))
        if hub is not None and now - last["publish"] >= 0.1:
            last["publish"] = now
            try:
                publish(state, source, ov)
            except Exception:  # a dashboard bug must never stop the control loop
                if not last.get("publish_failed"):
                    log.exception("dashboard publish failed (control continues)")
                last["publish_failed"] = True

    def publish(state, source, ov):
        d, dust = vision.detection, vision.dust
        t_capture, t_detect = vision.detection_times
        hub.publish(dict(
            state=state[0], reason=state[1], source=source, request=hub.request,
            latched=arbiter.latched.reason if arbiter.latched else "",
            motor=ctrl.status(),
            override=dict(manual=ov.manual, open=ov.open_pressed, close=ov.close_pressed),
            camera=dict(fps=round(vision.fps, 1), target_fps=cfg.camera.fps,
                        problem=vision.camera_problem(), error=vision.error,
                        dropped=vision.health.dropped, process_ms=round(vision.process_ms, 1)),
            armed=vision.armed, ready=vision.ready,
            impact=dict(level=d.level, ttc=round(d.ttc, 3), area=d.area, points=d.points),
            dust=dict(level=dust.level, severity=dust.severity, contrast=dust.contrast,
                      haze=dust.haze, particles=dust.particles, duration=round(dust.duration, 1)),
            latency=dict(capture_to_detect_ms=round((t_detect - t_capture) * 1000, 1)
                         if vision.ready and t_detect else None,
                         ttc_threshold=round(ttc_threshold(cfg), 2),
                         close_time=cfg.motion.close_time_s, last=latency.last),
            external_hazards=cfg.dashboard.external_hazards))

    try:
        sd_notify("READY=1")
        log.info("shield run: homing to the open stop")
        ctrl.home()
        ctrl.loop(on_tick=supervise)
    finally:
        vision.stop()
        override.close()
        ctrl.shutdown()
    return 0


def cmd_calibrate(cfg, args) -> int:
    from .check import calibrate
    return calibrate(cfg)


def cmd_check(cfg, args) -> int:
    from . import check
    if not 0 < args.duty <= 0.5:
        sys.exit("--duty must be in (0, 0.5]")
    return check.run(cfg, motor=not args.no_motor, camera=not args.no_camera, yes=args.yes,
                     duty=args.duty)


def main(argv=None) -> int:
    p = argparse.ArgumentParser(prog="shield", description="Armor-DILLO shield controller")
    p.add_argument("--config", help="config file (default: $SHIELD_CONFIG or repo config.yaml)")
    p.add_argument("-v", "--verbose", action="store_true")
    sub = p.add_subparsers(dest="cmd", required=True)
    c = sub.add_parser("check", help="first-power-up bring-up (shell disconnected)")
    c.add_argument("--no-motor", action="store_true", help="skip the motor tests")
    c.add_argument("--no-camera", action="store_true")
    c.add_argument("--yes", action="store_true", help="skip the motor confirmation prompt")
    c.add_argument("--duty", type=float, default=0.25, help="spin-test duty, max 0.5")
    c.set_defaults(func=cmd_check)
    sub.add_parser("calibrate", help="measure travel + close time (shell attached)").set_defaults(
        func=cmd_calibrate)
    sub.add_parser("home", help="find the open stop and zero").set_defaults(func=cmd_home)
    s = sub.add_parser("set", help="home, then move to a coverage 0..1")
    s.add_argument("coverage", type=float)
    s.add_argument("--speed", type=float, help="speed limit, counts/s")
    s.set_defaults(func=cmd_set)
    sub.add_parser("run", help="autonomous controller").set_defaults(func=cmd_run)
    args = p.parse_args(argv)

    try:
        cfg = config.load(args.config)
    except config.ConfigError as exc:
        print(f"config error: {exc}", file=sys.stderr)
        return 2
    setup_logging(cfg, args.verbose)
    # SIGTERM/SIGHUP (systemd stop, VS Code stop) -> normal exit, so every finally runs and
    # the motor coasts. SIGKILL can't be caught: safe_stop.sh (ExecStopPost) covers that.
    for sig in (signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, lambda signum, frame: sys.exit(128 + signum))
    try:
        return args.func(cfg, args)
    except KeyboardInterrupt:
        return 130


if __name__ == "__main__":
    sys.exit(main())
