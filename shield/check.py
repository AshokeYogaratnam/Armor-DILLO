"""`shield check`: first-power-up bring-up. Run it with the shell DISCONNECTED from the motor.

  platform     Pi model, Python
  boot config  pwm-2chan overlay + camera lines in /boot/firmware/config.txt
  pinmux       GPIO18/19 muxed to PWM, on the RP1 channels we expect
  gpio         chip found, EN pins claimed low, PWM backend, encoder, override inputs, limits
  enable gate  with EN low, PWM on each channel must NOT move the motor     (asks first)
  motor spin   short low-duty spin each way: counts, direction, encoder sign (asks first)
  camera       detected, frame rate, exposure sanity
"""
import platform
import re
import shutil
import subprocess
import sys
import time
from pathlib import Path

from . import motor as m

counts = {"PASS": 0, "WARN": 0, "FAIL": 0, "SKIP": 0}


def report(status: str, name: str, detail: str) -> None:
    counts[status] += 1
    print(f"[{status}] {name}: {detail}")


def pinctrl(pins) -> dict[int, str]:
    exe = shutil.which("pinctrl")
    if not exe:
        return {}
    out = subprocess.run([exe, "get", ",".join(map(str, pins))], capture_output=True,
                         text=True, timeout=2).stdout
    return {int(line.split(":")[0]): line.strip() for line in out.splitlines()
            if line.split(":")[0].strip().isdigit()}


def check_platform() -> None:
    report("PASS" if sys.version_info >= (3, 11) else "FAIL", "python", platform.python_version())
    model = Path("/proc/device-tree/model")
    name = model.read_text(errors="replace").strip("\x00\n ") if model.exists() else "unknown"
    report("PASS" if "Raspberry Pi 5" in name else "WARN", "platform", name)


def check_boot_config(cfg) -> None:
    path = next((p for p in (Path("/boot/firmware/config.txt"), Path("/boot/config.txt"))
                 if p.exists()), None)
    if path is None:
        report("FAIL", "boot config", "config.txt not found")
        return
    text = path.read_text(errors="replace")
    overlay = re.search(r"^\s*dtoverlay=pwm-2chan(.*)$", text, re.M)
    want = {cfg.pins.rpwm, cfg.pins.lpwm}
    if not overlay:
        report("FAIL", "pwm overlay", f"missing in {path}. Add under [all]: "
               "dtoverlay=pwm-2chan,pin=18,func=2,pin2=19,func2=2 and reboot")
    else:
        params = dict(kv.split("=", 1) for kv in overlay.group(1).split(",") if "=" in kv)
        got = {int(params.get("pin", 12)), int(params.get("pin2", 13))}
        report("PASS" if got == want else "FAIL", "pwm overlay",
               overlay.group(0).strip() + ("" if got == want else
               f" -> pins {sorted(got)}, config expects {sorted(want)} (default is 12/13)"))
    cam = re.search(r"^\s*(camera_auto_detect=1|dtoverlay=ov5647.*)$", text, re.M)
    report("PASS" if cam else "WARN", "camera config",
           cam.group(1) if cam else "no camera_auto_detect=1 or dtoverlay=ov5647")


def check_pinmux(cfg) -> None:
    lines = pinctrl([cfg.pins.rpwm, cfg.pins.lpwm])
    if not lines:
        report("WARN", "pinmux", "pinctrl not available; cannot verify pin functions")
        return
    for pin, name in ((cfg.pins.rpwm, "RPWM"), (cfg.pins.lpwm, "LPWM")):
        line = lines.get(pin, "?")
        hit = re.search(r"PWM0_CHAN(\d)", line)
        expected = m.RP1_PWM_CHANNEL.get(pin)
        if hit and int(hit.group(1)) == expected:
            report("PASS", "pinmux", f"{name} GPIO{pin} -> PWM0 channel {expected}")
        else:
            report("FAIL", "pinmux", f"{name} GPIO{pin} not on PWM0 channel {expected}: '{line}'")


def check_inputs(chip: int, cfg) -> None:
    import lgpio
    p = cfg.pins
    states = {}
    for name, pin in (("mode switch", p.mode_switch), ("OPEN", p.btn_open), ("CLOSE", p.btn_close)):
        m.claim(lgpio.gpio_claim_input, chip, pin, lgpio.SET_PULL_UP)
        states[name] = lgpio.gpio_read(chip, pin) == 0  # active-low
        lgpio.gpio_free(chip, pin)
    report("PASS", "override", f"mode={'MANUAL' if states['mode switch'] else 'AUTO'} "
           f"OPEN={'pressed' if states['OPEN'] else 'released'} "
           f"CLOSE={'pressed' if states['CLOSE'] else 'released'}")
    if states["OPEN"] or states["CLOSE"]:
        report("WARN", "override", "a button reads pressed at idle: check wiring (to GND when pressed)")
    if cfg.limits.enabled:
        limits = m.LimitSwitches(chip, cfg)
        at_open, at_closed = limits.read()
        limits.close()
        report("WARN" if at_open and at_closed else "PASS", "limits",
               f"open={at_open} closed={at_closed}")


def check_enable_gate(drv: m.Bts7960, enc: m.Encoder, cfg, duty: float) -> None:
    """EN low: PWM must not move the motor. On hardware PWM also drive 100% and confirm the
    right pin goes high, which proves the channel <-> pin mapping."""
    for pwm, pin, name in ((drv.rpwm, cfg.pins.rpwm, "RPWM"), (drv.lpwm, cfg.pins.lpwm, "LPWM")):
        drv.coast()
        c0 = enc.count
        level = None
        try:
            pwm.set(duty)  # raw channel write with EN low, bypassing the driver on purpose
            time.sleep(0.15)
            if not enc.count - c0 and drv.hardware_pwm:
                pwm.set(1.0)
                level = pinctrl([pin]).get(pin, "")
        finally:
            pwm.set(0.0)
            drv.coast()
        moved = enc.count - c0
        if moved:
            report("FAIL", "enable gate", f"motor moved {moved} counts with EN LOW and {name} on: "
                   "EN is not gating the driver (tied high?). Fix before anything else!")
        elif level is None or re.search(r"\|\s*hi", level):
            report("PASS", "enable gate", f"{name}: EN low keeps the driver off"
                   + (f"; GPIO{pin} drives high" if level else ""))
        else:
            report("FAIL", "enable gate", f"{name} at 100% but GPIO{pin} not high ('{level}'): "
                   "PWM channel mapping is wrong")


def spin(drv: m.Bts7960, enc: m.Encoder, duty: float, seconds: float = 0.4) -> int:
    c0, t0 = enc.count, time.monotonic()
    try:
        while time.monotonic() - t0 < seconds:
            drv.set_duty(duty)
            if time.monotonic() - t0 > 0.25 and enc.count == c0:
                break  # not moving: don't keep pushing
            time.sleep(0.005)
    finally:
        drv.brake()
    time.sleep(0.3)
    drv.coast()
    return enc.count - c0


def check_motor(drv: m.Bts7960, enc: m.Encoder, cfg, duty: float) -> None:
    fwd = spin(drv, enc, duty)
    rev = spin(drv, enc, -duty)
    for name, n in (("forward/RPWM", fwd), ("reverse/LPWM", rev)):
        report("PASS" if n else "WARN", "motor spin", f"{name} at {duty:.0%}: {n:+d} counts "
               f"({enc.output_revs(n):+.3f} gear revs)")
    if fwd == 0 and rev == 0:
        report("FAIL", "encoder", "no counts either way: check motor power (fuse, switch, B+/B-, "
               f"M+/M-) and encoder wiring (3V3, GND, A=GPIO{cfg.pins.enc_a}, B=GPIO{cfg.pins.enc_b})")
    elif fwd == 0 or rev == 0:
        report("WARN", "encoder", "moved only one way: is the shell still attached? re-run")
    elif (fwd > 0) == (rev > 0):
        report("FAIL", "encoder", "same sign both ways: one encoder channel is dead or miswired")
    else:
        flip = fwd < 0
        report("WARN" if flip else "PASS", "encoder", "forward counts "
               + ("DOWN: set encoder.invert to true in config.yaml" if flip else "UP"))
    if enc.errors:
        report("WARN", "encoder", f"{enc.errors} missed/illegal transitions (noise or loose wire)")
    report("PASS", "direction", f"config says {cfg.motor.close_direction} CLOSES the shield. "
           "Watch which way the spool turned on the forward spin; if forward would OPEN it, "
           "set motor.close_direction to the other value")


def check_camera(cfg) -> None:
    from .vision import Camera
    try:
        cam = Camera(cfg)
    except Exception as exc:
        report("FAIL", "camera", str(exc))
        return
    try:
        frames = [cam.read() for _ in range(60)]
    except Exception as exc:
        report("FAIL", "camera", f"capture failed: {exc}")
        return
    finally:
        cam.close()
    fps = (len(frames) - 1) / max(frames[-1][1] - frames[0][1], 1e-6)
    gray = frames[-1][0]
    mean = float(gray.mean())
    report("PASS" if fps >= 0.9 * cfg.camera.fps else "WARN", "camera",
           f"{cam.model} {gray.shape[1]}x{gray.shape[0]} at {fps:.1f} fps "
           f"(target {cfg.camera.fps}), mean level {mean:.0f}")
    if mean < 5 or mean > 250:
        report("WARN", "camera", "image is black or saturated: lens cap / exposure?")


def confirm() -> bool:
    if not sys.stdin.isatty():
        print("motor tests need confirmation: re-run with --yes")
        return False
    print("\nNext: brief low-duty spins in BOTH directions. Disconnect the shell from the motor\n"
          "and keep hands clear.")
    return input("Type 'yes' to continue: ").strip().lower() == "yes"


def run(cfg, *, motor: bool = True, camera: bool = True, yes: bool = False,
        duty: float = 0.25) -> int:
    print("shield check\n")
    check_platform()
    check_boot_config(cfg)
    check_pinmux(cfg)
    chip = drv = enc = None
    try:
        chip = m.open_gpiochip()
        drv = m.Bts7960(chip, cfg)
        report("PASS" if drv.hardware_pwm else "WARN", "pwm backend",
               f"hardware {drv.rpwm.path} / {drv.lpwm.path}" if drv.hardware_pwm
               else "SOFTWARE PWM fallback (fix the overlay before real use)")
        report("PASS", "motor driver", "EN pins claimed low (coast)")
        enc = m.Encoder(chip, cfg)
        a, b = enc.levels()
        report("PASS", "encoder idle", f"A={a} B={b}")
        check_inputs(chip, cfg)
        if not motor:
            report("SKIP", "motor", "--no-motor")
        elif yes or confirm():
            check_enable_gate(drv, enc, cfg, duty)
            check_motor(drv, enc, cfg, duty)
        else:
            report("SKIP", "motor", "not confirmed")
    except Exception as exc:
        report("FAIL", "hardware", f"{type(exc).__name__}: {exc}")
    finally:
        for res in (enc, drv):
            if res is not None:
                res.close()
        if chip is not None:
            import lgpio
            lgpio.gpiochip_close(chip)
    if camera:
        check_camera(cfg)
    print(f"\n{counts['FAIL']} FAIL, {counts['WARN']} WARN, {counts['PASS']} PASS")
    return 1 if counts["FAIL"] else 0


def calibrate(cfg) -> int:
    """Measure full travel (open stop -> closed stop) and full-speed close time, and write
    them to config.yaml. Run with the shell ATTACHED; the shield travels end to end."""
    from . import config
    from .controller import Controller
    if sys.stdin.isatty():
        print("Calibrate drives the shield into both hard stops at low duty, then does one\n"
              "full-speed close. Keep hands clear of the shell.")
        if input("Type 'yes' to continue: ").strip().lower() != "yes":
            return 1
    ctrl = Controller(cfg)

    def run_until(mode_done):
        ctrl.loop(until=lambda: mode_done() or bool(ctrl.fault))
        if ctrl.fault:
            raise RuntimeError(ctrl.fault)

    try:
        print("1/4 homing into the open stop...")
        ctrl.home()
        run_until(lambda: ctrl.mode != "homing")
        print("2/4 seeking the closed stop...")
        ctrl.seek_closed()
        run_until(lambda: ctrl.mode != "seeking")
        travel = ctrl.seek_result
        if travel is None or travel <= 2 * cfg.motion.soft_margin_counts:
            raise RuntimeError(f"travel of {travel} counts is too short: did the shell move?")
        ctrl.set_travel(travel)
        print(f"    travel {travel} counts = {ctrl.encoder.output_revs(travel):.2f} gear revs")
        print("3/4 back to open...")
        ctrl.open()
        run_until(lambda: ctrl.mode != "moving")
        print("4/4 full-speed close...")
        peak, furthest, reached = 0.0, 0, None
        t0 = time.monotonic()
        ctrl.emergency_close()

        def track():
            nonlocal peak, furthest, reached
            peak = max(peak, ctrl.velocity)
            furthest = max(furthest, ctrl.position)
            if reached is None and ctrl.position >= ctrl.hi - cfg.motion.tolerance_counts:
                reached = time.monotonic()  # covered: this is what protection depends on
        ctrl.loop(on_tick=track, until=lambda: ctrl.mode != "moving" or bool(ctrl.fault))
        if ctrl.fault:
            raise RuntimeError(ctrl.fault)
        close_time = (reached or time.monotonic()) - t0
        over = furthest - ctrl.hi
        print(f"    closed in {close_time:.3f} s, top speed {peak:.0f} counts/s "
              f"({peak / cfg.encoder.counts_per_motor_rev / cfg.encoder.gear_ratio:.0f} gear rev/s)")
        print(f"    overshoot past the soft limit: {over} counts "
              f"({travel - furthest} counts short of the hard stop)")
        if travel - furthest <= 0:
            print("    WARNING: it hit the closed hard stop at speed. Raise motion.soft_margin_counts "
                  "or pid.kd, then re-run calibrate")
        ctrl.open()  # the testbed rests open
        run_until(lambda: ctrl.mode != "moving")
    except RuntimeError as exc:
        print(f"calibrate FAILED: {exc}")
        return 1
    finally:
        ctrl.shutdown()
    config.save(cfg, {"motion.travel_counts": travel, "motion.close_time_s": round(close_time, 2),
                      "motion.free_speed": round(peak)})
    print(f"\nwrote to {cfg.path}: travel_counts={travel}, close_time_s={close_time:.2f}, "
          f"free_speed={peak:.0f} counts/s")
    if peak < cfg.motion.max_speed:
        print(f"note: max_speed ({cfg.motion.max_speed}) is above the measured top speed; "
              f"consider lowering it to ~{0.9 * peak:.0f}")
    return 0
