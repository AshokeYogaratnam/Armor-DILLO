# Armor-DILLO

Control software for an autonomous, armadillo-style telescoping shield over a stationary
testbed. One REV HD Hex motor (BTS7960 driver) drives the first shell segment, which carries
the rest. A Pi camera watches for falling objects and dust, and the shield closes in time
to protect the testbed. It rests **open**. Everything runs on the Raspberry Pi 5: develop
in VS Code on the Pi and run against the real hardware.

## Status

| Step | State |
|---|---|
| 1. Motor + encoder + `shield check` | done |The sh
| 2. Controller (`set_coverage`, homing, stall) + state machine + override + watchdog | done |
| 3. Vision impact detector + camera health + latency logging | done |
| 4. Dust detector + policy | done |
| 5. Dashboard | **done** |

## Setup (once, on the Pi)

1. Add these under `[all]` in `/boot/firmware/config.txt`:
   ```
   dtoverlay=pwm-2chan,pin=18,func=2,pin2=19,func2=2
   camera_auto_detect=1
   ```
   The overlay defaults to GPIO12/13, so the `pin=` parameters are required. Don't enable
   audio overlays: GPIO18/19 are also the PCM pins.
2. `bash install.sh`, then reboot.
3. Open the folder in VS Code. It picks up `.venv` automatically.

## First power-up, in order

| Step | Command | Shell |
|---|---|---|
| 1 | `shield check` – wiring, PWM, encoder sign, camera | **disconnected** |
| 2 | fix `encoder.invert` / `motor.close_direction` in `config.yaml` if check says so | |
| 3 | `shield calibrate` – travel + close time, written to `config.yaml` | attached |
| 4 | `shield set 0.5`, `shield set 1`, `shield set 0` | attached |
| 5 | `shield run`, open http://127.0.0.1:8000 – try the switch, buttons, a ball drop | attached |
| 6 | `bash install.sh --enable` – start `shield run` on every boot | |

Run any of these from VS Code (**Run and Debug** → pick one) or a terminal
(`.venv/bin/shield ...`). Tests: `.venv/bin/pytest`.

**The service and VS Code fight over the GPIO pins.** Once the service is enabled,
`sudo systemctl stop shield` before running anything from VS Code (the error tells you if
you forget). Service logs: `journalctl -u shield -f`, plus `logs/shield.log`.

**If the motor is ever left running** (e.g. Python was killed): `./safe_stop.sh`.

## Dashboard

`shield run` serves it at **http://127.0.0.1:8000**. Open that in Chromium on the Pi.

- State badge and reason, coverage bar (with target and request markers), coverage slider,
  OPEN / CLOSE, and a big EMERGENCY CLOSE.
- Live camera preview (~10 fps) with the tracked object's box and TTC, plus a dust severity
  bar. "DISARMED" when the shield isn't fully open in AUTO.
- Falling object level/TTC, the four dust meters (severity, contrast, haze, particles),
  camera FPS/health, live capture → detect latency against the TTC threshold and close
  time, the last hazard close's full latency row, motor status, override switch state, and
  the event log.

Authority never moves to the dashboard. Its buttons are *requests* through the same arbiter:
the MANUAL switch always wins (the buttons grey out), a hazard close beats a dashboard OPEN,
and EMERGENCY CLOSE is treated as a hazard, so it latches like one. It can't touch the
watchdog, stall detection or camera fail-safe. It runs on its own thread; if it dies, control
carries on. It binds to 127.0.0.1 and has no login, so only set `dashboard.host: 0.0.0.0` on
a network you trust.

`POST /hazard` (`{"level": "critical", "kind": "rock", "confidence": 0.9, "ttc_s": 1.2,
"source": "rover"}`) lets another system report a hazard. It's off unless
`dashboard.external_hazards: true`. A critical report is an emergency close; a warning is logged.

## How it behaves

- **One command:** `Controller.set_coverage(x, speed_limit)`, 0 = open, 1 = closed.
  `open()`, `close()` and `emergency_close()` (max speed, profile bypassed) all call it.
  A 200 Hz loop tracks it: trapezoidal profile → PID + velocity feedforward → BTS7960.
- **Startup:** homes gently into the open stop (stall = found it), then rests open.
  Each process start homes again, since position isn't stored.
- **Priority:** MANUAL switch > hazard close > requests > rest open. In MANUAL, one press
  of OPEN or CLOSE sends the shield all the way to that end; it holds otherwise.
- **Hazard closes latch.** The camera is under the shell, so once closed it can't tell
  whether the hazard has passed. The shield stays closed (the dashboard can't open it)
  until someone retracts it: switch to MANUAL, press OPEN, switch back to AUTO. That also
  re-arms detection.
- **Faults** (shown in the log with a reason):
  - *stalled*: the shell moved, then stopped against something.
  - *motor power lost*: PWM applied but the encoder never moved (E-stop, fuse, or a jam).
  
  The motor brakes and autonomy stops until someone presses a button in MANUAL (or restarts).
- **Watchdog:** if the control loop goes quiet for 200 ms, a watchdog thread brakes the
  motor; systemd kills and restarts the process after 2 s of silence.
- **Holding:** the motor brakes and friction holds the shell (`motion.hold: brake`). If the
  shell is pushed more than `drift_counts` off target, it corrects.
- **Soft limits:** the shield never drives past `soft_margin_counts` inside either hard stop.

## Falling-object detection

Runs on its own thread at camera rate and never blocks the control loop. It only watches
while the shield is **fully open in AUTO**. Otherwise the shell is in view. Each time it
re-arms (e.g. after a manual retract), it spends `impact.warmup_s` learning the sky first.

1. Background subtraction finds blobs; frame differencing keeps only the moving ones.
2. Sparse Lucas-Kanade flow tracks corners on the biggest moving blob. How fast they spread
   apart is the expansion rate. A plain ball with few corners falls back to blob-area growth.
3. Time to contact = 1 / expansion rate, averaged over a few frames. Under the threshold
   (`close_time_s + ttc_margin_s`) for 2 frames in a row → **emergency close**.

TTC assumes constant speed; a falling ball speeds up, so it arrives a bit sooner than the
estimate. `ttc_margin_s` covers that: raise it if the latency log shows thin margins.

**Camera health** (it is the only hazard sensor): frozen, black, saturated or missing
frames, or FPS under `camera.min_fps`, while armed → the fail-safe (`camera.fail_safe:
close`). The camera thread keeps trying to reopen the camera.

## Dust detection

Three scores, each 0..1, against a clear-air baseline learned for `dust.warmup_s` at arming
(then slowly tracked while the air is clear, so passing clouds and sunsets don't trigger it):

- **contrast:** drop in Laplacian variance. Fine detail washes out.
- **haze:** the grayscale dark channel (local minimum brightness) rises toward white.
- **particles:** moving specks per frame (small-particle flow density).

Severity is their weighted mean, smoothed, with hysteresis between levels:

| Level | Default action |
|---|---|
| WARNING (severity >= 0.30, clears < 0.20) | logged only; full close if it lasts `sustained_s` (10 s) |
| SEVERE (>= 0.60, back to warning < 0.50) | full close after `severe_hold_s` (1 s) |

Set `policy.dust_partial_close: true` to have a WARNING close partially (coverage = severity)
instead. Any partial close puts the shell in view, so detection stops until a manual retract.
`policy.shed_cycles` > 0 makes a manual OPEN after a dust close wiggle the shell first to
shed dust.

The thresholds are first guesses from synthetic frames. Tune them against real footage;
the dashboard (step 5) shows the live scores.

## Policy (`shield/policy.py`)

Falling object CRITICAL → emergency close > camera fail-safe → close > dust (as above) >
falling object WARNING (TTC under twice the threshold) → log only. Every decision change is
logged, and every close latches until a manual retract.

**Latency log:** every hazard close writes a row to `logs/latency.csv`: capture → detect,
detect → command, close time, and margin = TTC − (capture → command + close time). A
negative margin means the shield closed too late. Equivalently, the camera must see an
object coming from at least

    range >= object_speed * (detect_latency + decision + close_time + margin)

## Layout

```
config.yaml         every pin and tunable (calibrate writes travel/close time back here)
shield/
  controller.py     set_coverage(), the 200 Hz loop, homing, stall/power-loss, watchdog
  state.py          command priority + state names
  override.py       AUTO/MANUAL switch, OPEN/CLOSE buttons
  motor.py          BTS7960 (never both PWM), hardware PWM (+ software fallback), encoder
  vision.py         camera, impact (LK flow -> TTC), dust, camera health, latency log
  policy.py         hazard fusion -> one Hazard for the arbiter
  dashboard.py      FastAPI + WebSocket + MJPEG; static/ index.html, app.js, style.css
  check.py          `shield check`, `shield calibrate`
  config.py         loads/saves config.yaml
  main.py           `shield` CLI, logging (console + logs/shield.log)
tests/              pure-logic pytest, no hardware
docs/wiring.md      pin map
shield.service      systemd unit (installed by install.sh)
safe_stop.sh        EN low + PWM 0 without Python
```
- http://127.0.0.1:8000