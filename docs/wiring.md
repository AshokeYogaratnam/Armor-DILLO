# Wiring and pin map

All GPIO numbers are BCM, 3.3 V logic. Every pin can be changed in `config.yaml`
(`pins:`); `shield check` verifies the result.

## BTS7960 motor driver

| BTS7960 pin | Pi                       | Notes |
|-------------|--------------------------|-------|
| RPWM        | GPIO19 (pin 35)          | forward, hardware PWM (RP1 PWM0 channel 3) |
| LPWM        | GPIO18 (pin 12)          | reverse, hardware PWM (RP1 PWM0 channel 2) |
| R_EN        | GPIO23 (pin 16)          | driven together with L_EN; high = enabled |
| L_EN        | GPIO24 (pin 18)          | |
| VCC         | 3V3 (pin 1)              | **must be 3.3 V**: at 5 V the 74HC244 will not reliably read 3.3 V inputs |
| GND         | GND (pin 14)             | shared logic ground |
| R_IS, L_IS  | **not connected**        | can reach supply voltage on a fault; never wire directly to the Pi |
| B+ / B-     | battery after Fuse A + main switch | |
| M+ / M-     | HD Hex motor             | swap if `shield check` shows the wrong close direction (or set `motor.close_direction`) |

Truth table: EN low = coast; EN high + both PWM low = brake; RPWM = forward; LPWM = reverse.
The software never drives RPWM and LPWM together, brakes before reversing, and keeps EN
low whenever it is not running. The board's 30k pull-downs keep EN low while the Pi boots.

## HD Hex encoder

| Encoder | Pi |
|---------|----|
| A       | GPIO5 (pin 29) |
| B       | GPIO6 (pin 31) |
| VCC     | 3V3 (pin 17) |
| GND     | GND (pin 25) |

28 counts per motor revolution (x4 decoding). Internal pull-ups are enabled.

## Manual override (active-low, internal pull-ups, debounced)

| Input | Pi | Wiring |
|-------|----|--------|
| AUTO/MANUAL toggle | GPIO17 (pin 11) | MANUAL position connects GPIO17 to GND. A broken wire reads AUTO, so autonomy (hazard protection) stays on. |
| OPEN button  | GPIO27 (pin 13) | to GND when pressed |
| CLOSE button | GPIO22 (pin 15) | to GND when pressed |

## Optional limit switches (off by default; set `limits.enabled: true`)

| Switch | Pi |
|--------|----|
| open   | GPIO20 (pin 38), to GND when hit |
| closed | GPIO21 (pin 40), to GND when hit |

## Camera

Arducam 5MP (OV5647) on a CSI port. **The Pi 5 uses 22-pin mini CSI connectors**, so a
15-pin camera needs a 22-to-15-pin adapter cable.

## Power

Battery -> Fuse A -> main switch -> (BTS7960 B+/B-) and (power board -> USB-C -> Pi).
The main switch kills the Pi too, which is an unclean shutdown (see setup_pi5.md). If a
motor-only E-stop is added later, the software detects "PWM applied, encoder not moving"
and reports a fault instead of retrying forever.
