#!/bin/sh
# Force the BTS7960 to COAST without Python: EN low, then every RP1 header PWM channel to 0.
# Use it if the motor is ever left running (e.g. Python was killed). Sysfs PWM keeps
# running after a process dies, so this matters. The systemd unit also runs it on stop.
# Usage: ./safe_stop.sh [EN pins, default 23,24]
EN_PINS="${1:-23,24}"

if command -v pinctrl >/dev/null 2>&1; then
    pinctrl set "$EN_PINS" op dl
else
    echo "safe_stop: pinctrl not found; relying on the driver's pull-downs on EN" >&2
fi

for chip in /sys/class/pwm/pwmchip*; do
    case "$(readlink -f "$chip")" in *1f00098000.pwm*) ;; *) continue ;; esac  # never the fan PWM
    for ch in "$chip"/pwm[0-9]*; do
        [ -d "$ch" ] || continue
        echo 0 > "$ch/duty_cycle" 2>/dev/null
        echo 0 > "$ch/enable" 2>/dev/null
    done
done
echo "safe_stop: EN low, PWM 0 (motor coasting)"
