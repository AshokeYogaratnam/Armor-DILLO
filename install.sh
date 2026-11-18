#!/usr/bin/env bash
# One-time setup on the Raspberry Pi 5 (Raspberry Pi OS 64-bit, Bookworm or newer).
#   bash install.sh            deps, venv, permissions, systemd unit (installed, not enabled)
#   bash install.sh --enable   also start `shield run` on every boot
set -euo pipefail
REPO="$(cd "$(dirname "$0")" && pwd)"

echo "==> apt packages"
# picamera2, lgpio, opencv and numpy MUST come from apt: the pip builds don't work on the
# Pi 5 or are compiled against a different numpy than picamera2.
sudo apt-get update
sudo apt-get install -y python3-venv python3-pip python3-setuptools python3-lgpio \
    python3-picamera2 python3-opencv python3-numpy python3-yaml python3-pytest \
    python3-fastapi python3-uvicorn python3-websockets

echo "==> virtualenv (sees the apt packages)"
python3 -m venv --system-site-packages "$REPO/.venv"
"$REPO/.venv/bin/pip" install --no-deps --no-build-isolation -e "$REPO"
"$REPO/.venv/bin/python" -c "import lgpio, picamera2, cv2, numpy, yaml, fastapi, uvicorn, websockets; print('    imports ok')"

echo "==> permissions (GPIO, camera, sysfs PWM without sudo)"
sudo usermod -aG gpio,video "$USER"
echo 'SUBSYSTEM=="pwm*", ACTION=="add|change", RUN+="/bin/sh -c '"'"'chgrp -R gpio /sys%p && chmod -R g+rw /sys%p'"'"'"' \
    | sudo tee /etc/udev/rules.d/99-shield-pwm.rules >/dev/null
sudo udevadm control --reload-rules
chmod +x "$REPO/safe_stop.sh"

echo "==> systemd unit"
sed -e "s#@REPO@#$REPO#g" -e "s#@USER@#$USER#g" "$REPO/shield.service" \
    | sudo tee /etc/systemd/system/shield.service >/dev/null
sudo systemctl daemon-reload
if [[ "${1:-}" == "--enable" ]]; then
    sudo systemctl enable --now shield
    echo "    enabled: starts on boot. Logs: journalctl -u shield -f"
else
    echo "    installed, not enabled (bash install.sh --enable once calibrated)"
fi

echo "==> boot config"
BOOT=/boot/firmware/config.txt
grep -Eq '^\s*dtoverlay=pwm-2chan,pin=18,func=2,pin2=19,func2=2' "$BOOT" \
    && echo "    PWM overlay present" \
    || echo "    MISSING: add 'dtoverlay=pwm-2chan,pin=18,func=2,pin2=19,func2=2' under [all] in $BOOT"
grep -Eq '^\s*(camera_auto_detect=1|dtoverlay=ov5647)' "$BOOT" \
    || echo "    MISSING: add 'camera_auto_detect=1' to $BOOT"

echo
echo "Done. Reboot (group changes + config.txt), then: .venv/bin/shield check"
