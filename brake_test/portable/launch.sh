#!/usr/bin/env bash
set -euo pipefail

ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)
RUNTIME="$ROOT/.brake-test-runtime"
VENV="$RUNTIME/venv"
UI=http://127.0.0.1:8781/
SERIAL_PORT=${AUTOKART_SERIAL_PORT:-auto}

fail() { printf '\nAutoKart Brake Test: %s\n' "$*" >&2; read -r -p "Press Enter to close..." _ || true; exit 1; }

if [[ -f "$RUNTIME/installed-ok" && -x "$VENV/bin/python3" ]] && \
        "$VENV/bin/python3" -c 'import serial' >/dev/null 2>&1; then
    touch "$RUNTIME/installed-usb-serial-v1"
fi

if [[ ! -f "$RUNTIME/installed-usb-serial-v1" || ! -x "$VENV/bin/python3" ]]; then
    printf '%s\n' "First launch: installing the AutoKart Brake Test runtime."
    bash "$ROOT/brake_test/portable/install.sh"
fi

case "$(uname -m)" in
    x86_64) SDK_TARGET=x86_64-linux-gnu; OPENCV_THREADS=8 ;;
    aarch64|arm64) SDK_TARGET=aarch64-linux-gnu; OPENCV_THREADS=4 ;;
    *) fail "Unsupported CPU architecture: $(uname -m)" ;;
esac

exec 9>"$RUNTIME/launcher.lock"
flock -n 9 || fail "The AutoKart Brake Test is already running."

export PYTHONPATH="$ROOT/autonomy_stack_phase1/phase_01_perception/src:$ROOT/kart/ros2_ws/src/autokart_vehicle_geometry:$ROOT/tools"
export LD_LIBRARY_PATH="$ROOT/autonomy_stack_phase1/vendor/linux_ros/linux/libs/lib/$SDK_TARGET${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"

APP_PID=
safety_lock() {
    curl --silent --show-error --max-time 3 \
        -H 'Content-Type: application/json' \
        -d '{"action":"lock"}' "$UI/api/control" >/dev/null 2>&1 || \
        printf '%s\n' "CRITICAL: launcher exit could not confirm ESP32 serial brake" >&2
    if [[ -n ${APP_PID:-} ]]; then
        kill "$APP_PID" >/dev/null 2>&1 || true
    fi
}
trap safety_lock EXIT INT TERM HUP

mkdir -p "$RUNTIME/logs"
LOG="$RUNTIME/logs/brake-test-$(date -u +%Y%m%dT%H%M%SZ).log"
printf '%s\n' "Starting AutoKart Brake Test" "Log: $LOG"
printf '%s\n' "ESP32 transport: USB serial ($SERIAL_PORT)" \
    "The dashboard auto-discovers the controller and safely disables START until verified."
LAN_IPS=$(ip -o -4 addr show scope global 2>/dev/null | \
    awk '{split($4, address, "/"); print address[1]}' || true)
if [[ -n ${LAN_IPS// } ]]; then
    printf '%s\n' "Dashboard addresses for phones/laptops on this router:"
    for address in $LAN_IPS; do
        case "$address" in
            *:*) continue ;;
            127.*|169.254.*) continue ;;
            *) printf '  http://%s:8781/\n' "$address" ;;
        esac
    done
fi

"$VENV/bin/python3" "$ROOT/tools/bldc_encoder_camera_ui.py" \
    --serial-port "$SERIAL_PORT" \
    --serial-baud 115200 \
    --trigger-gap 1.20 \
    --pitch-deg 16.37 \
    --display-confidence 0.35 \
    --confidence 0.80 \
    --confirmations 1 \
    --opencv-threads "$OPENCV_THREADS" \
    --presentation-mode \
    > >(tee -a "$LOG") 2>&1 &
APP_PID=$!

for _ in $(seq 1 80); do
    if curl --fail --silent --max-time 1 "$UI/api/status" >/dev/null 2>&1; then
        if [[ ${AUTOKART_SKIP_BROWSER:-0} != 1 ]]; then
            xdg-open "$UI" >/dev/null 2>&1 || true
        fi
        printf '%s\n' "Dashboard opened. Use START only with the kart jacked and restrained."
        wait "$APP_PID"
        exit $?
    fi
    if ! kill -0 "$APP_PID" >/dev/null 2>&1; then
        wait "$APP_PID" || true
        fail "Dashboard failed to start. Review $LOG"
    fi
    sleep 0.25
done

fail "Dashboard did not become ready. Review $LOG"
