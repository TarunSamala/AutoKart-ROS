#!/usr/bin/env bash
set -euo pipefail

ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)
RUNTIME="$ROOT/.brake-test-runtime"
VENV="$RUNTIME/venv"

say() { printf '\n[AutoKart setup] %s\n' "$*"; }
fail() { printf '\n[AutoKart setup] ERROR: %s\n' "$*" >&2; exit 1; }

case "$(uname -m)" in
    x86_64) SDK_TARGET=x86_64-linux-gnu ;;
    aarch64|arm64) SDK_TARGET=aarch64-linux-gnu ;;
    *) fail "Supported targets are x86_64 Ubuntu 22.04 and 64-bit Raspberry Pi Ubuntu 22.04." ;;
esac

if [[ -r /etc/os-release ]]; then
    # shellcheck disable=SC1091
    source /etc/os-release
    case "${ID:-}:${VERSION_ID:-}:$SDK_TARGET" in
        ubuntu:22.04:*) ;;
        debian:12:aarch64-linux-gnu) ;;
        *) fail "Supported OS targets are Ubuntu 22.04 and 64-bit Raspberry Pi OS 12; detected ${PRETTY_NAME:-unknown OS}." ;;
    esac
fi

python3 - <<'PY' || fail "Python 3.10 or newer is required by the pinned ONNX Runtime."
import sys
raise SystemExit(0 if sys.version_info >= (3, 10) else 1)
PY

say "Installing Ubuntu runtime and HP60C build dependencies"
sudo apt-get update
sudo apt-get install -y \
    python3 python3-venv python3-pip python3-opencv python3-numpy \
    python3-scipy python3-yaml python3-serial cmake g++ make curl xdg-utils iproute2 \
    libusb-1.0-0 ca-certificates

mkdir -p "$RUNTIME"
if [[ ! -x "$VENV/bin/python3" ]]; then
    python3 -m venv --system-site-packages "$VENV"
fi

say "Installing pinned CPU inference runtime"
"$VENV/bin/python3" -m pip install --upgrade "pip<26"
"$VENV/bin/python3" -m pip install "onnxruntime==1.23.2"

MODEL="$ROOT/models/yolo26s/best.onnx"
[[ -f "$MODEL" ]] || fail "YOLO26s model is missing: $MODEL"
EXPECTED_MODEL_SHA=425a89cef2eda9db9c84fbf30cad187e510510b95d7b7e9904003fd22cec1158
ACTUAL_MODEL_SHA=$(sha256sum "$MODEL" | awk '{print $1}')
[[ $ACTUAL_MODEL_SHA == "$EXPECTED_MODEL_SHA" ]] || fail "YOLO26s checksum mismatch."

SDK_LIB="$ROOT/autonomy_stack_phase1/vendor/linux_ros/linux/libs/lib/$SDK_TARGET/libAngstrongCameraSdk.so"
[[ -f "$SDK_LIB" ]] || fail "HP60C vendor library for $SDK_TARGET is missing."

say "Building native HP60C bridge for $SDK_TARGET"
bash "$ROOT/autonomy_stack_phase1/phase_01_perception/build_native.sh"

if [[ -f /run/.containerenv || -n ${CONTAINER_ID:-} ]]; then
    # A rootless Distrobox shares the host's USB devices but cannot reload the
    # host udev daemon.  Attempting it is both ineffective and caused the Arch
    # desktop launcher to exit during first-run setup.
    say "Distrobox detected; using host HP60C USB permissions"
else
    say "Installing HP60C and ESP32 USB permissions"
    sudo install -m 0644 \
        "$ROOT/autonomy_stack_phase1/vendor/linux_ros/linux/scripts/angstrong-camera.rules" \
        /etc/udev/rules.d/99-angstrong-camera.rules
    sudo install -m 0644 \
        "$ROOT/brake_test/portable/99-autokart-esp32.rules" \
        /etc/udev/rules.d/99-autokart-esp32.rules
    LOGIN_USER=${SUDO_USER:-$USER}
    if getent group dialout >/dev/null 2>&1; then
        sudo usermod -aG dialout "$LOGIN_USER"
    fi
    sudo udevadm control --reload-rules
    sudo udevadm trigger
fi

"$VENV/bin/python3" - <<'PY'
import cv2
import numpy
import onnxruntime
import scipy
import serial
import yaml
print("Python runtime verified")
print("OpenCV", cv2.__version__)
print("ONNX Runtime", onnxruntime.__version__, onnxruntime.get_available_providers())
print("PySerial", serial.__version__)
PY

touch "$RUNTIME/installed-usb-serial-v1"

if [[ ${AUTOKART_SKIP_DESKTOP:-0} != 1 ]]; then
    DESKTOP_DIR=${XDG_DESKTOP_DIR:-"$HOME/Desktop"}
    APPLICATIONS_DIR="$HOME/.local/share/applications"
    mkdir -p "$DESKTOP_DIR" "$APPLICATIONS_DIR"
    for destination in \
        "$DESKTOP_DIR/AutoKart Brake Test.desktop" \
        "$APPLICATIONS_DIR/autokart-brake-test.desktop"
    do
        sed \
            -e "s|@ROOT@|$ROOT|g" \
            "$ROOT/brake_test/portable/AutoKart Brake Test.desktop.in" > "$destination"
        chmod +x "$destination"
    done
fi

say "Installation complete"
printf '%s\n' "Unplug/reconnect the HP60C once, connect the ESP32 by USB," \
    "then launch AutoKart Brake Test again. Internet Wi-Fi may remain connected." \
    "On Raspberry Pi, reboot once after first installation so dialout access is active."
