#!/usr/bin/env bash
set -euo pipefail
python3 - <<'PY'
from importlib.util import find_spec
from shutil import which
from kart_sim.backends import detect_chrono
from kart_sim.ros2_sil import Ros2Runtime

ros = Ros2Runtime.detect()
print(f"python: available")
print(f"pytest: {bool(find_spec('pytest'))}")
print(f"rclpy: {ros.rclpy_available}")
print(f"ros2_cli: {ros.cli_available}")
print(f"colcon: {bool(which('colcon'))}")
print(f"gz: {bool(which('gz'))}")
print(f"gazebo_classic: {bool(which('gazebo'))}")
print(f"chrono: {detect_chrono().available}")
PY
