#!/usr/bin/env bash
# One command: build (if needed), start the ROS 2 stack and the dashboard gateway.
# The kart stays DISARMED until you press REQUEST ARM on the dashboard (and every startup check is READY).
#   scripts/start_autokart.sh [--build] [--mode LIVE|REPLAY|SIMULATED]
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
WS="$ROOT/ros2_ws"
MODE=LIVE; BUILD=0
while [[ $# -gt 0 ]]; do case "$1" in --build) BUILD=1;; --mode) MODE="$2"; shift;; esac; shift; done

source "${ROS_SETUP:-/opt/ros/humble/setup.bash}"
if [[ $BUILD -eq 1 || ! -d "$WS/install" ]]; then
  (cd "$WS" && colcon build --symlink-install)
fi
source "$WS/install/setup.bash"
mkdir -p "$ROOT/logs"

# Read the configured throttle cap out of the same YAML so the dashboard never shows a baked-in number.
CONFIG="${PARAMS:-$WS/src/autokart_bringup/config/autokart.yaml}"
CAP_V="$(grep -m1 'throttle_v_max:' "$CONFIG" | sed -E 's/.*throttle_v_max:[[:space:]]*([0-9.]+).*/\1/')"
CAP_V="${CAP_V:-2.0}"

pids=()
cleanup() { echo "stopping..."; for p in "${pids[@]}"; do kill "$p" 2>/dev/null || true; done; wait 2>/dev/null || true; }
trap cleanup EXIT INT TERM

python3 "$ROOT/dashboard/ros_gateway/gateway.py" --mode "$MODE" --throttle-cap-v "$CAP_V" >"$ROOT/logs/gateway.log" 2>&1 & pids+=($!)
ros2 launch autokart_bringup autokart.launch.py data_mode:="$MODE" params:="$CONFIG" 2>&1 | tee "$ROOT/logs/ros.log" &
pids+=($!)
echo "Dashboard: http://localhost:8080/index.html   (vehicle is DISARMED)"
wait
