#!/usr/bin/env bash
set -euo pipefail
project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$project_root"
export PYTHONPATH="$project_root${PYTHONPATH:+:$PYTHONPATH}"
python3 -m kart_sim.cli ros2 status
python3 -m kart_sim.cli ros2 validate configs/autonomy/ros2_sil.yaml
if [[ "${1:-}" != "--native" ]]; then
  echo "Deterministic mock SIL contract validated. Pass --native to build and launch the installed ROS 2 graph."
  exit 0
fi
if [[ ! -r /opt/ros/humble/setup.bash ]] || ! command -v colcon >/dev/null 2>&1; then
  echo "NOT_EXECUTED_EXTERNAL_DEPENDENCY: ROS 2 Humble/colcon unavailable" >&2
  exit 3
fi
set +u
source /opt/ros/humble/setup.bash
set -u
colcon build --base-paths ros2_ws/src --build-base /tmp/autokart-ros-build --install-base /tmp/autokart-ros-install --log-base /tmp/autokart-ros-log
set +u
source /tmp/autokart-ros-install/setup.bash
set -u
exec ros2 launch autokart_bringup full_stack.launch.py
