#!/usr/bin/env bash
set -euo pipefail
project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ ! -r /opt/ros/humble/setup.bash ]] || ! command -v colcon >/dev/null 2>&1; then
  echo "NOT_EXECUTED_EXTERNAL_DEPENDENCY: ROS 2 Humble/colcon unavailable" >&2
  exit 3
fi
set +u
source /opt/ros/humble/setup.bash
set -u
exec colcon --log-base /tmp/autokart-ros-log build \
  --base-paths "$project_root/ros2_ws/src" \
  --build-base /tmp/autokart-ros-build \
  --install-base /tmp/autokart-ros-install \
  --event-handlers console_cohesion+
