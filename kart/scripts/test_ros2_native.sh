#!/usr/bin/env bash
set -euo pipefail
project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ ! -r /tmp/autokart-ros-install/setup.bash ]]; then
  "$project_root/scripts/build_ros2.sh"
fi
set +u
source /opt/ros/humble/setup.bash
source /tmp/autokart-ros-install/setup.bash
set -u
export ROS_DOMAIN_ID="${AUTOKART_ROS_DOMAIN_ID:-77}"
export ROS_LOG_DIR="${AUTOKART_ROS_LOG_DIR:-/tmp/autokart-ros-runtime-log}"
mkdir -p "$ROS_LOG_DIR"
exec python3 "$project_root/tools/ros2_native_smoke.py"
