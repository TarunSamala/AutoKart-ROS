#!/usr/bin/env bash
set -eo pipefail

bundle_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
session_name=${1:-tarun-first-ai-$(date +%Y%m%d-%H%M%S)}
output_dir="$bundle_root/bags/$session_name"

if [[ -e "$output_dir" ]]; then
    echo "Session already exists: $output_dir" >&2
    exit 1
fi

source /opt/ros/humble/setup.bash
source "$bundle_root/ros2_ws/install/local_setup.bash"
mkdir -p "$bundle_root/bags"

echo "Recording stationary camera/perception evidence to $output_dir"
echo "Stop with Ctrl+C. Do not connect motor battery or actuator authority."
ros2 bag record -o "$output_dir" \
  /hp60c/rgb/image_raw /hp60c/depth/image_raw \
  /hp60c/rgb/camera_info /hp60c/depth/camera_info \
  /hp60c/status /hp60c/diagnostics \
  /perception/status /perception/diagnostics \
  /perception/detections /perception/cones

