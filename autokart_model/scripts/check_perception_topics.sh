#!/usr/bin/env bash
set -eo pipefail

bundle_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
source /opt/ros/humble/setup.bash
source "$bundle_root/ros2_ws/install/local_setup.bash"

echo "ROS nodes:"
ros2 node list

echo "Required topics and types:"
for topic in \
  /hp60c/rgb/image_raw /hp60c/depth/image_raw /hp60c/rgb/camera_info \
  /hp60c/status /hp60c/diagnostics /perception/status \
  /perception/diagnostics /perception/detections /perception/cones; do
    printf '%s: ' "$topic"
    ros2 topic type "$topic" 2>/dev/null || echo MISSING
done

echo "One camera status sample:"
timeout 10 ros2 topic echo /hp60c/status --once || true
echo "One perception status sample:"
timeout 10 ros2 topic echo /perception/status --once || true
echo "One perception diagnostic sample:"
timeout 10 ros2 topic echo /perception/diagnostics --once || true

echo "NOTE: no qualified /perception/cones message is expected while live metric calibration is unverified."

