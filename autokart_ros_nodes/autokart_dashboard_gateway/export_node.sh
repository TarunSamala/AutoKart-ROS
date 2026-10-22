#!/usr/bin/env bash
# Packages autokart_dashboard_gateway (this standalone wrapper only; it has
# no autokart_msgs colcon dependency of its own -- autokart_msgs is imported
# at runtime by the real, reused gateway module once loaded) into a tarball
# that can be dropped into any ROS 2 Humble workspace's src/ and built with:
# colcon build --packages-select autokart_dashboard_gateway
#
# This bundle does NOT include kart/ros2_ws/src/autokart_dashboard_gateway
# (the real gateway implementation) or autonomy_stack_phase1/phase_01_perception
# (its transitive autokart_perception dependency, required even without
# camera display -- see DEPENDENCIES.md). Without both, this node builds and
# launches but serves only a degraded /health endpoint with no ROS node and
# no actuation path at all.
set -euo pipefail

if [[ $# -ne 1 ]]; then
  echo "usage: $0 <output_directory>" >&2
  exit 1
fi

out_dir="$1"
mkdir -p "$out_dir"

script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)

stage=$(mktemp -d)
trap 'rm -rf "$stage"' EXIT

mkdir -p "$stage/src"
rsync -a --exclude 'build' --exclude 'install' --exclude 'log' --exclude '__pycache__' \
  --exclude '*.pyc' "$script_dir/" "$stage/src/autokart_dashboard_gateway/"

cat > "$stage/README_EXPORT.md" <<'EOF'
# autokart_dashboard_gateway export bundle

Contents: autokart_dashboard_gateway (this standalone wrapper node) only.

Build:
    colcon build --packages-select autokart_dashboard_gateway
    source install/setup.bash
    ros2 launch autokart_dashboard_gateway dashboard_gateway.launch.py \
      config_file:=$(ros2 pkg prefix autokart_dashboard_gateway)/share/autokart_dashboard_gateway/config/dashboard_gateway_synthetic.yaml

With the synthetic/default config (no gateway_package_path or
perception_package_path set), this serves ONLY a degraded /health endpoint
(gateway_available: false) -- no ROS node, no publisher, no actuation path
of any kind.

To exercise the real Mission Control gateway, also copy:
  - kart/ros2_ws/src/autokart_dashboard_gateway  (gateway_package_path)
  - autonomy_stack_phase1/phase_01_perception/src (perception_package_path,
    REQUIRED even with enable_camera_feed: false -- see DEPENDENCIES.md)
  - kart/ros2_ws/src/autokart_msgs (needed by the real gateway's own
    make_ros_node() at call time, for mode: "live")
into this workspace's src/, set both path parameters in a config override,
and rebuild/relaunch.

This node is READ-ONLY by design: the only effect a browser can have on the
vehicle is a bounded, whitelisted request string evaluated by
autokart_safety (node 10), never a direct actuator command. See CONTRACT.md
and reports/SENIOR_REVIEWER_REPORT.md inside src/autokart_dashboard_gateway/.
EOF

tarball="$out_dir/autokart_dashboard_gateway_export_$(date -u +%Y%m%dT%H%M%SZ).tar.gz"
tar -czf "$tarball" -C "$stage" .
echo "Wrote $tarball"
