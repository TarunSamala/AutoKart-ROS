#!/usr/bin/env bash
# Packages autokart_perception_adapter plus its one shared-interface dependency
# (autokart_msgs) into a tarball droppable into any ROS 2 Humble workspace's
# src/ and built with: colcon build --packages-up-to autokart_perception_adapter
set -euo pipefail

if [[ $# -ne 1 ]]; then
  echo "usage: $0 <output_directory>" >&2
  exit 1
fi

out_dir="$1"
mkdir -p "$out_dir"

script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
repo_root=$(cd -- "$script_dir/../.." && pwd)
msgs_src="$repo_root/kart/ros2_ws/src/autokart_msgs"

if [[ ! -d "$msgs_src" ]]; then
  echo "ERROR: expected autokart_msgs at $msgs_src, not found. Export aborted (no undisclosed" \
       "dependency substitution)." >&2
  exit 1
fi

stage=$(mktemp -d)
trap 'rm -rf "$stage"' EXIT

mkdir -p "$stage/src"
rsync -a --exclude 'build' --exclude 'install' --exclude 'log' --exclude '__pycache__' \
  --exclude '*.pyc' --exclude '.pytest_cache' "$script_dir/" "$stage/src/autokart_perception_adapter/"
rsync -a --exclude 'build' --exclude 'install' --exclude 'log' --exclude '__pycache__' \
  --exclude '*.pyc' "$msgs_src/" "$stage/src/autokart_msgs/"

cat > "$stage/README_EXPORT.md" <<'EOF'
# autokart_perception_adapter export bundle

Contents: autokart_perception_adapter (this node) and autokart_msgs (its one
shared ROS interface dependency, copied verbatim).

Build:
    colcon build --packages-up-to autokart_perception_adapter
    source install/setup.bash
    ros2 launch autokart_perception_adapter perception_adapter.launch.py \
      config_file:=$(ros2 pkg prefix autokart_perception_adapter)/share/autokart_perception_adapter/config/synthetic.yaml

This bundle does NOT include the real autokart_perception algorithm package
(YOLO26s/ONNX/pipeline), the ONNX model, or ROS message_filters' runtime
beyond what ROS 2 Humble already ships. backend: synthetic (the default) has
no further dependency and is what the bundled tests exercise. backend:
algorithm requires the real autokart_perception package to be made available
separately and its path set via the algorithm_package_path parameter -- see
DEPENDENCIES.md inside src/autokart_perception_adapter/.
EOF

tarball="$out_dir/autokart_perception_adapter_export_$(date -u +%Y%m%dT%H%M%SZ).tar.gz"
tar -czf "$tarball" -C "$stage" .
echo "Wrote $tarball"
