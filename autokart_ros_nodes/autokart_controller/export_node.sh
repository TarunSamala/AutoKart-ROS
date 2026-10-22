#!/usr/bin/env bash
# Packages autokart_controller plus its one shared-interface dependency (autokart_msgs) into a
# tarball that can be dropped into any ROS 2 Humble workspace's src/ and built with:
# colcon build --packages-up-to autokart_controller
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
  --exclude '*.pyc' "$script_dir/" "$stage/src/autokart_controller/"
rsync -a --exclude 'build' --exclude 'install' --exclude 'log' --exclude '__pycache__' \
  --exclude '*.pyc' "$msgs_src/" "$stage/src/autokart_msgs/"

cat > "$stage/README_EXPORT.md" <<'EOF'
# autokart_controller export bundle

Contents: autokart_controller (this node) and autokart_msgs (its one shared ROS interface
dependency, copied verbatim from the source repository).

Build:
    colcon build --packages-up-to autokart_controller
    source install/setup.bash
    ros2 launch autokart_controller controller.launch.py \
      config_file:=$(ros2 pkg prefix autokart_controller)/share/autokart_controller/config/controller_builtin.yaml

This bundle does NOT include the kart_sim package (pursuit_backend: reused). That
remains a separately documented dependency -- see DEPENDENCIES.md inside
src/autokart_controller/. The default pursuit_backend: builtin configuration has no
further dependency and is what the bundled tests exercise. It also does not include
autokart_planner (node 08); test_pair_with_planner.py is skipped in this bundle.
EOF

tarball="$out_dir/autokart_controller_export_$(date -u +%Y%m%dT%H%M%SZ).tar.gz"
tar -czf "$tarball" -C "$stage" .
echo "Wrote $tarball"
