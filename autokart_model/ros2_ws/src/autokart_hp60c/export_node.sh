#!/usr/bin/env bash
# Packages autokart_hp60c plus its one shared-interface dependency
# (autokart_msgs) into a tarball that can be dropped into any ROS 2 Humble
# workspace's src/ and built with: colcon build --packages-up-to autokart_hp60c
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
if [[ ! -d "$msgs_src" && -d "$script_dir/../autokart_msgs" ]]; then
  msgs_src="$script_dir/../autokart_msgs"
fi

if [[ ! -d "$msgs_src" ]]; then
  echo "ERROR: expected autokart_msgs at $msgs_src, not found. Export aborted (no undisclosed" \
       "dependency substitution)." >&2
  exit 1
fi

stage=$(mktemp -d)
trap 'rm -rf "$stage"' EXIT

mkdir -p "$stage/src"
rsync -a --exclude 'build' --exclude 'install' --exclude 'log' --exclude '__pycache__' \
  --exclude '*.pyc' "$script_dir/" "$stage/src/autokart_hp60c/"
rsync -a --exclude 'build' --exclude 'install' --exclude 'log' --exclude '__pycache__' \
  --exclude '*.pyc' "$msgs_src/" "$stage/src/autokart_msgs/"

cat > "$stage/README_EXPORT.md" <<'EOF'
# autokart_hp60c export bundle

Contents: autokart_hp60c (this node) and autokart_msgs (its one shared ROS
interface dependency, copied verbatim from the source repository).

Build:
    colcon build --packages-up-to autokart_hp60c
    source install/setup.bash
    ros2 launch autokart_hp60c hp60c.launch.py \
      config_file:=$(ros2 pkg prefix autokart_hp60c)/share/autokart_hp60c/config/hp60c_synthetic.yaml

This bundle does NOT include the HP60C vendor SDK, native bridge library or
camera configuration file (backend: hp60c). Those remain a separately
documented hardware dependency — see DEPENDENCIES.md inside
src/autokart_hp60c/. The default `backend: synthetic` configuration has no
further dependency and is what the bundled tests exercise.
EOF

tarball="$out_dir/autokart_hp60c_export_$(date -u +%Y%m%dT%H%M%SZ).tar.gz"
tar -czf "$tarball" -C "$stage" .
echo "Wrote $tarball"
