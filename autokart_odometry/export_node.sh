#!/usr/bin/env bash
# Packages autokart_odometry plus its one shared-interface dependency
# (autokart_msgs) into a tarball that can be dropped into any ROS 2 Humble
# workspace's src/ and built with: colcon build --packages-up-to autokart_odometry
#
# The reused algorithm package (kart/ros2_ws/src/autokart_localization) is
# deliberately NOT bundled: it is a runtime dependency resolved via the
# algorithm_package_path parameter (see DEPENDENCIES.md). Pass
# --with-algorithm to additionally copy it into the bundle under
# third_party/autokart_localization (NOT into src/, so colcon does not build
# or install it as a second localization node).
set -euo pipefail

with_algorithm=0
if [[ "${1:-}" == "--with-algorithm" ]]; then
  with_algorithm=1
  shift
fi
if [[ $# -ne 1 ]]; then
  echo "usage: $0 [--with-algorithm] <output_directory>" >&2
  exit 1
fi

out_dir="$1"
mkdir -p "$out_dir"

script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
repo_root=$(cd -- "$script_dir/../.." && pwd)
msgs_src="$repo_root/kart/ros2_ws/src/autokart_msgs"
algo_src="$repo_root/kart/ros2_ws/src/autokart_localization"

if [[ ! -d "$msgs_src" ]]; then
  echo "ERROR: expected autokart_msgs at $msgs_src, not found. Export aborted (no undisclosed" \
       "dependency substitution)." >&2
  exit 1
fi
if [[ $with_algorithm -eq 1 && ! -d "$algo_src" ]]; then
  echo "ERROR: --with-algorithm requested but $algo_src not found. Export aborted." >&2
  exit 1
fi

stage=$(mktemp -d)
trap 'rm -rf "$stage"' EXIT

excludes=(--exclude 'build' --exclude 'install' --exclude 'log' --exclude '__pycache__' --exclude '*.pyc'
          --exclude '.pytest_cache')
mkdir -p "$stage/src"
rsync -a "${excludes[@]}" "$script_dir/" "$stage/src/autokart_odometry/"
rsync -a "${excludes[@]}" "$msgs_src/" "$stage/src/autokart_msgs/"
if [[ $with_algorithm -eq 1 ]]; then
  mkdir -p "$stage/third_party"
  rsync -a "${excludes[@]}" "$algo_src/" "$stage/third_party/autokart_localization/"
fi

cat > "$stage/README_EXPORT.md" <<'EOF'
# autokart_odometry export bundle

Contents: src/autokart_odometry (this node), src/autokart_msgs (its one
shared ROS interface dependency, copied verbatim), and -- only if exported
with --with-algorithm -- third_party/autokart_localization (the reused
WheelOdometry source, kept OUT of src/ so colcon never builds it as a second
localization node).

Build:
    colcon build --packages-up-to autokart_odometry
    source install/setup.bash
    ros2 launch autokart_odometry odometry.launch.py

The node REQUIRES the reused algorithm: set algorithm_package_path to the
absolute path of third_party/autokart_localization (or any autokart_localization
checkout). Without it the node fails closed: /odom/status reports
ALGORITHM_PATH_NOT_CONFIGURED and no /odom or TF is published. There is no
local copy of the algorithm. Tests honour AUTOKART_LOCALIZATION_PATH.
EOF

suffix=""
[[ $with_algorithm -eq 1 ]] && suffix="_with_algorithm"
tarball="$out_dir/autokart_odometry_export${suffix}_$(date -u +%Y%m%dT%H%M%SZ).tar.gz"
tar -czf "$tarball" -C "$stage" .
echo "Wrote $tarball"
