#!/usr/bin/env bash
# Packages autokart_safety plus its one shared-interface dependency
# (autokart_msgs) into a tarball that can be dropped into any ROS 2 Humble
# workspace's src/ and built with: colcon build --packages-up-to autokart_safety
#
# This bundle does NOT include kart/ros2_ws/src/autokart_vcu_safety (the real
# safety-core dependency, loaded lazily via the vcu_safety_package_path
# parameter at runtime, never at build time). Without it, this node builds
# and launches but fails closed forever (state=UNAVAILABLE, arm never
# allowed) -- see DEPENDENCIES.md.
set -euo pipefail

if [[ $# -ne 1 ]]; then
  echo "usage: $0 <output_directory>" >&2
  exit 1
fi

out_dir="$1"
mkdir -p "$out_dir"

script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
repo_root=$(cd -- "$script_dir/../.." && pwd)
msgs_src="${AUTOKART_MSGS_SRC:-$repo_root/kart/ros2_ws/src/autokart_msgs}"

if [[ ! -d "$msgs_src" ]]; then
  echo "ERROR: expected autokart_msgs at $msgs_src, not found. Export aborted (no undisclosed" \
       "dependency substitution)." >&2
  exit 1
fi

stage=$(mktemp -d)
trap 'rm -rf "$stage"' EXIT

mkdir -p "$stage/src"
rsync -a --exclude 'build' --exclude 'install' --exclude 'log' --exclude '__pycache__' \
  --exclude '*.pyc' "$script_dir/" "$stage/src/autokart_safety/"
rsync -a --exclude 'build' --exclude 'install' --exclude 'log' --exclude '__pycache__' \
  --exclude '*.pyc' "$msgs_src/" "$stage/src/autokart_msgs/"

cat > "$stage/README_EXPORT.md" <<'EOF'
# autokart_safety export bundle

Contents: autokart_safety (this node) and autokart_msgs (its one shared ROS
interface dependency, copied verbatim from the source repository).

Build:
    colcon build --packages-up-to autokart_safety
    source install/setup.bash
    ros2 launch autokart_safety safety.launch.py \
      config_file:=$(ros2 pkg prefix autokart_safety)/share/autokart_safety/config/safety_synthetic.yaml

This bundle does NOT include kart/ros2_ws/src/autokart_vcu_safety (the real
VCU arming safety supervisor core this node reuses via a runtime, lazily
imported path parameter). Without it, the node builds, launches and publishes
/autokart/safety/status, but with state=UNAVAILABLE and arm_allowed=false
forever -- it never authorizes ARM. To exercise the real arming logic, also
copy kart/ros2_ws/src/autokart_vcu_safety into this workspace's src/ and set
vcu_safety_package_path to its path in a config override. See DEPENDENCIES.md
inside src/autokart_safety/ for the exact contract.

This node is a SOFTWARE arming authority only. It is not the physical E-stop
and does not replace one -- see CONTRACT.md inside src/autokart_safety/.
EOF

tarball="$out_dir/autokart_safety_export_$(date -u +%Y%m%dT%H%M%SZ).tar.gz"
tar -czf "$tarball" -C "$stage" .
echo "Wrote $tarball"
