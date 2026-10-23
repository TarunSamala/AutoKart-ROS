#!/usr/bin/env bash
# Packages autokart_system_monitor alone -- it has no shared-interface
# dependency on autokart_msgs (uses only standard diagnostic_msgs) and no
# path-parameterized algorithm dependency (resource sampling is optional
# psutil, imported lazily at runtime; heartbeat tracking is pure dynamic ROS
# graph introspection). Build with:
# colcon build --packages-select autokart_system_monitor
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
  --exclude '*.pyc' "$script_dir/" "$stage/src/autokart_system_monitor/"

cat > "$stage/README_EXPORT.md" <<'EOF'
# autokart_system_monitor export bundle

Contents: autokart_system_monitor only. No autokart_msgs dependency (uses
standard diagnostic_msgs only); no path-parameterized algorithm dependency.

Build:
    colcon build --packages-select autokart_system_monitor
    source install/setup.bash
    ros2 launch autokart_system_monitor system_monitor.launch.py \
      config_file:=$(ros2 pkg prefix autokart_system_monitor)/share/autokart_system_monitor/config/system_monitor_synthetic.yaml

Optional runtime dependency: `psutil` (CPU/RAM sampling). Not installed?
This node still builds, launches, and publishes -- it reports
resource sampling as UNAVAILABLE rather than crashing or fabricating a
reading. See DEPENDENCIES.md inside src/autokart_system_monitor/.

This node REPORTS only -- it has no safety authority and cannot gate, stop,
or authorize anything. See CONTRACT.md.
EOF

tarball="$out_dir/autokart_system_monitor_export_$(date -u +%Y%m%dT%H%M%SZ).tar.gz"
tar -czf "$tarball" -C "$stage" .
echo "Wrote $tarball"
