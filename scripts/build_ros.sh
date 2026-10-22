#!/usr/bin/env bash
# Build kart/ros2_ws into a stable repo-local isolated dir ($ROOT/.ros_build/{build,install,log}).
# Never touches the in-tree kart/ros2_ws/{build,install,log} (they hold stale absolute-path caches).
# Usage: scripts/build_ros.sh [extra colcon build args, e.g. --packages-select autokart_dashboard_gateway]
set -euo pipefail
# shellcheck source=autokart_env.sh
source "$(dirname "${BASH_SOURCE[0]}")/autokart_env.sh"

WS_SRC="$ROOT/kart/ros2_ws/src"
[[ -d "$WS_SRC" ]] || { ak_err "missing $WS_SRC"; exit 1; }
command -v colcon >/dev/null || { ak_err "colcon not found (apt install python3-colcon-common-extensions)"; exit 1; }
[[ -f "$AUTOKART_ROS_DISTRO_SETUP" ]] || { ak_err "missing $AUTOKART_ROS_DISTRO_SETUP"; exit 1; }

set +u
# shellcheck disable=SC1090
source "$AUTOKART_ROS_DISTRO_SETUP"
set -u

mkdir -p "$AUTOKART_ROS_BUILD_DIR"/{build,install,log}
ak_info "building $WS_SRC -> $AUTOKART_ROS_BUILD_DIR"
start=$(date +%s)
# Build from a neutral cwd so colcon never picks up stray paths.
cd "$AUTOKART_ROS_BUILD_DIR"
colcon --log-base "$AUTOKART_ROS_BUILD_DIR/log" build \
  --base-paths "$WS_SRC" \
  --build-base "$AUTOKART_ROS_BUILD_DIR/build" \
  --install-base "$AUTOKART_ROS_BUILD_DIR/install" \
  --event-handlers console_cohesion- console_direct- \
  "$@"
ak_info "build OK in $(( $(date +%s) - start ))s; install: $AUTOKART_ROS_INSTALL"
ak_info "packages: $(ls "$AUTOKART_ROS_INSTALL" | grep -v -E '\.(bash|sh|zsh|ps1|py)$|COLCON_IGNORE' | tr '\n' ' ')"
