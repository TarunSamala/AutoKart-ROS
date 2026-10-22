#!/usr/bin/env bash
# Single non-interactive CI entry point for this repo's fast, hermetic unit-test suites: perception,
# kart simulator, the tools/ suite, the perception ROS adapter, and the phase_sim frontend (vitest +
# tsc). Starts nothing, opens no serial/camera device, sends no ARM/CMD. Prints a PASS/FAIL table and
# exits non-zero if any suite failed -- suitable as a CI job step.
#
# Also covers the three ROS packages DEMO DRIVE touches (autokart_vcu_safety, autokart_vehicle_
# bridge, autokart_dashboard_gateway) -- pure-logic/aiohttp tests only, no rclpy node spin, no
# serial port, no camera.
#
# Deliberately NOT covered here (each already has its own dedicated, slower entry point -- running
# them all on every CI push would be wasteful and some need hardware that CI does not have):
#   - the fresh ROS overlay colcon build (scripts/build_ros.sh)
#   - vcu_firmware's host test suite (`make -C vcu_firmware test`) and `pio run`
#   - anything in scripts/test_session.sh stage 1 onward (needs the operator physically present)
# scripts/test_session.sh's own stage 0 covers a subset of what this script covers (perception, kart,
# tools) using the same invocations; this script adds the ROS-adapter and frontend suites and is meant
# for CI, not an interactive session.
set -uo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"

declare -a NAMES=() RESULTS=()
# A suite function returns 0 for pass, 1+ for fail, and SKIP_RC specifically to mean "did not run
# because a prerequisite was missing" -- shown as SKIPPED, never counted as a pass it never earned.
SKIP_RC=77
run_suite() {
  local name="$1"; shift
  echo "=== $name ==="
  "$@"
  local rc=$?
  if (( rc == 0 )); then
    NAMES+=("$name"); RESULTS+=("PASS")
  elif (( rc == SKIP_RC )); then
    NAMES+=("$name"); RESULTS+=("SKIPPED")
  else
    NAMES+=("$name"); RESULTS+=("FAIL(rc=$rc)")
  fi
}

perception_suite() {
  (cd "$ROOT/autonomy_stack_phase1/phase_01_perception" && \
    PYTHONDONTWRITEBYTECODE=1 PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONPATH="src:$ROOT/kart" \
    python3 -m pytest -p no:cacheprovider -q tests)
}
kart_suite() {
  (cd "$ROOT/kart" && PYTHONDONTWRITEBYTECODE=1 PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONPATH=. \
    python3 -m pytest -p no:cacheprovider -q tests)
}
tools_suite() {
  (cd "$ROOT" && PYTHONDONTWRITEBYTECODE=1 PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
    PYTHONPATH="autonomy_stack_phase1/phase_01_perception/src:kart/ros2_ws/src/autokart_vehicle_geometry:kart/ros2_ws/src/autokart_vehicle_bridge:kart/ros2_ws/src/autokart_nodes/autokart_planner:tools" \
    python3 -m pytest -p no:cacheprovider -q tools/test_*.py)
}
perception_ros_adapter_suite() {
  local overlay="${AUTOKART_PHASE1_OVERLAY:-$ROOT/.ros_phase1_overlay/install-estop-full}"
  if [[ ! -f "/opt/ros/humble/setup.bash" || ! -f "$overlay/setup.bash" ]]; then
    echo "SKIPPED: ROS Humble or the built overlay ($overlay) is not available here" >&2
    return "$SKIP_RC"
  fi
  (
    set +u
    source /opt/ros/humble/setup.bash
    source "$overlay/setup.bash"
    set -u
    export PYTHONDONTWRITEBYTECODE=1 PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
    export PYTHONPATH="$ROOT/autonomy_stack_phase1/phase_01_perception/src:$ROOT/kart/ros2_ws/src/autokart_nodes/autokart_perception:${PYTHONPATH:-}"
    cd "$ROOT/kart/ros2_ws/src/autokart_nodes/autokart_perception"
    python3 -m pytest -p no:cacheprovider -q test
  )
}
demo_drive_ros_bundle_suite() {
  local overlay="${AUTOKART_PHASE1_OVERLAY:-$ROOT/.ros_phase1_overlay/install-estop-full}"
  if [[ ! -f "/opt/ros/humble/setup.bash" || ! -f "$overlay/setup.bash" ]]; then
    echo "SKIPPED: ROS Humble or the built overlay ($overlay) is not available here" >&2
    return "$SKIP_RC"
  fi
  (
    set +u
    source /opt/ros/humble/setup.bash
    source "$overlay/setup.bash"
    set -u
    export PYTHONDONTWRITEBYTECODE=1 PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
    export PYTHONPATH="$ROOT/kart/ros2_ws/src/autokart_vcu_safety:$ROOT/kart/ros2_ws/src/autokart_vehicle_bridge:$ROOT/kart/ros2_ws/src/autokart_dashboard_gateway:${PYTHONPATH:-}"
    for pkg in autokart_vcu_safety autokart_vehicle_bridge autokart_dashboard_gateway; do
      (cd "$ROOT/kart/ros2_ws/src/$pkg" && python3 -m pytest -p no:cacheprovider -q test) || exit 1
    done
  )
}
frontend_suite() {
  if [[ ! -d "$ROOT/phase_sim/node_modules" ]]; then
    echo "SKIPPED: phase_sim/node_modules not installed (run npm install in phase_sim first)" >&2
    return "$SKIP_RC"
  fi
  (cd "$ROOT/phase_sim" && npx vitest run) && (cd "$ROOT/phase_sim" && npx tsc -b)
}

run_suite "perception"              perception_suite
run_suite "kart simulator"          kart_suite
run_suite "tools"                   tools_suite
run_suite "perception ROS adapter"  perception_ros_adapter_suite
run_suite "demo drive ROS bundle"   demo_drive_ros_bundle_suite
run_suite "phase_sim frontend"      frontend_suite

echo
echo "=== SUMMARY ==="
failures=0
for i in "${!NAMES[@]}"; do
  printf '%-24s %s\n' "${NAMES[$i]}" "${RESULTS[$i]}"
  [[ "${RESULTS[$i]}" == PASS ]] || failures=$((failures + 1))
done
if (( failures > 0 )); then
  echo "RESULT: $failures suite(s) failed." >&2
  exit 1
fi
echo "RESULT: all suites passed."
exit 0
