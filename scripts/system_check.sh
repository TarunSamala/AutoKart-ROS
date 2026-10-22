#!/usr/bin/env bash
# Read-only phase-1 preflight. Does not open the serial port or request ARM/RST.
set -uo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
OVERLAY="${AUTOKART_PHASE1_OVERLAY:-$ROOT/.ros_phase1_overlay/install}"
reasons=()
no_go() { reasons+=("$1"); echo "FAIL: $1"; }
pass() { echo "PASS: $1"; }
if [[ -e "${AUTOKART_VCU_PORT:-/dev/ttyACM0}" ]]; then pass "serial device present"; else no_go "VCU serial device absent"; fi
if [[ ! -f "$OVERLAY/setup.bash" ]]; then
  no_go "separate ROS overlay absent: $OVERLAY"
  printf 'FINAL: NO-GO (%s reason(s))\n' "${#reasons[@]}"
  exit 1
fi
set +u
source /opt/ros/humble/setup.bash
source "$OVERLAY/setup.bash"
set -u
export PYTHONPATH="$ROOT/autonomy_stack_phase1/phase_01_perception/src:$ROOT/kart/ros2_ws/src/autokart_vehicle_geometry:${PYTHONPATH:-}"
export ROS_DOMAIN_ID="${AUTOKART_LIVE_ROS_DOMAIN_ID:-0}"
health="$(timeout 4 ros2 topic echo --once /autokart/vcu/health 2>/dev/null || true)"
if [[ "$health" == *'"dac_ok": true'* ]]; then pass "VCU dac_ok telemetry"; else no_go "VCU dac_ok absent or false"; fi
if [[ "$health" == *'"hello_fresh": true'* ]]; then pass "VCU HELLO fresh"; else no_go "VCU HELLO absent or stale"; fi
if [[ "$health" == *'"protocol_ok": true'* ]]; then pass "VCU protocol revision matches bridge"; else no_go "VCU protocol mismatch or absent HELLO"; fi
if [[ "$health" == *'"state": "SAFE"'* ]]; then pass "VCU SAFE"; else no_go "VCU not SAFE"; fi
if [[ "$health" == *'"estop_input_fitted": false'* ]]; then
  echo "AMBER: E-STOP INPUT NOT FITTED - MANUAL POWER CUT ONLY"
elif [[ "$health" == *'"estop_ok": true'* ]]; then pass "fitted E-stop status input OK"; else no_go "fitted E-stop status input not OK or telemetry absent"; fi
if timeout 4 ros2 topic echo --once /autokart/perception/rgb >/dev/null 2>&1 &&
   timeout 4 ros2 topic echo --once /autokart/perception/depth >/dev/null 2>&1; then
  pass "RGB and depth topics have live messages"
else no_go "RGB or depth topic has no live message"; fi
nodes="$(timeout 5 ros2 node list 2>/dev/null || true)"
for node in autokart_vehicle_bridge autokart_localization autokart_perception autokart_mapping autokart_planner autokart_controller autokart_vcu_safety autokart_dashboard_gateway; do
  if grep -qx "/$node" <<< "$nodes"; then pass "node $node"; else no_go "node $node absent"; fi
done
python3 -m autokart_vehicle_geometry.geometry 2>/dev/null || no_go "vehicle geometry not loadable"
if python3 "$ROOT/tools/validate_config_schema.py" >/dev/null 2>&1; then
  pass "config schema versions match"
else
  no_go "config schema version mismatch or missing (see tools/validate_config_schema.py)"
fi
for parameter in camera_to_bumper_m stop_gap_from_bumper_m brake_margin_m brake_min_mps2 t_latency lateral_margin_m; do
  value="$(timeout 3 ros2 param get /autokart_planner "$parameter" 2>/dev/null || true)"
  echo "planner $parameter: ${value:-UNAVAILABLE}"
  [[ -n "$value" ]] || no_go "planner parameter $parameter unavailable"
done
positive_param() {
  local node="$1" parameter="$2" value number
  value="$(timeout 3 ros2 param get "$node" "$parameter" 2>/dev/null || true)"
  echo "$node $parameter: ${value:-UNAVAILABLE}"
  number="${value##* }"
  if python3 -c 'import math,sys; value=float(sys.argv[1]); raise SystemExit(0 if math.isfinite(value) and value > 0 else 1)' "$number" 2>/dev/null; then
    pass "$parameter measured and configured"
  else
    no_go "$parameter unmeasured, zero, or unavailable"
  fi
}
positive_param /autokart_planner t_latency
if [[ "$health" == *'"speed_measured": true'* ]]; then
  pass "ground speed measured"
else
  no_go "no measured ground speed; 2 V is a DAC ceiling, not a speed limit"
fi
odom_verified="$(timeout 3 ros2 param get /autokart_mapping live_odom_verified 2>/dev/null || true)"
echo "mapping live_odom_verified: ${odom_verified:-UNAVAILABLE}"
[[ "$odom_verified" == *"True"* || "$odom_verified" == *"true"* ]] || no_go "no qualified encoder-free pose source for mapping/controller"
for flag in calibration_verified vehicle_config_verified braking_verified; do
  value="$(timeout 3 ros2 param get /autokart_vcu_safety "$flag" 2>/dev/null || true)"
  echo "safety $flag: ${value:-UNAVAILABLE}"
  [[ "$value" == *"True"* || "$value" == *"true"* ]] || no_go "$flag is not true"
done
safety="$(timeout 4 ros2 topic echo --once /autokart/safety/status 2>/dev/null || true)"
if [[ "$safety" == *'"startup_ready": true'* ]]; then
  pass "supervisor ready with manual power cut confirmed for this ARM"
else no_go "supervisor stages not ready or manual power cut not confirmed"; fi
if ((${#reasons[@]})); then
  printf 'FINAL: NO-GO (%s reason(s))\n' "${#reasons[@]}"
  printf ' - %s\n' "${reasons[@]}"
  exit 1
fi
echo "FINAL: GO for operator preflight only. Physical battery/supply disconnect is the only emergency stop."
