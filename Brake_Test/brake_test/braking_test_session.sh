#!/usr/bin/env bash
# Integrated, fail-closed Phase-1 braking qualification workflow.
#
# This is the single operator entry point for the complete braking chain:
# HP60C perception -> mapping -> BRAKING_TEST planner -> controller candidate ->
# safety supervisor -> VCU bridge -> measured braking -> integrated stop trigger.
#
# It never sets verification flags, changes commissioning values, flashes firmware,
# requests ARM, or bypasses the production wiring lock. Physical stages remain
# interactive and require the operator/observer confirmations in their underlying tools.
set -uo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"

SESSION_TS="${AUTOKART_SESSION_TS:-$(date -u +%Y%m%dT%H%M%SZ)}"
SESSION_DIR="$ROOT/results/braking_session_${SESSION_TS}"
STATE_FILE="$SESSION_DIR/state.txt"

usage() {
  cat <<'EOF'
Usage:
  brake_test/run.sh                 # guided stage menu
  brake_test/run.sh --stage N       # run one stage (0-9)
  brake_test/run.sh --status        # non-actuating snapshot + refreshed readiness report
  brake_test/run.sh --help

Stages:
  0  Software self-tests (no hardware)
  1  Start complete BRAKING_TEST graph + stationary camera-rate evidence
  2  Stationary HP60C depth/mount fit (interactive, no motion)
  3  Ground-speed/odometry calibration (hand push; ROS graph must be stopped)
  4  Live perception-to-VCU latency measurement (graph running)
  5  Direct measured braking test (physical motion; ROS graph must be stopped)
  6  Consolidated readiness report
  7  Runway calculation from measured inputs
  8  Integrated perception/planner/controller/VCU stop trigger (wheels off ground)
  9  First-run checklist (does not arm or move the kart)

Passing software or stationary stages never authorizes motion. Stage 5 can request
physical motion and retains its own CLEAR20/POWERCUT/OBSERVER confirmations.
EOF
}

ensure_session() {
  mkdir -p "$SESSION_DIR"
  touch "$STATE_FILE"
}

log_stage() {
  local stage="$1" status="$2"
  ensure_session
  printf '%s stage=%s status=%s\n' "$(date -u +%FT%TZ)" "$stage" "$status" >> "$STATE_FILE"
}

run_logged() {
  local stage="$1"; shift
  ensure_session
  local log="$SESSION_DIR/stage_${stage}.log"
  echo "--- stage $stage: $* ---" | tee -a "$log"
  if "$@" 2>&1 | tee -a "$log"; then
    log_stage "$stage" "PASS"
    return 0
  else
    local rc=$?
    log_stage "$stage" "FAIL(rc=$rc)"
    echo "STAGE $stage FAILED (exit $rc). See $log." >&2
    return "$rc"
  fi
}

graph_running() {
  local pid_file="$ROOT/.run/phase1/ros.pid" pid
  [[ -f "$pid_file" ]] || return 1
  pid="$(cat "$pid_file" 2>/dev/null || true)"
  [[ "$pid" =~ ^[0-9]+$ ]] && kill -0 "$pid" 2>/dev/null
}

require_graph_stopped_for_serial() {
  if graph_running || pgrep -af 'autokart_vehicle_bridge' >/dev/null 2>&1; then
    echo "NO-GO: the ROS vehicle bridge may own the VCU serial port." >&2
    echo "Run scripts/stop_phase1.sh, verify VCU SAFE, then repeat this stage." >&2
    return 2
  fi
}

stage_0_inner() {
  (cd "$ROOT/autonomy_stack_phase1/phase_01_perception" && \
    PYTHONDONTWRITEBYTECODE=1 PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONPATH="src:$ROOT/kart" \
    python3 -m pytest -p no:cacheprovider -q tests) || return 1
  (cd "$ROOT/kart" && PYTHONDONTWRITEBYTECODE=1 PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONPATH=. \
    python3 -m pytest -p no:cacheprovider -q tests) || return 1
  (cd "$ROOT" && PYTHONDONTWRITEBYTECODE=1 PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
    PYTHONPATH="autonomy_stack_phase1/phase_01_perception/src:kart/ros2_ws/src/autokart_vehicle_geometry:kart/ros2_ws/src/autokart_vehicle_bridge:kart/ros2_ws/src/autokart_nodes/autokart_planner:tools" \
    python3 -m pytest -p no:cacheprovider -q tools/test_*.py) || return 1
}

stage_0() { run_logged 0 stage_0_inner; }

stage_1() {
  echo "Starting the complete live BRAKING_TEST graph DISARMED: perception, mapping, planner,"
  echo "controller, safety supervisor, VCU bridge, run logger, and dashboard gateway."
  "$ROOT/scripts/start_phase1.sh" || return $?
  echo "Waiting 8 s for live topics before the stationary camera measurement."
  sleep 8
  run_logged 1 python3 tools/camera_rate_check.py --duration 20 --out-dir "$SESSION_DIR"
}

stage_2() {
  echo "STATIONARY ONLY: use tape-measured cone stations; this stage never writes calibration."
  run_logged 2 python3 tools/depth_check.py --fit-pitch --out-dir "$SESSION_DIR"
}

stage_3() {
  echo "HAND PUSH ONLY: traction isolated; this requires a real installed ground-speed encoder."
  require_graph_stopped_for_serial || return $?
  run_logged 3 python3 tools/encoder_calibration.py
}

stage_4() {
  if ! graph_running; then
    echo "NO-GO: Phase-1 graph is not running. Run stage 1 first." >&2
    return 2
  fi
  run_logged 4 python3 tools/latency_report.py --live --seconds 30 --out-dir "$SESSION_DIR"
}

stage_5() {
  echo "PHYSICAL MOTION STAGE: open 20 m strip, physical power cut, and second observer required."
  require_graph_stopped_for_serial || return $?
  local port
  read -r -p "VCU serial port [/dev/ttyACM0]: " port
  port="${port:-/dev/ttyACM0}"
  run_logged 5 python3 tools/braking_test.py "$port" --out-dir "$SESSION_DIR"
}

stage_6() {
  run_logged 6 python3 tools/readiness_report.py --out "$SESSION_DIR/readiness_report.json"
}

stage_7() {
  local v_run a_verified t_latency margin
  echo "Enter measured values only; do not substitute configured or simulated values."
  read -r -p "v_run (m/s): " v_run
  read -r -p "a_verified (m/s^2): " a_verified
  read -r -p "t_latency (s): " t_latency
  read -r -p "margin (m): " margin
  run_logged 7 python3 tools/runway_check.py --v-run "$v_run" --a-verified "$a_verified" \
    --t-latency "$t_latency" --margin "$margin"
}

stage_8() {
  if ! graph_running; then
    echo "NO-GO: Phase-1 graph is not running. Run stage 1 first." >&2
    return 2
  fi
  echo "WHEELS OFF GROUND. This observer sends no ARM/CMD; arm only through the approved dashboard flow."
  run_logged 8 python3 tools/stop_trigger_test.py
}

stage_9() { run_logged 9 "$ROOT/scripts/first_run.sh"; }

STAGE_NAMES=(
  "0 software self-tests (all suites, no hardware)"
  "1 complete BRAKING_TEST graph + stationary camera rate"
  "2 stationary HP60C depth/mount fit"
  "3 measured ground-speed/odometry calibration (hand push)"
  "4 end-to-end perception-to-VCU latency"
  "5 direct physical braking measurement"
  "6 consolidated readiness report"
  "7 measured runway calculation"
  "8 integrated stop-trigger chain (wheels off)"
  "9 first-run checklist (no automatic ARM)"
)

run_stage() {
  case "$1" in
    0) stage_0 ;; 1) stage_1 ;; 2) stage_2 ;; 3) stage_3 ;; 4) stage_4 ;;
    5) stage_5 ;; 6) stage_6 ;; 7) stage_7 ;; 8) stage_8 ;; 9) stage_9 ;;
    *) echo "Unknown stage $1" >&2; return 2 ;;
  esac
}

show_status() {
  echo "Integrated braking-test status (non-actuating; no graph start, ARM, flash, or command):"
  if [[ -e "${AUTOKART_VCU_PORT:-/dev/ttyACM0}" ]]; then
    echo "  VCU serial: PRESENT (${AUTOKART_VCU_PORT:-/dev/ttyACM0})"
  else
    echo "  VCU serial: ABSENT (${AUTOKART_VCU_PORT:-/dev/ttyACM0})"
  fi
  if graph_running; then echo "  Phase-1 graph: RUNNING"; else echo "  Phase-1 graph: STOPPED"; fi
  if [[ -f "$ROOT/models/yolo26s/best.onnx" ]]; then echo "  YOLO26s model: PRESENT"; else echo "  YOLO26s model: ABSENT"; fi
  if [[ -f "$ROOT/autonomy_stack_phase1/phase_01_perception/native/build-x86_64-linux-gnu/libhp60c_bridge.so" ]]; then
    echo "  HP60C native bridge: PRESENT"
  else
    echo "  HP60C native bridge: ABSENT"
  fi
  echo
  python3 "$ROOT/tools/readiness_report.py"
}

guided_menu() {
  ensure_session
  echo "Integrated AutoKart Phase-1 braking session"
  echo "Evidence directory: $SESSION_DIR"
  echo "Nothing here sets verified flags, flashes, arms, or releases the brake automatically."
  while true; do
    echo
    echo "Stages:"
    local name
    for name in "${STAGE_NAMES[@]}"; do echo "  $name"; done
    read -r -p "Enter stage 0-9, 's' to skip-and-log, 't' for status, or 'q' to quit: " choice
    case "$choice" in
      q|Q) echo "Session state saved in $STATE_FILE"; return 0 ;;
      t|T) show_status || true ;;
      s|S)
        read -r -p "Stage number to skip: " skip_n
        log_stage "$skip_n" "SKIPPED"
        ;;
      [0-9])
        if ! run_stage "$choice"; then
          echo "Stage $choice stopped fail-closed. Fix the reported condition before continuing." >&2
          return 1
        fi
        ;;
      *) echo "Unrecognized input." ;;
    esac
  done
}

case "${1:-}" in
  "") guided_menu ;;
  --stage)
    [[ $# -eq 2 ]] || { usage >&2; exit 2; }
    ensure_session
    echo "Evidence directory: $SESSION_DIR"
    run_stage "$2"
    ;;
  --status)
    [[ $# -eq 1 ]] || { usage >&2; exit 2; }
    show_status
    ;;
  --help|-h) usage ;;
  *) usage >&2; exit 2 ;;
esac
