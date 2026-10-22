#!/usr/bin/env bash
# Guided, resumable test session. Stages 0-9, logs saved under results/session_<ts>/. Each stage can be
# skipped or repeated; the menu re-shows after every stage. Stops immediately on any stage reporting a
# fault/abort (non-zero exit from a stage that is not an explicit "skip"). Sets no verified flag; writes
# no config. The operator must be present for every physical stage (3 onward).
set -uo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"

SESSION_TS="${AUTOKART_SESSION_TS:-$(date -u +%Y%m%dT%H%M%SZ)}"
SESSION_DIR="$ROOT/results/session_${SESSION_TS}"
mkdir -p "$SESSION_DIR"
STATE_FILE="$SESSION_DIR/state.txt"
touch "$STATE_FILE"
echo "Session log directory: $SESSION_DIR"

log_stage() {
  local stage="$1" status="$2"
  printf '%s stage=%s status=%s\n' "$(date -u +%FT%TZ)" "$stage" "$status" >> "$STATE_FILE"
}

run_logged() {
  local stage="$1"; shift
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
  echo "Starting the phase-1 graph if not already running, then measuring camera rate and rejections for 20 s."
  "$ROOT/scripts/start_phase1.sh" || true
  sleep 8
  run_logged 1 python3 tools/camera_rate_check.py --duration 20 --out-dir "$SESSION_DIR"
}

stage_2() { run_logged 2 python3 tools/depth_check.py --fit-pitch --out-dir "$SESSION_DIR"; }
stage_3() { echo "Stop the ROS vehicle bridge first (scripts/stop_phase1.sh) before this opens the serial port."
  run_logged 3 python3 tools/encoder_calibration.py; }
stage_4() { run_logged 4 python3 tools/latency_report.py --live --seconds 30 --out-dir "$SESSION_DIR"; }
stage_5() { echo "Open 20 m clear strip required. Shortest run time first; the tool enforces this itself."
  read -r -p "VCU serial port [/dev/ttyACM0]: " port; port="${port:-/dev/ttyACM0}"
  run_logged 5 python3 tools/braking_test.py "$port" --out-dir "$SESSION_DIR"; }
stage_6() { run_logged 6 python3 tools/readiness_report.py --out "$SESSION_DIR/readiness_report.json"; }
stage_7() {
  read -r -p "v_run (m/s): " v_run; read -r -p "a_verified (m/s^2): " a_verified
  read -r -p "t_latency (s): " t_latency; read -r -p "margin (m): " margin
  run_logged 7 python3 tools/runway_check.py --v-run "$v_run" --a-verified "$a_verified" \
    --t-latency "$t_latency" --margin "$margin"
}
stage_8() { echo "Wheels OFF THE GROUND. This stage never sends ARM/CMD itself; arm independently via the dashboard."
  run_logged 8 python3 tools/stop_trigger_test.py; }
stage_9() { run_logged 9 "$ROOT/scripts/first_run.sh"; }

STAGE_NAMES=(
  "0 software self-test (all suites, read-only)"
  "1 stationary camera rate + rejection table"
  "2 depth_check --fit-pitch"
  "3 encoder_calibration (hand push, 3 trials)"
  "4 latency_report"
  "5 braking_test (typed confirmations, open 20 m strip)"
  "6 readiness_report"
  "7 runway_check"
  "8 stop_trigger_test (wheels off)"
  "9 first_run"
)

run_stage() {
  local n="$1"
  case "$n" in
    0) stage_0 ;; 1) stage_1 ;; 2) stage_2 ;; 3) stage_3 ;; 4) stage_4 ;;
    5) stage_5 ;; 6) stage_6 ;; 7) stage_7 ;; 8) stage_8 ;; 9) stage_9 ;;
    *) echo "Unknown stage $n" >&2; return 2 ;;
  esac
}

echo "AutoKart guided test session. Nothing here sets a verified flag or arms the vehicle on its own."
while true; do
  echo
  echo "Stages:"
  for name in "${STAGE_NAMES[@]}"; do echo "  $name"; done
  read -r -p "Enter a stage number to run (0-9), 's' to skip-and-log, or 'q' to quit: " choice
  case "$choice" in
    q|Q) echo "Session state saved in $STATE_FILE"; exit 0 ;;
    s|S) read -r -p "Stage number to skip: " skip_n; log_stage "$skip_n" "SKIPPED"; continue ;;
    [0-9])
      if ! run_stage "$choice"; then
        echo "Stopping the session on stage $choice fault. Resume later with the same stages; nothing was auto-retried."
        exit 1
      fi
      ;;
    *) echo "Unrecognized input." ;;
  esac
done
