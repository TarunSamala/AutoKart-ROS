#!/usr/bin/env bash
# Starts the phase-1 graph (if not already running), requires system_check.sh GO, shows the pre-run
# checklist, and requires a typed confirmation. Sends no motion request and sets no verified flag.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
RUN="$ROOT/.run/phase1"
if [[ ! -f "$RUN/ros.pid" ]] || ! kill -0 "$(cat "$RUN/ros.pid" 2>/dev/null)" 2>/dev/null; then
  echo "Phase-1 graph not running: starting it with scripts/start_phase1.sh first."
  "$ROOT/scripts/start_phase1.sh"
  echo "Waiting for the graph to come up before checking readiness..."
  sleep 8
fi
gate_output="$("$ROOT/scripts/system_check.sh" 2>&1)" || gate_status=$?
printf '%s\n' "$gate_output"
if [[ "${gate_status:-0}" -ne 0 || "$gate_output" != *"FINAL: GO for operator preflight only"* ]]; then
  echo "NO-GO: first-run checklist refused because system_check.sh did not pass." >&2
  exit 2
fi
readiness_output="$(python3 "$ROOT/tools/readiness_report.py" 2>&1)" || readiness_status=$?
printf '%s\n' "$readiness_output"
if [[ "${readiness_status:-0}" -ne 0 || "$readiness_output" != *"READY FOR FIRST RUN"* ]]; then
  echo "NO-GO: first-run checklist refused because tools/readiness_report.py reported NOT READY." >&2
  exit 2
fi
active_gap="$(awk '/^planner stop_gap_from_bumper_m:/ {print $NF}' <<< "$gate_output" | tail -1)"
if ! python3 - "$active_gap" <<'PY'
import math, sys
try:
    gap = float(sys.argv[1])
    assert math.isfinite(gap) and gap >= 0.15
except (ValueError, IndexError, AssertionError):
    raise SystemExit(1)
PY
then
  echo "NO-GO: active live stop gap is unavailable or below the 0.15 m red zone." >&2
  exit 2
fi
cat <<EOF
PRE-RUN OPERATOR CHECKLIST — no ARM or throttle is sent by this script:
  20 m clear straight area; nobody in front of or behind the kart.
  Physical battery/supply disconnect within reach and a second observer present.
  Steering mechanically fixed straight and checked on the actual kart.
  Cone center on the marked stop line; tape the front-bumper-to-center gap.
  Active commissioning stop gap: ${active_gap} m. The 0.15 m red zone is not an approved target until measured.
  Confirm measured speed, braking, camera calibration, output polarity, and latency records.
EOF
read -r -p "After all physical checks, type CHECKED to record operator review: " answer
if [[ "$answer" != "CHECKED" ]]; then
  echo "NOT EXECUTED: operator review was not confirmed."
  exit 2
fi
echo "CHECKLIST REVIEWED. This script did not arm or move the kart; use the approved dashboard flow only."
