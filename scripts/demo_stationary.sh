#!/usr/bin/env bash
# Stationary, no-motion demo: starts the phase-1 graph (DISARMED, zero commands ever sent by this
# script), then tells the operator to open Mission Control and move a cone by hand to watch the live
# cone distance, bumper-to-cone gap and BRAKE decision update. SIMULATED STOP - NO MOTION: the kart
# never moves in this demo; "BRAKE decision" is the real planner's output, but nothing acts on it.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
RUN="$ROOT/.run/phase1"
if [[ ! -f "$RUN/ros.pid" ]] || ! kill -0 "$(cat "$RUN/ros.pid" 2>/dev/null)" 2>/dev/null; then
  echo "Starting the phase-1 graph for the stationary demo..."
  "$ROOT/scripts/start_phase1.sh"
  sleep 8
fi
echo
echo "=== SIMULATED STOP - NO MOTION ==="
echo "The vehicle stays DISARMED. This script sends no ARM, CMD, DIS or RST and sets no verified flag."
echo "Open Mission Control (the phase-1 graph just printed its dashboard URL above, normally"
echo "http://127.0.0.1:5173/ops) and watch: live cone distance, bumper-to-cone gap, the planner's"
echo "BRAKE decision, and the 0.15 m red zone, all clearly labelled SIMULATED STOP - NO MOTION."
echo "Move a cone by hand in front of the camera and watch the numbers update in real time."
echo "Run scripts/system_check.sh at any point for the current GO/NO-GO reasons."
echo "Stop this demo with scripts/stop_phase1.sh when done."
