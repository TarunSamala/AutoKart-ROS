#!/usr/bin/env bash
set -euo pipefail
project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$project_root"
export PYTHONPATH="$project_root${PYTHONPATH:+:$PYTHONPATH}"
fault="${1:-vcu_freeze}"
case "$fault" in
  steering_jam) scenario="configs/scenarios/vcu_steering_jam.yaml" ;;
  brake_cable_break) scenario="configs/scenarios/vcu_brake_cable_break.yaml" ;;
  stuck_motor_torque) scenario="configs/scenarios/vcu_motor_stuck_torque.yaml" ;;
  vcu_freeze) scenario="configs/scenarios/vcu_task_freeze.yaml" ;;
  physical_estop) scenario="configs/scenarios/vcu_physical_estop.yaml" ;;
  wet_surface_stop) scenario="configs/scenarios/vcu_wet_surface_stop.yaml" ;;
  *)
    echo "usage: $0 steering_jam|brake_cable_break|stuck_motor_torque|vcu_freeze|physical_estop|wet_surface_stop" >&2
    exit 2 ;;
esac
exec python3 -m kart_sim.cli run "$scenario" --output-dir runs/fault-demo
