#!/usr/bin/env bash
set -euo pipefail
project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$project_root"
export PYTHONPATH="$project_root${PYTHONPATH:+:$PYTHONPATH}"
event="${1:-}"
case "$event" in
  acceleration)
    track="configs/tracks/acceleration_event.yaml"; steps="${AUTOKART_STEPS:-1400}"; speed="${AUTOKART_SPEED:-6.0}" ;;
  skidpad)
    track="configs/tracks/skidpad_event.yaml"; steps="${AUTOKART_STEPS:-1800}"; speed="${AUTOKART_SPEED:-3.0}" ;;
  autocross|cone_track)
    track="configs/tracks/cone_track.yaml"; steps="${AUTOKART_STEPS:-2200}"; speed="${AUTOKART_SPEED:-3.0}" ;;
  trackdrive|endurance)
    track="configs/tracks/cone_track.yaml"; steps="${AUTOKART_STEPS:-10000}"; speed="${AUTOKART_SPEED:-3.0}" ;;
  emergency_brake_test)
    exec python3 -m kart_sim.cli run configs/scenarios/vcu_physical_estop_acceleration.yaml --output-dir runs/formula_student ;;
  *)
    echo "usage: $0 acceleration|skidpad|autocross|trackdrive|emergency_brake_test" >&2
    exit 2 ;;
esac
exec python3 -m kart_sim.cli fast --track "$track" --controller pure_pursuit --speed "$speed" --steps "$steps" --output "runs/formula_student/${event}.jsonl"
