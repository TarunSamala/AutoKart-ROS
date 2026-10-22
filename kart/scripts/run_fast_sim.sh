#!/usr/bin/env bash
set -euo pipefail
project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$project_root"
export PYTHONPATH="$project_root${PYTHONPATH:+:$PYTHONPATH}"
exec python3 -m kart_sim.cli fast \
  --track configs/tracks/cone_track.yaml \
  --controller pure_pursuit \
  --speed 3.0 \
  --steps "${AUTOKART_STEPS:-1500}" \
  --output "${AUTOKART_OUTPUT:-runs/fast/cone_track.jsonl}"
