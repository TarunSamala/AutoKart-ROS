#!/usr/bin/env bash
set -euo pipefail
project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$project_root"
export PYTHONPATH="$project_root${PYTHONPATH:+:$PYTHONPATH}"
exec python3 -m kart_sim.cli multi-race \
  --track configs/tracks/cone_track.yaml \
  --agents "${AUTOKART_AGENTS:-3}" \
  --steps "${AUTOKART_STEPS:-500}"
