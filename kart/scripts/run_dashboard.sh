#!/usr/bin/env bash
set -euo pipefail
project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$project_root"
export PYTHONPATH="$project_root${PYTHONPATH:+:$PYTHONPATH}"
exec python3 -m kart_sim.cli dashboard launch \
  configs/scenarios/dashboard_research_lap.yaml \
  --host "${AUTOKART_HOST:-127.0.0.1}" \
  --port "${AUTOKART_PORT:-8000}"
