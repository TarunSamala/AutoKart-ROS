#!/usr/bin/env bash
# One disarmed HP60C/replay/simulation diagnostic dashboard on Jarvis.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PYTHON="${AUTOKART_PYTHON:-python3}"
if [[ ! -f "$ROOT/phase_sim/dist/index.html" ||
      "$ROOT/phase_sim/src/live/LiveMissionControl.tsx" -nt "$ROOT/phase_sim/dist/index.html" ||
      "$ROOT/phase_sim/src/live/liveMissionControl.css" -nt "$ROOT/phase_sim/dist/index.html" ||
      "$ROOT/phase_sim/src/main.tsx" -nt "$ROOT/phase_sim/dist/index.html" ]]; then
  (cd "$ROOT/phase_sim" && npm run build)
fi
export PYTHONPATH="$ROOT/autonomy_stack_phase1/phase_01_perception/src:$ROOT/kart${PYTHONPATH:+:$PYTHONPATH}"
exec "$PYTHON" -m autokart_perception.live_mission "$@"
