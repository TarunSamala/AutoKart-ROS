#!/usr/bin/env bash
set -euo pipefail
project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$project_root"
python3 -m pytest tests/realtime -v
PYTHONPATH=. python3 -m kart_sim.phase16 faults \
  --output artifacts/phase16/faults_under_load.json
if [[ "${AUTOKART_SKIP_NATIVE:-0}" != "1" ]]; then
  "$project_root/scripts/run_gazebo_realtime_sil.sh"
  PYTHONPATH=. python3 -m kart_sim.phase16 gate \
    --run-dir artifacts/phase16/realtime_full_lap \
    --output artifacts/phase16/realtime_full_lap/qualification_gate.json
fi
