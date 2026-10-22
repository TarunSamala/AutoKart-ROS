#!/usr/bin/env bash
set -euo pipefail
project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
run_dir="${1:-$project_root/artifacts/phase16/realtime_full_lap}"
output="${2:-$run_dir/qualification_gate.json}"
cd "$project_root"
PYTHONPATH=. python3 -m kart_sim.phase16 gate \
  --run-dir "$run_dir" --output "$output"
