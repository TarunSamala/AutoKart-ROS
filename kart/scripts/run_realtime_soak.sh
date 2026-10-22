#!/usr/bin/env bash
set -euo pipefail
project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
duration_s="${1:-1800}"
output="${2:-$project_root/artifacts/phase16/soak/soak_results.json}"
cd "$project_root"
PYTHONPATH=. python3 -m kart_sim.phase16 soak \
  --duration-s "$duration_s" --output "$output"
