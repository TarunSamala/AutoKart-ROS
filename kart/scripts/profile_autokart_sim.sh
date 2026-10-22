#!/usr/bin/env bash
set -euo pipefail
project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
output_dir="${1:-$project_root/artifacts/phase16/profile}"
mkdir -p "$output_dir"
cd "$project_root"
PYTHONPATH=. python3 -m kart_sim.phase16 host \
  --output "$output_dir/host_profile.json"
PYTHONPATH=. python3 -m kart_sim.phase16 physics-rates \
  --output "$output_dir/physics_rate_qualification.json"
printf 'Phase 16 profile artifacts: %s\n' "$output_dir"
