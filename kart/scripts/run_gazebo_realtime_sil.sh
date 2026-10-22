#!/usr/bin/env bash
set -euo pipefail
project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
exec "$project_root/scripts/run_gazebo_follower.sh" \
  --scenario "$project_root/configs/scenarios/gazebo_realtime_sil_full_lap.yaml" \
  --preset "$project_root/configs/presets/gazebo_realtime_sil.yaml" \
  --output-dir "${1:-$project_root/artifacts/phase16/realtime_full_lap}"
