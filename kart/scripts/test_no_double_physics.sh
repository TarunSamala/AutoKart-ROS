#!/usr/bin/env bash
set -euo pipefail
project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$project_root"
python3 -m kart_sim.gazebo_full_lap no-double-physics \
  configs/scenarios/gazebo_follower_full_lap.yaml \
  --output-dir artifacts/gazebo_follower/no_double_physics
