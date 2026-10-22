#!/usr/bin/env bash
set -euo pipefail
project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$project_root"
world="${1:-simulation/worlds/autokart_empty_test_world.sdf}"
if [[ "${world}" == "--validate-only" ]]; then
  python3 -m kart_sim.cli robot validate simulation/models/autokart/model.sdf
  for candidate in simulation/worlds/*.sdf; do
    python3 -m kart_sim.cli robot validate "$candidate"
  done
  exit 0
fi
if ! command -v gazebo >/dev/null 2>&1; then
  echo "NOT_EXECUTED_EXTERNAL_DEPENDENCY: Gazebo Classic executable is unavailable" >&2
  exit 3
fi
export GAZEBO_MODEL_PATH="$project_root/simulation/models${GAZEBO_MODEL_PATH:+:$GAZEBO_MODEL_PATH}"
export GAZEBO_RESOURCE_PATH="$project_root/simulation${GAZEBO_RESOURCE_PATH:+:$GAZEBO_RESOURCE_PATH}"
echo "Physics authority: internal; Gazebo role: visual_follower"
echo "Gazebo contains no vehicle-drive plugin. Start the state bridge separately after ROS workspace build."
if [[ "${AUTOKART_HEADLESS:-0}" == "1" ]]; then
  exec gzserver --verbose "$world"
fi
exec gazebo --verbose "$world"
