#!/usr/bin/env bash
set -euo pipefail
project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
artifact_directory="${1:-$project_root/artifacts/gazebo_follower/full_lap}"
cd "$project_root"
python3 -m kart_sim.gazebo_full_lap report "$artifact_directory"
