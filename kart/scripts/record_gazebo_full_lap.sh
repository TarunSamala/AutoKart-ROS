#!/usr/bin/env bash
set -euo pipefail
project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$project_root"
export AUTOKART_RECORD_ROSBAG=1
exec ./scripts/run_gazebo_follower.sh "$@"
