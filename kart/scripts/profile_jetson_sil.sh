#!/usr/bin/env bash
set -euo pipefail
project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
"$project_root/scripts/check_jetson_environment.sh"
output="${1:-$project_root/artifacts/phase16/jetson}"
mkdir -p "$output"
if command -v tegrastats >/dev/null 2>&1; then
  tegrastats --interval 1000 --logfile "$output/tegrastats.log" &
  stats_pid=$!
  trap 'kill "$stats_pid" >/dev/null 2>&1 || true' EXIT
fi
"$project_root/scripts/run_gazebo_realtime_sil.sh" "$output/full_lap"
"$project_root/scripts/report_realtime_performance.sh" \
  "$output/full_lap" "$output/qualification_gate.json"
