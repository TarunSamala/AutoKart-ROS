#!/usr/bin/env bash
set -euo pipefail
project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
run_dir="${1:-$project_root/artifacts/phase16/realtime_final}"
host="${AUTOKART_DASHBOARD_HOST:-127.0.0.1}"
port="${AUTOKART_DASHBOARD_PORT:-8000}"
telemetry="$run_dir/internal_vehicle_log.jsonl"
if [[ ! -f "$telemetry" ]]; then
  printf 'Phase 16 telemetry not found: %s\n' "$telemetry" >&2
  exit 2
fi
cd "$project_root"
printf 'AutoKart Phase 16 dashboard: http://%s:%s\n' "$host" "$port"
printf 'Evidence directory: %s\n' "$run_dir"
exec env PYTHONPATH=. python3 -m kart_sim.cli dashboard serve \
  "$telemetry" \
  --track configs/tracks/research_circle.yaml \
  --kart configs/karts/prototype_dynamic.yaml \
  --host "$host" \
  --port "$port" \
  --rate-hz 30
