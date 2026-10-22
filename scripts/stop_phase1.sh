#!/usr/bin/env bash
# Stop phase-1 processes. Vehicle bridge sends DIS on ROS shutdown.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
RUN="$ROOT/.run/phase1"
for name in ui ros; do
  file="$RUN/$name.pid"
  [[ -f "$file" ]] || continue
  pid="$(cat "$file")"
  if [[ "$pid" =~ ^[0-9]+$ ]] && kill -0 "$pid" 2>/dev/null; then
    kill -INT -- "-$pid" 2>/dev/null || kill -INT "$pid" 2>/dev/null || true
    for _ in {1..50}; do kill -0 "$pid" 2>/dev/null || break; sleep 0.1; done
    if kill -0 "$pid" 2>/dev/null; then kill -TERM -- "-$pid" 2>/dev/null || true; fi
  fi
done
echo "Phase-1 software stop requested. Confirm VCU SAFE and use the physical battery/supply cut if needed."
