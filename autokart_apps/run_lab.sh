#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export PYTHONPATH="$ROOT/autonomy_stack_phase1/phase_01_perception/src:$ROOT/kart${PYTHONPATH:+:$PYTHONPATH}"
python3 -m autokart_perception.lab_api --host 127.0.0.1 --port 8767 &
api_pid=$!
trap 'kill "$api_pid" 2>/dev/null || true' EXIT INT TERM
cd "$ROOT/phase_sim"
npm run dev -- --host 127.0.0.1
