#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export PYTHONPATH="$ROOT/autonomy_stack_phase1/phase_01_perception/src:$ROOT/kart${PYTHONPATH:+:$PYTHONPATH}"
if (($# == 0)); then
  set -- --source sample --input "$ROOT/autokart_apps/examples/straight.jsonl"
fi
exec python3 -m autokart_perception.application "$@"
