#!/usr/bin/env bash
# Repeat reported AUT-03 local tests with no code modification.
set -u
ROOT="${AUTOKART_ROOT:-$HOME/Desktop/autonomous_kart}"
P="$ROOT/autonomy_stack_phase1/phase_01_perception"
K="$ROOT/kart"
LAB="$ROOT/phase_sim"
fail=0
printf '=== AUT-03 LOCAL REGRESSION ===\n'
if [[ -d "$P/tests" && -d "$P/src" ]]; then
 (cd "$P" && PYTHONPATH="src:$K" python3 -m pytest -p no:cacheprovider -q tests) || fail=1
else echo 'BLOCKED: perception tests missing'; fail=1; fi
if [[ -d "$K/tests" ]]; then
 (cd "$K" && PYTHONPATH=. python3 -m pytest -p no:cacheprovider -q tests) || fail=1
else echo 'NOT_TESTED: kart tests missing'; fi
if [[ -f "$LAB/package.json" && -d "$LAB/node_modules" ]]; then
 (cd "$LAB" && npm run test -- --run) || fail=1
else echo 'NOT_TESTED: LAB dependencies absent (install intentionally not automatic)'; fi
if (( fail )); then echo 'AUT-03 BASELINE FAIL / BLOCKED'; exit 1; fi
echo 'AUT-03 BASELINE EXECUTED; review test output for any skipped suites.'
