#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export PYTHONDONTWRITEBYTECODE=1 PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
export PYTHONPATH="$ROOT/autonomy_stack_phase1/phase_01_perception/src:$ROOT/kart${PYTHONPATH:+:$PYTHONPATH}"
python3 -m pytest -p no:cacheprovider -q \
  "$ROOT/autonomy_stack_phase1/phase_01_perception/tests/test_shared_core.py" \
  "$ROOT/autonomy_stack_phase1/phase_01_perception/tests/test_application_integration.py" \
  "$ROOT/autonomy_stack_phase1/phase_01_perception/tests/test_geometry.py" \
  "$ROOT/autonomy_stack_phase1/phase_01_perception/tests/test_gabriel.py" \
  "$ROOT/autonomy_stack_phase1/phase_01_perception/tests/test_gates.py" \
  "$ROOT/autonomy_stack_phase1/phase_01_perception/tests/test_track_geometry.py" \
  "$ROOT/autonomy_stack_phase1/phase_01_perception/tests/test_pipeline.py" \
  "$ROOT/autonomy_stack_phase1/phase_01_perception/tests/test_safety.py" \
  "$ROOT/kart/tests/unit/test_pure_pursuit_wheelbase.py" \
  "$ROOT/kart/tests/unit/test_autonomy.py" \
  "$ROOT/kart/tests/unit/test_formula_student.py"
cd "$ROOT/phase_sim"
npm run test -- --run
npm run build
