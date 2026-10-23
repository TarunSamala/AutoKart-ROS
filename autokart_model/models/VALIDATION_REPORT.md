# Validation Report

Date: 2026-10-22

## Copy and provenance validation

- `SOURCE_MANIFEST.csv`: 130 organized artifact rows; 130 have verified source/hash provenance; 0 unresolved.
- Google Drive: 56/56 downloaded files match a second authenticated source stream by SHA-256; 0 mismatches.
- Duplicate detection: 9 SHA-256 groups, 13 extra copies across run-local contexts/original filenames.
- Original active model hashes were re-read after organization and remain unchanged.

## Model load tests

- ONNX: 7 physical files (6 unique hashes), all passed `onnx.checker`, ONNX Runtime session construction, and zero-input CPU inference.
- PyTorch: 16 physical checkpoints, all passed Ultralytics deserialization/model construction.
- NCNN: required `model.ncnn.param`, `model.ncnn.bin`, `metadata.yaml`, and `model_ncnn.py` are present. Runtime load was not executed because the NCNN Python module is not installed.

These are load/smoke checks, not accuracy or performance qualification. Detailed results are in `MODEL_LOAD_RESULTS.json`.

## Derived graphs

Twelve plots were reconstructed from four actual `results.csv` files: losses, metrics, and learning rates for each run. Each plot title states `RECONSTRUCTED FROM results.csv`. No PR curve, confusion matrix, or metric was synthesized when the required data were absent.

## Existing perception regression tests

Command:

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
PYTHONPATH=autonomy_stack_phase1/phase_01_perception/src \
python3 -m pytest -q \
  autonomy_stack_phase1/phase_01_perception/tests/test_detector.py \
  autonomy_stack_phase1/phase_01_perception/tests/test_live_mission.py
```

Result: **19 passed in 3.50 s**.

## Runtime path integrity

- ROS launch still references `models/yolo26s/best.onnx` and `best.json`.
- Live Mission default still resolves ConeNet v1 from `training_runs/conenet_v1_20261014_114447`.
- ConeSlayer rollback still resolves from its original perception-model directory.
- No source configuration or algorithm was changed.

No perception service was running at final inspection. “Active model” therefore means configured runtime path, not a currently executing process.
