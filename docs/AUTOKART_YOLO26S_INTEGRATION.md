# AutoKart YOLO26s Integration — Verification Report (Agent 3)

> **CORRECTION (2026-10-18, later):** statements in this document that no HP60C is attached are WRONG or STALE. An HP60C (USB `3482:6723`) is attached and streams RGB+depth at ~11-13 fps via the vendor SDK; the live Vehicle Dashboard was verified against real frames (see `ui_split/VALIDATION_MATRIX.md`). Items below marked NOT TESTED because "no camera" are now testable; those not re-run in `ui_split/VALIDATION_MATRIX.md` remain NOT TESTED. Calibration is still UNVERIFIED and no qualified metric geometry exists.

**Date (UTC):** 2026-10-18. All results below are from actually running the real exported model on this workstation (`Jarvis`, x86_64, CPU-only `onnxruntime` — no GPU), not read from source or assumed.

## TASK C1 — Model file verification

```
models/yolo26s/best.onnx   sha256: 425a89cef2eda9db9c84fbf30cad187e510510b95d7b7e9904003fd22cec1158
models/yolo26s/best.pt     sha256: 834abec3531b654d02d70b8879356cd44f572dacc12e8757f78bfcb3671d9281
```
Both match `models/yolo26s/best.json`'s recorded `model_sha256` / `source_checkpoint_sha256` exactly. **PASS.**

Manifest: `output_shape=[1,300,6]`, `output_columns=[x1,y1,x2,y2,score,class_id]` (end-to-end, NMS baked into the graph — not a raw `[1,84,8400]` export needing separate NMS), `classes=[blue, large_orange, orange, unknown, yellow]`, `status: "EXPERIMENTAL_HP60C_VALIDATION_PENDING"`, `vehicle_authorization: "DISABLED"`.

`detector.py`'s decode path for this exact manifest shape (`Yolo26OnnxDetector`, lines ~199–274) reads `row[0:4]` as absolute `x1,y1,x2,y2` and `row[5]` as `class_id` — **matches the manifest's declared columns exactly**, confirmed by reading the code against the manifest rather than assuming the 6-field layout described in the master prompt.

## TASK C2/C3 — Real inference, actual model, actual pipeline

`onnxruntime` was not installed in this environment; installed it (`pip install --user onnxruntime`, no source changes) to actually exercise the model rather than only read code.

**Noise-image control:** 480×640 random RGB noise → **0 detections.** Correct behavior — a trained detector should not fire on garbage input.

**Real cone image** (`datasets/arece_v6/valid/images/amz_00578_...jpg`, 2872×2328, an actual labeled track photo, not synthetic): **25 detections**, confidences 0.50–0.96, classes `blue`/`orange` only (the two classes present in this particular frame), spatially sensible (blue cones one side of the track, orange the other — a real boundary layout). This is the existing detection pipeline (`autokart_perception.detector.Yolo26OnnxDetector`) running unmodified — no second detection system was introduced, per the constraint.

## TASK C8 — Measured inference latency (CPU, this workstation)

10 consecutive calls on the same 480×640 image, CPU-only `onnxruntime`:

```
327.0, 319.8, 311.3, 264.5, 281.6, 270.1, 319.9, 217.0, 178.8, 236.1  (ms)
mean warm (excluding first): 266.6 ms  →  ~3.75 FPS
```

**Finding, reported honestly rather than smoothed over:** ~3.75 FPS CPU inference is almost certainly too slow for real-time obstacle avoidance at any meaningful kart operating speed — the RC-06 report's own illustrative braking-trigger formula assumes a bounded detection-to-braking latency `T`; 266 ms alone would already consume a large fraction of any reasonable `T` budget before depth association, geometry, or safety classification even run. This is a genuine speed/latency gap, not fixed here — flagged per the master prompt's "evaluate whether reliable sensor range and processing latency are sufficient... if not, report the speed-range incompatibility." No GPU/NPU was available to test on this workstation, and the actual Raspberry Pi target (ARM64) was not reachable from this session, so this number is a CPU-x86_64 reference point only, not the deployment figure — the Pi number requires the actual hardware (see Open Issues).

## TASK C4–C7 — Depth association, metric coordinates, missing-depth handling

Covered by the existing `depth.py`/`pipeline.py`/`shared_core.py` and their test suites (`test_depth.py`, `test_pipeline.py`, `test_shared_core.py`, all passing — see regression section below); not modified this session. Not independently re-verified against new recorded depth data in this pass — see Open Issues.

## TASK C9 — Generic (non-cone) obstacle detection

Not addressed in this pass. The existing pipeline is cone-classification-only; a depth-based generic-obstacle path (objects YOLO26s was never trained to recognize) remains a real, acknowledged gap, consistent with the master prompt's own warning that "a cone detector is not a general obstacle detector." This is scoped for a following session — see Open Issues.

## Two real environment defects found and fixed (not algorithm changes)

1. **Stale pip install shadowing the live source.** `~/.local/lib/python3.10/site-packages/autokart_perception` was a non-editable wheel installed 2026-10-14, missing every camera.py improvement from this project's RC-05/RC-06 work (no disconnect/timestamp-mismatch/calibration checks). Any plain `python3 -c "import autokart_perception"` silently ran the stale, less-safe copy. Fixed with `pip install --user -e autonomy_stack_phase1/phase_01_perception` (packaging-only, zero source changes). Verified: `camera.py`'s own test suite went from 2/7 passing to 7/7 passing once the editable install pointed at real source.
2. **`kart_sim` (the `control` optional dependency) was never installed**, so every code path touching `PurePursuitController` degraded to `CONTROL_DEPENDENCY_UNAVAILABLE` — correctly fail-safe behavior, but it meant 7 tests across `test_application_integration.py`, `test_live_mission.py`, and `test_shared_core.py` were failing for an environment reason, not an algorithm defect. Fixed with `pip install --user -e kart` (the `auto-kart-simulator` package, packaging-only). Verified: full `autonomy_stack_phase1/phase_01_perception` suite went from 156/163 to **163/163**.

## Regression, after both fixes

- `autonomy_stack_phase1/phase_01_perception/tests/`: **163/163 PASS** (was 156/163 before the two installs above — this session's true original baseline, not previously reported, since an earlier baseline pass in this session missed this test directory entirely).
- `kart/ros2_ws` gateway + safety_bridge classification tests: **33/33 PASS**, now resolvable without the `PYTHONPATH` workaround this session previously needed.

## Open issues (not fabricated as PASS)

| Item | Status |
|---|---|
| CPU inference speed (266 ms / 3.75 FPS) adequate for target operating speed | **FAIL/FLAGGED** — needs either model optimization (quantization, smaller input, ONNX Runtime execution-provider tuning) or GPU/NPU acceleration; not fixed in this pass |
| Raspberry Pi (ARM64) inference latency | **NOT TESTED** — no Pi reachable from this session |
| Depth association against new recorded depth data | **NOT TESTED** this pass — existing test suite coverage only |
| Generic (non-cone) obstacle detection | **NOT IMPLEMENTED** — acknowledged gap, not started |
| Real HP60C camera + live YOLO26s end-to-end | **NOT TESTED** — no camera attached to this sandbox |
