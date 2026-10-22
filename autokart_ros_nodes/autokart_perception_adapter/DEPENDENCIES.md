# External dependencies — autokart_perception_adapter

## ROS package dependencies

- `rclpy`, `sensor_msgs`, `message_filters` — ROS 2 Humble core/common, assumed present on any
  Humble install (`message_filters` ships with `ros-humble-message-filters`, confirmed importable in
  this dev environment).
- `autokart_msgs` — this repository's shared interface package. Only `ConeArray`/`Cone` and
  `SensorDeliveryMetadata` are used. Copy it alongside this node to reuse elsewhere (interface-only,
  no further dependency chain) — `export_node.sh` does this automatically.

## Algorithm dependency (`backend: algorithm` only)

With `backend: synthetic` (the default), there is no further dependency at all — builds and all
tests pass on a plain Ubuntu 22.04 + ROS 2 Humble machine with nothing else installed (`cv2`,
`onnxruntime` etc. not required).

With `backend: algorithm`, at runtime (not build time) this node imports and reuses:
- `autokart_perception.application.CoreRuntime`, `autokart_perception.pipeline.Pipeline`,
  `autokart_perception.detector.Yolo26OnnxDetector`, `autokart_perception.types.{Frame,Intrinsics}`,
  from `autonomy_stack_phase1/phase_01_perception/src/autokart_perception/` in the main
  `autonomous_kart` repository (confirmed canonical via `docs/decisions/CANONICAL_COMPONENTS.md`).
- which itself needs `numpy`, `opencv-python` (`cv2`), `onnxruntime` — all three confirmed importable
  in this dev environment, but not vendored or re-declared by THIS package; they are the algorithm
  package's own dependencies, inherited only when `backend: algorithm` is actually selected.
- a trained YOLO26s ONNX model + its manifest (`model_path`/`model_manifest` parameters) if live
  detection is wanted; without them, the underlying `Pipeline` runs in its own built-in synthetic
  geometry mode (not this node's dummy-image `standalone_timer` mode — a different, independent
  synthetic path that already existed in the algorithm package before this session).

**To reuse this node with real detection in a clean workspace:**
1. Make the `autokart_perception` algorithm package's `src/` directory available on the target
   machine (with its own Python dependencies: numpy, opencv-python, onnxruntime).
2. Set `algorithm_package_path`, `model_path`, `model_manifest` in `config/topics_live.yaml` to their
   actual paths — never hardcode a `/home/mark1/...` path into source.
3. Set `backend: algorithm`, `input_mode: topics`, and point `rgb_topic`/`depth_topic`/
   `camera_info_topic` at a running camera node (e.g. `autokart_hp60c`).

No path in this package's source is hardcoded to this repository's location. A wrong/missing
dependency path fails closed (`DEPENDENCY_PATH_NOT_FOUND`/`SDK_ADAPTER_IMPORT_FAILED` on
`/perception/status`, repeated every second, not a one-shot message), not silently.

## What was NOT verified in this session

No HP60C camera, no real cone track, no surveyed ground truth. The `algorithm` backend's *import and
call path* is tested against the real algorithm package in this dev environment (that package IS
present here — see `test/test_backends.py::test_algorithm_backend_real_import_runs_synthetic_geometry`,
which exercises it with the package's own built-in synthetic geometry, no model needed), but running
it against a real YOLO26s model and real camera frames is **NOT RUN** — no model path was supplied,
no camera attached. The detection-accuracy, localization-error, and precision/recall acceptance
items from the master architecture doc's node 02 row are **NOT RUN**, not claimed passing.
