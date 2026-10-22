# Developer report — autokart_perception_adapter

## Audit performed before writing code

- Read the existing ROS perception wrapper in full:
  `kart/ros2_ws/src/autokart_nodes/autokart_perception/autokart_perception_ros/node.py` (375 lines).
  It embeds HP60C camera acquisition AND YOLO inference AND qualification gating in one process/node,
  via `self.core.sdk.process_frame(frame, pipeline=self.pipeline)` where `self.core = CoreRuntime()`.
- Read the underlying algorithm package in full:
  `autonomy_stack_phase1/phase_01_perception/src/autokart_perception/{application,pipeline,detector,
  synthetic,types,calibration_store,recording}.py`. Found that `pipeline.py`'s `Pipeline` class is
  **monolithic**: a single `Result` carries cones AND Delaunay/Gabriel graph AND gates AND boundaries
  AND centerline AND curvature profile all together — i.e. it already does perception *and* a large
  chunk of what the master architecture doc calls node 06 (mapping) and parts of node 08 (planner,
  curvature). This is a real cross-cutting finding, not specific to this node: the mapping agent
  (building node 06 in parallel, from the *different* `kart/ros2_ws/src/autokart_nodes/autokart_mapping`
  implementation) may be auditing a separate, non-identical mapping algorithm than this one. Recorded
  here and in the top-level `AUDIT_PHASE0.md` rather than silently reconciled.
- Confirmed `autokart_msgs/Cone.msg`/`ConeArray.msg` already fit the cone output contract; no new
  message type needed for this node's scope.

## Decision: standalone node, consumes topics, does not embed a camera

Per the mission statement's reuse-not-rewrite instruction, the embedded capture-thread perception
node in `kart/ros2_ws` was not touched. `autokart_perception_adapter` is additive: it subscribes to
RGB/depth/CameraInfo (by default `autokart_hp60c`'s topics from node 01 in this library) via
`message_filters.ApproximateTimeSynchronizer`, instead of opening the camera itself — this is the
actual node-02-vs-node-01 boundary the master architecture doc specifies, which the current
production code does not have (camera + perception are one process there).

**Naming decision, explicit because it matters**: the ament/Python package name here is
`autokart_perception_adapter`, NOT `autokart_perception`. Two existing packages already use
`autokart_perception` as a name (the algorithm package, and the ROS wrapper package in
`kart/ros2_ws/src/autokart_nodes/autokart_perception`). Reusing that name a third time would create a
real ament-package-name collision risk if this library and `kart/ros2_ws` were ever colcon-built in
the same workspace, and a Python-import collision risk between this node's own package and the
lazily-imported algorithm package if both were ever named identically. Chose a distinct name instead
of following the master doc's "ROS node name can be `autokart_perception`" suggestion literally,
because that suggestion predates discovering this specific repository already has two different
things named exactly that.

## What was implemented

- `backends.py`: `SyntheticConeSource` (zero dependency, deterministic 4-cone layout, matches node
  01's `SyntheticBackend` role) and `AlgorithmBackend` (thin, lazy-import adapter around the real
  `CoreRuntime`/`Pipeline`/`Yolo26OnnxDetector`, dependency path from a parameter, never hardcoded;
  reuses the SAME LIVE-provenance qualification logic — `LIVE_METRIC_CALIBRATION_UNVERIFIED` gating —
  already proven in the embedded ROS node's `qualified()` function, reimplemented minimally here
  since it's a small, pure boolean check, not a large algorithm).
- `node.py`: `PerceptionAdapterNode`, two input modes (`topics` for real integration,
  `standalone_timer` for zero-dependency portability/testing — mirrors node 01's design intent of
  "buildable/testable/launchable with nothing else running"), publishes `/perception/cones`
  (`autokart_msgs/ConeArray`, **only when qualified**, never an unqualified/diagnostic cone) and
  `/perception/status` (`autokart_msgs/SensorDeliveryMetadata`, every cycle, success or fault).
- Full ament_python package, launch file, two config profiles, 11 tests (7 unit incl. a real
  algorithm-package import test, 1 ROS integration, 3 fault-injection).
- `export_node.sh`, `CONTRACT.md`, `DEPENDENCIES.md`, `README.md`.

## Known gaps (explicit)

- `/perception/detections` (raw pre-qualification boxes) and `/perception/obstacles` (non-cone 3D
  obstacles) from the master doc's node 02 row are **not implemented this session** — scoped out to
  keep the two topics that are implemented (`cones`, `status`) solid and tested rather than four
  topics half-done. This is a deliberate reduction, stated here and in `CONTRACT.md`, not a gap
  discovered later.
- Real YOLO26s inference against a live camera is **NOT RUN** (no model path given in this session's
  tests, no camera hardware). Only the algorithm backend's import/call path is exercised (against the
  package's own built-in synthetic-geometry fallback, which runs when no detector model is given —
  this is a genuinely real code path in the existing package, not a fake stand-in I invented).
- Intrinsics come from `CameraInfo.k` only; distortion (`CameraInfo.d`) is read by nothing in this
  node (node 01 publishes a placeholder `[0,0,0,0,0]` anyway — see node 01's own CONTRACT.md).
