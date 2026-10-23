# External dependencies — autokart_mapping

## ROS package dependencies (ordinary colcon deps)

- `rclpy`, `nav_msgs` — ROS 2 Humble core.
- `autokart_msgs` — this repository's shared interface package (`kart/ros2_ws/src/autokart_msgs`).
  Uses `Cone`, `ConeArray`, `MissionDebug`. To reuse this node in another workspace, copy
  `autokart_msgs` alongside it (interface-only, no further dependency chain).
- `python3-numpy` — used transitively by the reused algorithm package (see below); declared directly
  since this package's own `requirements` on it are real (the reused code imports numpy).

## Algorithm dependency (always required for a valid output; no "synthetic" fallback)

Unlike `autokart_hp60c` (where a dependency-free `SyntheticBackend` fully substitutes for the camera
SDK), this node has no meaningful "fake algorithm": its entire purpose is to run the real,
already-existing Delaunay/Gabriel/gate/centerline/skidpad-fit algorithm, so there is nothing useful to
synthesize in its place. The package still **builds** and its ROS-adapter-only unit tests still pass
with zero dependency present (see `test/test_algorithm.py`'s path-not-found/import-failure cases); any
test that exercises real mapping output is skipped, not faked, when the dependency is absent (see
`test/test_engine.py`, `test/test_node_integration.py`).

At runtime (not build time), this node imports and reuses:

- `autokart_perception.application.CoreRuntime` (constructs `AlgorithmCore` with the package's pinned
  default configuration, `autokart_perception/data/perception.yaml`) and
  `autokart_perception.shared_core.{AlgorithmCore, PoseEstimate, fit_skidpad_topology}`, from
  `autonomy_stack_phase1/phase_01_perception/src/autokart_perception/` in the main `autonomous_kart`
  repository. That module in turn imports `.boundaries`, `.centerline`, `.curvature`, `.gabriel`,
  `.gates`, `.geometry`, `.safety`, `.types` — all pure Python + numpy, no native library, no camera,
  no GPU, no model file required for this node's own `mission="STRAIGHT_TRACK"` evaluation path.

**To reuse this node in a clean workspace:**
1. Copy (or otherwise make available) the `autokart_perception` algorithm package's `src/` directory
   (it is pure Python + numpy — no native build step, unlike HP60C's `libhp60c_bridge.so`).
2. Set the `algorithm_package_path` parameter in `config/mapping.yaml` to its actual path on that
   machine — never hardcode a `/home/mark1/...` path into source.

No path in this package's source code is hardcoded to this repository's location. If the dependency
path is wrong or missing, the node fails closed at startup (`DEPENDENCY_PATH_NOT_FOUND` /
`ALGORITHM_IMPORT_FAILED` on `/autokart_mapping/status`, republished every 100 ms), not silently.

## What was NOT verified in this session

- No physically surveyed cone course was available (no hardware/track in this environment). The
  skidpad ring fit and exit-corridor pairing are exercised only against synthetic/simulated cone
  layouts (see `test/test_engine.py`), matching the existing, reused adapter's own test scope. Residual
  accuracy against a real surveyed course is **NOT RUN** — see reports/VALIDATOR_REPORT.md.
- `backend: LIVE` odometry covariance/verification gating is implemented and unit-tested for its
  rejection paths, but no LIVE sensor pipeline exists in this environment to exercise the accepted
  path end to end.
