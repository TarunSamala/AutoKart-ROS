# Developer report — autokart_hp60c

## Audit performed before writing any code

- Searched the repository for an existing standalone HP60C camera *node*. None exists. Camera
  acquisition today is embedded inside the live-mode capture thread of
  `kart/ros2_ws/src/autokart_nodes/autokart_perception/autokart_perception_ros/node.py`
  (`PerceptionNode._capture_loop`), which opens `autokart_perception.camera.HP60C` directly in the
  same process that also runs YOLO inference and publishes cones.
- Found the actual SDK-owning class: `autonomy_stack_phase1/phase_01_perception/src/autokart_perception/camera.py`
  (`HP60C`, ctypes wrapper around `libhp60c_bridge.so`, built from
  `autonomy_stack_phase1/phase_01_perception/native/hp60c_bridge.cpp`). It already does the
  timestamp/format/calibration validation the master architecture doc section 3 asks for
  (`STALE_FRAME`, `RGB_DEPTH_FRAME_MISMATCH`, `TIMESTAMP_MISMATCH`, `CALIBRATION_UNAVAILABLE`, etc.).
  Confirmed as the canonical algorithm source via `docs/decisions/CANONICAL_COMPONENTS.md`.
- Confirmed `autokart_msgs` (the canonical shared interfaces, `kart/ros2_ws/src/autokart_msgs`)
  already has `SensorDeliveryMetadata.msg`, which fits the camera status contract without inventing
  a new message type.
- Confirmed ROS 2 Humble + the existing `kart/ros2_ws` install are present and working in this dev
  environment (no Raspberry Pi, no HP60C hardware here).

## Decision: new standalone node, not a rewrite of the embedded capture thread

The embedded capture thread in the live perception node is a carefully written, already-working
design (single mutable "latest frame" slot, decoupled capture/inference, proven in its own test
suite). Per the mission statement ("do not rewrite working algorithms unnecessarily," "do not
replace existing implementations before auditing and testing them"), I did not touch it.

Instead, `autokart_ros_nodes/autokart_hp60c` is an **additive**, standalone node built for the
reusable library this session was asked to produce. It:
- reuses the exact same `HP60C` class (imported, not copied or reimplemented) for the `hp60c`
  backend,
- publishes the standard contract the master architecture doc specifies for node 01
  (`sensor_msgs/Image` RGB + depth, `sensor_msgs/CameraInfo` for each, status), which the embedded
  capture thread does not do today (it publishes RGB/depth on perception-specific topics, not
  CameraInfo, and does not run as an independently launchable node),
- adds a `synthetic` backend with zero external dependency, so the package is buildable/testable/
  runnable on any machine, which the embedded version is not (it requires the YOLO model + manifest
  + camera library + configuration to even start in live mode).

Migrating the live production pipeline to consume this node's topics instead of opening the camera
in-process is explicitly **out of scope for this change** — it is a separate, cross-node integration
decision (changes latency/threading assumptions in a path that is currently working) and belongs in
Phase 5 (cross-node ROS replay) of the master architecture doc's development order, reviewed on its
own.

## What was implemented

- `autokart_hp60c/backends.py`: `CameraBackend` protocol, `SyntheticBackend` (no dependency),
  `HP60CBackend` (thin adapter over the reused `HP60C` class, dependency path resolved at runtime via
  a parameter — never hardcoded).
- `autokart_hp60c/node.py`: `HP60CNode`, one background capture thread (same ownership pattern as
  the proven embedded design: exactly one SDK caller), publishes `/hp60c/rgb/image_raw`,
  `/hp60c/depth/image_raw`, `/hp60c/{rgb,depth}/camera_info`, `/hp60c/status`
  (`autokart_msgs/SensorDeliveryMetadata`). Fails closed and keeps reporting on any backend error;
  never publishes a fabricated frame.
- Full ament_python package (`package.xml`, `setup.py`, `setup.cfg`, `resource/`), launch file,
  two config profiles (`synthetic`, `live` — the latter with all hardware paths left blank and
  documented as placeholders, per "no hardcoded project-home paths" and "no guessed physical
  calibration").
- 11 tests: 7 unit (backend logic, no ROS needed), 1 ROS integration (real rclpy node + subscriber,
  checks actual message contents), 3 fault-injection (persistent disconnect, transient-fault
  recovery, missing-dependency fail-closed path).
- `export_node.sh`: bundles this node + its one real dependency (`autokart_msgs`) into a tarball.
- `CONTRACT.md`, `DEPENDENCIES.md`, `README.md` (this directory).

## Known gaps (explicit, not hidden)

- `backend: hp60c`'s actual `read()` call against real hardware is **not exercised** — no camera in
  this environment. Only its failure paths (missing path, import failure) are tested. See
  `VALIDATOR_REPORT.md` for exact IMPLEMENTED/TESTED/BLOCKED status per capability.
- No `PointCloud2` output (optional per spec, explicitly deferred, not forgotten).
- No static `base_link -> hp60c_*_optical_frame` TF publication — mount geometry is unmeasured.
- `CameraInfo.d` (distortion) is a placeholder `[0,0,0,0,0]`; the SDK calibration blob read by `HP60C`
  does not expose distortion coefficients in the fields currently parsed by `camera.py`. Documented
  in `CONTRACT.md`, not silently assumed to mean "no distortion."
