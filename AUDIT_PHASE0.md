# Phase 0 — repository audit (for autokart_ros_nodes)

Performed before writing any node code, per the mission statement's Phase 0/19 instructions.

## Workspace duplication finding

Two ROS 2 workspaces contain packages with identical names:

- `kart/ros2_ws/src/` — **confirmed canonical** by this repository's own
  [`docs/decisions/CANONICAL_COMPONENTS.md`](../docs/decisions/CANONICAL_COMPONENTS.md) and
  [`PROJECT_INDEX.md`](../PROJECT_INDEX.md). Contains the active VCU bridge, wheel-encoder odometry,
  VCU safety supervisor, perception/mapping/planner/controller ROS adapters, and `autokart_msgs`.
- `autokart/pi/ros2_ws/src/` — a second set of packages with the same names
  (`autokart_controller`, `autokart_mapping`, `autokart_perception`, `autokart_planner`,
  `autokart_safety`, `autokart_vehicle_bridge`, `autokart_msgs`) that is **not referenced anywhere**
  in the canonical-components or repository-disposition docs.

Per the user's explicit decision this session, **`kart/ros2_ws` is treated as the sole source** for
this library going forward. `autokart/pi/ros2_ws` was not opened, read in detail, copied from, built
against, or modified — it is left exactly as found, flagged here as an unresolved duplication risk
for a future, separately-approved cleanup pass (see the main repository's own
`docs/decisions/REPOSITORY_DISPOSITION.md` for the precedent/process used for other archive
candidates in this repo).

## Node-count reality check (master architecture doc §0/§19)

`kart/ros2_ws/src/` contains 16 packages (`autokart_bringup`, `autokart_control`,
`autokart_dashboard_gateway`, `autokart_description`, `autokart_diagnostics`, `autokart_gazebo`,
`autokart_localization`, `autokart_msgs`, `autokart_nodes/{autokart_controller,autokart_mapping,
autokart_perception,autokart_planner}`, `autokart_planning`, `autokart_safety_bridge`,
`autokart_scenarios`, `autokart_sim_bridge`, `autokart_vcu_safety`, `autokart_vehicle_bridge`,
`autokart_vehicle_geometry`). This is a package count, not a verified running-node count (no
`ros2 node list` was run against a live launch in this session — no bringup launch was started as
part of this audit, per the architecture doc's "do not auto-open another camera or serial port
during audit" instruction and because no hardware is present to usefully launch against).

**No existing standalone node matches "node 01: camera-only driver publishing
`sensor_msgs/Image`+`CameraInfo`"** from the 12-node registry. Camera acquisition today is embedded
inside `kart/ros2_ws/src/autokart_nodes/autokart_perception/autokart_perception_ros/node.py`'s live-mode
capture thread, which opens `autokart_perception.camera.HP60C` directly in the same process as YOLO
inference. The actual SDK-owning class (`HP60C`, ctypes wrapper over `libhp60c_bridge.so`) lives in
`autonomy_stack_phase1/phase_01_perception/src/autokart_perception/camera.py` — confirmed as the
canonical algorithm source via `docs/decisions/CANONICAL_COMPONENTS.md`.

## Interfaces found (reused, not reinvented)

`kart/ros2_ws/src/autokart_msgs/msg/`: `ActuatorFeedback`, `AuthoritativeVehicleSnapshot`,
`BenchVcuStatus`, `Cone`, `ConeArray`, `GazeboFollowerState`, `MissionDebug`, `MotionCommand`,
`SafetyEnvelope`, `SensorDeliveryMetadata`, `VcuState`, `VehicleState`, `WheelState`, `WheelStates`.
See `INTERFACE_FREEZE_PHASE1.md` for which of these this library's first node reuses.

## Toolchain found available in this dev environment

- ROS 2 Humble at `/opt/ros/humble`, `colcon` present, `kart/ros2_ws` already built
  (`kart/ros2_ws/install/setup.bash` sources `autokart_msgs` successfully).
- No HP60C camera, no Raspberry Pi, no ESP32-S3/VCU connected. All hardware-dependent acceptance
  items in any node's validator report are explicitly `NOT RUN`, not assumed passing.

## Node 02 findings (autokart_perception_adapter)

- **Package-name collision avoided deliberately**: this repository already has TWO things named
  `autokart_perception` (the non-ROS algorithm package at
  `autonomy_stack_phase1/phase_01_perception/src/autokart_perception`, and the ROS wrapper ament
  package at `kart/ros2_ws/src/autokart_nodes/autokart_perception`). Node 02 in this library is
  therefore named `autokart_perception_adapter`, not `autokart_perception`, to avoid a third
  ament-package-name collision and a Python-import collision with the lazily-imported algorithm
  package. See `autokart_perception_adapter/reports/DEVELOPER_REPORT.md` for the full reasoning.
- **Monolithic algorithm finding**: `autonomy_stack_phase1/.../autokart_perception/pipeline.py`'s
  `Pipeline` class does not just detection+3D localization — its `Result` also carries Delaunay/
  Gabriel graph, gates, boundaries, centerline and curvature profile, i.e. it already overlaps with
  what the master architecture doc calls node 06 (mapping) and part of node 08 (planner curvature).
  This may or may not be the same geometry algorithm a concurrent workstream building node 06 from
  `kart/ros2_ws/src/autokart_nodes/autokart_mapping` is auditing — **not reconciled in this session**;
  flagged here so a future pass checks whether these are two independent implementations of
  overlapping geometry (a duplication risk analogous to the `autokart/pi/ros2_ws` finding above) or
  deliberately separate by design.

## Remaining work (not done this session, scoped out explicitly per the user's own pacing decision)

Nodes 03–12 (IMU, VCU bridge, odometry, mapping, mission, planner, controller, safety, dashboard
gateway, system monitor) are being built in parallel workstreams following this same audit-first,
reuse-not-rewrite, real-test discipline — see each node's own `reports/` once built. Each has
substantial existing, mature implementations already in `kart/ros2_ws/src/` (e.g. `autokart_vcu_safety`,
`autokart_localization`, `autokart_planning`) that must be read and audited with the same care given
to nodes 01/02 before deciding, per node, what is genuinely new-standalone-wrapper work versus what
would duplicate an already-working single-owner component.
