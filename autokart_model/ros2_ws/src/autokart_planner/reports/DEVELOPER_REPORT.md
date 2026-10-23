# Developer report — autokart_planner

## Audit performed before writing any code

- Read `kart/ros2_ws/src/autokart_nodes/autokart_planner/autokart_planner/{node.py,mission.py}` in
  full. Found that `mission.py`'s `PlannerEngine` is almost entirely a **mission state machine**
  (SKIDPAD gate-crossing progress, BRAKING_TEST corridor/dead-reckoning logic) wrapped around a
  stop-envelope check — not a general curvature/speed-profile planner. Per the mission statement,
  that FSM is being extracted into a separate node (06/07) by another agent this session, so this
  node does not import or depend on `mission.py`.
- Traced the actual underlying algorithms `mission.py` depends on:
  - `autokart_perception.curvature.profile_centerline`
    (`autonomy_stack_phase1/phase_01_perception/src/autokart_perception/curvature.py`) — discrete
    three-point signed-circle curvature fit, the real curvature algorithm already used by
    `kart/ros2_ws`'s `autokart_mapping`.
  - `autokart_perception.shared_core.BrakeCapability.stopping_distance` — exactly
    `v*t_latency + v^2/(2a) + margin`, i.e. the master architecture doc's stop-envelope formula,
    already implemented and unit-tested elsewhere.
  - `autokart_vehicle_geometry.geometry.GEOMETRY` (`kart/ros2_ws/src/autokart_vehicle_geometry`) —
    the single source of measured bumper/camera geometry
    (`kart/ros2_ws/src/autokart_bringup/config/vehicle_geometry.yaml`). Confirmed this file has
    **no wheelbase, braking-deceleration, or lateral-acceleration figures** — only camera/bumper
    geometry. Those three constants genuinely do not exist anywhere in this repository as measured
    values (matches `PROJECT_INDEX.md`: "stopping performance remain unknown or unverified").
- Confirmed `autokart_msgs/SafetyEnvelope.msg` (unused by the live `kart/ros2_ws` stack today) is
  a close, standard fit for "stop-envelope/validity status" output, and `nav_msgs/Path` for the
  candidate path — both reused as-is, no new message types needed for those two. See CONTRACT.md
  for why the third output (speed profile) ended up on `std_msgs/String`, not
  `autokart_msgs/MissionDebug`: **`MissionDebug` is present in the source tree but not built into
  the already-built `kart/ros2_ws/install` this library's README instructs sourcing** — verified
  directly (`python3 -c "import autokart_msgs.msg as m; print(dir(m))"` after sourcing exactly
  that README's commands), not assumed. A related, smaller finding:
  `SafetyEnvelope.data_source` is also declared in the `.msg` source but absent from the installed
  package; handled with a `hasattr()` guard rather than a hard dependency.

## Decision: new standalone node, reusing the algorithm layer, not the mission layer

`autokart_ros_nodes/autokart_planner` is additive and does not modify
`kart/ros2_ws/src/autokart_nodes/autokart_planner`. It:
- reuses the exact, already-tested curvature and stopping-distance algorithms (lazily imported,
  `algorithm_backend: reused`, path never hardcoded — see DEPENDENCIES.md),
- also ships a zero-dependency `builtin` backend implementing the literal formulas from the master
  architecture doc, so this package builds/tests with nothing beyond ROS 2 Humble (same
  portability bar as `autokart_hp60c`'s `SyntheticBackend`), cross-validated against the reused
  backend in `test_algorithms.py`,
- implements its own obstacle-clearance (point-to-polyline distance) logic fresh rather than
  reusing `mission.py`'s private `_polyline_closest` helper, since that module is explicitly in
  flux this session and the function itself is textbook computational geometry, not a tuned
  algorithm worth preserving verbatim,
- never encodes a guessed physical constant as if measured: `a_lat_max_mps2` is an explicit
  UNVERIFIED nominal placeholder, and `a_brake_min_mps2`/`t_total_latency_s` default to `0.0`
  (unconfigured, fails closed), mirroring the existing `kart/ros2_ws` planner's own
  `brake_min_mps2: 0.0` convention rather than inventing a different pattern.

## What was implemented

- `autokart_planner/algorithms.py`: `AlgorithmBackend` protocol, `BuiltinAlgorithmBackend`
  (zero-dependency finite-difference curvature + stopping-distance formulas),
  `ReusedAlgorithmBackend` (lazy adapter over the real `autokart_perception` modules).
- `autokart_planner/planner_core.py`: `PlannerCore`, the ROS-independent engine — validity gating
  (odom freshness, path freshness/shape, configuration qualification), curvature-capped speed
  profile, forward obstacle clearance, stop-envelope validity.
- `autokart_planner/geometry_params.py`: measured-geometry defaults (a point-in-time copy of
  `vehicle_geometry.yaml`) plus an optional direct-YAML-read override, with no ROS-package import
  needed either way.
- `autokart_planner/node.py`: `PlannerNode`, ROS wrapper — subscribes odometry, reference path,
  obstacles; publishes path/envelope/speed-profile; timeout-driven and startup-driven fault
  reporting that never goes silent.
- Full ament_python package, launch file, two config profiles (`builtin`, `reused`).
- 40 tests: 18 algorithm unit tests (+ cross-validation), 15 planner-core unit tests, 1 ROS
  integration test, 6 fault-injection tests.
- `export_node.sh`, `CONTRACT.md`, `DEPENDENCIES.md`, `README.md` (this directory).

## Known gaps (explicit, not hidden)

- No acceleration-limit computation (`SafetyEnvelope.maximum_acceleration_mps2` is always `0.0`)
  — out of scope for the formulas this node was asked to implement.
- `algorithm_backend: reused`'s successful computation path is cross-validated against the
  zero-dependency builtin formulas on synthetic geometry only, not against real sensor data or a
  surveyed track (no hardware in this environment).
- `allowed_region` (a 2D drivable-region polygon) on `SafetyEnvelope` is left at its zero-length
  default; this node does not compute one.
