# External dependencies — autokart_mission

## ROS package dependencies (ordinary colcon deps)

- `rclpy`, `nav_msgs` — ROS 2 Humble core.
- `autokart_msgs` — `Cone`, `ConeArray`, `MissionDebug`, `ActuatorFeedback`. Copy alongside this
  package to reuse it elsewhere (interface-only, no further dependency chain).

## Algorithm dependency: THREE lazy-import paths, not one

Unlike `autokart_hp60c`/`autokart_mapping` (one external path each), this node reuses
`autokart_planner.mission.PlannerEngine`, which itself has two further pure-Python dependencies that
must already be importable when it is imported. All three are resolved lazily (never at module import
time, never hardcoded), via three ROS parameters:

1. **`algorithm_package_path`** → `autonomy_stack_phase1/phase_01_perception/src` — gives
   `autokart_perception.shared_core.{PoseEstimate, BrakeCapability}`, which `mission.py` imports at
   its own module level.
2. **`vehicle_geometry_package_path`** → `kart/ros2_ws/src/autokart_vehicle_geometry` — gives
   `autokart_vehicle_geometry.geometry.GEOMETRY`, which `mission.py` also imports at its own module
   level. **This one is unusual**: `autokart_vehicle_geometry/geometry.py` runs
   `GEOMETRY = load_geometry()` at import time, which resolves
   `kart/ros2_ws/src/autokart_bringup/config/vehicle_geometry.yaml` either via `ament_index` (if a
   workspace with `autokart_bringup` installed is sourced) or by walking **upward from
   `geometry.py`'s own file location** looking for a sibling `autokart_bringup/config/
   vehicle_geometry.yaml`. That means `vehicle_geometry_package_path` must point at the **real,
   on-disk** `kart/ros2_ws/src/autokart_vehicle_geometry` directory (whose parent, `kart/ros2_ws/src`,
   still has `autokart_bringup` as a sibling) — not a relocated copy of just that one folder. See
   `test/test_algorithm.py::test_rejects_when_vehicle_geometry_yaml_cannot_be_found`, which
   demonstrates this fails closed (not silently with a guessed/default geometry) when that sibling
   relationship is broken.
3. **`planner_package_path`** → `kart/ros2_ws/src/autokart_nodes/autokart_planner` — gives
   `autokart_planner.mission` itself (`PlannerEngine`, `SkidpadProgress`). This is the **live
   production planner package's own source directory**; this node imports it as a read-only
   dependency and does not modify it.

**Import order matters**: (1) and (2) must be on `sys.path` and successfully imported *before* (3),
since `mission.py`'s own top-level `from autokart_perception... import ...` / `from
autokart_vehicle_geometry... import ...` statements run the instant it is imported.
`autokart_mission/algorithm.py` does this in the correct order.

**To reuse this node in a clean workspace:**
1. Copy all three source directories above onto the target machine (all are pure Python; (2) also
   needs its real `vehicle_geometry.yaml`, i.e. keep `autokart_bringup` as `autokart_vehicle_geometry`'s
   sibling, or supply an equivalent sibling layout).
2. Set the three `*_package_path` parameters to their actual paths.

No path in this package's source code is hardcoded to this repository's location. If any of the three
is wrong, missing, or the geometry YAML cannot be found, the node fails closed at startup
(`DEPENDENCY_PATH_NOT_FOUND:<label>:<path>` / `ALGORITHM_IMPORT_FAILED:<module>:<...>` on
`/autokart_mission/status`, republished every 100 ms), not silently.

## What is NOT reused: `autokart_planner`'s own path-finding/planner logic

`autokart_planner.mission.PlannerEngine.evaluate()` also internally computes a candidate path
(`path_odom`). This node reads that return value and discards `path_odom` before publishing (see
CONTRACT.md) rather than republishing it — node 08 (`autokart_planner`, not yet built in this
library) is the intended owner of path candidates, and this package does not duplicate or shortcut
that boundary by exposing path data under a different topic name.

## What was NOT verified in this session

- No physically surveyed skidpad course or real VCU/actuator feedback transport was available (no
  hardware/track in this environment). The gate-crossing/winding-number FSM and braking stop-distance
  logic are exercised only against synthetic/simulated fixtures (see `test/test_mission_fsm.py`,
  ported from the existing, reused adapter's own test suite). **NOT RUN**: real-vehicle braking
  performance, real `ActuatorFeedback` stop confirmation, and the braking FSM named-state gap noted in
  CONTRACT.md (which is a pre-existing property of the reused code, not something this node could
  "run" its way out of).
