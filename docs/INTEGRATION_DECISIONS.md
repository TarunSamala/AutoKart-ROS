# Integration decisions (RC-02)

## RC-03–05 implementation addendum (2026-10-16)

The prior package grouping and unique ROS package names are unchanged (16
packages). Three new executables are installed by the existing `ament_cmake`
packages: `perception_node`, `mapping_node`, and `planner_node`. A new
`rc03_rc05_diagnostics.launch.py` starts them disarmed; its gateway option is
read-only and its odometry input remains external. No controller executable,
`MotionCommand` publisher or ESP32 transport was added.

RC-03 uses the Phase 1 `CoreRuntime`, `Pipeline`, `Replay`, `Synthetic`, `HP60C`
and `Yolo26OnnxDetector`; it does not copy inference or depth code. The model
is pinned by the existing manifest and checksum. The current `best.onnx` is
explicitly experimental. `LIVE` coordinates remain diagnostic because the
surveyed RAW16 scale, registration and camera extrinsics are absent. Synthetic
or recorded synthetic cones are source tagged and paired with a status message.

RC-04 uses the Phase 1 `AlgorithmCore` bounded `LocalConeMap`, Delaunay,
Gabriel, boundary/gate/centerline geometry and `fit_skidpad_topology`. A narrow
fix to that authoritative topology fit excludes the ambiguous crossover band;
it made the exact ±9.125 m simulated dual loop fit without inserting course
truth into observations. Pose jumps, stale/duplicate samples, frame mismatch
and missing source qualification clear the current map.

RC-05 uses the observed topology from RC-04 and the shared-core
`BrakeCapability`/`PoseEstimate` contracts. Its internal mission adapter adds
right/right/left/left gate and winding bookkeeping and an observed-circle
crossover candidate. The braking adapter checks the continuous corridor
segments and stopping envelope. STOP completion waits for near-zero odometry
speed plus fresh valid actuator feedback. These are diagnostic/SIL paths only;
`autokart_controller` remains a future milestone.

The dashboard gateway now preserves stage source/status in a separate
`diagnostics` field and displays odom-frame tracked cones. `base_link` raw
cones are not rendered as world coordinates. The existing Mission Control UI
shows source, validity, frame, sample time and rejection reasons. It does not
animate missing physical telemetry or grant VCU authority.

## Package ownership and name conflicts

The existing 13 ROS `package.xml` files have 13 unique package names. There is no duplicate ROS package name in the current colcon workspace. The name `autokart_perception` occurs in two distinct systems: an empty ROS package shell and the authoritative Python import package. They are not two ROS packages, but a ROS Python wrapper must not install another module called `autokart_perception`.

The existing ROS shell was relocated from `kart/ros2_ws/src/autokart_perception/` to `kart/ros2_ws/src/autokart_nodes/autokart_perception/`. Its ROS package name remains `autokart_perception`; its wrapper import namespace is `autokart_perception_ros`. CMake installs the Python algorithm package directly from `autonomy_stack_phase1/phase_01_perception/src/autokart_perception/` into the ROS install space. No duplicate algorithm source is tracked under `kart/ros2_ws/`.

`autokart_planning` and `autokart_control` are existing empty ROS shells (plus controller configuration in the latter). No package, launch file or source reference to either package name was found in the current ROS workspace. Keep both shells intact for compatibility; new wrapper package names are `autokart_planner` and `autokart_controller`. These are semantically related but not duplicate package names. `autokart_mapping` is new.

`kart/ros2_ws/src/autokart_nodes/` is a grouping directory only and intentionally has no `package.xml`. Colcon discovers the four child packages recursively. Existing `autokart_localization`, `autokart_sim_bridge`, `autokart_dashboard_gateway`, `autokart_msgs`, and bringup packages retain their current paths and names.

## Algorithm source of truth

| Function | Authoritative import path | Adapter rule |
|---|---|---|
| Camera acquisition, detector, metric depth | `autokart_perception.camera`, `.detector`, `.depth`, `.pipeline` from Phase 1 `src/` | Import the existing library; do not duplicate or retrain YOLO26s. |
| Delaunay/Gabriel/boundaries/gates/centerline/curvature | `autokart_perception.geometry`, `.gabriel`, `.boundaries`, `.gates`, `.centerline`, `.curvature` | Keep their tested contracts; adapter performs ROS/message conversion only. |
| Mapping, observed skidpad topology, shared mission candidate | `autokart_perception.shared_core` | Preserve fail-closed freshness and diagnostic-only output. |
| Pure Pursuit | `kart_sim.autonomy.PurePursuitController` from `kart/kart_sim` | Wrap its measured wheel-angle candidate; do not copy or translate to actuator pulses. |

Do not rely on `phase_01_perception/build/lib` or linked worktrees as imports; the authoritative source is the `src/` tree. The Phase 1 source is untracked in its nested Git checkout, so commits do not yet identify the complete tested source.

## Current scope

This stage creates the grouping/package skeleton and freezes the initial message ownership. It adds no node entry points and changes no model, sensor calibration, ROS schema, algorithm, safety rule, dashboard UI, or VCU transport. The ROS build now fails explicitly if the authoritative Phase 1 source package is absent. A ROS install-space import check is required after build to verify that this source, rather than a stale user-site copy, resolves. This is not evidence of Pi runtime or physical sensor validity.

## Structural rollback

To restore the former perception package path, move `kart/ros2_ws/src/autokart_nodes/autokart_perception/` back to `kart/ros2_ws/src/autokart_perception/`, restore its original four-line `ament_cmake` CMake file and original package description, and revert the package-path assertion in `kart/tests/integration/test_ros2_workspace.py`. Move the three newly added adapter package directories and grouping README to an external recovery folder rather than deleting them. Then rerun the kart suite and 13-package colcon build. The Phase 1 Python source was not changed by this migration.
