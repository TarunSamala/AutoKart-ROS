# AutoKart ROS 2 System Inventory (Agent 1 — Phase 1 baseline)

> **CORRECTION (2026-10-18, later):** statements in this document that no HP60C is attached are WRONG or STALE. An HP60C (USB `3482:6723`) is attached and streams RGB+depth at ~11-13 fps via the vendor SDK; the live Vehicle Dashboard was verified against real frames (see `ui_split/VALIDATION_MATRIX.md`). Items below marked NOT TESTED because "no camera" are now testable; those not re-run in `ui_split/VALIDATION_MATRIX.md` remain NOT TESTED. Calibration is still UNVERIFIED and no qualified metric geometry exists.

**Date (UTC):** 2026-10-18. **Method:** direct inspection of `kart/ros2_ws/src` — package.xml/CMakeLists/setup.py, not assumed.

## Environment reality (corrects assumptions in the master execution prompt)

This session runs on a workstation named **`Jarvis`**, `x86_64`, Intel Core i7-9750H, Ubuntu with ROS 2 Humble — **not a Raspberry Pi, not ARM64**. `lsusb` shows a laptop webcam ("HP Wide Vision HD Camera", `/dev/video0,1`) and no HP60C or other depth-camera device. There is **no `autokart_hp60c` ROS package** — it does not exist under this or any other name in `kart/ros2_ws/src`. The real HP60C driver code lives outside the ROS workspace, at `autonomy_stack_phase1/phase_01_perception/src/autokart_perception/camera.py`, wrapped into ROS 2 by `kart/ros2_ws/src/autokart_nodes/autokart_perception` (package name `autokart_perception`, not `autokart_hp60c`).

**Consequence:** Agent 2 (camera) and Agent 7 (Raspberry Pi deployment) tasks that require physically connecting the HP60C or running on Pi hardware are **NOT TESTABLE in this environment** and will be marked `NOT TESTED` / `BLOCKED` with an exact procedure, never fabricated. `autokart_bringup` is the real existing bring-up package (not a package to be created).

## Package inventory (16 packages, real names)

| Package | Build type | Role |
|---|---|---|
| `autokart_msgs` | ament_cmake | Custom message definitions (Cone, ConeArray, SafetyEnvelope, VcuState, MotionCommand, …) |
| `autokart_bringup` | ament_cmake | Launch files: `cone_track`, `fast_sim`, `full_stack`, `gazebo_sim`, `rc03_rc05_diagnostics`, `skidpad` |
| `autokart_description` | ament_cmake | URDF/xacro, rviz configs |
| `autokart_control` | ament_cmake | Controller config (`autokart_msgs` only dep) |
| `autokart_diagnostics` | ament_cmake | `rclpy` diagnostics node, bench VCU status |
| `autokart_gazebo` | ament_cmake | C++ Gazebo plugins (`gazebo_follower_plugin.cpp`, `gazebo_sensor_delivery_plugin.cpp`) |
| `autokart_localization` | ament_cmake | (no declared exec deps — stub/placeholder) |
| `autokart_planning` | ament_cmake | (no declared exec deps — stub/placeholder) |
| `autokart_safety_bridge` | ament_cmake | **Front safety envelope monitor** (RC-05/RC-06, this session's prior work). Only node under this package. |
| `autokart_scenarios` | ament_cmake | Scenario contract config |
| `autokart_sim_bridge` | ament_python | `vcu_bridge.py` (sim VCU), `gazebo_full_lap.py`, `gazebo_sync_probe.py` |
| `autokart_dashboard_gateway` | ament_python | ROS→WebSocket/HTTP Mission Control gateway (`gateway.py`, `core.py`, `camera_feed.py`) |
| `autokart_nodes/autokart_perception` | nested, ament_cmake+python | ROS adapter over the authoritative Phase-1 perception library (HP60C camera, YOLO26s detector, depth pipeline) |
| `autokart_nodes/autokart_mapping` | nested | Mapping adapter |
| `autokart_nodes/autokart_planner` | nested | Planner adapter (mission/skidpad logic) |
| `autokart_nodes/autokart_controller` | nested | Controller adapter |

`autokart_localization` and `autokart_planning` declare no exec dependencies and contain no source beyond CMakeLists/package.xml — they are placeholders, not implemented nodes.

## Dependency graph (exec_depends, from package.xml)

```
autokart_bringup
  -> autokart_description, autokart_sim_bridge, autokart_perception,
     autokart_mapping, autokart_planner, autokart_controller,
     autokart_dashboard_gateway

autokart_perception   -> autokart_msgs, sensor_msgs, std_srvs
autokart_mapping      -> autokart_msgs, autokart_perception, nav_msgs
autokart_planner      -> autokart_msgs, autokart_perception, autokart_mapping, nav_msgs, geometry_msgs
autokart_controller    -> autokart_msgs, autokart_mapping, autokart_planner, nav_msgs
autokart_safety_bridge -> autokart_msgs, nav_msgs
autokart_dashboard_gateway -> autokart_msgs, nav_msgs, sensor_msgs, autokart_perception (for CameraFeed's live_render)
autokart_sim_bridge    -> autokart_msgs, nav_msgs, sensor_msgs, diagnostic_msgs, rosgraph_msgs
autokart_gazebo        -> autokart_description
autokart_diagnostics   -> std_msgs, autokart_msgs
```

## Real baseline (this session, before any new changes)

- `colcon build` (full workspace, isolated build/install dir): **16/16 packages built, 0 failures.**
- Python regression across all 8 packages with a `test/` directory: **79/79 PASS** (run directly with `pytest`, not via `colcon test`, because this sandbox's `ament_cmake_pytest` cannot find the system `pytest` module — a pre-existing packaging quirk unrelated to source changes; `python3 -m pytest` runs the same test files directly and they pass).
- `phase_sim` (TypeScript/React frontend) vitest suite: **43/43 PASS**, `tsc --noEmit` clean, production build clean.
- **Total: 122/122 automated tests passing, 16/16 packages building, before this phase's changes begin.**

## Custom messages (`autokart_msgs`)

`ActuatorFeedback`, `AuthoritativeVehicleSnapshot`, `BenchVcuStatus`, `Cone`, `ConeArray`, `GazeboFollowerState`, `MissionDebug`, `MotionCommand`, `SafetyEnvelope` (extended this session with `data_source`), `SensorDeliveryMetadata`, `VcuState`, `VehicleState`, `WheelState`, `WheelStates`.

## Launch files (8 total)

- `autokart_bringup`: `cone_track`, `fast_sim`, `full_stack`, `gazebo_sim`, `rc03_rc05_diagnostics`, `skidpad`
- `autokart_nodes/autokart_perception`: `perception_replay.launch.py`
- `autokart_safety_bridge`: `front_safety_envelope_monitor.launch.py` (this session's prior work)

No existing launch target combines camera + perception + safety + gateway into one "complete diagnostic system" bring-up. `full_stack.launch.py` currently only launches `autokart_sim_bridge`'s `vcu_bridge` — it is a simulation-bridge launch, not a full hardware diagnostic stack launch, despite the name.
