# autokart_planner

Standalone trajectory/curvature/collision/speed-profile/stop-envelope planner. Candidate output
only — never actuates the vehicle. See `CONTRACT.md` for the exact interface and `DEPENDENCIES.md`
for what this package needs from elsewhere.

## Quick start (no hardware, no external algorithm package required)

```bash
source /opt/ros/humble/setup.bash
source /home/mark1/Desktop/autonomous_kart/kart/ros2_ws/install/setup.bash   # gives you autokart_msgs
colcon build --packages-select autokart_planner
source install/setup.bash
ros2 launch autokart_planner planner.launch.py config_file:=$(ros2 pkg prefix autokart_planner)/share/autokart_planner/config/planner_builtin.yaml
# in other shells:
ros2 topic pub /autokart/odom nav_msgs/msg/Odometry '{...}'
ros2 topic pub /autokart_planner/reference_path nav_msgs/msg/Path '{...}'
ros2 topic echo /autokart_planner/safety_envelope
```

This runs the built-in, zero-dependency curvature/stopping-distance formulas (`algorithm_backend:
builtin`). With `a_brake_min_mps2: 0.0` (the shipped default), every cycle reports
`valid=false`/`BRAKE_CAPABILITY_UNQUALIFIED` until you configure a real, measured braking
deceleration — this is deliberate fail-closed behavior, not a bug; see CONTRACT.md.

## Running with the real, reused curvature/stopping-distance algorithm

See `DEPENDENCIES.md`. Set `algorithm_backend: reused` and `algorithm_package_path` in
`config/planner_reused.yaml`, then:

```bash
ros2 launch autokart_planner planner.launch.py config_file:=/path/to/planner_reused.yaml
```

## Tests

```bash
source /opt/ros/humble/setup.bash
source <this_or_any_workspace>/install/setup.bash   # needs autokart_msgs built
cd autokart_ros_nodes/autokart_planner
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test/ -q
```

- `test/test_algorithms.py` — unit tests for the curvature/stopping-distance backends, plus a
  cross-validation of the zero-dependency builtin formulas against the real, reused algorithm
  (skipped, not failed, if that algorithm package isn't present) (18 tests).
- `test/test_planner_core.py` — unit tests for the ROS-independent planning engine: validity
  gating, curvature speed cap, obstacle clearance, stop envelope (13 tests).
- `test/test_node_integration.py` — real rclpy node + real subscriber, checks topic contents
  (1 test).
- `test/test_fault_injection.py` — missing odom, stale odom, too-short path, path timeout
  (repeats, not one-shot), missing algorithm dependency (repeats, not one-shot) (5 tests).

All 37 pass as of this writing; see `reports/VALIDATOR_REPORT.md` for the exact run log.

## Reusing this node elsewhere

```bash
./export_node.sh /path/to/export/dir
```

packages this node plus the `autokart_msgs` interface package it depends on into a self-contained
tarball you can drop into any ROS 2 Humble workspace's `src/` and `colcon build`.

## Reports

- `reports/DEVELOPER_REPORT.md`
- `reports/SENIOR_REVIEWER_REPORT.md`
- `reports/VALIDATOR_REPORT.md`

## Status

IMPLEMENTED and TESTED (software-only, synthetic paths/obstacles + fault-injection). The `reused`
algorithm backend's successful-computation path is cross-validated against the zero-dependency
builtin formulas on synthetic geometry, not against real sensor data or a surveyed track — see
DEPENDENCIES.md and the validator report. `a_lat_max_mps2`/`a_brake_min_mps2`/`t_total_latency_s`
are UNVERIFIED/unconfigured by default (see CONTRACT.md). **Software-only acceptance is not a
vehicle-readiness claim.**

## Integration contract update 2026-10-22

Canonical mission input is `/mission/state`; outputs are `/planning/*` with legacy
aliases. `/odom`, capture-aligned obstacle coverage and matched coverage status
are mandatory. See CONTRACT.md for the current contract; older report snapshots
record earlier standalone behavior and do not establish acceptance. Default
acceleration, curvature and braking values fail closed. No physical outputs.
