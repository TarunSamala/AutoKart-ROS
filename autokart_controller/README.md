# autokart_controller

Standalone Pure Pursuit + longitudinal-PID **command candidate** node. No vehicle authority — see
`CONTRACT.md` ("Authority boundary") first, and `DEPENDENCIES.md` for what it needs from elsewhere.

## Quick start (no hardware, no kart_sim needed)

```bash
source /opt/ros/humble/setup.bash
source <a workspace providing autokart_msgs>/install/setup.bash
colcon build --packages-select autokart_controller
source install/setup.bash
ros2 launch autokart_controller controller.launch.py
ros2 topic echo /autokart_controller/motion_command_candidate
```

With no planner running it reports `valid: false` (`PATH_MISSING_OR_STALE`) every 0.1 s — by design.
Run `autokart_planner` (node 08) alongside it with streaming odometry to get valid candidates.

## Using the real, reused kart_sim implementation

Set `pursuit_backend: reused` and `kart_sim_package_path` in `config/controller_reused.yaml`:

```bash
ros2 launch autokart_controller controller.launch.py config_file:=/path/to/controller_reused.yaml
```

## Tests

```bash
source /opt/ros/humble/setup.bash
source <workspace with autokart_msgs>/install/setup.bash
cd autokart_ros_nodes/autokart_controller
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test/ -q
```

- `test/test_pursuit_backends.py` — builtin backend unit tests, reused-backend failure paths
  (incl. path-provenance mismatch), builtin-vs-reused cross-validation (skipped if `kart_sim` absent) (20).
- `test/test_controller_core.py` — ROS-independent gating, speed capping, profile alignment,
  state reset across a reject (16).
- `test/test_node_integration.py` — real rclpy node fed node-08-shaped messages (1).
- `test/test_fault_injection.py` — missing/stale odom, invalid/mismatched/missing envelope, expired
  path, repeated stop reporting, never-received path, malformed profile, missing dependency,
  unqualified wheelbase, out-of-order arrival (12).
- `test/test_pair_with_planner.py` — the real node 08 driving this node end to end (1; skipped when
  node 08 isn't importable, e.g. in this package's own export bundle).

50 pass in place. See `reports/VALIDATOR_REPORT.md` for the run log and the clean-workspace results.

## Reusing elsewhere

`./export_node.sh /path/to/export/dir` bundles this package plus `autokart_msgs`.

## Status

IMPLEMENTED, TESTED (software-only). `wheelbase_m`, `maximum_steering_rad`,
`maximum_steering_rate_radps` are UNVERIFIED nominal values. No closed-loop, hardware or track
validation. **Not a vehicle-readiness claim.**
