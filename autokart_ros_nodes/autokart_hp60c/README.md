# autokart_hp60c

Sole HP60C camera SDK owner. Publishes stamped RGB, depth and CameraInfo for the live/replay
autonomy stack. See `CONTRACT.md` for the exact interface and `DEPENDENCIES.md` for what this
package needs from elsewhere.

## Quick start (no hardware required)

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select autokart_hp60c
source install/setup.bash
ros2 launch autokart_hp60c hp60c.launch.py config_file:=$(ros2 pkg prefix autokart_hp60c)/share/autokart_hp60c/config/hp60c_synthetic.yaml
# in another shell:
ros2 topic echo /hp60c/status
```

This runs the built-in `SyntheticBackend` — deterministic fake frames, `quality=OK`,
never claims `LIVE` provenance. It exists so this package is buildable, testable and runnable with
zero hardware/vendor-SDK dependency.

## Running against real HP60C hardware

See `DEPENDENCIES.md`. Set `backend: hp60c` and the three path parameters in
`config/hp60c_live.yaml`, then:

```bash
ros2 launch autokart_hp60c hp60c.launch.py config_file:=/path/to/hp60c_live.yaml
```

Camera acquisition errors surface on `/hp60c/status` (`quality=INVALID`, `fault_state=<reason>`);
the node keeps retrying and never fabricates a frame.

## Tests

```bash
source /opt/ros/humble/setup.bash
source <this_or_any_workspace>/install/setup.bash   # needs autokart_msgs built
cd autokart_ros_nodes/autokart_hp60c
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test/ -q
```

- `test/test_backends.py` — unit tests, no ROS runtime needed (7 tests).
- `test/test_node_integration.py` — real rclpy node + real subscriber, checks topic contents (1 test).
- `test/test_fault_injection.py` — camera disconnect, transient fault + recovery, missing dependency (3 tests).

All 11 pass as of this writing; see `reports/VALIDATOR_REPORT.md` for the exact run log and what was
verified in a clean, separate colcon workspace (`export_node.sh`).

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

IMPLEMENTED and TESTED (software-only, synthetic + fault-injection). The `hp60c` backend's
dependency-failure paths are tested; the live hardware read path is **NOT RUN** (no camera attached
in this environment) — see DEPENDENCIES.md and the validator report. **Software-only acceptance is
not a vehicle-readiness claim.**
