# autokart_perception_adapter

Node 02 in the `autokart_ros_nodes` library: YOLO26s + RGB/depth fusion -> qualified 3D cone
positions. Consumes camera topics (by default `autokart_hp60c`'s, node 01 in this library) rather
than opening a camera itself. See `CONTRACT.md` for the exact interface, `DEPENDENCIES.md` for what
it needs from elsewhere.

## Quick start (no hardware, no other node required)

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-up-to autokart_perception_adapter
source install/setup.bash
ros2 launch autokart_perception_adapter perception_adapter.launch.py \
  config_file:=$(ros2 pkg prefix autokart_perception_adapter)/share/autokart_perception_adapter/config/synthetic.yaml
# in another shell:
ros2 topic echo /perception/cones
```

This uses `backend: synthetic` + `input_mode: standalone_timer`: a fixed, always-valid 4-cone layout,
with zero dependency beyond `autokart_msgs`.

## Running against autokart_hp60c + the real algorithm

```bash
ros2 launch autokart_hp60c hp60c.launch.py config_file:=.../hp60c_synthetic.yaml &   # or hp60c_live.yaml
ros2 launch autokart_perception_adapter perception_adapter.launch.py \
  config_file:=/path/to/topics_live.yaml   # set algorithm_package_path etc. first, see DEPENDENCIES.md
```

## Tests

```bash
source /opt/ros/humble/setup.bash
source <workspace>/install/setup.bash   # needs autokart_msgs built
cd autokart_ros_nodes/autokart_perception_adapter
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test/ -q
```

- `test/test_backends.py` — 7 unit tests (6 no-ROS + 1 real-algorithm-import, auto-skipped if the
  algorithm package checkout isn't present).
- `test/test_node_integration.py` — 1 real-rclpy test (standalone timer mode, checks actual published
  cone/status content).
- `test/test_fault_injection.py` — 3 tests: unqualified result never publishes cones, unexpected
  backend exception reports a fault instead of crashing, missing dependency keeps reporting (not
  one-shot).

All 11 pass as of this writing, including in a clean export-bundle workspace with only
`autokart_perception_adapter` + `autokart_msgs` present (the real-algorithm test correctly
auto-skips there). See `reports/VALIDATOR_REPORT.md`.

## Reusing this node elsewhere

```bash
./export_node.sh /path/to/export/dir
```

## Reports

- `reports/DEVELOPER_REPORT.md`
- `reports/SENIOR_REVIEWER_REPORT.md`
- `reports/VALIDATOR_REPORT.md`

## Status

IMPLEMENTED and TESTED (software-only: synthetic backend + the real algorithm package's import/call
path, both verified). `/perception/detections` and `/perception/obstacles` are explicitly deferred
(not built this session). Real YOLO26s inference against a live camera and real cone geometry is
**NOT RUN** — no model path supplied, no camera attached, no surveyed track. Software-only acceptance
is not a vehicle-readiness claim.
