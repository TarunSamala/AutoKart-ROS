# autokart_mapping

Standalone cone-track mapping node (node 06 of the 12-node architecture): observed cone map,
Delaunay/Gabriel/gate/centerline geometry, and observed skidpad ring topology, reused unmodified from
`autokart_perception.shared_core`. See `CONTRACT.md` for the exact interface and `DEPENDENCIES.md` for
what this package needs from elsewhere.

## Quick start

```bash
source /opt/ros/humble/setup.bash
source /home/mark1/Desktop/autonomous_kart/kart/ros2_ws/install/setup.bash   # gives autokart_msgs
colcon build --packages-select autokart_mapping
source install/setup.bash
ros2 launch autokart_mapping mapping.launch.py \
  config_file:=$(ros2 pkg prefix autokart_mapping)/share/autokart_mapping/config/mapping.yaml \
  # then override algorithm_package_path -- see DEPENDENCIES.md; it is blank by default and the
  # node fails closed (fault status, no claimed map) until it is set.
```

With `algorithm_package_path` unset, the node builds, launches and keeps publishing a fault status on
`/autokart_mapping/status` rather than crashing or fabricating a map. Point it at
`autonomy_stack_phase1/phase_01_perception/src` (this repository's canonical algorithm source) to get
real mapping output; feed it cone observations (`/autokart_mapping/cones_in`,
`autokart_msgs/ConeArray`, `base_link`) and pose (`/autokart/odom`, `nav_msgs/Odometry`) and read
`/autokart_mapping/cones` (the observed map, `odom` frame) and `/autokart_mapping/status` (diagnostic
geometry, JSON).

## Tests

```bash
source /opt/ros/humble/setup.bash
source <this_or_any_workspace>/install/setup.bash   # needs autokart_msgs built
cd autokart_ros_nodes/autokart_mapping
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test/ -q
```

- `test/test_algorithm.py` — unit tests for the lazy-import wrapper, no ROS/algorithm dependency
  needed (3 tests; 1 skipped without the `autonomy_stack_phase1` checkout).
- `test/test_engine.py` — unit tests for the ROS-free mapping engine against the **real** reused
  algorithm (ported in behavior from the existing adapter's own test suite); skipped without the
  `autonomy_stack_phase1` checkout (4 tests).
- `test/test_node_integration.py` — real rclpy node + real subscriber/publisher, checks topic
  contents (1 test; skipped without the checkout).
- `test/test_fault_injection.py` — missing-dependency fail-closed path (always runs) and unqualified
  cone-frame rejection (skipped without the checkout) (2 tests).

See `reports/VALIDATOR_REPORT.md` for the exact run log, pass counts and what was verified in a clean,
separate colcon workspace (`export_node.sh`).

## Reusing this node elsewhere

```bash
./export_node.sh /path/to/export/dir
```

packages this node plus the `autokart_msgs` interface package into a self-contained tarball. It does
**not** bundle the `autokart_perception` algorithm package (pure Python, no native build, but large
and separately versioned) — see DEPENDENCIES.md for how to supply it on the target machine.

## Reports

- `reports/DEVELOPER_REPORT.md`
- `reports/SENIOR_REVIEWER_REPORT.md`
- `reports/VALIDATOR_REPORT.md`

## Status

IMPLEMENTED and TESTED (software-only, synthetic cone/pose fixtures over the real, reused geometry
algorithm). Surveyed-course residual accuracy is **NOT RUN** (no track/hardware in this environment).
**Software-only acceptance is not a vehicle-readiness claim.**
