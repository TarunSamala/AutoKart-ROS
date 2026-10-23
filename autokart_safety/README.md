# autokart_safety

Software-only vehicle arming/authorization authority. Reuses the real,
already-reviewed `autokart_vcu_safety` state machine via a lazy,
path-parameterized import (never reimplemented). See `CONTRACT.md` for the
exact interface and `DEPENDENCIES.md` for what this package needs from
elsewhere.

**This node is not the physical E-stop and does not replace it.** See
`CONTRACT.md`'s first section before relying on anything this node reports.

## Quick start (no hardware, no dependency required)

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select autokart_safety
source install/setup.bash
ros2 launch autokart_safety safety.launch.py config_file:=$(ros2 pkg prefix autokart_safety)/share/autokart_safety/config/safety_synthetic.yaml
# in another shell:
ros2 topic echo /autokart/safety/status
```

With no `vcu_safety_package_path` set, this publishes `state=UNAVAILABLE`,
`arm_allowed=false` forever — the package builds, launches, and never
authorizes arming with zero external dependency.

## Running against the real autokart_vcu_safety core

See `DEPENDENCIES.md`. Set `vcu_safety_package_path` in a config override to
point at `kart/ros2_ws/src/autokart_vcu_safety` (or a copy of it), e.g.:

```bash
ros2 launch autokart_safety safety.launch.py config_file:=/path/to/safety_live.yaml
```

Arming still requires `calibration_verified`, `vehicle_config_verified` and
`braking_verified` to be set `true` in that same config override — they
default `false` and this node never sets them itself.

## Tests

```bash
source /opt/ros/humble/setup.bash
source <this_or_any_workspace>/install/setup.bash   # needs autokart_msgs built
cd autokart_ros_nodes/autokart_safety
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test/ -q
```

- `test/test_adapter.py` — unit tests for the lazy-import adapter, no ROS
  runtime needed (7 tests; most run against the real checked-out
  `kart/ros2_ws/src/autokart_vcu_safety`, skipped if that checkout is absent).
- `test/test_node_integration.py` — real rclpy node + real subscriber (2
  tests: fails closed without the dependency; never auto-arms with the real
  core loaded).
- `test/test_fault_injection.py` — real rclpy node, real `autokart_msgs`
  types (4 tests: ready-but-unverified arm rejected, arm succeeds once fully
  ready+verified, E-stop forces disarm while armed, stale VCU heartbeat
  blocks arm).

All 13 pass as of this writing (`test_node_integration.py`/
`test_fault_injection.py` tests are skipped, not counted as passing, if the
real `kart/ros2_ws/src/autokart_vcu_safety` checkout is not present). See
`reports/VALIDATOR_REPORT.md` for the exact run log and clean-workspace
verification.

## Reusing this node elsewhere

```bash
./export_node.sh /path/to/export/dir
```

packages this node plus `autokart_msgs` into a self-contained tarball. The
real `autokart_vcu_safety` core dependency is **not** bundled (see
`DEPENDENCIES.md`) — without it, the exported node still builds and runs, but
never authorizes arming.

## Reports

- `reports/DEVELOPER_REPORT.md`
- `reports/SENIOR_REVIEWER_REPORT.md`
- `reports/VALIDATOR_REPORT.md`

## Status

IMPLEMENTED and TESTED (software-only). The real `autokart_vcu_safety` core's
own extensive pure-logic test suite
(`kart/ros2_ws/src/autokart_vcu_safety/test/test_safety_core.py`) is reused
unmodified and is NOT duplicated by this package — this package tests its
own ROS wrapper, the lazy-import adapter, and the fail-closed/never-auto-arm
properties specific to this node. Real VCU firmware / vehicle bridge / E-stop
hardware interaction is **NOT RUN** (no hardware in this environment).
**Software-only acceptance is not a vehicle-readiness claim, and this node's
STOP is not a substitute for the physical battery/supply disconnect.**
