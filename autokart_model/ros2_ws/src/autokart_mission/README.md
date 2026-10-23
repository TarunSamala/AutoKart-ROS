# autokart_mission

Standalone mission/braking-FSM node (node 07 of the 12-node architecture): wraps the existing,
already-tested `autokart_planner.mission.PlannerEngine` (skidpad gate-crossing/winding-number progress
FSM, braking stop-distance logic with dead-reckoning) as its own node, separate from the planner (node
08, not yet built in this library). See `CONTRACT.md` for the exact interface — including an honest
accounting of where the reused FSM's state names and completeness diverge from the master
architecture's own vocabulary — and `DEPENDENCIES.md` for the three lazy-import paths this node needs.

## Quick start

```bash
source /opt/ros/humble/setup.bash
source /home/mark1/Desktop/autonomous_kart/kart/ros2_ws/install/setup.bash   # gives autokart_msgs
colcon build --packages-select autokart_mission
source install/setup.bash
ros2 launch autokart_mission mission.launch.py \
  config_file:=$(ros2 pkg prefix autokart_mission)/share/autokart_mission/config/mission_skidpad.yaml
```

With the three `*_package_path` parameters unset (the shipped default), the node builds, launches and
keeps publishing a fault status on `/autokart_mission/status` rather than crashing or fabricating a
mission decision. Point them at the real `autonomy_stack_phase1/phase_01_perception/src`,
`kart/ros2_ws/src/autokart_vehicle_geometry` and `kart/ros2_ws/src/autokart_nodes/autokart_planner`
(this repository's canonical sources) to get real output; feed it pose (`/autokart/odom`) and a
mapping status (`/autokart_mapping/status`, e.g. from `autokart_mapping`, node 06 in this library) and
read `/autokart_mission/status`.

## Tests

```bash
source /opt/ros/humble/setup.bash
source <this_or_any_workspace>/install/setup.bash   # needs autokart_msgs built
cd autokart_ros_nodes/autokart_mission
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test/ -q
```

- `test/test_algorithm.py` — unit tests for the three-path lazy-import wrapper (5 tests; 2 skipped
  without the full `kart`+`autonomy_stack_phase1` checkout).
- `test/test_mission_fsm.py` — behavioral parity tests against the **real** reused `PlannerEngine`,
  ported from the existing adapter's own test suite, including the explicit winding-number +
  gate-crossing requirement and the braking-FSM gap assertion (3 tests; skipped without the checkout).
- `test/test_node_integration.py` — real rclpy node + real subscriber/publisher (1 test; skipped
  without the checkout).
- `test/test_fault_injection.py` — missing-dependency fail-closed path (always runs) and malformed
  mapping-status rejection (skipped without the checkout) (2 tests).

See `reports/VALIDATOR_REPORT.md` for the exact run log, pass counts, and a real cross-node run
against `autokart_mapping` (node 06, this library).

## Reusing this node elsewhere

```bash
./export_node.sh /path/to/export/dir
```

packages this node plus `autokart_msgs`. It does **not** bundle `autokart_perception`,
`autokart_vehicle_geometry`, or `autokart_planner` (the three reused dependencies) — see
DEPENDENCIES.md for exactly what to supply on the target machine and why the geometry one needs its
real sibling directory layout, not just a copied folder.

## Reports

- `reports/DEVELOPER_REPORT.md`
- `reports/SENIOR_REVIEWER_REPORT.md`
- `reports/VALIDATOR_REPORT.md`

## Status

IMPLEMENTED and TESTED (software-only, synthetic/simulated fixtures over the real, reused
`PlannerEngine`). Real actuator-feedback stop confirmation and real braking performance are **NOT
RUN** (no hardware/track in this environment). The braking mission's FSM-completeness gap (see
CONTRACT.md) is a property of the reused code, reported, not fixed by this node.
**Software-only acceptance is not a vehicle-readiness claim.**
