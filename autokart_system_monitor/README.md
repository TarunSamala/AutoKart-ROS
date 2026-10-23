# autokart_system_monitor

Health and startup-evidence reporting node: ROS node heartbeat/liveness
(dynamic topic discovery, graceful when dependencies are not running) plus
optional CPU/RAM, published as standard `diagnostic_msgs/DiagnosticArray`
on `/diagnostics`, `/system/health` and `/system/startup_status`. See
`CONTRACT.md` and `DEPENDENCIES.md`.

**This node reports; it has no safety authority** and cannot gate, stop or
authorize anything (contrast with `autokart_safety`, node 10).

## Quick start (no hardware, no other node required)

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select autokart_system_monitor
source install/setup.bash
ros2 launch autokart_system_monitor system_monitor.launch.py
# in another shell:
ros2 topic echo /system/health
```

With nothing else running, every watched topic reports `STALE` "never seen"
and `resources` reports real CPU/RAM (or `WARN` unavailable if `psutil` is
missing). Start e.g. `autokart_safety` and its `/autokart/safety/status`
entry turns `OK` within `topic_discovery_period_s` + one publish period.

## Tests

```bash
source /opt/ros/humble/setup.bash
source <any workspace>/install/setup.bash
cd autokart_ros_nodes/autokart_system_monitor
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test/ -q
```

- `test/test_monitor_core.py` — pure health/startup-latch logic, no ROS (9).
- `test/test_resource_backends.py` — psutil present / simulated missing /
  disabled / unknown name (5).
- `test/test_node_integration.py` — real rclpy: standalone with nothing
  running; dependency starts then stops → OK then STALE; best-effort
  publisher detected (QoS regression test); startup latch never un-latches (4).
- `test/test_fault_injection.py` — psutil missing at node level; invalid
  backend name fails loudly (2).

All 20 pass, including from a clean exported workspace with only
`/opt/ros/humble` sourced. See `reports/VALIDATOR_REPORT.md`.

## Reusing this node elsewhere

```bash
./export_node.sh /path/to/export/dir
```

Self-contained: no `autokart_msgs` or other project dependency is bundled
or needed.

## Reports

- `reports/DEVELOPER_REPORT.md`
- `reports/SENIOR_REVIEWER_REPORT.md`
- `reports/VALIDATOR_REPORT.md`

## Status

IMPLEMENTED and TESTED (software-only). Target-hardware (Raspberry Pi)
resource figures and threshold tuning are **NOT RUN**.
