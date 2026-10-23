# External dependencies — autokart_system_monitor

## ROS package dependencies (ordinary colcon deps)

- `rclpy`, `diagnostic_msgs`, `rosidl_runtime_py` — all ROS 2 Humble core.
- **No `autokart_msgs` dependency.** Heartbeat topics are subscribed with
  their message type resolved at runtime from the ROS graph, so this node
  needs no compile-time knowledge of `SensorDeliveryMetadata`, `MissionDebug`,
  or any other project message. (The *runtime* environment must of course
  have whatever message packages the watched topics use installed for type
  resolution to succeed; if a type cannot be resolved, that topic logs a
  warning, stays "never seen", and is retried — it does not crash the node.)

## Optional runtime dependency: `psutil`

Imported lazily in `resource_backends.PsutilResourceBackend`. Not
installed → the resources component reports `WARN` /
`PSUTIL_IMPORT_FAILED` and `cpu_percent`/`ram_percent` = `"null"`. Verified
both ways in this session: real `psutil` 5.9.0 (installed on this machine)
and a simulated missing import (`test/test_fault_injection.py`,
`test/test_resource_backends.py`).

## No reused-algorithm dependency

Unlike nodes 10 and 11, there was no existing system-monitor implementation
to wrap: `kart/ros2_ws/src/autokart_diagnostics/` contains only an
`ament_cmake` shell and `scripts/bench_vcu_status` (a read-only NOOP bench
packet projector). That script was read for conventions; what was reused is
its *pattern* (fail-closed reasons, a periodic expiry timer that keeps
publishing a stale status rather than going silent, an explicit
"read-only, no authority" statement), not its code — it decodes a
VCU-specific packet and has nothing generic to import. This node's logic
(`monitor_core.py`) is therefore new, and was reviewed and tested as new
code rather than as a reused algorithm.

## What was NOT verified in this session

- No Raspberry Pi: CPU/RAM figures were sampled on this development laptop
  only; thresholds (`cpu_warn_pct`/`ram_warn_pct` = 90 %) are generic
  defaults, not tuned against the target computer.
- Only `autokart_safety` (node 10) was actually run alongside this node as a
  real heartbeat source; `/hp60c/status` was correctly reported "never seen"
  because no camera node was running.
