# Developer report — autokart_system_monitor

## Audit performed before writing any code

- `kart/ros2_ws/src/autokart_diagnostics/`: read in full — an `ament_cmake`
  shell (`CMakeLists.txt` installs one script; `package.xml` declares
  rclpy/std_msgs/autokart_msgs) and `scripts/bench_vcu_status`, a read-only
  projector of an 8-word NOOP bench packet onto
  `autokart_msgs/BenchVcuStatus`. **There is no existing ROS system-monitor
  node to reuse**, so this package is largely new code, as the brief
  anticipated. Conventions adopted from `bench_vcu_status` (pattern, not
  code): explicit fail-closed reason strings, a periodic timer that keeps
  publishing a stale status instead of going silent, and an explicit
  "read-only, no authority" statement.
- Message audit: `autokart_msgs/SensorDeliveryMetadata` is a per-sensor
  delivery record (sequence, drops, queue depth) — the wrong shape for a
  multi-component health report. `BenchVcuStatus` is VCU-bench-specific. Per
  the architecture's "standard messages first" rule, all three outputs use
  standard `diagnostic_msgs/DiagnosticArray`; **no new message type was
  introduced**.
- `psutil` 5.9.0 is installed on this machine, but it is treated as optional
  (it is not a declared dependency and may be absent on a target).

## Design

- `monitor_core.py` — pure, ROS-free logic (same pure-logic/ROS-wrapper split
  as `autokart_vcu_safety`): `evaluate_health()` (per-component
  OK/WARN/STALE plus a worst-of rollup) and `StartupLatch` (a one-way "seen
  at least once" record per topic).
- `resource_backends.py` — `PsutilResourceBackend` (thin wrapper around real
  psutil; primes `cpu_percent` so the first sample is not psutil's documented
  meaningless 0.0), `UnavailableResourceBackend` (the dependency-free
  default/fallback; never fabricates a number), and `build_resource_backend()`
  (missing psutil → fallback, unknown name → loud error).
- `node.py` — dynamic heartbeat subscriptions: each topic's type is resolved
  from the live graph via `rosidl_runtime_py` and retried on a timer, so the
  node needs no `autokart_msgs` build dependency, works with nothing else
  running, and can watch a new topic with a config change only. Publishes
  `/diagnostics`, `/system/health`, `/system/startup_status`.
- **No safety authority:** no publisher other than the three diagnostic
  topics; nothing here can gate or command anything. This is stated in
  `CONTRACT.md`, `package.xml`, and the module docstring, explicitly
  contrasted with node 10.

## What was implemented

Package skeleton (ament_python, `resource/`, `setup.cfg`), launch file, two
configs, `export_node.sh` (self-contained: no project dependency bundled or
needed), `CONTRACT.md`, `DEPENDENCIES.md`, `README.md`, and 21 tests (9
pure-logic, 5 resource-backend, 4 real-rclpy integration, 3
fault-injection).

## Known gaps

- One shared heartbeat timeout; liveness only (payload validity is not
  inspected — that is node 10's job for its own inputs).
- CPU/RAM figures and the 90 % warn thresholds have only been exercised on
  this development laptop, not the target Raspberry Pi.
