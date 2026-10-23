# External dependencies — autokart_controller

## ROS package dependencies (ordinary colcon deps)

- `rclpy`, `nav_msgs`, `geometry_msgs`, `std_msgs`, `builtin_interfaces` — ROS 2 Humble core.
- `autokart_msgs` (`kart/ros2_ws/src/autokart_msgs`) — `MotionCommand`, `SafetyEnvelope`,
  `ActuatorFeedback` are used. Copied into the export bundle by `export_node.sh`.

No dependency on `autokart_planner` (node 08): the two interoperate over topics only. The one test
that runs both together (`test/test_pair_with_planner.py`) is skipped when node 08 isn't importable.

## Algorithm dependency (`pursuit_backend: reused` only)

With the default `pursuit_backend: builtin` there is none: the package builds and tests on a plain
Ubuntu 22.04 + ROS 2 Humble machine.

With `pursuit_backend: reused`, at runtime this node imports and directly calls the unmodified
`kart_sim.autonomy.PurePursuitController` and `kart_sim.autonomy.LongitudinalPidController`
(plus `kart_sim.types.PlanarKartState`/`SimulationError`) from `kart/kart_sim/` — the same
implementation `kart/ros2_ws`'s `autokart_controller/engine.py` wraps (`docs/NODE_CONTRACTS.md`
names it as the controller's algorithm source). It does **not** import `autokart_controller.engine`
from `kart/ros2_ws`: that module is a ROS-adapter-shaped wrapper inside the live workspace; this node
reuses the algorithm layer it wraps, the same choice `autokart_hp60c` made for the HP60C SDK class.

`kart_sim.autonomy` pulls in `kart_sim.vcu` → `communication`, `fault_manager`,
`safety_diagnostics` (all stdlib-only; verified importable with `python3 -I`).

**To use it:** copy the `kart/kart_sim/` package to the target machine, set
`kart_sim_package_path` to the directory that *contains* `kart_sim/` (see
`config/controller_reused.yaml`), set `pursuit_backend: reused`. Never hardcoded in source.

Failure modes, all fail-closed at startup and reported every 0.1 s:
- `DEPENDENCY_PATH_NOT_FOUND` — the path doesn't exist;
- `SDK_ADAPTER_IMPORT_FAILED` — `kart_sim.autonomy`/`types` can't be imported from it;
- `DEPENDENCY_PATH_MISMATCH` — the modules imported, but **not from the configured path**.

### Why the path-provenance check exists (found during this node's development)

This dev machine has an editable pip install of the simulator (`~/.local/.../
__editable___auto_kart_simulator_0_1_0_finder.py`), which registers a **meta-path finder** that
resolves `kart_sim` before `sys.path` is consulted. Putting the configured path at the front of
`sys.path` (the fix already applied in node 08) is therefore not enough: a test pointing
`kart_sim_package_path` at a directory with no `autonomy.py` silently "succeeded" by importing the
real `kart_sim` from elsewhere. The backend now verifies each imported module's `__file__` lies
under the configured path. The same check was added to node 08's `ReusedAlgorithmBackend`.
`autokart_hp60c/backends.py` has neither guard (not modified; outside this task's scope).

## What was NOT verified

No vehicle, no steering actuator, no track. The reused backend is cross-validated against the
builtin one (single-shot over four lateral offsets, and a six-step sequence exercising rate limiter
and PID state) to `1e-6`, on synthetic straight paths only. Closed-loop tracking quality, tuning, and
every UNVERIFIED constant in CONTRACT.md remain unvalidated.
