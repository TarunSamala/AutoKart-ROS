# External dependencies — autokart_safety

## ROS package dependencies (ordinary colcon deps)

- `rclpy`, `std_msgs` — ROS 2 Humble core.
- `autokart_msgs` — this repository's shared interface package
  (`kart/ros2_ws/src/autokart_msgs`). Uses `VcuState`, `ActuatorFeedback`,
  `MissionDebug`. To reuse this node in another workspace, copy
  `autokart_msgs` alongside it.

## Algorithm dependency: `autokart_vcu_safety` (runtime path parameter, not a colcon dependency)

This node does **not** vendor or reimplement the VCU arming state machine. It
lazily imports the real `autokart_vcu_safety.core` module (`SafetyLogic`,
`Inputs`, `Config`, `VcuView`, `evaluate_startup`) from
`kart/ros2_ws/src/autokart_vcu_safety/autokart_vcu_safety/core.py` at
**runtime**, via the `vcu_safety_package_path` ROS parameter — never
hardcoded in source.

**`vcu_safety_package_path` must be the ROS package directory that CONTAINS
the nested `autokart_vcu_safety` Python package** — i.e.
`.../kart/ros2_ws/src/autokart_vcu_safety` (the parent of
`autokart_vcu_safety/core.py`), **not** `.../kart/ros2_ws/src` itself. The
outer `src/` directory has no `__init__.py` at the `autokart_vcu_safety`
level in a way that resolves `.core` — pointing one level too high silently
produces an empty namespace package import failure
(`SDK_ADAPTER_IMPORT_FAILED:ModuleNotFoundError:No module named
'autokart_vcu_safety.core'`), caught by this package's own tests.

To reuse this node in a clean workspace against the real core:
1. Copy (or otherwise make available) `kart/ros2_ws/src/autokart_vcu_safety`
   onto the target machine.
2. Set `vcu_safety_package_path` in a config override (see
   `config/safety_live.yaml`) to that directory's absolute path.
3. The three `*_verified` parameters remain `false` by default even with the
   real core loaded — a person must explicitly set them `true` per-deployment
   after completing the bench checklist (`docs/BENCH_TESTS.md` in the main
   repository). This node never sets them itself.

If the dependency path is wrong, missing, or the imported module lacks the
expected API, this node fails closed at startup
(`DEPENDENCY_PATH_NOT_SET` / `DEPENDENCY_PATH_NOT_FOUND` /
`SDK_ADAPTER_IMPORT_FAILED` / `SDK_ADAPTER_API_MISMATCH` surfaced in
`/autokart/safety/status`'s `reason_codes`), not silently — see CONTRACT.md.

## Related, separate component NOT depended on

`kart/ros2_ws/src/autokart_safety_bridge` (`front_safety_envelope_monitor`)
is a diagnostic-only front-obstacle envelope classifier
(`vehicle_authority` permanently `OFF`). This node does not import, call, or
depend on it in any way. See CONTRACT.md and `reports/DEVELOPER_REPORT.md`.

## What was NOT verified in this session

- No physical VCU, ESP32 firmware, or vehicle bridge was available. This
  node's ROS wiring and the reused core's state machine are exercised
  end-to-end over real `rclpy` pub/sub with real `autokart_msgs` types and
  synthetic `VcuState`/`ActuatorFeedback` samples standing in for the real
  vehicle bridge/odometry/perception/mapping/planner/controller nodes — see
  `reports/VALIDATOR_REPORT.md`. The actual VCU firmware's behavior when it
  receives `ARM`/`DIS`/`RST` is unchanged and un-exercised by this node
  (that is `autokart_vehicle_bridge`'s own, separately tested, concern).
- `calibration_verified`, `vehicle_config_verified`, `braking_verified`
  remain `false` in every config shipped with this package. Nothing in this
  node sets them to `true` anywhere, in any profile.
