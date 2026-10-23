# Developer report — autokart_safety

## Audit performed before writing any code

- Read both existing safety-shaped packages in `kart/ros2_ws/src/` in full, per
  the mission's explicit instruction that they are distinct:
  - `autokart_vcu_safety/{core.py,node.py}` + `config/vcu_safety.yaml` +
    `test/test_safety_core.py` — the actual VCU **arming authority**:
    `BOOT -> DISARMED -> READY -> ARMED -> ARMING -> ... -> FAULT` state
    machine, default-DISARMED, never-auto-arms, gated by freshness/validity
    of six upstream signals plus three `*_verified` flags (all default
    `false` in `config/vcu_safety.yaml`) plus a manual-power-cut
    confirmation that is cleared on any link loss or fault.
  - `autokart_safety_bridge/{envelope.py,node.py}` + `test/test_envelope.py` +
    `test/test_node_contract.py` — `front_safety_envelope_monitor`, a
    **diagnostic-only** front-obstacle classifier. It refuses to start unless
    `vehicle_authority=="OFF"` and `safety_monitor_mode=="DIAGNOSTIC_ONLY"`,
    rejects any runtime parameter change away from those values, and has no
    VCU/brake/throttle publisher or client anywhere in its code. Confirmed by
    reading the whole file, not just its docstring.
- Confirmed `PROJECT_INDEX.md`'s framing matches: these are deliberately
  different components, not two versions of the same thing.
- Checked `autokart_msgs/msg/` for an existing fit: `VcuState`,
  `ActuatorFeedback`, `MissionDebug` all already have the exact fields the
  reused core consumes/produces (`estop_input_fitted`, `active_faults`,
  `payload_json`, etc.) — no new message type introduced.
- Read the repository root `PROJECT_INDEX.md`/`README.txt` safety boundary in
  full: three verified flags false, E-stop GPIO7 has no status wire fitted,
  manual battery/supply disconnect is the only proven emergency stop. This is
  reflected verbatim in `CONTRACT.md`'s opening section and in `node.py`'s
  module docstring — deliberately, not just by reference.

## Decision: reuse the real `autokart_vcu_safety` core via a lazy import, do not rewrite it

Per the master architecture doc's "do not reimplement working algorithms"
rule and this session's explicit instruction, `autokart_safety/adapter.py`
imports `kart/ros2_ws/src/autokart_vcu_safety/autokart_vcu_safety/core.py`'s
`SafetyLogic`, `Inputs`, `Config`, `VcuView`, `evaluate_startup` directly —
the same state machine, the same `evaluate_startup` stage table, the same
fault logic — at runtime, from a path given by the `vcu_safety_package_path`
ROS parameter. `node.py` is a new, standalone ROS wrapper (new subscriptions/
publishers, new entry point, new package metadata) but contains no
reimplementation of arming logic.

This mirrors `autokart_hp60c.backends.HP60CBackend`'s exact pattern for the
HP60C SDK: a thin adapter, a runtime path parameter (never hardcoded), and an
explicit, typed failure mode
(`DEPENDENCY_PATH_NOT_SET`/`DEPENDENCY_PATH_NOT_FOUND`/
`SDK_ADAPTER_IMPORT_FAILED`/`SDK_ADAPTER_API_MISMATCH`) if the dependency is
missing or wrong — except here, because this is a safety authority rather
than a sensor driver, "dependency unavailable" must fail toward **permanently
refusing to arm**, not toward a synthetic stand-in that claims to work. There
is deliberately no "synthetic safety logic" fallback: `SyntheticInputsSource`
in `adapter.py` only ever feeds the real, imported core deterministic
freshness data for testing — it never substitutes for the core's own
decision logic.

`autokart_safety_bridge`'s `front_safety_envelope_monitor` is documented (not
silently dropped) as a related, separate, diagnostic-only component this node
does not subsume. **Recommendation, not a decision made here:** if the
12-node architecture's single "autokart_safety" node is meant to be the one
place all safety-relevant classification lives, a future, separately
reviewed change could have this node *subscribe* to
`front_safety_envelope_monitor`'s `/safety/envelope` output as one more
runtime-fault input (analogous to how it already treats perception/mapping/
planner/controller staleness) — but that changes the envelope monitor's
consumer contract and deserves its own audit, not a decision folded into this
build.

## What was implemented

- `autokart_safety/adapter.py`: `SafetyCoreError`, `load_safety_core()` (lazy
  import with path-shadowing protection — see reviewer report for a real bug
  found and fixed here), `SyntheticInputsSource` (dependency-free freshness
  source for tests).
- `autokart_safety/node.py`: `AutokartSafetyNode`, same topic wiring as the
  original `autokart_vcu_safety` node (`/vcu/state`, `/vehicle/actuator_feedback`,
  five `MissionDebug` status topics, `/autokart/safety/request` in,
  `/autokart/safety/status` + `/dashboard/debug` + `/autokart/vcu/request`
  out), but fails closed (`state=UNAVAILABLE`, `arm_allowed=false`, repeated
  every tick, never one-shot) if the core dependency cannot be loaded.
- Full ament_python package skeleton, launch file, two config profiles
  (`synthetic` — no dependency path, fails closed by design;
  `live` — dependency path and `*_verified` flags left blank/false,
  documented as placeholders).
- 13 tests: adapter unit tests (lazy import success/failure/API-mismatch,
  a reimport-after-bad-path regression test), real-rclpy node integration
  (fails closed without dependency; never auto-arms with the real core),
  real-rclpy fault injection (ready-but-unverified arm rejected, arm
  succeeds once fully ready+verified, E-stop forces disarm while armed,
  stale VCU heartbeat blocks arm).
- `export_node.sh`, `CONTRACT.md`, `DEPENDENCIES.md`, `README.md`.

## Known gaps (explicit, not hidden)

- No real VCU firmware, ESP32, or vehicle bridge in this environment — this
  node's forwarded `ARM`/`DIS`/`RST` strings are observed on
  `/autokart/vcu/request`, not verified against actual firmware behavior.
  That is `autokart_vehicle_bridge`'s own, separately tested, concern.
- The reused core's own extensive pure-logic test suite
  (`kart/ros2_ws/src/autokart_vcu_safety/test/test_safety_core.py`, 17 tests
  covering demo-drive, stop-latch, arming-timeout, etc.) is **not copied into
  this package** — it already exists, already passes, and copying it would
  duplicate rather than reuse. This package's own tests focus on what is new
  here: the ROS wrapper, the lazy-import adapter, and the fail-closed
  behavior specific to this standalone node.
- `calibration_verified`/`vehicle_config_verified`/`braking_verified` are
  `false` in every shipped config. No test in this package sets them `true`
  except as an explicit, in-test parameter override used to exercise the
  positive "arm succeeds" path — never as a shipped default.
