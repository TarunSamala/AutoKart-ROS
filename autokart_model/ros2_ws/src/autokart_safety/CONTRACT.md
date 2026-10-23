# Takeover integration contract (2026-10-22)

This takeover adapter has a permanent software output inhibit. It reuses
`autokart_vcu_safety.core.SafetyLogic` for diagnostic evaluation, but only
STOP/DIS requests can leave its legacy request publisher. ARM,
confirm_manual_power_cut and demo_drive_start are rejected. All profiles
publish DISARMED, arm_allowed=false, vehicle_authorized=false and an invalid,
zero-speed stop candidate on `/vehicle/authorized_command` (MissionDebug).
Setting verification parameters true cannot remove the inhibit.

Canonical `/safety/status` aliases the existing `/autokart/safety/status`.
The authorized-command payload carries schema_version, sequence,
sample_time_ns, frame_id, source=UNVERIFIED, valid=false, output_inhibited,
stop_requested, target_speed_mps, target_steering_rad and reason_codes.
This is an intentionally unauthorized candidate, not a hardware command.
`/control/request` is observed as MissionDebug. Translational speed is
observed from `/odom` nav_msgs/Odometry; motor_speed_radps is never converted
to m/s without measured wheel geometry. Speed receipt has a 0.2 s age gate.
Source LIVE_PHYSICAL is required for any diagnostic upstream freshness gate;
receipt alone does not establish source integrity or physical qualification.
The measured ground-speed, brake, ACK/session and physical stop evidence
remain unqualified and motion remains NO-GO.

## Historical package contract

# autokart_safety — interface contract

## What this node is

A **software-only** arming/authorization authority. It is the only node in
this library that may request `ARM` of the vehicle bridge, reusing the real,
already-reviewed `autokart_vcu_safety` state machine
(`BOOT -> DISARMED -> READY -> ARMED -> ... -> STOP_VERIFIED/FAULT`, see
`kart/ros2_ws/src/autokart_vcu_safety/autokart_vcu_safety/core.py`) via a
lazy, path-parameterized import (`adapter.py`) -- never reimplemented.

## What this node is NOT

**This node is not the physical E-stop and must never be treated as one.**
Per this repository's `PROJECT_INDEX.md`/`README.txt`: the E-stop input at
GPIO7 has no status wire fitted, and the manual battery/supply disconnect is
the *only* proven emergency stop. A `STOP`/`DIS` published by this node
depends on this node's own process, the ROS graph, the USB link and the
ESP32 firmware all continuing to function correctly -- it has no independent,
hardware-enforced guarantee of stopping the vehicle. Treat every `STOP`/
`FAULT` state this node reports as a *request*, not a proof of a stopped
vehicle, until the physical disconnect is also used.

This node also does not replace or subsume
`kart/ros2_ws/src/autokart_safety_bridge` (`front_safety_envelope_monitor`).
That is a **separate, diagnostic-only** front-obstacle envelope classifier
with `vehicle_authority` permanently `OFF` -- it has no arming authority and
this node does not call into it or depend on it. See
`reports/DEVELOPER_REPORT.md` for a recommendation (not a decision made here)
about eventually unifying the two under one "autokart_safety" node.

## Topics

| Topic | Type | Direction | Notes |
|---|---|---|---|
| `/vcu/state` | `autokart_msgs/VcuState` | sub | VCU telemetry; freshness gates arming (`vcu_timeout_s`, default 0.3s in the reused core's `Config`) |
| `/vehicle/actuator_feedback` | `autokart_msgs/ActuatorFeedback` | sub | axle-derived speed; overspeed is a runtime fault while ARMED |
| `/autokart/safety/request` | `std_msgs/String` | sub | ONLY channel through which anything (including the dashboard gateway, node 11) may influence this node: one of `arm`, `stop`, `reset_fault`, `confirm_manual_power_cut`, `demo_drive_start`, `demo_drive_stop`. Anything else is logged and dropped. |
| `/autokart/perception/status`, `/autokart/mapping/status`, `/autokart/planner/status`, `/autokart/controller/status`, `/autokart/localization/status` | `autokart_msgs/MissionDebug` | sub | freshness/validity gates for arming and runtime fault detection (`source=="LIVE"` and `valid==true` required to count as fresh) |
| `/autokart/safety/status` | `autokart_msgs/MissionDebug` | pub | this node's own state machine status (state, `arm_allowed`, `stages`, `reasons`, `vehicle_authorized` -- always `false`), one message per 0.1s tick |
| `/dashboard/debug` | `autokart_msgs/MissionDebug` | pub | same payload, `stage="safety"`, for the dashboard gateway (node 11) to display |
| `/autokart/vcu/request` | `std_msgs/String` | pub | forwarded `ARM`/`DIS`/`RST` decision for the (not bundled here) vehicle bridge to act on |

## Default-DISARMED, never-auto-arm

On construction, `state = "DISARMED"`. No code path in the reused core or
this wrapper transitions out of `DISARMED` without an explicit `"arm"` entry
in `requests` for that tick -- there is no startup auto-arm, no
reconnection auto-arm, and no auto-arm-on-input-becoming-fresh. See
`kart/ros2_ws/src/autokart_vcu_safety/test/test_safety_core.py` (reused,
unmodified) and this package's own
`test/test_node_integration.py::test_safety_node_never_auto_arms_on_startup_with_real_core`.

## Arming gates (all must hold, every tick, or arm is refused)

- Every upstream input (`vcu`, `odom`, `perception`, `mapping`, `planner`,
  `controller`) measurably fresh (age below its configured timeout).
- `vcu.state == "SAFE"`.
- E-stop input: `FAULT` if fitted and active; `NOT_FITTED` (amber, not a
  hard block by itself, but it still requires `manual_power_cut_confirmed`).
- `calibration_verified`, `vehicle_config_verified`, `braking_verified` all
  `true` -- **read-only parameters this node never writes**, default `false`.
- `manual_power_cut_confirmed` -- a fresh, per-arm-attempt operator
  confirmation via the `confirm_manual_power_cut` request, which this node
  itself clears on any VCU-link loss, fault, or stale upstream input (see
  `evaluate_startup`/`SafetyLogic.step` in the reused core).

A `READY`-everything-else-but-unverified arm attempt (e.g. fresh inputs, VCU
`SAFE`, but `calibration_verified=false`) is **refused**, not granted -- see
`test/test_fault_injection.py::test_ready_but_unverified_arm_attempt_is_rejected`.

## Fault behavior

Any runtime fault (VCU heartbeat loss, VCU-reported fault, E-stop active,
overspeed, any upstream status going stale) while `ARMED` forces `FAULT` and
publishes `DIS`. A `STOP` request latches until an explicit `reset_fault`,
and `reset_fault` itself is refused while the triggering cause is still
present. See the reused core's own test suite and this package's
`test/test_fault_injection.py`.

## Dependency-unavailable behavior (software failure, distinct from E-stop)

If `vcu_safety_package_path` is unset, wrong, or the imported module does not
expose the expected API, this node **never falls back to a different
arming policy** -- it publishes `state="UNAVAILABLE"`, `arm_allowed=false` on
every tick, forever (not one-shot), and never forwards any VCU request. See
`adapter.py` and `test/test_node_integration.py::test_safety_node_fails_closed_without_core_dependency`.

## Parameters

See `config/safety_synthetic.yaml` (no dependency, fails closed) and
`config/safety_live.yaml` (points at the real core; `*_verified` flags still
default `false`). `vcu_safety_package_path` selects the dependency path (see
DEPENDENCIES.md).
