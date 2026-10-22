# autokart_vcu_bridge — interface contract

## Role

The sole USB/serial VCU endpoint owner. It relays an **already-decided, already-authorized** candidate
motion command into the AutoKart laptop<->VCU wire protocol (`docs/VCU_PROTOCOL.md`) and publishes
telemetry-derived feedback. **It does not decide motion authority** — see "Safety gating" below. No raw
PWM/DAC/stepper control and no E-stop override happen in this node; the VCU firmware's own
SAFE/ARMED/FAULT state machine remains the actual safety authority for whether a command is obeyed.

## Topics

| Topic | Type | Direction | Notes |
|---|---|---|---|
| `candidate_topic` (param, default `/autokart/controller/motion_command_candidate`) | `autokart_msgs/MotionCommand` | subscribe | The existing, disarmed candidate per `docs/NODE_CONTRACTS.md`. `target_speed_mps`/`valid`/`expiry` are used; `steering_angle_rad` is **never actuated** (observed only, per the wire protocol). `expiry == 0` (both fields) is treated as "no expiry" — a documented convention of this node, since the field's zero-sentinel semantics are not otherwise specified. |
| `safety_armed_topic` (param, default `/vcu_bridge/safety_armed`) | `std_msgs/Bool` | subscribe | **Placeholder upstream-safety input.** With nothing publishing here, this input is absent/stale and the reused `CommandGate` fails closed (no nonzero throttle ever passes). See "Relationship to node 10" below. |
| `request_topic` (param, default `/vcu_bridge/request`) | `std_msgs/String` | subscribe | `ARM`/`DIS`/`RST`, relayed directly to the VCU's own SAFE/ARMED/FAULT state machine (see "ARM/DIS/RST relay" below). |
| `/vehicle/actuator_feedback` | `autokart_msgs/ActuatorFeedback` | publish | From VCU telemetry. `valid=false` whenever telemetry is stale or the real conversion law (`core.raw_to_rad`) is unavailable — never fabricated. |
| `/vcu/state` | `autokart_msgs/VcuState` | publish | From VCU telemetry. `state="UNKNOWN"`, `limiting_reason="NO_VCU_TELEMETRY"` whenever telemetry is stale. |
| `/vcu_bridge/status` | `autokart_msgs/SensorDeliveryMetadata` | publish | Bridge-level health: `quality=OK` only when the backend is healthy AND the real control law (`core.py`) is available; `INVALID` with an explicit `fault_state` otherwise (reused message type, same pattern as `autokart_hp60c`/`autokart_imu`, not a new one). |

## Safety gating ("it does not decide motion authority")

Nonzero throttle is sent **only when all of** hold, computed every tick by the reused
`autokart_vehicle_bridge.core.CommandGate`:
1. `safety_armed` is `True` and was received within `safety_timeout_s` (default 0.3 s) — see the
   placeholder note above; nothing in this library publishes this Bool today, so it is always `False`.
2. The VCU's own telemetry reports `ARMED` and is fresh (received within 0.3 s).
3. The candidate command is fresh (received within `cmd_timeout_s`, default 0.25 s) and `valid=true` and
   not expired.

This node never originates authorization 1 — it only ever relays an externally-supplied decision,
exactly as the mission scope requires ("it relays an already-decided authorized command").

## Relationship to node 10 (`autokart_safety`)

`autokart_safety` (node 10) was built concurrently in this library during this session. It publishes its arming decision as `MissionDebug` JSON on `/autokart/safety/status` and forwards ARM/DIS/RST on `/autokart/vcu/request`. This node does **not** consume that JSON status. Nothing publishes the `safety_armed` Bool, so throttle stays gated off when the two run together (fail closed). `request_topic` can be set to `/autokart/vcu/request` (same String ARM/DIS/RST semantics). Wiring `safety_armed` to node 10's status is an identified integration gap, NOT RUN.

## ARM/DIS/RST relay — a deliberate scope simplification

Unlike the real `kart/ros2_ws` vehicle-bridge node, this bridge does **not** itself gate ARM/DIS/RST
requests with a two-phase `ARMING`/`safety_stamp` handshake (`core.arm_request_approved`) before relaying
them. That handshake exists in the real node because an upstream `autokart_vcu_safety` supervisor
originates a two-stage ARMING approval. When this node was designed no such supervisor existed in this
library, and node 10 (built concurrently) is not yet wired to this node (see above). Reusing
`arm_request_approved` without that wiring would require fabricating the upstream state, which this node
will not do. Instead, `/vcu_bridge/request` is relayed directly to the
backend; the VCU firmware's own state machine (SAFE/ARMED/FAULT, NAK codes) remains the actual authority
for whether ARM succeeds. **Nonzero throttle remains gated by `safety_armed` regardless of ARM state** —
a successful ARM does not by itself authorize any motion.

## Fault behavior

On any backend fault (missing dependency, unavailable/failed serial port, unexpected exception), this
node publishes `/vcu_bridge/status` with `quality=INVALID` and an explicit `fault_state`, keeps retrying
every tick, and continues publishing `/vehicle/actuator_feedback` (`valid=false`) and `/vcu/state`
(`state="UNKNOWN"`) rather than going silent. It never fabricates a plausible-looking telemetry value.

If `algorithm_package_path` does not resolve to the real `autokart_vehicle_bridge.core` module, the node
still runs (even with the `synthetic` backend fully portable) but falls back to the only safe degenerate
policy it is willing to invent itself — throttle always 0 mV, brake always 100% — and reports this on
every status message as `CORE_LOGIC_UNAVAILABLE:<reason>` (quality `INVALID`). It never reimplements
`ThrottleController`/`CommandGate`/`raw_to_rad`/`brake_pct_for_candidate` locally.

## Known, documented interface gaps (not silently added as new message fields)

Per `docs/VCU_PROTOCOL.md`'s `TEL` frame, the VCU telemetry stream carries `ack_seq` (last command
sequence number acknowledged) and `link_age_ms` (link staleness as the VCU itself sees it), and the
master architecture doc's own list of fields this node class should track includes a `boot_id` concept.
**None of `autokart_msgs/ActuatorFeedback`, `VcuState`, or `SensorDeliveryMetadata` has a field for any of
these**, and the real wire protocol itself has **no `boot_id` concept at all** (`HELLO` carries only
`fw_version`/`proto_version`, not a boot identifier) — this is a genuine gap in the current firmware
protocol, not something this node failed to parse. `ack_seq`/`link_age_ms` are logged
(`get_logger().debug`), not published in any typed message. This is a real, documented gap per the
mission's instruction to say so explicitly rather than silently inventing new message fields — see
DEVELOPER_REPORT.md.

## What this node is not

It does not decide ARM authorization policy, does not implement E-stop logic, does not run a demo-drive
feature (the real `autokart_vehicle_bridge` node's `demo_drive.py` is out of scope here), and does not
replace or modify `kart/ros2_ws/src/autokart_vehicle_bridge`, which remains the live production path.
**If this node is ever run alongside the real `autokart_vehicle_bridge` node against the same physical
VCU, both would be independent USB/serial writers to the same wire — a single-owner-of-the-serial-port
hazard the operator must avoid** (same category of hazard `autokart_odometry` documents for the
`odom -> base_link` TF in this same batch). This node does not and cannot resolve that system-level
conflict by itself.
