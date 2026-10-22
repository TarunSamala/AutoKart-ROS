# ROS node contracts (draft, RC-02)

## VCU Stage 0 bench status adapter (2026-10-17)

`autokart_diagnostics/bench_vcu_status` is a **read-only** decoder for the
existing NOOP-only ESP32-S3 micro-ROS bench firmware. It subscribes to
`/autokart/bench/status` (`std_msgs/UInt64MultiArray`, exactly eight words as
specified in `/home/mark1/Desktop/autokart-remote/docs/INTERFACE_CONTRACT.md`)
and publishes `/autokart/bench/vcu_status`
(`autokart_msgs/BenchVcuStatus`). It does not publish to the bench command
topic, `/control/motion_command`, `/vcu/state`, or any actuator interface.

The typed status carries a **host receipt timestamp**, the VCU's separate
monotonic uptime, boot ID, last accepted NOOP sequence/age, Agent flag, raw
bench fault bits, packet validity and staleness. `source=BENCH_USB_NOOP` and
`vehicle_authorized=false` are fixed. A missing packet for 500 ms publishes
`stale=true`, `packet_valid=false`; malformed, regressed, wrong-version or
non-SAFE packets are rejected immediately. `packet_valid=true` means only that
the bench status packet obeyed this format. It does **not** certify actuator
feedback, physical stopping, or an independent E-stop.

This separate bench topic is not ingested as LIVE vehicle state by the
dashboard gateway. Production VCU request/feedback IDL, transport integrity,
as-built sensor mapping and physical authorization remain open M4 gates.

## RC-03–05 frozen software contract (2026-10-16)

The following interface is for diagnostic and replay integration. It grants no VCU
authority. `MissionDebug` is a status companion; consumers must match its
`measurement_time`, `cycle` and source before accepting a `ConeArray`. A missing
or mismatched companion invalidates the data. No new ROS message schema is added.

| Producer | Topic / type | Frame and time | Validity and expiry |
|---|---|---|---|
| Perception | `/perception/cones` / `autokart_msgs/ConeArray` | `base_link`, metric X forward/Y left; capture time in `measurement_time`, publication time in `delivery_time` | Only qualified observations. `ground_truth=false` even for recorded observations; source lives in companion. Never emit LIVE cones while RAW16 scale, registration or extrinsics are unverified. |
| Perception | `/autokart/perception/status` / `MissionDebug`, stage `PERCEPTION`; `/dashboard/debug`, stage `perception` | Same capture time and cycle as the cone sample | JSON: `schema_version=1`, `source` (`LIVE`, `REPLAY`, `SIMULATED`), `valid`, `reason_codes`, `frame_id`, `sample_time_ns`, `expires_ns`, `sequence`, model/calibration provenance, `vehicle_authorized=false`. Status is sent even when cones are withheld. |
| External estimator/fixture | `/autokart/odom` / `nav_msgs/Odometry` | Header `odom`, child `base_link`, same clock domain as cone/status sample | Pose and twist covariance finite; <=150 ms sample age in LIVE, monotonically increasing timestamps. REPLAY/SIMULATED must use an explicitly tagged run; no fabricated LIVE odometry. |
| Mapping | `/autokart/mapping/cones` / `ConeArray` | `odom` frame, observed map; measurement time of latest accepted input | Tracked IDs belong to `LocalConeMap`; source IDs remain in perception status. No publication of a valid observed map on invalid odom/perception. Bounded to 256 items; expires after 10 s. |
| Mapping | `/autokart/mapping/status` / `MissionDebug`, stage `MAPPING`; `/dashboard/debug`, stage `mapping` | Same latest input time and mapping cycle | JSON: source, validity, reasons, pose stamp, tracked map count, Delaunay/Gabriel/gates and observed skidpad hypothesis. A rejected update clears the current published map and is reflected in status. |
| Controller | `/autokart/controller/motion_command_candidate` / `autokart_msgs/MotionCommand`; `/autokart/controller/status` / `MissionDebug`, stage `CONTROL`; `/dashboard/debug`, stage `control` | Inputs: planner path+status (matching `sample_time_ns`), `/autokart/odom` (<=150 ms from path stamp), optional `/vehicle/actuator_feedback` | Candidate only, `valid=false` with zero speed and a reason code on any missing/stale/mismatched input; expiry is checked in the sample time domain (pose stamp), never host wall clock. Published to a non-authoritative topic; not subscribed by `vcu_bridge`; `vehicle_authorized=false`. A future VCU node subscribes to the candidate topic and applies its own gates. |
| Planner | `/autokart/planner/path_candidate` / `nav_msgs/Path`; `/autokart/planner/status` / `MissionDebug`, stage `PLANNING`; `/dashboard/debug`, stage `planning` | Path in `odom` with expiry in status | Read-only path/speed/STOP candidate. No `MotionCommand`, `/control/motion_command`, or VCU publisher. Invalid input publishes an empty path and reason; no path persists after expiry. |

All timestamps are integer nanoseconds in JSON and ROS `builtin_interfaces/Time`
on the wire. A replay clock is not compared to the host wall clock; replay
fixtures must carry the same time domain for frames and odometry. LIVE samples
are checked against the node ROS clock. A reset, backwards stamp, duplicate
sequence, missing TF, stale source status, or source mismatch clears downstream
state. `SIMULATED` and `REPLAY` never imply measured camera or braking capability.
The current `ConeArray` cannot express source or validity, which is why these
paired status messages are mandatory. The gateway remains read-only.

`autokart_mapping` defaults to `source_mode=SIMULATED`; `LIVE` additionally
requires explicit `live_odom_verified=true` and nonzero odometry covariance.
It uses the latest odometry sample no later than capture and no more than
150 ms old. Its output IDs are map IDs; RC-03 IDs are per observation. The
`MissionDebug` mapping geometry is consumed only by the disarmed diagnostic
planner in this sprint. A typed map/topology transport and qualified TF policy
remain prerequisites before controller or VCU integration.

These contracts reuse the current ROS messages and existing algorithm APIs. They do not authorize live actuation. Topic names and rates remain to be frozen before node entry points are added; this document records the data ownership and invalidation rules so implementation does not invent sensor state.

| Package | Existing input contract | Existing output contract | Algorithm source of truth | Gate |
|---|---|---|---|---|
| `autokart_perception` | Paired RGB/depth SDK frames with matched capture time, camera info, calibration and model provenance. No ROS source topic is currently selected. | `autokart_msgs/ConeArray` plus `SensorDeliveryMetadata`; positions must declare `base_link`, units, uncertainty, source and capture/delivery times. | `autonomy_stack_phase1/phase_01_perception/src/autokart_perception/{camera,detector,depth,pipeline}.py` | No measured cone output until the camera configuration, calibration and target model are selected and tested. |
| `autokart_mapping` | `ConeArray` in `base_link` plus fresh `nav_msgs/Odometry` for `odom -> base_link`. | Existing `MissionDebug` stage payload for diagnostics; a persistent map message/schema is not present yet. | `phase_01_perception/src/autokart_perception/{geometry,gabriel,boundaries,gates,centerline,curvature,shared_core}.py` | Reject stale, invalid, unsupported-frame or unqualified input; clear prior valid map/path state. |
| `autokart_planner` | Fresh observed map, odometry, mission state, obstacle observations, footprint and qualified stop capability. Some required input contracts are absent. | Candidate path/diagnostic through existing messages; add a typed path contract only with migration tests. | `shared_core.AlgorithmCore` and existing mission/planning algorithms; do not encode FKDC world truth as observations. | No valid path means invalid candidate/hold. Brake planning requires measured or explicitly validated capability. |
| `autokart_controller` | Candidate trajectory, synchronized odometry, measured VCU feedback and safety limits. | `autokart_msgs/MotionCommand` as a bounded, expiring **candidate** only. | `kart/kart_sim/kart_sim/autonomy.py::PurePursuitController`; current shared core invokes it optionally. | The node is not in `full_stack.launch.py`; no ROS-to-hardware transport is introduced. Invalid/stale input clears the prior candidate. |

All message outputs must preserve source (`LIVE`, `REPLAY`, or `SIMULATED`), frame, units, sequence, timestamp/freshness, validity and rejection reason. The existing `ConeArray` has no explicit source or per-cone covariance field, and `MissionDebug` is JSON diagnostic data, not a typed control input. These gaps remain recorded rather than silently inferred. Any schema changes require migration and round-trip tests.

## Existing input and output package contracts

- `autokart_msgs` defines `ConeArray`, `VehicleState`, `ActuatorFeedback`, `SafetyEnvelope`, `MotionCommand`, `VcuState`, and `MissionDebug`.
- `autokart_sim_bridge/vcu_bridge.py` subscribes to `control/motion_command` and stores the latest high-level request; it does not communicate with an ESP32.
- `autokart_dashboard_gateway` consumes ROS diagnostics/messages and exposes read-only HTTP/WebSocket/replay data to the existing `phase_sim` UI.
- No IMU driver, odometry estimator, mission supervisor, or verified VCU telemetry transport is added by these four packages.
