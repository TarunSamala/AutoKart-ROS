# autokart_planner — interface contract

## Scope: planner-proper, not the mission FSM

This node computes, for an already-chosen reference path: per-point curvature, a curvature-capped
speed profile, forward obstacle clearance, and the braking stop envelope. It has **no mission
state machine** -- no SKIDPAD/BRAKING_TEST progress tracking, no gate-crossing logic. That FSM
lives in `kart/ros2_ws/src/autokart_nodes/autokart_planner/autokart_planner/mission.py` and is
being extracted into a separate standalone node (06/07) by another agent in this session (see
`../AUDIT_PHASE0.md` and the top-level `../README.md`). This node does not import or depend on
that module's internal shape.

## Topics subscribed

| Topic (param) | Type | Expected shape |
|---|---|---|
| `odom_topic` (default `/autokart/odom`) | `nav_msgs/Odometry` | `pose.pose.position.{x,y}` in `odom` frame; `twist.twist.linear.{x,y}` speed. Must arrive at least every `odom_max_age_s` (default 0.15 s) or the next path evaluation is rejected `ODOM_MISSING_OR_STALE`. |
| `reference_path_topic` (default `/autokart_planner/reference_path`) | `nav_msgs/Path` | `header.frame_id="odom"`, `header.stamp` = the sample time this path is valid for, `>=2` poses, finite `position.{x,y}`. This is the node's one required "mission/map" input — in the full stack it is expected to come from the mission/mapping node (06/07), not yet built in this library; a synthetic generator is sufficient for this node's own tests (see `test/`). |
| `obstacles_topic` (default `/autokart_planner/obstacles`) | `autokart_msgs/ConeArray` | **Must already be expressed in the `odom` frame** (`frame_id="odom"`, `ground_truth=false`) — this node does no frame transform. A cone's `position.{x,y}` and `position_std_m` are read as a circular obstacle (`radius_m = max(0, position_std_m)`). A mismatched frame is logged and the update is dropped, not transformed or guessed. |

## Topics published

| Topic | Type | Rate |
|---|---|---|
| `/autokart_planner/path` | `nav_msgs/Path` | one per processed reference-path message (or per 0.1 s timeout tick when input is stale) |
| `/autokart_planner/safety_envelope` | `autokart_msgs/SafetyEnvelope` | same cadence |
| `/autokart_planner/speed_profile` | `std_msgs/String` (JSON) | same cadence — see "Why std_msgs/String, not MissionDebug" below |

`/autokart_planner/path` carries the (unmodified) candidate path when valid, and an **empty**
`poses` list when invalid — matching this repository's own documented convention
(`docs/NODE_CONTRACTS.md`: "Invalid input publishes an empty path and reason; no path persists
after expiry").

### `/autokart_planner/safety_envelope` (`autokart_msgs/SafetyEnvelope`) field mapping

| Field | Meaning here |
|---|---|
| `maximum_speed_mps` | `min()` over the whole published speed profile (the most conservative upcoming commanded speed), `0.0` when invalid |
| `maximum_acceleration_mps2` | not computed by this node (`0.0`) — no acceleration-limit formula was in scope for node 08 |
| `guaranteed_deceleration_mps2` | echoes the `a_brake_min_mps2` parameter (see "Physical constants" below) |
| `clear_distance_m` | forward arc-length distance (along the path, from the vehicle's projected position) to the nearest footprint-intersecting obstacle, or to the path's end if none |
| `stopping_distance_m` | `d_required = v*t_total_latency + v^2/(2*a_brake_min) + d_margin`, evaluated at the odometry-reported current speed |
| `lateral_limit_mps2` | echoes the `a_lat_max_mps2` parameter |
| `limiting_reason` | `""` when valid; otherwise the reason code that invalidated this cycle (e.g. `CORRIDOR_BLOCKED`, `ODOM_MISSING_OR_STALE`) |
| `valid` | `clear_distance_m >= stopping_distance_m` and all upstream input/configuration checks passed |
| `allowed_region` | not populated (left as the message's zero-length default) — this node does not compute a 2D drivable-region polygon |
| `data_source` | **only set if the field exists** — see "Environment finding" below |

### `/autokart_planner/speed_profile` (`std_msgs/String`, JSON) payload schema

```json
{
  "schema_version": 1, "source": "SIMULATED", "processed_at_monotonic_ns": 0,
  "valid": true, "reason_codes": [], "decision": "PATH_CANDIDATE",
  "sample_time_ns": 0, "sequence": 0, "frame_id": "odom", "stage": "PLANNING",
  "speed_profile_mps": [0.0, ...], "curvature_profile_m_inv": [0.0, ...],
  "vehicle_authorized": false, "published_to_vcu": false
}
```
`speed_profile_mps[i]` / `curvature_profile_m_inv[i]` align index-for-index with the published
`/autokart_planner/path`'s `poses[i]` when `valid=true`; both arrays are empty when invalid.

### Why `std_msgs/String`, not `autokart_msgs/MissionDebug` — environment finding

This repository's own convention (see `docs/NODE_CONTRACTS.md`, and `kart/ros2_ws`'s existing
`autokart_planner`/`autokart_controller` nodes) is to carry exactly this kind of non-actuating,
structured diagnostic payload on `autokart_msgs/MissionDebug.payload_json`, and that was this
node's first design. However: **`MissionDebug.msg` exists in the canonical source tree
(`kart/ros2_ws/src/autokart_msgs/msg/MissionDebug.msg`) but is verifiably NOT built into the
already-built `kart/ros2_ws/install` this library's own top-level README instructs sourcing** —
confirmed with `python3 -c "import autokart_msgs.msg as m; print(dir(m))"` after sourcing exactly
the commands this README specifies: `MissionDebug` is absent from the installed package's
`__init__.py`, i.e. a stale-install/source drift in this dev environment, not introduced by this
change. Depending on an interface this environment cannot actually provide would make this node
fail to build/import here, so `std_msgs/String` (the most standard message available) carries the
same JSON payload instead. If `kart/ros2_ws`'s `autokart_msgs` is ever rebuilt and `MissionDebug`
becomes importable, migrating is a one-import, one-type swap, not a redesign.

(A related, smaller finding: the installed `autokart_msgs/SafetyEnvelope` is also missing the
`data_source` field that the current `.msg` source declares — handled with a `hasattr()` guard,
see `node.py::publish`, rather than reported as a hard failure, since the rest of that message is
usable.)

## Formulas (master architecture doc §9/§10, as implemented in `algorithms.py`/`planner_core.py`)

- **Curvature**: `kappa = (x'y'' - y'x'') / (x'^2 + y'^2)^1.5`, via centered finite differences on
  the path's point sequence (`BuiltinAlgorithmBackend`), or the existing, already-tested discrete
  three-point signed-circle-fit curvature in `autokart_perception.curvature.profile_centerline`
  (`ReusedAlgorithmBackend`, `algorithm_backend: reused`). See DEPENDENCIES.md for which to use
  when, and `test_algorithms.py` for the cross-validation that proves they agree.
- **Curvature speed cap**: `v_curve = sqrt(a_lat_max_mps2 / max(abs(kappa), curvature_eps_m_inv))`,
  then `target_speed = min(speed_limit_mps, v_curve)` per path point.
- **Stop envelope**: `d_required = v*t_total_latency_s + v^2/(2*a_brake_min_mps2) + d_margin_m`,
  evaluated with the odometry-reported current speed. Exactly the formula already implemented (and
  independently tested) as `autokart_perception.shared_core.BrakeCapability.stopping_distance` —
  `BuiltinAlgorithmBackend` re-derives the same one-line formula for zero-dependency operation,
  cross-validated against the reused implementation in `test_algorithms.py`.

## Physical constants — what is measured vs. UNVERIFIED (read before trusting any output)

| Parameter | Status | Why |
|---|---|---|
| `footprint_width_m` | MEASURED (when left at its 0.0 default) | derived from `bumper_width_m` + `2*lateral_margin_m`, a point-in-time copy of `kart/ros2_ws/src/autokart_bringup/config/vehicle_geometry.yaml`'s measured values (see `geometry_params.py`); set `vehicle_geometry_yaml_path` to read that file directly instead of trusting the copy. |
| `a_lat_max_mps2` (default `2.0`) | **UNVERIFIED nominal placeholder** | this repository has not measured this kart's peak lateral grip; see `reports/SENIOR_REVIEWER_REPORT.md`. |
| `a_brake_min_mps2` (default `0.0`) | **unconfigured by default, on purpose** | `0.0` is not a guessed deceleration — it makes every stop-envelope evaluation fail closed (`STOPPING_DISTANCE_BACKEND_ERROR:...BRAKE_CAPABILITY_UNQUALIFIED`) until an operator sets a real, measured value. See `PROJECT_INDEX.md`: "stopping performance remain unknown or unverified." |
| `t_total_latency_s` (default `0.0`) | unconfigured by default | same reasoning as above; `0.0` only means "not yet measured," not "zero latency." |
| `d_margin_m` (default `1.0`) | commissioning default, not measured | matches the existing `kart/ros2_ws` planner's own `brake_margin_m` default and rationale. |

## Fault behavior

On any of: missing/stale odometry, a too-short/non-finite/expired/future-timestamped reference
path, an unqualified brake capability or lateral limit, a malformed obstacle, or an unavailable
algorithm dependency, this node:
- publishes an **empty** `/autokart_planner/path`,
- publishes `/autokart_planner/safety_envelope` with `valid=false` and a `limiting_reason`,
- publishes `/autokart_planner/speed_profile` with `valid=false`, empty profile arrays,
- **keeps repeating** that status at the timer's cadence (never a single report followed by
  silence) for as long as the condition persists,
- never reuses the previous cycle's path/speed profile.

This node never decides to stop the vehicle; it only reports a candidate. `autokart_safety`
(node 10, not built in this library) is the authority that must treat an invalid envelope as a
blocking input. `vehicle_authorized` is always `false` in this node's output.
