# autokart_controller — interface contract

## Authority boundary (read first)

This node produces a **command candidate only**. It has **no authority to command the vehicle**,
armed or disarmed:
- its output topic `/autokart_controller/motion_command_candidate` is not subscribed by any
  actuator/VCU transport in this library, and this node never publishes to `/control/*` or any VCU
  bridge topic;
- `MotionCommand.integrity` is always `0` (no VCU CRC signing — `kart_sim`'s `MotionCommand.signed()`
  is deliberately not used), so a VCU bridge applying its own integrity gate must reject it;
- every diagnostics payload carries `"vehicle_authorized": false, "published_to_vcu": false`.

`autokart_safety` (node 10, built by another agent, not part of this package) is the authority that
decides whether any candidate may ever reach an actuator. Consistent with `docs/NODE_CONTRACTS.md`'s
existing controller row: "Candidate only ... not subscribed by `vcu_bridge`; `vehicle_authorized=false`."

## Topics subscribed

| Topic (param) | Type | Expected shape / use |
|---|---|---|
| `odom_topic` (`/autokart/odom`) | `nav_msgs/Odometry` | pose x/y, yaw from quaternion, speed = `hypot(twist.linear.x, .y)`. Must have been received within `pose_max_age_s` (0.15 s) when a path is processed — i.e. it must **stream**. |
| `path_topic` (`/autokart_planner/path`) | `nav_msgs/Path` | `header.stamp` is the pairing key; `>=2` finite poses in `odom` frame. Default topic is node 08's output. |
| `safety_envelope_topic` (`/autokart_planner/safety_envelope`) | `autokart_msgs/SafetyEnvelope` | `timestamp` must equal the path's stamp; `valid` gates the cycle; `maximum_speed_mps` caps speed. |
| `speed_profile_topic` (`/autokart_planner/speed_profile`) | `std_msgs/String` (JSON) | node 08's payload: `sample_time_ns`, `speed_profile_mps[i]` aligned with path pose `i`. Optional. |
| `actuator_feedback_topic` (`/vehicle/actuator_feedback`) | `autokart_msgs/ActuatorFeedback` | `valid=false` ⇒ `brake_stuck` ⇒ drive forced to 0 (same convention as the existing `kart/ros2_ws` controller). Optional; no staleness check (see "Known gaps"). |

### Stamp pairing (why, and how)

Node 08 publishes path, envelope and speed profile for one sample time on three topics; ROS gives no
cross-topic ordering guarantee, and node 08 in fact publishes the path *first*. This node buffers
each by stamp (bounded to 8 entries each) and processes a path only when its **same-stamp** envelope
has arrived, plus its speed profile if the envelope is valid. If the profile hasn't arrived within
`pair_timeout_s` (0.1 s) the path is processed with the envelope's flat `maximum_speed_mps` cap
instead; if the envelope never arrives, the path is rejected `PLANNER_STATUS_MISSING`. An envelope
for a *different* stamp is never borrowed. (This pairing replaced a first draft that would have
emitted STOP forever when wired to the real node 08 — see `reports/SENIOR_REVIEWER_REPORT.md`.)

## Topics published

| Topic | Type | Rate |
|---|---|---|
| `/autokart_controller/motion_command_candidate` | `autokart_msgs/MotionCommand` | one per processed path, plus one per 0.1 s timer tick while input is stale/missing or the node is in a startup fault |
| `/autokart_controller/diagnostics` | `std_msgs/String` (JSON) | same cadence |

`MotionCommand` fields: `mode="speed_steering"`, `target_speed_mps`, `steering_angle_rad`,
`use_curvature=false`, `acceleration_limit_mps2=deceleration_limit_mps2=0.0` (not computed here),
`integrity=0`, `source="autokart_controller/<source>"`, `expiry = timestamp + command_validity_s`
(0.1 s, mirroring `kart_sim.autonomy.ControllerManager.validity_s`) when valid, `expiry = timestamp`
when invalid. (The existing `kart/ros2_ws` controller sets `expiry = timestamp` for every candidate,
i.e. each one is born expired; changed here so a consumer's expiry check is meaningful.)

Diagnostics JSON: `schema_version, source, valid, reason_codes, sample_time_ns, sequence, brake,
drive, cross_track_error_m, heading_error_rad, speed_tracking_error_mps, vehicle_authorized=false,
published_to_vcu=false`. `std_msgs/String`, not `autokart_msgs/MissionDebug`, for the same verified
environment reason as node 08 (`MissionDebug` is absent from the installed `kart/ros2_ws` interface
build — see `../autokart_planner/CONTRACT.md`).

## The ACTUAL Pure Pursuit formula (verified against `kart/kart_sim/autonomy.py`, not assumed)

The mission brief states `L_d = clamp(L0 + k_v*|v|, Lmin, Lmax)`, `alpha = atan2(y_la, x_la)`,
`delta = atan(2*L*sin(alpha)/L_d)`. The existing, tested implementation this node reuses is:

```
lookahead = max(0.1, lookahead_m + speed_lookahead_gain_s * max(0, u))     # L0=lookahead_m, k_v=gain
target    = first trajectory point, from the nearest point onward, with range >= lookahead
            (else the last point)
(x_local, y_local) = target rotated into the vehicle body frame
distance  = max(0.1, hypot(x_local, y_local))
requested = atan2(2 * wheelbase_m * y_local, distance**2)
steering  = clamp(clamp(requested, last - rate*dt, last + rate*dt), -max_steer, +max_steer)
```

Differences from the brief's notation, documented rather than "fixed":
1. **No upper lookahead clamp (`Lmax`)**; the lower clamp `Lmin` is a hardcoded `0.1 m`, not a
   parameter. Uses `max(0, u)`, not `|v|` (reverse speed contributes nothing).
2. `alpha` is never computed explicitly. `atan2(2L*y_local, distance^2)` is the curvature form
   (`kappa = 2*y_local/distance^2`, `delta = atan(L*kappa)`). Since `y_local = distance*sin(alpha)`,
   this equals `atan(2L*sin(alpha)/distance)` — the brief's formula with `L_d` replaced by the
   **actual range to the selected point** (which is >= the nominal lookahead, not equal to it).
3. Steering is **rate-limited** (`maximum_steering_rate_radps * dt`) before the magnitude clamp.
4. The speed command is the selected target point's speed, clamped to
   `[0, min(maximum_speed_mps, envelope maximum_speed_mps)]` by the reused `LongitudinalPidController`,
   which also yields `drive`/`brake` (kp=0.35, ki=0.12, kd=0, integral limit 4.0, conditional anti-windup).

Tracking diagnostics are reused, not recomputed: `cross_track_error_m = y_local` and
`heading_error_rad = wrap(target.heading - yaw)` from `PurePursuitController`'s `ControllerOutput`;
`speed_tracking_error_mps` from `LongitudinalOutput.error_mps`. (`ControllerManager.command()` drops
these when packaging a `MotionCommand`; this node calls the two controllers directly to keep them.)

## Physical constants — measured vs UNVERIFIED

| Parameter | Default | Status |
|---|---|---|
| `wheelbase_m` | 1.0 | **UNVERIFIED nominal.** Not in `vehicle_geometry.yaml` (which has only camera/bumper geometry); never measured in this repo. Same number as the existing controller's code default. |
| `maximum_steering_rad` | 0.6 (~34.4°) | **UNVERIFIED nominal mechanical limit.** No steering-travel measurement exists in this repo. |
| `maximum_steering_rate_radps` | 3.0 | **UNVERIFIED nominal.** |
| `lookahead_m`, `speed_lookahead_gain_s` | 1.5, 0.15 | tuning values from `kart_sim`, not physical constants |
| `maximum_speed_mps` | 5.0 | operational cap from the existing code default; the planner envelope normally caps lower |
| PID gains | 0.35 / 0.12 / 0 | inherited from `kart_sim.autonomy.LongitudinalPidController`, not tuned on hardware |

`wheelbase_m <= 0` or `maximum_steering_rad <= 0` fails closed at startup (`WHEELBASE_UNQUALIFIED` /
`STEERING_LIMIT_UNQUALIFIED`), reported every timer tick.

## Fault behavior

Any of: invalid planner envelope, envelope missing for the path's stamp, missing/stale odometry,
expired / future-stamped / too-short / non-finite path, invalid speed or dt, a backend compute error,
no path within `path_max_age_s` (including never receiving one at all), or a startup fault ⇒
`MotionCommand` with `valid=false`, `target_speed_mps=0.0`, `steering_angle_rad=0.0`, a reason code in
diagnostics — **repeated every 0.1 s while the condition persists**. A rejected cycle also resets the
steering rate-limiter memory, the PID state and the dt clock, so the first command after recovery is
computed fresh rather than shaped by pre-fault state. The previous valid command is never re-sent.

## Known gaps

- `ActuatorFeedback` has no staleness gate: the last-received `valid` flag persists.
- No `Lmax` lookahead clamp (faithful to the reused implementation, see above).
- `acceleration_limit_mps2`/`deceleration_limit_mps2` are not computed (`0.0`).
