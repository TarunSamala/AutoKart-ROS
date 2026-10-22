# autokart_mission — interface contract

## Topics subscribed

| Topic (param) | Type | QoS | Notes |
|---|---|---|---|
| `odom_topic` (default `/autokart/odom`) | `nav_msgs/Odometry` | sensor data | `child_frame_id` must be `base_link`. |
| `mapping_status_topic` (default `/autokart_mapping/status`) | `autokart_msgs/MissionDebug`, `stage="MAPPING"` | reliable, depth 10 | This node's own `autokart_mapping` default output topic (node 06, this library) — its JSON payload is passed to `PlannerEngine.evaluate(...)` directly, matching exactly what that reused function already expects (`valid`, `source`, `frame_id`, `sample_time_ns`, `tracked_count`, `geometry.{skidpad_topology,exit_centerline_odom,centerline,centerline_status}`). |
| `feedback_topic` (default `/vehicle/actuator_feedback`) | `autokart_msgs/ActuatorFeedback` | reliable, depth 10 | Optional; used only to confirm a physically-measured stop (`stop_feedback_valid`) before the skidpad FSM claims `STOP`. Absent/stale feedback is treated as `stop_feedback_valid=false` (fails closed to `STOP_PENDING`, never fabricates a stop confirmation). |
| `obstacles_topic` (default `/autokart_mission/obstacles`) | `autokart_msgs/ConeArray` | sensor data | `BRAKING_TEST` mission only; must be `frame_id=base_link`, non-ground-truth; only `semantic_class=0` (generic obstacle) cones are used, matching the existing, reused `autokart_planner` adapter's own convention. |

Unlike the existing, reused `autokart_planner/node.py`, this node does **not** separately subscribe the
mapping `ConeArray` (`/autokart_mapping/cones`) purely as a redundant early "are there any cones"
check — `PlannerEngine.evaluate()`'s own `mapping.get("tracked_count") == 0` check (read from the
JSON status payload) already covers that, so one less subscription is needed for equivalent
validation. This is a deliberate simplification, not an unnoticed omission.

## Topics published

| Topic (param) | Type | QoS | Rate |
|---|---|---|---|
| `status_topic` (default `/autokart_mission/status`) | `autokart_msgs/MissionDebug`, `stage="MISSION"` | reliable, depth 10 | one per processed mapping-status input, plus fault heartbeats every 100 ms when input is missing/stale or the algorithm dependency failed to load |

**This node never publishes a path, a `nav_msgs/Path`, or `autokart_msgs/MotionCommand`.** The
reused `PlannerEngine.evaluate(...)` internally computes a candidate path (`path_odom`) as part of
its single return value — that field is read and then **discarded**, never placed on any topic. Per
the master architecture's node registry, full path candidates are node 08 (`autokart_planner`)'s
responsibility, not node 07's; publishing `path_odom` here would blur that boundary. See
`reports/DEVELOPER_REPORT.md` for why this could not be avoided by *not computing* it (the two
responsibilities are not separable inside the existing, reused `mission.py` without modifying that
file, which is explicitly out of scope).

## Status payload (`MissionDebug.payload_json`)

JSON, `schema_version=1`: `mission` (`SKIDPAD`/`BRAKING_TEST`), `source`, `valid`, `reason_codes`,
`frame_id=odom`, `sample_time_ns`, `expires_ns`, `sequence`, `decision` (verbatim from
`PlannerEngine`, e.g. `PATH_CANDIDATE`/`STOP_CANDIDATE`/`EXIT_PATH_CANDIDATE`/`HOLD`/`STOP`),
`target_speed_mps`, `mission_state` (verbatim from `PlannerEngine` — `SkidpadProgress.snapshot()` for
`SKIDPAD`, an ad hoc `{"state": ...}` dict for `BRAKING_TEST`; see "Known FSM gaps" below),
`mission_constraints` (this node's own addition — see below), `stop_point_m` (informational; not a
command), `stopping_distance_m`, `vehicle_authorized` (always `false`), `published_to_vcu` (always
`false`).

### `mission_constraints` — the "outputs: mission constraints" requirement

`{"allowable_branch": <mission_state.state>, "lap_index": <right_laps+left_laps for SKIDPAD, else 0>,
"speed_cap_mps": <target_speed_mps if valid else 0.0>, "mode": <decision>}`. This is a **direct
relabeling of already-computed, already-tested `PlannerEngine` fields**, computed in
`autokart_mission/node.py::mission_constraints()` — not a new safety decision, not a renamed/second
copy of `mission_state`. It exists because the master architecture's node-07 spec names this exact
output shape; the underlying authoritative values are unchanged from `PlannerEngine`'s own,
already-tested output.

## Reused, not re-derived: signed winding number + gate-crossing lap confirmation

The skidpad lap-confirmation logic (`autokart_planner.mission.SkidpadProgress`, reused unmodified via
`planner_package_path`) accumulates a signed angular `travel` term equal to `direction * wrap(angle -
previous_angle)` per pose update — mathematically the same accumulated-winding-number quantity the
master architecture names (`W = (1/2*pi) * sum(wrap(theta_i - theta_i-1))`), just expressed in
radians rather than pre-divided by `2*pi` — **and** additionally requires a discrete gate-crossing
event (`previous_phase > 5.5 and phase < 0.7`, i.e. passing a fixed angular gate near the ring's
closing point) before a lap increments, with the accumulated travel required to be `>= 2*pi - 0.25`
at that crossing. A lap is **not** confirmed by elapsed time or angular progress alone. This was
verified, not assumed — see `test/test_mission_fsm.py::test_right_right_left_left_requires_all_four_gate_crossings_and_terminal_stop`,
ported from the existing, reused adapter's own test suite, and the terminal `STOP` additionally
requires `stop_feedback_valid=True` (a real actuator-feedback confirmation), not merely zero commanded
speed.

## Known FSM gaps against the master architecture's named-state vocabulary (found during this
## session's audit — reported, not silently filled)

- **Skidpad FSM states are logically equivalent but not spelled the same.** The architecture names
  `ENTRY->RIGHT_LOOP_1->RIGHT_LOOP_2->CROSSOVER->LEFT_LOOP_1->LEFT_LOOP_2->EXIT->STOPPING->
  STOP_VERIFIED`. The reused `SkidpadProgress` implements `ENTRY->RIGHT_1->RIGHT_2->TRANSITION->
  LEFT_1->LEFT_2->EXIT->STOP_PENDING->STOP` (plus a `HOLD` fault-ish state on pose-jump/direction
  violations). The transition structure and gate/winding confirmation requirement are the same; the
  literal names are not. This node does not rename them (that would require either editing the reused
  `mission.py`, out of scope, or inventing a second, untested classifier on top of it) — it reports
  `PlannerEngine`'s own state names verbatim in `mission_state.state`. A consumer that expects the
  architecture doc's literal strings will not find them; use this table: `RIGHT_LOOP_1`≈`RIGHT_1`,
  `RIGHT_LOOP_2`≈`RIGHT_2`, `CROSSOVER`≈`TRANSITION`, `LEFT_LOOP_1`≈`LEFT_1`,
  `LEFT_LOOP_2`≈`LEFT_2`, `STOPPING`≈`STOP_PENDING`, `STOP_VERIFIED`≈`STOP`. There is also no
  separate `FAULT` state — pose-jump/off-track conditions land in `HOLD`, which this node reports
  under `reason_codes`, not a `mission_state.state=="FAULT"` value.
- **Braking FSM has no named state machine at all.** The architecture names `WAITING->TRACK_VALID->
  READY->RUNNING->STOP_TRIGGERED->STOPPING->STOP_VERIFIED`, any state `->FAULT`. The reused
  `PlannerEngine._braking()` has **no equivalent state-machine object** — it returns ad hoc
  `mission_state={"state": "BRAKING_APPROACH"}` while cruising and `{"state": "BRAKE_PENDING"}` while
  a stop is pending, with no `WAITING`/`TRACK_VALID`/`READY`/`STOP_TRIGGERED`/`STOPPING`/
  `STOP_VERIFIED`/`FAULT` progression, confirmed directly by
  `test/test_mission_fsm.py::test_braking_fsm_has_no_named_waiting_track_valid_ready_running_stop_states`
  (a real assertion against the reused code's actual output, not just prose). **More significantly:
  `evaluate()` accepts `stop_feedback_valid` as a parameter but `_braking()` never receives or uses
  it** — so, unlike `SKIDPAD`'s `STOP_PENDING -> STOP` transition, `BRAKING_TEST` has no
  measured-stop-confirmation step analogous to `STOP_VERIFIED` at all; a persistent `STOP_CANDIDATE`
  decision is as far as the reused logic goes. This node does not invent a new, untested
  `STOP_VERIFIED`-equivalent state on top of `_braking()`'s output to paper over this — see the
  mission statement's own instruction not to silently fill FSM gaps with guessed logic. This is
  reported here as a real, pre-existing gap in the reused `mission.py`, not introduced or hidden by
  this node.

## No typed mission-constraints message exists yet

Same situation as `autokart_mapping`: `autokart_msgs` has no dedicated mission/FSM message type. This
node reuses `MissionDebug` (already-existing, already the convention for `autokart_planner`'s own
diagnostic output) rather than defining a new message type in this package.

## Parameters

See `config/mission_skidpad.yaml` / `config/mission_braking_test.yaml`. Three lazy-import paths are
required (`algorithm_package_path`, `vehicle_geometry_package_path`, `planner_package_path`) — see
DEPENDENCIES.md for why three, not one. Brake/geometry parameters mirror the existing, reused
`autokart_planner/node.py`'s own parameter names/semantics exactly, including its `-1.0` sentinel
pattern for `camera_to_bumper_m`/`lateral_margin_m`/`footprint_width_m` (negative means "use the
single `GEOMETRY` source of truth's derived default", never a second hardcoded literal).

## What this node is not

It is not the planner (node 08, not yet built in this library): it never publishes a path or a motor
command. It does not perform mapping/geometry (node 06, `autokart_mapping`, this library). It does
not replace or modify `kart/ros2_ws/src/autokart_nodes/autokart_planner/`, which remains the live
production path for the bundled planner+mission node. This is an additive, independent extraction for
the reusable library.
