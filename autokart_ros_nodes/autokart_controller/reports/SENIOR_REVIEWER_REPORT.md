# Senior reviewer report — autokart_controller

A distinct, sequential adversarial pass over the developer's source and tests (no multi-agent runtime
exists here; this is a genuine second look, not a simulated one). Every defect below was reproduced
on the pre-fix code by a test before the fix was trusted; the reproduction is described per item and
was re-run by the validator.

## Finding 0 — guessed physical constants presented as ordinary defaults (the "no guessed physical
calibration" violation the architecture doc warns against)

In the reused source, `kart/ros2_ws/.../autokart_controller/engine.py` (`ControllerEngine`),
`node.py` (`declare_parameter`) and `kart/kart_sim/autonomy.py` (`PurePursuitController`) all
hardcode `wheelbase_m=1.0`, `maximum_steering_rad=0.6`, `maximum_steering_rate_radps=3.0` with no
indication they are unmeasured; the repository's only measured-geometry file
(`kart/ros2_ws/src/autokart_bringup/config/vehicle_geometry.yaml`) has no wheelbase or steering
figure, and `PROJECT_INDEX.md` says vehicle physical capability is unverified. Note the steering
limit in the code is 0.6 rad (~34.4°), not the "±20°" the brief anticipated — checked, not assumed.
**Resolution in this node**: all three are ROS parameters labelled **UNVERIFIED nominal** at every
point of use (node.py comment, both config YAMLs, CONTRACT.md table); values kept identical to the
existing code rather than replaced with a different guess; `wheelbase_m<=0` and
`maximum_steering_rad<=0` fail closed (`WHEELBASE_UNQUALIFIED`, `STEERING_LIMIT_UNQUALIFIED`) — the
reused `PurePursuitController` itself validates wheelbase but not the steering limit, so the reused
backend now checks it too. The existing `kart/ros2_ws` code was not modified (out of scope).

## Defects found and fixed

1. **Wired to the real planner, the controller would have output STOP forever (cross-topic
   ordering).** The first draft processed on path arrival and required an *already-received*
   same-stamp envelope. Node 08 publishes path → envelope → speed profile (and ROS guarantees no
   cross-topic order regardless), so every cycle failed `PLANNER_INVALID`. The developer's own
   integration test published the envelope first and hid this. **Reproduced**:
   `test_path_arriving_before_its_envelope_is_still_processed` (planner's real order) and
   `test_real_planner_output_drives_a_valid_controller_candidate` (the real `PlannerNode` driving
   this node) both fail with the first-draft logic temporarily restored (`2 failed`). **Fixed**:
   stamp-keyed pairing buffers (bounded), processing when path + same-stamp envelope (+ profile if
   valid) are present, `pair_timeout_s` fallback to the envelope's flat cap, and
   `PLANNER_STATUS_MISSING` rejection when the envelope never arrives
   (`test_path_whose_envelope_never_arrives_is_rejected_not_dropped`). A startup warning flags
   `pair_timeout_s + timer period >= path_max_age_s`, which would otherwise silently expire every
   late-profile path (found when a test using `path_max_age_s=0.15` failed for exactly that reason).

2. **Pre-fault controller state leaked into the first command after recovery.** The steering
   rate-limiter memory and PID integral survived a rejected cycle, although the node had just
   published steering 0. **Reproduced**: `test_controller_state_does_not_leak_across_a_reject` —
   on-path, straight-ahead pose immediately after a fault produced **0.0805 rad** of steering instead
   of 0. A related path: `dt` was computed from wall time across the whole fault gap, so a multi-
   second gap scaled `rate*dt` enough to disable the steering rate limiter and dumped `error*dt` into
   the integral. **Fixed**: `ControllerCore.reject()` calls `backend.reset()` (builtin: zero state;
   reused: fresh `PurePursuitController` + public `LongitudinalPidController.reset()` — no private
   attribute poking), and the node resets its dt clock after any invalid cycle.

3. **Configured dependency path silently not honoured (`pursuit_backend: reused`).** This machine has
   an editable pip install whose meta-path finder resolves `kart_sim` before `sys.path`, so a
   `kart_sim_package_path` pointing at a directory with no `autonomy.py` "succeeded" by importing the
   real `kart_sim` from elsewhere — i.e. operators could believe they were running code from path A
   while running code from path B. Node 08's earlier front-of-`sys.path` fix does not cover this.
   **Reproduced**: `test_reused_backend_rejects_path_without_kart_sim` failed (`DID NOT RAISE`) on the
   first draft; the deterministic `test_reused_backend_rejects_module_resolved_outside_configured_path`
   fails with the check removed. **Fixed**: `DEPENDENCY_PATH_MISMATCH` unless each imported module's
   `__file__` lies under the configured path. Same check ported to node 08 (its equivalent test also
   fails with the check removed). `autokart_hp60c` has the same gap; not modified, flagged.

4. **`MotionCommand.expiry` equalled `timestamp`** (inherited from the existing node), so every
   candidate was born expired and a consumer's expiry check could not distinguish fresh from stale.
   Changed to `timestamp + command_validity_s` (0.1 s, mirroring `ControllerManager.validity_s`) for
   valid candidates; invalid ones still expire at their stamp. Judgement-level change, documented in
   CONTRACT.md; node 10 (not built here) must decide how it uses expiry.

## Checked and found correct

- Builtin vs reused parity: single-shot over four lateral offsets and a six-step stateful sequence
  agree to 1e-6 on steering, speed, brake, cross-track and heading error.
- Live hand-check (validator run): kart at y=-0.3, v=0.4 m/s, straight path → lookahead 1.56 m →
  target (2,0) → `atan2(2·1·0.3, 4.09) = 0.14566` rad, matching the published 0.1456603.
- Fault paths all publish `valid=false`, speed 0, steering 0, repeatedly (not one-shot), including
  never-received path, missing dependency, unqualified wheelbase.
- No hardcoded project path in source (grep: only config comments and docs).

## Not re-verified

The `kart_sim` algorithms themselves (pre-existing, unchanged) beyond formula reading and parity;
PID gains and lookahead tuning are not validated against any vehicle.
