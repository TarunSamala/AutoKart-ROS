# Independent validator report — autokart_controller

Sequential validation pass after the reviewer's fixes. All commands below were actually run in this
session; outputs are quoted, not summarized from memory.

## Environment

ROS 2 Humble, colcon. No vehicle, no steering actuator, no VCU, no track. `kart/ros2_ws/install`
sourced for in-place runs only; the clean-workspace runs source `/opt/ros/humble` alone
(`env -i` shell).

## Runs

1. **In-place, full suite** (`PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test/ -q`):
   `50 passed`, repeated three times before the final pair test was added (`49 passed` ×3) to check
   for timing flakiness — none observed.
2. **Fix reproductions** (each fix temporarily reverted, tests re-run, fix restored):
   - ordering/pairing (first-draft "process on path arrival" restored): `2 failed`
     (`test_real_planner_output_drives_a_valid_controller_candidate`,
     `test_path_arriving_before_its_envelope_is_still_processed`); restored → `50 passed`.
   - state reset: before the fix, `test_controller_state_does_not_leak_across_a_reject` failed with
     `AssertionError: 0.08050637711236486`.
   - path provenance (check removed): `2 failed` in `test_pursuit_backends.py`; restored → `20 passed`.
3. **Cross-validation actually exercised** (not skipped): `kart/kart_sim/autonomy.py` present; the
   6 reused-backend success/parity tests (1 build + 4 parametrized offsets + 1 multi-step) ran and passed.
4. **Clean-workspace portability**:
   - `export_node.sh` → bundle with exactly `autokart_controller/` + `autokart_msgs/`.
   - Extracted into `scratchpad/autokart_controller_ws2`; `colcon build --packages-up-to
     autokart_controller` with only Humble sourced → `Summary: 2 packages finished [20.6s]`.
   - Tests inside the bundle → `49 passed, 1 skipped` (the skip is
     `test_pair_with_planner.py: could not import 'autokart_planner.node'` — expected, node 08 is not
     in this bundle).
5. **Live launch from clean workspaces** (planner bundle + controller bundle overlaid; planner given
   test-only braking values `a_brake_min_mps2=2.0, t_total_latency_s=0.1` from a scratch YAML that
   says they are not measurements):
   - `ros2 topic list` showed `/autokart_controller/{diagnostics,motion_command_candidate}` and all
     five `/autokart_planner/*` topics.
   - No inputs: candidate `valid: false, target_speed_mps: 0.0, steering_angle_rad: 0.0`.
   - Streaming odom (y=-0.3, 0.4 m/s) + straight reference path at 20 Hz: planner envelope
     `valid: true, stopping_distance_m: 1.08` (= 0.04 + 0.04 + 1.0 ✓); controller candidate
     `valid: true, target_speed_mps: 0.8, steering_angle_rad: 0.1456602937669184, integrity: 0,
     source: autokart_controller/SIMULATED`; diagnostics `cross_track_error_m: 0.3,
     heading_error_rad: 0.0, speed_tracking_error_mps: 0.4, vehicle_authorized: false,
     published_to_vcu: false`. Steering hand-checked: 0.14566 rad ✓.
   - After the stream stopped: candidate `valid: false`, speed 0, steering 0,
     `reason_codes: ["PLANNER_INVALID"]` (planner meanwhile reporting `PATH_MISSING_OR_STALE`).
   - Cleanup: launch/node processes killed; `ps` afterwards: "no leftover node/launch processes".
     The shared `ros2-daemon` (CLI discovery daemon) was left running.

## Acceptance matrix (software-only)

| Capability | Status | Evidence |
|---|---|---|
| Standalone ament_python build | VALIDATED | in place + clean workspace |
| Candidate + diagnostics topics | VALIDATED | topic list/echo, live |
| Reuses existing Pure Pursuit (`kart_sim`) unmodified | VALIDATED | reused backend + 1e-6 parity |
| Actual formula documented vs brief | VALIDATED | CONTRACT.md, live hand-check |
| Tracking diagnostics (xte, heading, speed error) | VALIDATED | integration test + live echo |
| STOP on missing/stale odom, invalid/missing/mismatched envelope, expired/short path, timeout, startup fault | VALIDATED | 12 fault tests + live post-stream check |
| Works with real node 08 output | VALIDATED | end-to-end test + live chain |
| UNVERIFIED constants labelled, non-positive values fail closed | VALIDATED | config/CONTRACT review, fault test |
| `colcon test` hook | BLOCKED | same setuptools `tests_require` tooling gap as autokart_hp60c; pytest run directly instead |
| Closed-loop tracking quality / tuning | NOT RUN | no plant model run, no vehicle |
| Wheelbase, steering limit, steering rate measurement | NOT RUN | requires the physical kart |
| Behaviour with node 10 (safety) consuming candidates | NOT RUN | node 10 not part of this task |

**Software-only acceptance; not a vehicle-readiness claim.** No authority flag is set anywhere.
