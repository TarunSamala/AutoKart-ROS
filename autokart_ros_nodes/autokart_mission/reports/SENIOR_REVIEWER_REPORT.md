# Senior reviewer report — autokart_mission

Performed as a distinct, sequential review pass over the developer's actual source (same convention as
`autokart_hp60c`'s and `autokart_mapping`'s own reviewer passes: a genuine sequential second look, not
a simulated second agent).

## Defect found and fixed (same class as autokart_mapping's, applied proactively here)

**All three lazy-import paths had the same blank-path-resolves-to-cwd hazard** that
`autokart_mapping`'s reviewer pass found and fixed (`Path("").expanduser().is_dir()` is `True`).
Because this node's `MissionAlgorithm` validates three independent paths
(`algorithm_package_path`, `vehicle_geometry_package_path`, `planner_package_path`), the same defect
pattern applied three times over — an unconfigured deployment (any one of the three parameters left
at its shipped blank default) could, on a host where the corresponding package happens to be globally
importable some other way, silently "work" instead of failing closed. This was caught and fixed in
`autokart_mission/algorithm.py` *before* first test execution (informed directly by the mapping
package's own reviewer finding, applied here as a proactive fix rather than waiting to rediscover it
independently) — each of the three parameters is explicitly checked for blank/whitespace-only input
and rejected with a labeled `<PARAM>_NOT_CONFIGURED` error before any `Path(...).is_dir()` call.
**Verified** via `test/test_algorithm.py::test_blank_paths_fail_closed_instead_of_resolving_to_cwd`,
which exercises all three parameter positions independently.

## Checked and found correct

- **Winding-number + gate-crossing requirement is real, not merely plausible from reading the
  source.** Ran `test/test_mission_fsm.py::test_right_right_left_left_requires_all_four_gate_crossings_and_terminal_stop`
  against the actual, unmodified `PlannerEngine`/`SkidpadProgress` (not a mock): confirmed 2 right
  laps, 0 left laps, state `TRANSITION` after the right-loop sequence; 2 right, 2 left, state `EXIT`
  after the left-loop sequence; `STOP_PENDING` (not `STOP`) while `stop_feedback_valid=False` at the
  exit point; `STOP` only once `stop_feedback_valid=True`. This is the specific property the mission
  statement asked to verify, verified against real behavior rather than taken on the developer's word.
- **Braking FSM gap is real, not a misreading of the source.** Ran
  `test/test_mission_fsm.py::test_braking_fsm_has_no_named_waiting_track_valid_ready_running_stop_states`
  and independently re-read `_braking()`'s full 110-line body: confirmed `stop_feedback_valid` is a
  parameter of `evaluate()` but `_braking(mapping, pose, stamp, obstacles)` is called with no such
  argument and never references it; confirmed every `mission_state` value `_braking()` can return is
  one of `{"state": "BRAKING_APPROACH"}` / `{"state": "BRAKE_PENDING"}` — no occurrence of `WAITING`,
  `TRACK_VALID`, `READY`, `RUNNING`, `STOP_TRIGGERED`, `STOPPING`, `STOP_VERIFIED`, or `FAULT` anywhere
  in that function. Agrees with the developer's report; not rubber-stamped.
- **`path_odom` is genuinely discarded, not accidentally leaked.** Grepped `autokart_mission/node.py`
  for `path_odom` and for any `Path`/`MotionCommand` message construction: neither appears. The
  `publish()` method's payload dict has no path-shaped field. Confirmed by reading the publisher
  list in `MissionNode.__init__`: exactly one publisher (`status_pub`, `MissionDebug`).
- **`mission_constraints` is a pure relabeling, not a second decision path.** Checked
  `mission_constraints()` in `node.py`: it reads only already-computed `result` dict fields
  (`mission_state`, `target_speed_mps`, `decision`, `valid`) and performs no new threshold comparison,
  physics, or FSM logic of its own — confirmed it cannot diverge from `PlannerEngine`'s own output in
  any case that matters (the `speed_cap_mps=0.0 if not valid` branch is the only conditional, and it
  matches `PlannerEngine`'s own convention of zeroing speed on any invalid result).
- **Three-path import order.** Confirmed by direct execution (not just reading) that importing
  `autokart_vehicle_geometry.geometry` before `autokart_planner.mission` is necessary and sufficient:
  `mission.py`'s own `from autokart_vehicle_geometry.geometry import GEOMETRY` statement runs at
  `mission.py`'s import time, so `autokart_vehicle_geometry` must already be on `sys.path` by then.
  `MissionAlgorithm.__init__`'s path insertion loop runs before any of the three module imports, in
  the dict's insertion order (`algorithm_package_path`, `vehicle_geometry_package_path`,
  `planner_package_path`), so all three paths are present on `sys.path` before any import is
  attempted — order of path *insertion* therefore does not matter, only order of *import*, which is
  correctly algorithm/geometry before planner in the code.
- **No hidden/hardcoded path**: grepped the package for `/home/mark1` and absolute paths; none in
  source.

## Not independently re-verified by this reviewer

- The reused `PlannerEngine`'s own stop-distance/dead-reckoning physics and the native skidpad
  ring-fit geometry it consumes (from `autokart_mapping`'s shared dependency) were authored before
  this session; this review did not re-derive that physics beyond what the existing, reused adapter's
  own test suite (ported here) already establishes. If a latent bug exists there, this node inherits
  it unmodified — not introduced or hidden by this change.
- Real actuator-feedback hardware timing/latency characteristics: **NOT RUN**, no VCU/actuator
  transport exists in this environment (see DEPENDENCIES.md).
