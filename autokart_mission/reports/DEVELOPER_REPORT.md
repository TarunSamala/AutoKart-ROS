# Developer report — autokart_mission

## Audit performed before writing any code

- Read `kart/ros2_ws/src/autokart_nodes/autokart_planner/autokart_planner/mission.py` (431 lines) in
  full: `SkidpadProgress` (gate-crossing/winding-number progress FSM), `PlannerEngine`
  (`_skidpad`/`_braking`/`_transition_path`, stop-distance physics with dead-reckoning).
- Read `kart/ros2_ws/src/autokart_nodes/autokart_planner/test/test_mission_adapter.py` (368 lines) and
  `kart/ros2_ws/src/autokart_nodes/test/ros_skidpad_route_probe.py` (201 lines) in full to confirm
  expected behavior and how the skidpad route is currently exercised end-to-end via real ROS nodes.
- Also read `kart/ros2_ws/src/autokart_nodes/autokart_planner/autokart_planner/node.py` in full (the
  existing `PlannerNode` ROS wrapper) to confirm exactly how `PlannerEngine` is constructed and driven
  from ROS topics/parameters, since this node's own wiring mirrors that construction (same parameter
  names/semantics) rather than guessing at it.
- Read `autokart_vehicle_geometry/geometry.py` and confirmed, by direct experiment
  (`sys.path`-only import with no ROS/colcon involved), that `mission.py` plus its two transitive
  dependencies (`autokart_perception.shared_core`, `autokart_vehicle_geometry.geometry`) import and
  run as **pure Python**, with no ROS dependency of their own — confirming the mission statement's own
  preference ("if it's cleanly separable as a pure-Python import, prefer that over duplicating code")
  applies here.

## Decision: reuse `PlannerEngine` wholesale via a 3-path lazy import, not a rewrite

`SkidpadProgress`'s gate-crossing/winding-number logic and `PlannerEngine._braking`'s stop-distance
physics are **not separable** from the rest of `PlannerEngine` (path-candidate generation, dead
reckoning) without editing `mission.py`, which is explicitly out of scope (live production source,
not to be modified). Rather than re-deriving the FSM/physics to produce a "mission-only" subset, this
node imports the whole module (`autokart_planner.mission.PlannerEngine`/`SkidpadProgress`) lazily via
`autokart_mission/algorithm.py`, runs it unmodified, and **discards** the one field
(`result["path_odom"]`) that belongs to the planner's (node 08's) responsibility rather than
publishing it — see CONTRACT.md. This required three lazy-import paths, not one, because
`mission.py` itself imports two further pure-Python packages at its own module level
(`autokart_perception.shared_core`, `autokart_vehicle_geometry.geometry`) — see DEPENDENCIES.md for
the exact import-order and sibling-directory requirements this implies, especially for
`autokart_vehicle_geometry`'s own source-tree search for `vehicle_geometry.yaml`.

## What was implemented

- `autokart_mission/algorithm.py`: `MissionAlgorithm`, lazy-imports `autokart_perception.shared_core`,
  `autokart_vehicle_geometry.geometry`, then `autokart_planner.mission` (in that order), each path
  validated independently and never hardcoded.
- `autokart_mission/node.py`: `MissionNode`, ROS pub/sub wiring matching the existing, reused
  `PlannerNode`'s own parameter names/semantics (brake/geometry parameters), publishing
  `mission_constraints` (allowable_branch/lap_index/speed_cap_mps/mode) as a direct, documented
  relabeling of already-computed `PlannerEngine` fields.
- Full ament_python package, launch file, two config profiles (`mission_skidpad.yaml`,
  `mission_braking_test.yaml`).
- 10 tests: 5 unit (three-path lazy-import wrapper, including the blank-path fail-closed fix ported
  from `autokart_mapping`'s own reviewer-found defect, and a dedicated test for the
  `autokart_vehicle_geometry` sibling-directory hazard) + 3 unit (behavioral parity against the REAL
  reused `PlannerEngine`, ported from the existing adapter's own test suite, including an explicit
  assertion that the winding-number **and** gate-crossing requirement is real, and an explicit
  assertion that the braking FSM gap is real) + 1 ROS integration (real rclpy node, real
  `PlannerEngine`) + 1 ROS fault injection (malformed mapping status) + 1 ROS fault injection (missing
  dependency, always runs).
- `export_node.sh`, `CONTRACT.md`, `DEPENDENCIES.md`, `README.md` (this directory).

## The specific gap this session was asked to look for: found and reported, not silently filled

Per the mission statement's explicit instruction to check whether the existing mission FSM really
implements both missions with winding-number + gate-crossing confirmation (not elapsed time alone),
and to report rather than guess if it does not:

- **Skidpad**: implements both requirements for real (`SkidpadProgress.travel` accumulates signed
  angular progress equivalent to the architecture's winding-number formula, **and** a discrete gate
  crossing is additionally required before a lap increments) — verified with a real test against the
  unmodified reused code, not assumed. State *names* differ from the architecture doc's literal
  vocabulary (reported, with a mapping table, in CONTRACT.md) but the transition structure is
  equivalent.
- **Braking**: has **no equivalent named state machine at all**, and critically, `evaluate()` accepts
  `stop_feedback_valid` but `_braking()` never receives or uses it — so there is no
  `STOP_VERIFIED`-equivalent measured-stop confirmation for `BRAKING_TEST`, unlike `SKIDPAD`'s
  `STOP_PENDING -> STOP` transition. This is a genuine, pre-existing gap in the reused `mission.py`,
  confirmed with a real test
  (`test/test_mission_fsm.py::test_braking_fsm_has_no_named_waiting_track_valid_ready_running_stop_states`)
  against its actual, unmodified output — not an assumption, and not something this node invented a
  new, untested classifier to paper over. See CONTRACT.md's "Known FSM gaps" section.

## Known gaps (explicit, not hidden)

- No real `ActuatorFeedback`/VCU transport exists in this environment to exercise the `STOP_PENDING ->
  STOP` (skidpad) transition against genuine measured stop feedback — tested only via a synthetic
  `ActuatorFeedback` message with `valid=true`.
- This node does not publish a path (see CONTRACT.md for why that could not be avoided without editing
  the reused `mission.py`) — node 08 (`autokart_planner`) remains the intended path-candidate owner,
  not yet built in this library.
- The braking-FSM completeness gap above is reported, not fixed — fixing it would mean either editing
  `mission.py` (out of scope) or inventing new, untested state-machine logic in this node's own
  package with no existing test coverage to validate it against, which the mission statement's own
  instruction ("implement only what you can justify from the existing code/tests/docs") argues
  against doing unprompted.
