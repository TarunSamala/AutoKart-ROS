# Developer report — autokart_controller

## Audit performed before writing any code

- Read `kart/ros2_ws/src/autokart_nodes/autokart_controller/autokart_controller/{engine.py,node.py}`,
  its `test/test_controller_adapter.py`, `CMakeLists.txt` and `package.xml` in full.
- `engine.py` (`ControllerEngine`) is a freshness/validity gate around
  `kart_sim.autonomy.ControllerManager`; the actual Pure Pursuit math is
  `kart/kart_sim/autonomy.py::PurePursuitController.compute()`, read in full, along with
  `LongitudinalPidController`, `ControllerManager`, `kart_sim/types.py` and `kart_sim/vcu.py`'s
  `MotionCommand`. `kart_sim` lives outside `ros2_ws/src`; the existing CMake package installs it via
  `ament_python_install_package(... PACKAGE_DIR ../../../../kart_sim)`.
- **Formula check against the brief**: the implementation does not literally match
  `L_d = clamp(L0+k_v|v|, Lmin, Lmax)`, `delta = atan(2L sin(alpha)/L_d)`. Actual: lower clamp only
  (hardcoded 0.1 m, no `Lmax`), `max(0,u)` not `|v|`, steering = `atan2(2*L*y_local, distance^2)`
  where `distance` is the actual range to the selected point, then rate-limited, then magnitude-
  clamped. Equivalent to the brief's formula with `L_d` := actual target range. Documented verbatim
  in CONTRACT.md; not "corrected" (the brief says to document the actual formula).
- **Guessed physical constants found** (all hardcoded as dataclass defaults in `engine.py` *and*
  `kart_sim.autonomy.PurePursuitController`, and as `declare_parameter` defaults in `node.py`):
  `wheelbase_m=1.0`, `maximum_steering_rad=0.6` (~34.4°; the brief guessed "±20°", the code actually
  says 0.6 rad), `maximum_steering_rate_radps=3.0`, plus `maximum_speed_mps=5.0`,
  `acceleration_limit_mps2=3.0`, `deceleration_limit_mps2=5.0`. None is labelled as unmeasured
  anywhere in that code. `vehicle_geometry.yaml` (the repo's single measured-geometry source)
  contains **no** wheelbase or steering figure. See SENIOR_REVIEWER_REPORT.md finding 0.
- Tracking diagnostics: `PurePursuitController.compute()` already computes
  `cross_track_error_m`/`heading_error_rad`, `LongitudinalPidController` computes `error_mps`; but
  `ControllerManager.command()` and `ControllerEngine.evaluate()` both drop them.
- Interfaces: installed `autokart_msgs` has `MotionCommand`, `SafetyEnvelope`, `ActuatorFeedback`,
  but not `MissionDebug` (stale install; see node 08's CONTRACT.md). Diagnostics therefore use
  `std_msgs/String` JSON.

## Decisions

- Reuse layer: `kart_sim.autonomy.PurePursuitController` + `LongitudinalPidController`, called
  directly (not through `ControllerManager`, to keep the diagnostics; not by importing
  `autokart_controller.engine` from the live workspace, which is a ROS-adapter wrapper — same
  algorithm-layer choice `autokart_hp60c` made). Lazy import via `kart_sim_package_path`, never
  hardcoded. `ReusedPursuitBackend`.
- Zero-dependency default `BuiltinPursuitBackend`: the same formulas coded directly (line-by-line
  parity with `autonomy.py`), so the package builds/tests on bare Humble. Cross-validated against the
  reused backend to 1e-6 including multi-step stateful behaviour.
- Inputs are node 08's actual outputs (path + SafetyEnvelope + speed-profile JSON), so 08→09 is a
  working chain, verified end to end.
- The UNVERIFIED constants are kept as ROS parameters with the existing numbers (not re-guessed,
  not elevated to "measured") and labelled UNVERIFIED in node.py, both config files, CONTRACT.md;
  non-positive wheelbase/steering limit fail closed at startup.
- No VCU signing (`integrity=0`), `vehicle_authorized=false`: candidate only.

## What was implemented

`pursuit_backends.py`, `controller_core.py` (ROS-independent gating), `node.py` (stamp-paired
inputs, timer-driven repeated fault reporting), ament_python package, launch file, `builtin`/`reused`
configs, `export_node.sh`, CONTRACT/DEPENDENCIES/README, 50 tests (20 backend + 16 core + 1
integration + 12 fault-injection + 1 end-to-end with the real node 08).

## Known gaps

No closed-loop/plant simulation of tracking quality; no `Lmax`; ActuatorFeedback not staleness-gated;
acceleration/deceleration limits not computed; all physical constants UNVERIFIED.
