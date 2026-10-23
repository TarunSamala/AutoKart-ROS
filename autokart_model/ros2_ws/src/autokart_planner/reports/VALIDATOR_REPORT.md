# Independent validator report — autokart_planner

Performed as a distinct, sequential validation pass: built the package independently in a brand
new, isolated workspace, ran its tests there, and exercised the export/clean-workspace path end to
end, after the reviewer's three fixes.

## Environment

- ROS 2 Humble, `colcon` present.
- No kart hardware, no track survey. Dev-environment-only validation.
- `kart/ros2_ws` was already colcon-built in this repository (used as the `autokart_msgs` source
  for in-place runs); the clean-workspace portability run below does **not** source it.

## Runs performed and their actual output

1. **In-place unit/core tests** (no ROS sourced for the algorithm/core-only files):
   `python3 -m pytest test/test_algorithms.py test/test_planner_core.py -q` → `31 passed`, then
   `40 passed` after the reviewer's two regression tests were added
   (`test_zero_curvature_epsilon_rejects_instead_of_dividing_by_zero`,
   `test_negative_curvature_epsilon_also_rejects`) plus one more
   (`test_never_receiving_any_path_still_reports_stale_after_startup`).

2. **Full suite with ROS Humble + `kart/ros2_ws`'s `autokart_msgs` sourced**:
   `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test/ -q` → `40 passed` (18 algorithm +
   15 planner-core + 1 integration + 6 fault-injection).

3. **Cross-validation tests actually exercised the real dependency, not skipped**: confirmed
   `autonomy_stack_phase1/phase_01_perception/src/autokart_perception/curvature.py` is present in
   this dev environment, so `test_builtin_and_reused_curvature_agree_on_a_circular_arc` and
   `test_builtin_and_reused_stopping_distance_agree` ran for real (not via the `skipif` path) —
   confirmed by intentionally renaming the dependency directory and re-running, observing those
   two tests flip to `skipped` with the expected reason string, then restoring it.

4. **Import-precedence bug, independently reproduced**: reverting *only* the reviewer's `sys.path`
   front-insertion fix in `algorithms.py` was not, by itself, enough to reproduce the original
   failure, because the test file's own `_clean_autokart_perception_imports` autouse fixture (added
   alongside the fix) also restores `sys.path` after every test, independently masking the bug.
   To isolate the production fix's actual effect, reverted **both** `algorithms.py`'s fix **and**
   the fixture's `sys.path` restoration (keeping only its `sys.modules` cache cleanup) — i.e.
   reconstructed the exact condition this bug was first found under. Re-ran
   `test/test_algorithms.py -q` and observed `3 failed, 14 passed`, with
   `test_reused_backend_builds_against_real_dependency`,
   `test_builtin_and_reused_curvature_agree_on_a_circular_arc`, and
   `test_builtin_and_reused_stopping_distance_agree` all failing with
   `ModuleNotFoundError: No module named 'autokart_perception.curvature'` (the exact failure the
   reviewer's report describes). Restored both files and confirmed `17 passed` (the file's own
   count; `40 passed` for the full suite). Not taken on trust.

5. **Division-by-zero bug, independently reproduced**: with the reviewer's
   `CURVATURE_EPSILON_UNQUALIFIED` guard temporarily removed, constructed a `PlannerCore` with
   `curvature_eps_m_inv=0.0` and a straight 5-point path and confirmed
   `ZeroDivisionError: float division by zero` is raised. Restored the guard and confirmed the same
   call now returns `valid=False`, `reason_codes=["CURVATURE_EPSILON_UNQUALIFIED"]`.

6. **Clean-workspace portability acceptance test** (the mission's "most important requirement"):
   - `./export_node.sh /tmp/.../scratchpad/planner_export` → produced a tarball containing only
     `autokart_planner/` and `autokart_msgs/`.
   - Extracted into a brand-new workspace (`/tmp/.../scratchpad/planner_clean_ws`), **with the
     original repository's `kart/ros2_ws` install not sourced** (only
     `/opt/ros/humble/setup.bash`).
   - `colcon build --packages-up-to autokart_planner` → `Summary: 2 packages finished`, no errors
     (one harmless `tests_require`/byte-compiling warning, matching `autokart_hp60c`'s own
     documented tooling-gap note, not a failure).
   - `python3 -m pytest test/ -q` from inside the extracted bundle, this clean workspace's own
     `install/setup.bash` sourced → `40 passed`.
   - `ros2 launch autokart_planner planner.launch.py config_file:=.../planner_builtin.yaml` →
     process started; `ros2 topic list` showed all 5 expected topics
     (`/autokart_planner/{obstacles,path,reference_path,safety_envelope,speed_profile}`).
   - `ros2 topic echo /autokart_planner/safety_envelope --once` with **no reference path ever
     published** returned `valid: false`, `limiting_reason: PATH_MISSING_OR_STALE` — this is a live,
     end-to-end confirmation of the reviewer's fix #2 (previously this would have stayed
     completely silent).
   - Also observed the freshly-built-from-source `autokart_msgs` in this clean workspace *does*
     carry `data_source` (unlike the pre-existing `kart/ros2_ws/install`, see CONTRACT.md's
     "Environment finding") — `data_source: SIMULATED` appeared in the echoed message, consistent
     with this node's `hasattr()` guard correctly detecting and using the field when present.
   - Published a reference path via `ros2 topic pub --once` with an unset (zero) header stamp and
     confirmed the node correctly rejected it as `PATH_EXPIRED`/stale rather than treating a
     zero/placeholder timestamp as "now" — a useful, if accidental, additional live confirmation
     of the staleness-rejection path (the automated `test_node_integration.py`, which stamps with
     the real ROS clock, is the actual evidence for the "valid input" path).
   - No leftover `ros2 launch`/`planner_node` process after the run (checked with `pgrep -af`,
     force-killed two stragglers from the manual launch/pub commands; the shared `ros2-daemon`
     background service is not a leftover from this node and was left running).

## Acceptance matrix (software-only)

| Capability | Status | Evidence |
|---|---|---|
| Builds as standalone ament_python package | VALIDATED | colcon build, twice, independent workspaces |
| Publishes path/envelope/speed-profile topics | VALIDATED | `ros2 topic list` + live echo |
| Builtin (zero-dependency) curvature + stop-envelope formulas | VALIDATED | unit tests + live launch |
| Reused algorithm backend, successful path | VALIDATED (synthetic) | cross-validation tests against real `autokart_perception` code, confirmed actually exercised (not skipped) in this environment |
| Fault reporting: missing/stale odom, too-short path, expired/future-timestamped path | VALIDATED | fault-injection tests + live echo |
| Fault reporting: never-received path, missing algorithm dependency (repeats, not one-shot) | VALIDATED | regression tests added and confirmed, one live-verified |
| Fail-closed default braking/lateral configuration | VALIDATED | unit test + config file inspection |
| Portability: clean workspace, export bundle, build, launch | VALIDATED | full run recorded above |
| No hardcoded project-home paths in source | VALIDATED | grep check, defaults are empty strings |
| `colcon test` standard pytest hook | BLOCKED | same setuptools/pytest-runner incompatibility already documented for `autokart_hp60c`; worked around by running pytest directly |
| Curvature/stop-envelope accuracy against real sensor data or a surveyed track | NOT RUN | no hardware/track in this environment |
| `SafetyEnvelope.allowed_region` (drivable-region polygon) | NOT RUN | not computed by this node (see DEVELOPER_REPORT.md "Known gaps") |

## Explicit statement

**Software-only acceptance above does not authorize or imply vehicle motion.** This node never
sets `vehicle_authorized`; `a_lat_max_mps2` is an UNVERIFIED nominal placeholder and
`a_brake_min_mps2`/`t_total_latency_s` default to unconfigured (fail-closed) values. Per
`PROJECT_INDEX.md`, vehicle operation remains NO-GO and nothing in this change changes that.
