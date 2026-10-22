# Independent validator report — autokart_mission

Performed as a distinct, sequential validation pass: built the package independently, ran its tests
in-place and from a clean export, and exercised a real cross-node run against `autokart_mapping`
(node 06, this library).

## Environment note (shared with autokart_mapping's own validator report)

`kart/ros2_ws/install` is stale relative to `kart/ros2_ws/src/autokart_msgs` (missing `MissionDebug`).
Not rebuilt in place (out of scope); a scratch copy of `autokart_msgs` was built in this session's own
scratchpad workspace and sourced for all runs below instead. See `autokart_mapping/reports/
VALIDATOR_REPORT.md` for the full finding.

## Runs performed and their actual output

1. **In-place colcon build**: `colcon build --packages-select autokart_mission` →
   `Summary: 1 package finished`, no errors (same harmless `tests_require` deprecation warning as
   every other package in this library).

2. **Full suite, all three dependencies present**:
   `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test/ -q` → **10 passed**.

3. **Live node, real pub/sub, real `PlannerEngine`** (`ros2 run autokart_mission mission_node
   --ros-args -p algorithm_package_path:=... -p vehicle_geometry_package_path:=... -p
   planner_package_path:=... -p mission:=SKIDPAD -p brake_min_mps2:=3.0 -p brake_delay_s:=0.2 -p
   brake_provenance:=SIMULATED`): published a synthetic odom pose on the right-loop entry gate plus a
   matching skidpad `skidpad_topology` payload on `/autokart_mapping/status`; observed
   `/autokart_mission/status` respond `valid: true, decision: "PATH_CANDIDATE",
   mission_constraints.allowable_branch: "RIGHT_1", speed_cap_mps: 1.0`. **PASS**. (First attempt,
   without setting `brake_min_mps2`, correctly produced
   `reason_codes: ["BRAKE_OR_VEHICLE_LIMIT_UNQUALIFIED"]` — this is the reused `PlannerEngine`'s own,
   correct fail-closed behavior for an unqualified brake capability, not a defect; the probe script
   was corrected to configure a brake, not the node.)

4. **Fault injection, actually executed**:
   - All three dependency paths missing → `/autokart_mission/status` observed repeating `valid:
     false`, `reason_codes` containing `DEPENDENCY_PATH_NOT_FOUND:algorithm_package_path:...` (first
     path checked) at least twice within 3 s. **PASS**.
   - Malformed `payload_json` on `/autokart_mapping/status` → observed `valid: false`,
     `"MAPPING_STATUS_MALFORMED"` in `reason_codes`. **PASS**.
   - Pre-fix reproduction of the reviewer's proactively-applied defect: confirmed (by temporarily
     reverting the blank-path guard in a scratch copy of `algorithm.py` and re-running
     `test_blank_paths_fail_closed_instead_of_resolving_to_cwd`) that the test fails without the
     guard — `MissionAlgorithm("", "/some/path", "/some/path")` does not raise — before confirming the
     shipped code (with the guard) passes.

5. **Clean-workspace portability test**:
   - `./export_node.sh` → tarball containing only `autokart_mission/` and `autokart_msgs/`.
   - Extracted alongside a separately exported `autokart_mapping` tarball into one clean workspace
     (`autokart_msgs` de-duplicated between the two bundles — both bundles are independently correct
     on their own; the duplication only appears when combining two single-node exports into one
     workspace, which `export_node.sh`'s own `README_EXPORT.md` does not claim to prevent).
   - `colcon build` (only `/opt/ros/humble` sourced) → `Summary: 3 packages finished`, no errors.
   - `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test/ -q` from the extracted bundle →
     **3 passed, 7 skipped** (the 7 correctly skip: they need the full `kart/ros2_ws` +
     `autonomy_stack_phase1` checkout, deliberately not bundled — see DEPENDENCIES.md).
   - `ros2 run autokart_mission mission_node` with no `*_package_path` overrides (shipped default
     config) → started with no errors in its log, topics (`/autokart_mission/obstacles`,
     `/autokart_mission/status`) present via `ros2 topic list`.
   - `ros2 run autokart_mission mission_node --ros-args -p algorithm_package_path:=<real path> -p
     vehicle_geometry_package_path:=<real path> -p planner_package_path:=<real path> ...` from the
     same clean install → started with no errors.
   - **Cross-node run** (both packages from their clean, exported installs, together): launched
     `autokart_mapping` and `autokart_mission`, fed a 96-cone skidpad ring + odom pose into
     `autokart_mapping`'s input topics via a Python probe, and observed `autokart_mission`'s
     `/autokart_mission/status` respond `valid: true, decision: "PATH_CANDIDATE",
     mission_constraints.allowable_branch: "RIGHT_1"`, confirming `autokart_mapping`'s real output
     (not a stub) correctly drives `autokart_mission`'s real `PlannerEngine`, from clean, independently
     exported installs of both packages. **PASS**.
   - No leftover `ros2`/`mapping_node`/`mission_node` process after any run (`pgrep -af`, stragglers
     killed before finishing).

## Acceptance matrix

| Capability | Status | Evidence |
|---|---|---|
| Builds as standalone ament_python package | VALIDATED | colcon build, in-place and from a clean export |
| Publishes `/autokart_mission/status` with mission constraints | VALIDATED | live probe + integration test |
| Never publishes a path or motor command | VALIDATED | reviewer's grep + publisher-list check; no `Path`/`MotionCommand` publisher exists |
| Reuses `PlannerEngine`/`SkidpadProgress` without reimplementation | VALIDATED | `DEPENDENCIES.md` + `algorithm.py` import path |
| Winding-number + gate-crossing lap confirmation (not time alone) | VALIDATED (reused, not re-derived) | real test against unmodified `PlannerEngine` |
| Braking FSM named-state gap | CONFIRMED AS A REAL, PRE-EXISTING GAP, NOT FIXED | real test asserting the gap against unmodified `PlannerEngine`; see CONTRACT.md |
| Fault reporting: missing dependency keeps reporting (not one-shot) | VALIDATED | fault-injection test + live status observed |
| Fault reporting: malformed mapping status rejected, not fabricated | VALIDATED | fault-injection test |
| Blank-path fail-closed (defect proactively applied from autokart_mapping's finding) | VALIDATED (fixed + regression-tested, pre-fix reproduced) | test + manual revert-and-rerun |
| Portability: clean workspace, export bundle, build, launch | VALIDATED | full run recorded above |
| Cross-node interoperation with `autokart_mapping` | VALIDATED | real pub/sub run, both from clean exported installs |
| No hardcoded project-home paths in source | VALIDATED | grep check, defaults are empty strings |
| `colcon test` standard pytest hook | BLOCKED | same setuptools/`tests_require` incompatibility recorded across this whole library — a tooling characteristic of this environment |
| Real actuator-feedback/VCU stop confirmation | NOT RUN | no hardware/transport in this environment |
| Real braking-course stop-distance validation | NOT RUN | no physical track in this environment |

## Explicit statement

**Software-only acceptance above does not authorize or imply vehicle motion, physical braking
performance, or calibration correctness.** `vehicle_authorized`, `calibration_verified`,
`vehicle_config_verified` and `braking_verified` remain false/unset everywhere in this repository;
this node does not touch those flags and nothing above sets any of them. The braking FSM gap
documented in CONTRACT.md/DEVELOPER_REPORT.md is a property of the reused `mission.py`, confirmed real
by this validation pass, and is not resolved by anything in this package.
