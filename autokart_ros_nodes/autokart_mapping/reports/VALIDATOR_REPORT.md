# Independent validator report — autokart_mapping

Performed as a distinct, sequential validation pass: built the package independently, ran its tests
in-place and from a clean export, and exercised a real cross-node run against `autokart_mission`
(node 07, this library), after the reviewer's fix.

## Environment note recorded, not worked around in place

`kart/ros2_ws/install` (the environment's own stated "gives you autokart_msgs" source) is **stale**
relative to `kart/ros2_ws/src/autokart_msgs`: `MissionDebug.msg` and `BenchVcuStatus.msg` exist in the
source tree but are absent from the built install (`import autokart_msgs.msg` from that install is
missing `MissionDebug`). Per this session's explicit restriction not to touch anything outside
`autokart_ros_nodes/`, `kart/ros2_ws` was **not** rebuilt in place. Instead, a scratch copy of
`kart/ros2_ws/src/autokart_msgs` was built in this session's own scratchpad workspace and sourced for
all test/build/launch runs below. This is an environment-drift finding, not a defect in this node or
in `autokart_msgs` itself, and is reported here rather than silently worked around without mention.

## Runs performed and their actual output

1. **In-place colcon build** (symlinked scratch workspace, scratch `autokart_msgs` + this node):
   `colcon build --packages-select autokart_mapping` → `Summary: 1 package finished`, no errors (one
   harmless `tests_require` setuptools deprecation warning, same as `autokart_hp60c`'s own validator
   report already recorded — a tooling characteristic of this environment, not a failure).

2. **Full suite, real algorithm present** (`autonomy_stack_phase1` on the configured
   `algorithm_package_path`): `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test/ -q` →
   **11 passed**.

3. **Live node, real pub/sub** (`ros2 run autokart_mapping mapping_node --ros-args -p
   algorithm_package_path:=.../autonomy_stack_phase1/phase_01_perception/src`): published 6 synthetic
   cones (3 gates) + an `odom` pose via a Python probe; observed `/autokart_mapping/status` with
   `valid: true`, `tracked_count: 6`, `reason_codes: []`. **PASS**.

4. **Fault injection, actually executed**:
   - Missing `algorithm_package_path` → `/autokart_mapping/status` observed repeating
     `valid: false`, `reason_codes: ["ALGORITHM_PACKAGE_PATH_NOT_CONFIGURED"]` (post-fix) at least
     twice within 3 s, not once-then-silent. **PASS**.
   - Unqualified cone frame (`frame_id="camera_optical"`) → observed `valid: false`,
     `"UNQUALIFIED_CONE_FRAME"` in `reason_codes`, `tracked_count: 0`. **PASS**.
   - Pre-fix reproduction of the reviewer's defect (see SENIOR_REVIEWER_REPORT.md): confirmed blank
     `algorithm_package_path` loaded a live algorithm object with no error, before the fix; confirmed
     `startup_error = "ALGORITHM_PACKAGE_PATH_NOT_CONFIGURED"` after, in a rebuilt scratch workspace.

5. **Clean-workspace portability test** (the mission's "most important requirement"):
   - `./export_node.sh` → tarball containing only `autokart_mapping/` and `autokart_msgs/`.
   - Extracted into a brand-new directory alongside a separately exported `autokart_mission` tarball
     (both tarballs' `autokart_msgs` copies collapsed into one `src/autokart_msgs` — colcon refuses
     duplicate package names, caught and corrected during this run, not a defect in either
     `export_node.sh`: each script's own bundle is self-consistent on its own).
   - `colcon build` (only `/opt/ros/humble` sourced, **not** `kart/ros2_ws`, **not** the original
     `autonomy_stack_phase1` checkout on any path) → `Summary: 3 packages finished`, no errors.
   - `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test/ -q` from inside the extracted bundle →
     **4 passed, 7 skipped** (the 7 correctly skip: they require
     `autonomy_stack_phase1/phase_01_perception/src`, which this bundle deliberately does not
     include — this is the intended, honest behavior documented in DEPENDENCIES.md, not a test
     failure).
   - `ros2 launch autokart_mapping mapping.launch.py` with the shipped default config (blank
     `algorithm_package_path`) → `ros2 topic list` showed all 3 expected topics; the node logged
     `ERROR ... autokart_mapping startup failed: ALGORITHM_PACKAGE_PATH_NOT_CONFIGURED` and kept
     running (did not crash).
   - `ros2 run autokart_mapping mapping_node --ros-args -p algorithm_package_path:=<real path>` from
     the same clean install → started with no errors in its log, topics present, process cleanly
     killed afterward.
   - **Cross-node run** (also from this clean install): launched `autokart_mapping` and
     `autokart_mission` together, published a 96-cone skidpad ring layout + odom pose via a Python
     probe to `autokart_mapping`'s input topics, and observed `autokart_mission`'s
     `/autokart_mission/status` respond with `valid: true, decision: "PATH_CANDIDATE",
     mission_constraints.allowable_branch: "RIGHT_1"` — i.e. `autokart_mapping`'s real output fed
     `autokart_mission`'s real `PlannerEngine` end to end, from the clean, exported install of both
     packages. **PASS**.
   - No leftover `ros2`/`mapping_node`/`mission_node` process after any run (checked with `pgrep -af`,
     killed stragglers from each manual run before finishing).

## Acceptance matrix

| Capability | Status | Evidence |
|---|---|---|
| Builds as standalone ament_python package | VALIDATED | colcon build, in-place and from a clean export |
| Publishes `/autokart_mapping/cones`, `/autokart_mapping/status` | VALIDATED | `ros2 topic list` + live probe |
| Reuses Delaunay/Gabriel/gates/centerline/skidpad-fit without reimplementation | VALIDATED | `DEPENDENCIES.md` + `algorithm.py` import path, no duplicated geometry math |
| Preserves skidpad two-closed-loop/crossover logic | VALIDATED (not re-derived) | `fit_skidpad_topology` called unmodified; parity test ported from the existing adapter's own suite |
| Fault reporting: missing dependency keeps reporting (not one-shot) | VALIDATED | fault-injection test + live status observed |
| Fault reporting: unqualified cone frame rejected, not fabricated | VALIDATED | fault-injection test |
| Blank-path fail-closed (reviewer-found defect) | VALIDATED (fixed + regression-tested) | pre-fix reproduction + post-fix test |
| Portability: clean workspace, export bundle, build, launch | VALIDATED | full run recorded above |
| Cross-node interoperation with `autokart_mission` | VALIDATED | real pub/sub run, both from clean exported installs |
| No hardcoded project-home paths in source | VALIDATED | grep check, defaults are empty strings |
| `colcon test` standard pytest hook | BLOCKED | same setuptools/`tests_require` incompatibility already recorded in `autokart_hp60c`'s validator report — a tooling characteristic of this environment, worked around by running pytest directly (the real evidence above) |
| Surveyed-course skidpad residual accuracy | NOT RUN | no physical track/cones in this environment |
| `source_mode=LIVE` accepted path (vs. its rejection paths, which are tested) | NOT RUN | no LIVE sensor pipeline in this environment |
| Node-02 perception-status companion cross-check (`PERCEPTION_STATUS_MISMATCH`) | NOT RUN / not wired | node 02 does not exist in this library yet; see CONTRACT.md |

## Explicit statement

**Software-only acceptance above does not authorize or imply vehicle motion, physical sensor
deployment readiness, or calibration correctness.** `vehicle_authorized`, `calibration_verified`,
`vehicle_config_verified` and `braking_verified` remain false/unset everywhere in this repository;
this node does not touch those flags and nothing above sets any of them.
