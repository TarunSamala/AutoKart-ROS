# Independent validator report — autokart_safety

Performed as a distinct, sequential validation pass: built the package in an
isolated colcon workspace (not the repository's own `kart/ros2_ws`), ran its
tests, ran the reused core's own existing test suite unmodified, exercised
the export/clean-workspace path end to end (including relocating the real
`autokart_vcu_safety` dependency to an arbitrary path, not its original
in-repo location), and did a live `ros2 launch`/topic check — after the
reviewer's path-shadowing fix.

## Environment

- ROS 2 Humble, `colcon` present.
- No VCU, no ESP32, no vehicle bridge hardware attached to this machine —
  software-only validation; see "BLOCKED / NOT RUN" below.
- This package was built and tested against a **fresh copy** of
  `autokart_msgs` and a **fresh copy** of `autokart_vcu_safety`, each rsynced
  into an isolated scratch workspace — not by sourcing the repository's own
  `kart/ros2_ws/install` — so these results do not depend on anything
  pre-built in `kart/ros2_ws`, consistent with this session's instruction
  not to modify or rely on in-place state under `kart/ros2_ws`.

## Runs performed and their actual output

1. **Isolated-workspace colcon build** (fresh `autokart_msgs` copy + this
   package, `kart/ros2_ws` untouched):
   `colcon build --packages-up-to autokart_safety` →
   `Summary: 2 packages finished`, no errors (the same harmless
   `tests_require` setuptools deprecation warning as `autokart_hp60c`, not a
   failure).

2. **Full suite, real dependency available**
   (`PYTHONPATH` via sourced install; `vcu_safety_package_path` pointed at
   the real, in-repo `kart/ros2_ws/src/autokart_vcu_safety`):
   `python3 -m pytest test/ -q` → **13 passed**.

3. **Reused core's own existing test suite, run unmodified** (confirms this
   change did not somehow depend on behavior that test suite would catch):
   `cd kart/ros2_ws/src/autokart_vcu_safety && python3 -m pytest test/test_safety_core.py -q`
   → **17 passed**. Read-only: no file under `kart/ros2_ws` was modified to
   run this.

4. **Reviewer's bug, reproduced and then confirmed fixed**: reverted the
   `adapter.py` path-shadowing fix locally, re-ran
   `test/test_adapter.py::test_reimport_after_a_bad_path_loads_the_new_real_module`
   alone → failed with `SDK_ADAPTER_API_MISMATCH` (the stale stub module was
   returned instead of the real core). Restored the fix, re-ran → passed.
   Also re-ran the **full** `test/` suite pre-fix in one process (matching
   how the developer first discovered this) → 7 of 13 tests failed with the
   same stale-module symptom; post-fix, all 13 passed. Not just taken on
   trust from the reviewer's own description.

5. **Clean-workspace portability test, performed in a workspace with
   `kart/ros2_ws` NOT sourced and NOT on `PYTHONPATH`:**
   - `export_node.sh` → tarball containing only `autokart_safety/` and
     `autokart_msgs/`.
   - Extracted into a brand-new directory; `colcon build --packages-up-to
     autokart_safety` with only `/opt/ros/humble/setup.bash` sourced →
     `Summary: 2 packages finished`, no errors.
   - `python3 -m pytest test/ -q` (real dependency path not present in this
     clean tree) → **4 passed, 9 skipped** — the skip reason is explicit
     (`real kart/ros2_ws/src checkout not present`), not a silent pass; this
     confirms the package degrades to "fails closed, dependency-free" rather
     than erroring when the dependency is absent, matching
     `DEPENDENCIES.md`.
   - Copied `kart/ros2_ws/src/autokart_vcu_safety` into an **arbitrary,
     non-original path** inside this same clean workspace
     (`vcu_safety_dependency/autokart_vcu_safety`, nested one level deeper
     than in the source repository) and called `adapter.load_safety_core()`
     against that relocated path directly: resolved to
     `.../vcu_safety_dependency/autokart_vcu_safety/autokart_vcu_safety/core.py`,
     `SafetyLogic().state == "DISARMED"`, and an `arm` request with nothing
     fresh was correctly refused with explicit stage reasons. This confirms
     the dependency contract in `DEPENDENCIES.md` works when the dependency
     is copied somewhere other than its original repository path, not only
     at the exact path this session happened to develop against.
   - `ros2 launch autokart_safety safety.launch.py config_file:=.../safety_synthetic.yaml`
     → process started; `ros2 topic list` showed all 9 expected topics
     (`/autokart/safety/status`, `/autokart/safety/request`,
     `/autokart/vcu/request`, `/dashboard/debug`, `/vcu/state`,
     `/vehicle/actuator_feedback`, and the three/five status-input topics);
     `ros2 topic echo /autokart/safety/status --once` returned a well-formed
     message: `state=UNAVAILABLE` (no dependency path set in this
     synthetic-profile launch — correct, fail-closed, by design),
     `arm_allowed=false`.
   - No leftover `ros2`/`safety_node` process after the run — checked with
     `pgrep -af`, killed stragglers, confirmed clean with a second `pgrep`.

## Acceptance matrix (software-only)

| Capability | Status | Evidence |
|---|---|---|
| Builds as standalone ament_python package, `kart/ros2_ws` not sourced | VALIDATED | isolated-workspace colcon build, twice (develop tree + exported tarball) |
| Default DISARMED, never auto-arms (dependency available) | VALIDATED | `test_safety_node_never_auto_arms_on_startup_with_real_core` |
| Fails closed, repeats (not one-shot), when dependency unavailable | VALIDATED | `test_safety_node_fails_closed_without_core_dependency`, live launch with no path set |
| Ready-but-unverified arm attempt rejected | VALIDATED | `test_ready_but_unverified_arm_attempt_is_rejected` |
| Arm succeeds once fully ready + explicitly verified (software path only) | VALIDATED | `test_arm_succeeds_once_fully_ready_and_verified` |
| E-stop-active forces disarm while ARMED | VALIDATED | `test_estop_active_forces_disarm_while_armed` |
| Stale/never-seen VCU heartbeat blocks arm | VALIDATED | `test_stale_vcu_heartbeat_blocks_arm` |
| Reused core imported, not reimplemented; dependency relocatable | VALIDATED | `DEPENDENCIES.md` + adapter import test + relocated-copy manual verification above |
| Path-shadowing regression (reviewer's fix) | VALIDATED | reproduced pre-fix, confirmed post-fix, both in isolation and across the full suite |
| Reused core's own existing test suite | VALIDATED (unmodified, read-only) | 17/17 passed |
| `front_safety_envelope_monitor` documented as separate, not subsumed | VALIDATED (documentation check) | CONTRACT.md / DEVELOPER_REPORT.md; no import of it anywhere in this package |
| Portability: clean workspace, export bundle, build, launch, topic check | VALIDATED | full run recorded above |
| No hardcoded project-home paths in source | VALIDATED | grep check; only comments/prose, actual defaults are `""`/`false` |
| `colcon test` standard pytest hook | BLOCKED | same pre-existing `tests_require`/pytest-runner incompatibility already recorded for `autokart_hp60c` in this environment — tooling gap, not a node defect; worked around by running pytest directly (the real evidence above) |
| Real VCU firmware / ESP32 / vehicle bridge behavior on ARM/DIS/RST | NOT RUN | no hardware in this environment; this is `autokart_vehicle_bridge`'s own, separately tested, concern |
| Physical E-stop circuit / manual power-cut interaction | NOT RUN | no hardware; this node has no path to the physical circuit by design |

## Explicit statement

**Software-only acceptance above does not authorize or imply vehicle
motion.** This node is a software arming authority, not the physical E-stop.
Per `PROJECT_INDEX.md`/`README.txt`, the manual battery/supply disconnect
remains the only proven emergency stop in this repository, and nothing in
this change sets `calibration_verified`, `vehicle_config_verified`,
`braking_verified`, or any other safety gate to `true` in any shipped
config.
