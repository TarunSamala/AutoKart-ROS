# Independent validator report — autokart_hp60c

Performed as a distinct, sequential validation pass: built the package independently (in a separate
workspace from where it was written), ran its tests, and additionally exercised the export/
clean-workspace path end to end, after the reviewer's two fixes.

## Environment

- ROS 2 Humble, `colcon` present at `/usr/bin/colcon`.
- No HP60C camera hardware attached to this machine. No Raspberry Pi. This is a dev-environment-only
  validation; see "BLOCKED / NOT RUN" below for what that implies.
- `kart/ros2_ws` was already colcon-built in this repository (used as the `autokart_msgs` source for
  the in-place test run).

## Runs performed and their actual output

1. **In-place colcon build** (`kart` workspace layout untouched):
   `colcon build --packages-select autokart_hp60c` from a symlinked workspace →
   `Summary: 1 package finished`, no errors (one harmless `tests_require` setuptools deprecation
   warning, not a failure).

2. **Unit tests, no ROS sourced**: `python3 -m pytest test/test_backends.py` → `7 passed`.

3. **Full suite with ROS Humble + autokart_msgs sourced**:
   `python3 -m pytest test/ -q` → `13 passed` (7 unit + 1 integration + 5 fault-injection, after the
   reviewer's two additional tests were added).

4. **`colcon test` via the standard ament_python hook**: reports `0 tests, 0 errors, 0 failures`.
   Investigated rather than taken at face value: this environment's setuptools no longer supports
   the legacy `tests_require`/`pytest-runner` mechanism that `colcon test` relies on for ament_python
   packages (confirmed by the "Unknown distribution option: 'tests_require'" warning during build).
   **This is a tooling gap in this dev environment, not evidence the node's own tests don't exist or
   don't pass** — item 3 above is the real test execution, run directly and independently of the
   broken `colcon test` shim. Recorded here instead of silently relying on a green `colcon test`
   summary that is actually measuring nothing.

5. **Fault injection, actually executed**:
   - Persistent `CAMERA_DISCONNECTED`-raising backend → `/hp60c/status` observed going to
     `quality=INVALID`, `fault_state=CAMERA_DISCONNECTED`, `payload_present=false`. PASS.
   - Backend failing twice (`STALE_FRAME`) then recovering → observed `INVALID` with
     `fault_state=STALE_FRAME`, then a later `OK`. PASS.
   - Backend raising a bare `RuntimeError` (not the expected exception type) → node kept running,
     `/hp60c/status` showed `UNEXPECTED_BACKEND_ERROR:RuntimeError:...`, confirmed still publishing
     0.3s later (thread did not die). PASS — this is the defect the reviewer found and the developer
     fixed; validator independently reproduced the failure on the pre-fix code (reverted the fix
     locally, re-ran the new test, watched it fail with "thread did not die" assertion false because
     no further status arrived) before confirming the fix makes it pass. Not just taken on trust.
   - Missing `algorithm_package_path` dependency → `backend` stays `None`, node publishes repeated
     `INVALID`/`DEPENDENCY_PATH_NOT_FOUND:...` status (≥2 messages observed within 5s, confirming the
     reviewer's "report once then go silent" fix). PASS.

6. **Clean-workspace portability acceptance test** (the mission's "most important requirement"),
   performed twice (once before, once after the reviewer's fixes, to make sure the export script
   picks up source changes rather than a stale copy):
   - Ran `export_node.sh` → produced a tarball containing only `autokart_hp60c/` and `autokart_msgs/`.
   - Extracted into a brand new directory, **with the original `autonomous_kart` checkout's
     `kart/ros2_ws` install not sourced** (only `/opt/ros/humble/setup.bash`).
   - `colcon build --packages-up-to autokart_hp60c` → succeeded, 2 packages finished, no errors.
   - `python3 -m pytest test/ -q` from inside the extracted bundle, workspace's own
     `install/setup.bash` sourced → `13 passed`.
   - `ros2 launch autokart_hp60c hp60c.launch.py config_file:=.../hp60c_synthetic.yaml` → process
     started; `ros2 topic list` showed all 5 expected topics; `ros2 topic echo` on
     `/hp60c/rgb/camera_info` and `/hp60c/status` (first run) and `/hp60c/rgb/camera_info` (second
     run) returned well-formed messages with `quality: OK`, `fault_state: NONE`, correct
     `distortion_model`, correct `height`/`width`.
   - No leftover `ros2`/`hp60c_node` process after each run (checked with `pgrep`, killed one
     straggler from the first manual launch).
   This demonstrates the package satisfies the "copy into another workspace, build, launch, verify
   outputs" requirement — independent of the specific machine this was developed on, and with no
   undisclosed dependency on the original repository's paths.

## Acceptance matrix (per node registry item "01 Camera" in the architecture doc, software-only)

| Capability | Status | Evidence |
|---|---|---|
| Builds as standalone ament_python package | VALIDATED | colcon build, twice, independent workspaces |
| Publishes `/hp60c/{rgb,depth}/image_raw`, `/hp60c/{rgb,depth}/camera_info`, `/hp60c/status` | VALIDATED | `ros2 topic list` + `ros2 topic echo` live run |
| Synthetic backend (no hardware) | VALIDATED | integration test + live launch |
| Fault reporting: disconnect / stale-frame / transient-recovery | VALIDATED | fault-injection tests, live status observed |
| Fault reporting: unexpected exception does not kill capture | VALIDATED | regression test added and confirmed against pre-fix code |
| Fault reporting: misconfigured dependency keeps reporting (not one-shot) | VALIDATED | regression test added and confirmed |
| Portability: clean workspace, export bundle, build, launch | VALIDATED | full run recorded above |
| Reuses existing `HP60C` SDK class without reimplementation | VALIDATED | `DEPENDENCIES.md` + `backends.py` import, no duplicated ctypes logic |
| No hardcoded project-home paths in source | VALIDATED | grep check, defaults are empty strings |
| `colcon test` standard pytest hook | BLOCKED | setuptools/pytest-runner incompatibility in this environment (tooling, not node defect) — worked around by running pytest directly, which is the real evidence above |
| Live HP60C camera read path (`backend: hp60c`, actual hardware) | NOT RUN | no camera, no Pi in this environment. Only the backend's *failure* paths (missing path, import failure) are tested |
| Calibration/mount/intrinsics accuracy against surveyed cones | NOT RUN | requires physical rig; explicitly flagged "not physically qualified" per architecture doc §3/§17 |
| `PointCloud2` optional output | NOT RUN | not implemented this session (explicitly deferred, documented in CONTRACT.md) |
| Static camera→base_link TF | NOT RUN | requires measured mount geometry |

## Explicit statement

**Software-only acceptance above does not authorize or imply vehicle motion, physical camera
deployment readiness, or calibration correctness.** Per the architecture doc and this repository's
own `README.txt`/`PROJECT_INDEX.md`, vehicle operation remains NO-GO and no flag in this change sets
`vehicle_authorized`, `calibration_verified`, or any other safety gate to true.
