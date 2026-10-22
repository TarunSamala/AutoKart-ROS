# Independent validator report — autokart_perception_adapter

Distinct, sequential validation pass: built independently, ran tests, reproduced the reviewer's two
findings pre-fix, and ran a real clean-workspace portability test.

## Environment

ROS 2 Humble, no HP60C camera, no real cone track, no trained model path supplied. `cv2` and
`onnxruntime` happen to be installed in this dev environment (confirmed:
`python3 -c "import cv2, onnxruntime"` succeeds), which is why the `algorithm` backend's import path
could be exercised at all — this would not be true on a bare ROS 2 Humble install, which is exactly
why `backend: synthetic` has zero such dependency and is the default.

## Runs performed and actual output

1. `python3 -m pytest test/test_backends.py -q` (no ROS sourced) → 7 passed, including the
   real-algorithm-import test.
2. Full suite with ROS Humble + `autokart_msgs` sourced: `python3 -m pytest test/ -q` → 11 passed.
3. **Reproduced one of the two reviewer findings pre-fix; could not reliably reproduce the other
   on demand, and the report below says so rather than claiming otherwise:**
   - Finding 1 (sys.path/sys.modules test pollution): ran `test_backends.py` with the autouse
     isolation fixture commented out → the real-import test failed with `ModuleNotFoundError: No
     module named 'autokart_perception.application'` (the fake tmp-dir package from an earlier test
     shadowing the real one via stale `sys.path`). Restored the fixture → 7/7 pass again.
     **Cleanly reproduced, deterministically, every time.**
   - Finding 2 (shutdown race / `destroy_node` timer cancellation): the original warning
     (`"cannot use Destroyable because destruction was requested"`) was observed once, during this
     node's first clean-workspace portability run, before the `destroy_node()` override existed.
     Attempting to deliberately reproduce it afterward — with the override removed again — via 5
     repeated runs of the relevant pytest test, and separately via 3 repeated `ros2 run` +
     `SIGINT` cycles against a real colcon-built install, produced **no repeat of the warning** in
     any of the 8 attempts. This is consistent with a genuine but narrow race window rather than a
     deterministic bug, but it means the fix's benefit is not proven by reproduction the way finding
     1's is — it is a correct, defensive fix for a real hazard class (the same class
     `autokart_hp60c`'s reviewer did cleanly reproduce and confirm there), applied here on code
     inspection and one real observed instance, not on a repeatable test. Recorded honestly as
     IMPLEMENTED but not VALIDATED-by-reproduction, to avoid overstating what was actually confirmed.
   - One operational note from this reproduction attempt: the manual `ros2 run` loop left stray
     background node processes that cross-talked with a later pytest run over the shared DDS domain
     (seen as `test_missing_dependency_keeps_reporting_fault` failing once, after this, with mixed
     `OK`/`INVALID` statuses — not an unknown problem on inspection: `pgrep -af
     perception_adapter_node` confirmed 6 leftover processes from the race-reproduction loop). Killed
     them (`pkill -9 -f perception_adapter_node`) and reran the full suite clean (11/11). Recorded
     here because it is exactly the kind of leftover-process risk this report's own process checklist
     exists to catch, and because the failure was in this validator's own test harness, not the node.
4. **Fault injection, actually executed**:
   - `AlwaysUnqualifiedBackend` → `/perception/status` observed `quality=DIAGNOSTIC_ONLY`,
     `fault_state` containing `LIVE_METRIC_CALIBRATION_UNVERIFIED`; confirmed **no** `ConeArray`
     message arrived even after an additional 0.3s wait. PASS.
   - `RaisesUnexpectedErrorBackend` (plain `RuntimeError`, not `PerceptionBackendError`) →
     `/perception/status` showed `UNEXPECTED_BACKEND_ERROR:RuntimeError:...`, and the timer was
     confirmed still firing 0.3s later (node did not die). PASS.
   - Missing `algorithm_package_path` → `backend=None` at construction, ≥2 repeated `INVALID` /
     `DEPENDENCY_PATH_NOT_FOUND` status messages observed within 5s (not a single message then
     silence). PASS.
5. **Clean-workspace portability acceptance test**, performed twice (once before, once after the
   reviewer's destroy_node fix):
   - `export_node.sh` → tarball containing only `autokart_perception_adapter/` and `autokart_msgs/`.
   - Extracted into a brand-new directory, only `/opt/ros/humble/setup.bash` sourced (original repo's
     `kart/ros2_ws` install NOT sourced).
   - `colcon build --packages-up-to autokart_perception_adapter` → succeeded, 2 packages, no errors.
   - `python3 -m pytest test/ -q` inside the bundle → 11 passed (the real-algorithm-import test
     correctly auto-skipped in this isolated workspace, since the real algorithm package checkout is
     not part of the bundle — `pytest.mark.skipif` worked as intended, not silently passed).
   - `ros2 launch autokart_perception_adapter perception_adapter.launch.py
     config_file:=.../synthetic.yaml` → process started; `ros2 topic list` showed
     `/perception/cones` and `/perception/status`; `ros2 topic echo /perception/cones --once`
     returned 4 well-formed cones at the expected coordinates with `semantic_class` 1/2 (blue/yellow).
   - No leftover `perception_adapter_node` process after each run (checked with `pgrep`, killed one
     straggler after the first manual launch).

## Acceptance matrix (node 02, software-only)

| Capability | Status | Evidence |
|---|---|---|
| Builds as standalone ament_python package | VALIDATED | colcon build, twice, independent workspaces |
| Publishes `/perception/cones`, `/perception/status` | VALIDATED | `ros2 topic echo` live run |
| Synthetic backend (no hardware, no other node) | VALIDATED | integration test + live launch, standalone_timer mode |
| Topic-sync input mode (consumes node 01's topics) | IMPLEMENTED, NOT RUN end-to-end | message_filters wiring unit-reviewed; no cross-node run with autokart_hp60c performed this session (each node's own test suite covers its own node only) |
| Qualification gating: unqualified result never publishes a cone | VALIDATED | fault-injection test, 0.3s negative-wait check |
| Unexpected exception does not kill the node | VALIDATED | regression test, reproduced pre-fix failure |
| Missing dependency keeps reporting (not one-shot) | VALIDATED | regression test, ≥2 messages observed |
| Shutdown race (timer vs. destroy_node) | IMPLEMENTED (fix not reproducibly verified) | one real observation pre-fix; 8 deliberate reproduction attempts post-removal did not recur it — see note above |
| Real algorithm package import + call path | VALIDATED | dedicated test against the actual `autokart_perception` package present in this dev environment |
| Portability: clean workspace, export bundle, build, launch | VALIDATED | full run recorded above, twice |
| Real YOLO26s inference against a trained model | NOT RUN | no model path supplied this session |
| Real camera frames / real cone track | NOT RUN | no HP60C hardware, no surveyed track |
| `/perception/detections`, `/perception/obstacles` | NOT RUN (not implemented) | explicit scope reduction, see CONTRACT.md |
| Cross-node integration test with autokart_hp60c actually running | NOT RUN | each node's test suite is self-contained this session; a combined launch was not exercised |

## Explicit statement

Software-only acceptance above does not authorize or imply vehicle motion, live detection accuracy,
or calibration correctness. No flag in this change sets `vehicle_authorized`, `calibration_verified`,
or any other safety gate.
