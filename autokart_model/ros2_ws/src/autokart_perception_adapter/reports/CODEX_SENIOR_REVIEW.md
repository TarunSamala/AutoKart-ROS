# Current independent re-review disposition — 2026-10-22

**REVIEWED: no remaining P0/P1 source findings in the current diagnostic,
output-inhibited implementation. Physical qualification remains BLOCKED.**
The initial change requests below are historical review evidence and are
superseded by this re-review disposition.

Actual source and diffs were re-read after integrator fixes. Independent final
combined run used:

```bash
source /opt/ros/humble/setup.bash
source /home/mark1/Desktop/autonomous_kart/_autokart_integration/takeover_20261022/validation_ws/install/local_setup.bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONPATH="$PWD/autokart_hp60c:$PWD/autokart_perception_adapter:$PWD/autokart_mapping:$PWD/autokart_mission:${PYTHONPATH:-}" python3 -m pytest autokart_hp60c/test autokart_perception_adapter/test autokart_mapping/test autokart_mission/test autokart_mission/test/reviewer_stop_proof.py -q --import-mode=importlib
```

Result: **99 passed in 12.31s**, exit 0. This includes the eleven independent
reviewer regressions and the latest developer regressions. Earlier combined
failure was real and is retained below; the current combined run resolves it.
No hardware or physical stop tests were performed. This is a software review,
not vehicle motion approval. These nodes always report vehicle_authorized=false.

Re-read fixes: dependency module origins are verified; source normalization
happens before live model/calibration gates. Exact RGB/depth/info stamps,
registered dimensions and finite calibrated focal/principal-point checks are
present. LIVE source monotonic timestamp passes to the reused SDK while ROS
capture stamp remains on outputs; raw capture sequence is preserved. Invalid
metadata fails closed. Real arbitrary obstacle coverage remains unavailable,
which is accurately published as invalid rather than synthetic free space.

## Historical first review

# Independent Codex senior review — 2026-10-22

Reviewer owned nodes10–12, and did not implement nodes01/02/06/07. Reviewed
actual current source and git diff against baseline, not developer assertions.
No implementation edits to these four packages. New independent tests only:
`autokart_mission/test/reviewer_stop_proof.py`.

Independent test commands (from isolated autokart_ros_nodes):

```bash
source /opt/ros/humble/setup.bash
source /home/mark1/Desktop/autonomous_kart/_autokart_integration/takeover_20261022/validation_ws/install/local_setup.bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONPATH="$PWD/autokart_mission:${PYTHONPATH:-}" python3 -m pytest autokart_mission/test/reviewer_stop_proof.py -q --import-mode=importlib
```

Result **11 passed in 0.04s**, covering independent-ground-sensor failure,
wheel-only source rejection, brake/VCU/clock faults, duplicate ACK-like packet
not building hold, simulated hold never becoming physical proof, moving
feedback retaining latch and explicit disarmed reset.

Combined four-package existing-suite run with corresponding four package
PYTHONPATH entries: **1 failed, 63 passed in 10.67s**. The failure is
`autokart_perception_adapter/test/test_backends.py::test_algorithm_backend_real_import_runs_synthetic_geometry`,
SDK_ADAPTER_IMPORT_FAILED: No module named autokart_perception.application,
after camera backend path tests. Exposes cached dependency module/path
pollution; integrator was notified. This is not a passing validation result.

No hardware tests or physical acceptance conducted. All reviewed sources retain
vehicle_authorized=false; this review is not physical release authorization.
Line numbers refer to source as reviewed and may shift during fixes.

## Disposition: CHANGES REQUESTED

- [P1] `autokart_perception_adapter/backends.py:93-100`: same cached dependency
  path/origin defect as node01; combined suite currently fails. Explicit path
  must resolve actual code origin instead of silently accepting cached modules.

Findings fixed and re-read during this review: LIVE_PHYSICAL is normalized
before both model and MOUNT_VERIFIED gates; formerly only LIVE received the
physical calibration blocker. Synchronized RGB/depth/CameraInfo now require
exact capture stamps, matching RGB-info frame, compatible dimensions and
finite positive focal lengths/principal points. JSON scalar metadata now
fails closed rather than raising AttributeError. Metadata and images must
match before processing; output retains capture stamp. Synthetic sources stay
SIMULATED, obstacle coverage unavailable on real algorithm backend remains
explicitly invalid. This is correct diagnostic behavior; empty arbitrary
obstacle output is not measured free-space evidence.
