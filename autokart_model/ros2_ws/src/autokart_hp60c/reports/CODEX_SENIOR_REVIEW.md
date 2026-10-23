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

Re-read fixes: backend configured source path moves first and module file
origin is checked; wrong cached sources fail explicitly. Unaligned CameraInfo
uses k/p zero sentinel rather than fabricating registered depth calibration.
Source capture timestamp and raw monotonic timestamp remain explicit.
No remaining P0/P1; measured camera/depth calibration acceptance is blocked.

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

- [P1] `autokart_hp60c/backends.py:120-124`: configured algorithm path only
  enters sys.path if absent, and imports may reuse cached autokart_perception
  from another path. That defeats explicit dependency provenance and causes
  the observed combined-suite failure. Resolve/verify module origin against
  configured path and isolate cached conflicting modules; rerun combined tests.
- [P2] `autokart_hp60c/node.py:181-182` and `intrinsics_to_camera_info`: depth
  CameraInfo is filled with RGB dimensions/intrinsics and invented zero
  distortion regardless of frame.aligned. For unregistered frames, do not
  publish RGB intrinsics as depth calibration; use SDK depth calibration or
  publish an explicitly uncalibrated depth CameraInfo. Retain aligned=false.

Positive findings: source monotonic capture timestamp is preserved explicitly;
ROS capture time is translated using observed age rather than stamping at
publication. Nonmonotonic/stale source frames are rejected. Device owner lock
is acquired before SDK start. Physical depth scale and calibrated mount remain
unqualified. No new SDK opener was added outside this node.
