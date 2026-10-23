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

Re-read fixes: map expires_ns and capture age are rechecked immediately
before engine.evaluate, after waiting for companions. BRAKING_TEST requires
atomic same-stamp base_link obstacle-status coverage_complete=true, valid=true,
matching source and unexpired TTL; missing coverage cannot advance READY/RUNNING.
Matched odometry companions and capture-time interpolation are required.
STOP hold, duplicate/session/fault handling and reset remain as independently
tested. Skidpad continues using the existing winding/gate state machine and
now honors explicit interruption/E-stop latch. No remaining P0/P1 source finding.
P2: mission_braking_test.yaml's legacy obstacles_topic override was reported
for canonical alignment; integrated launch must use /perception/obstacles.

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

- [P1] `autokart_mission/node.py:312-324`: map TTL checked on receipt but not
  after waiting for odometry/obstacles; `_maybe_process` can consume a map
  after expires_ns while pose remains within 250ms. Recheck current map age
  and expiry immediately before evaluate; reject without refreshing capture.
- [P1] `autokart_mission/node.py:239-250,318-324`: BRAKING_TEST waits for an
  obstacle ConeArray but has no atomic source/validity/coverage companion.
  An empty array can become a clear corridor and valid PATH_CANDIDATE in
  reused legacy braking geometry, despite real arbitrary-obstacle coverage
  being declared unavailable by node02. Require stamped fresh matching
  obstacle coverage status before READY/RUNNING; missing coverage stays HOLD
  or latched STOP. Do not interpret missing observations as observed free space.

Positive findings: StopVerifier rejects wheel-only/odometry-derived ground
motion, requires independent ground evidence, finite stationary velocities,
clock/VCU/stop/fault evidence and brake feedback when instrumented. Session or
nonmonotonic/stale input resets hold; duplicate identity cannot accumulate
samples. Synthetic/replay verified holds never set physical_stop_verified.
BrakingFSM retains STOP latch after evidence loss, refuses restart, and reset
requires explicit disarmed + current verified hold. Mission wrapper reuses
existing winding/gate progress and adds interrupt/E-stop latch to skidpad as
well. Matched odometry source/capture interpolation was added during review.

Independent reviewer tests PASS as above. These tests do not replace physical
stop tests or establish that a self-reported ground sensor is commissioned.
