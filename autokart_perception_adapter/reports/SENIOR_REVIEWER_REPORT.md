# Senior reviewer report — autokart_perception_adapter

Distinct, sequential adversarial pass over the developer's source, diff and tests (no multi-agent
runtime wired up in this environment; a genuine second look, not a rubber stamp).

## Defects found and fixed

1. **Test-isolation bug: global `sys.path`/`sys.modules` mutation leaked between tests.**
   `AlgorithmBackend.__init__` permanently inserts its `algorithm_package_path` at `sys.path[0]` and
   never removes it — harmless in a real node process (one backend constructed once per process
   lifetime), but running `test_algorithm_backend_rejects_path_without_autokart_perception` (which
   creates a FAKE, empty `autokart_perception` package in a tmp dir) before
   `test_algorithm_backend_real_import_runs_synthetic_geometry` (which points at the REAL algorithm
   package) caused the real-import test to fail: the fake tmp-dir entry, still sitting earlier in
   `sys.path` from the prior test, shadowed the real package. The test passed in isolation and failed
   in the full suite — exactly the kind of order-dependent failure that's easy to miss by running
   tests one at a time during development. **Fixed** by adding an autouse pytest fixture
   (`_isolate_autokart_perception_import_state`) that snapshots/restores `sys.path` and purges
   `autokart_perception`-named entries from `sys.modules` around every test in `test_backends.py`.
   Verified: ran the full file before the fix (reproduced the failure), applied the fix, reran (all
   green).

2. **Shutdown race: a timer callback could publish during `destroy_node()`.** The node had no
   `destroy_node()` override; `rclpy`'s default teardown could run while a scheduled timer callback
   (the 1Hz fault-report timer, or the standalone-mode timer) was still invoking
   `self.*_pub.publish(...)`, surfacing as an unretrieved `"cannot use Destroyable because
   destruction was requested"` exception during this node's own clean-workspace portability test.
   Same class of bug `autokart_hp60c`'s review caught in its capture thread. **Fixed**: added a
   `destroy_node()` override that cancels both timers before calling `super().destroy_node()`. Unlike
   finding 1, the validator's deliberate reproduction attempts afterward did not recur this one on
   demand (see VALIDATOR_REPORT.md) — it is a correct defensive fix for a real, observed hazard
   class, not a fix confirmed by repeatable failing-then-passing reproduction.

## Checked and found correct

- **Cone publication gating**: traced every path to `self.cones_pub.publish(...)` — it is inside
  `if outcome.valid:` with no other call site. An unqualified or faulted result cannot reach
  `/perception/cones` under any backend or exception path (verified by
  `test_unqualified_result_never_publishes_cones`, which also waits 0.3s after the fault to confirm
  no cone arrives, not just checks immediately).
- **CameraInfo unpacking**: `fx,cx,fy,cy = k[0],k[2],k[4],k[5]` matches `k = [fx,0,cx,0,fy,cy,0,0,1]`
  as published by `autokart_hp60c` (checked against that node's own `node.py`, not assumed).
  `message_filters.Subscriber` instances are registered through the owning `Node` (confirmed by
  reading the installed `message_filters` package's source at
  `/opt/ros/humble/.../message_filters/__init__.py`'s `Subscriber.__init__`, which calls
  `node.create_subscription`), so they are destroyed automatically by `Node.destroy_node()` — no
  explicit cleanup needed beyond the timers.
  (did not spend further review time scanning `/opt/ros/humble` reportedly ships `message_filters`
  as a C++-backed package; the standard-library-adjacent convention it follows is well-established in
  ROS 2 and not specific to this project.)
- **Naming collision avoidance**: confirmed `autokart_perception_adapter` does not collide with either
  existing `autokart_perception` package (grepped both `kart/ros2_ws` and `autonomy_stack_phase1` for
  the exact string `autokart_perception_adapter` before this session — zero hits).
- **No actuation authority**: this node has no publisher that could plausibly be read as a motion
  command (`/perception/cones`, `/perception/status` only) — correctly out of scope per its row in
  the master architecture doc.

## Not independently re-verified by this reviewer

- The underlying `Pipeline`/`CoreRuntime`/YOLO26s detection math predates this session and is reused,
  not rewritten; this review did not re-derive its correctness, consistent with "do not
  rewrite/re-litigate working algorithms unnecessarily." The monolithic-`Pipeline`
  (perception+mapping+curvature bundled together) finding in the developer report is a structural/
  scope observation, not a correctness claim about that code.

## Post-integration fix (found by the autokart_nodes_bringup integration pass)

3. **LIVE cones could never qualify.** `AlgorithmBackend.process()` treated *every* reason from the
   reused algorithm as blocking. Node 02 has no pose input, so the algorithm always reports
   `INVALID_POSE`/`NO_VALID_PATH` (planning reasons), which meant a LIVE frame's cones were rejected
   even with a verified calibration. The production `qualified()` in
   `kart/ros2_ws/.../autokart_perception_ros/node.py` only blocks on `LIVE_CONE_BLOCKERS`
   (perception-integrity reasons) plus `CALIBRATION*`/`MOUNT*`. **Fixed** by porting that function
   exactly as `backends.qualify()`, including its REPLAY rule (a real recording stays unqualified
   unless its recorded source was synthetic). 4 regression tests added; the old all-reasons logic
   blocks the key case (confirmed by evaluating the old rule against it, not by a full revert).
   Suite 15/15; the 12-node `autokart_nodes_bringup` integration test still passes 3/3.
