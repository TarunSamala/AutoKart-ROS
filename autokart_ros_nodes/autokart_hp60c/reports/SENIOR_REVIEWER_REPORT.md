# Senior reviewer report — autokart_hp60c

Performed as a distinct, sequential review pass over the developer's actual source and diff (this
repository has no multi-agent runtime wired up; per the mission statement's own instruction, this is
a genuine sequential review, not a simulated second agent).

## Defects found and their resolution

1. **Dead/misleading conditional in `_publish_status`** —
   `cumulative_delivered = self.cumulative_captured if quality == "OK" else self.cumulative_captured`
   branched on `quality` but returned the same value either way, which reads as "intended to differ"
   but silently never does. **Fixed**: replaced with an unconditional assignment and a comment
   explaining why `delivered == captured` is actually correct here (no queue, synchronous publish),
   so a later reader doesn't "fix" it to carry a real distinction without justification.

2. **Capture thread could die silently on an unanticipated exception** — `_capture_loop` only caught
   `CameraBackendError`. Any other exception from `backend.read()` (e.g. a bug inside `HP60CBackend`,
   or a future backend that raises something else) would propagate out of the thread target,
   terminate the daemon thread, and leave `/hp60c/status` frozen at its last value — with the ROS
   node process otherwise still alive and apparently fine. This is exactly the "silent exception
   handling" failure mode the mission statement prohibits, and it would be invisible to anything
   checking only "is the node up." **Fixed**: added a catch-all `except Exception` branch that
   reports `UNEXPECTED_BACKEND_ERROR:<type>:<message>` via `/hp60c/status` and keeps the loop
   running. **Verified with a new test** (`test_unexpected_exception_reports_fault_and_keeps_running`)
   using a backend that raises a plain `RuntimeError`.

3. **`backend is None` (misconfigured dependency) published exactly one fault status, then the
   thread returned and went silent forever** — a persistent misconfiguration (e.g., wrong
   `algorithm_package_path` left in a live config file) would show one warning and then nothing,
   relying entirely on an external staleness watchdog (node 12, not built in this session) to notice.
   **Fixed**: this path now loops and republishes the fault every second until the node is stopped,
   matching the behavior of every other fault path in this node. **Verified** with
   `test_missing_backend_keeps_reporting_fault_not_just_once`.

## Checked and found correct

- **Units/frames**: RGB published as `rgb8` (matches the ctypes buffer's `(h,w,3) uint8` layout
  exactly, no channel reordering needed since the native bridge already fills an RGB buffer, not
  BGR — confirmed against `camera.py`'s buffer dtype/shape, did not re-derive from the C++ source
  since that native layer is explicitly out of scope/unchanged). Depth published as `16UC1` raw
  (not pre-scaled to metres), with the scale factor documented rather than silently applied —
  correct per the master architecture doc's explicit warning not to assume mm.
- **CameraInfo math**: `k = [fx,0,cx,0,fy,cy,0,0,1]` is the standard row-major 3x3 intrinsic matrix;
  checked against `sensor_msgs/CameraInfo`'s documented layout. `p` mirrors `k` into 3x4 with no
  baseline, correct for a non-stereo monocular+depth source (RGB and depth are not treated as a
  stereo pair anywhere in this node).
- **Single camera-SDK-owner rule**: `HP60CBackend` is only ever constructed from within
  `HP60CNode.__init__`/`build_backend`; nothing else in this package opens the SDK. The existing
  embedded capture thread in the live perception node is a separate, independent owner today — this
  is flagged as a known cross-node concern in `CONTRACT.md` and `DEVELOPER_REPORT.md`, not silently
  ignored. It is out of scope to resolve in this node's package alone (that's a system-level,
  cross-package integration decision for a later phase).
- **No hidden/hardcoded path**: grepped the package for `/home/mark1` and absolute paths; the only
  occurrences are in `config/hp60c_live.yaml`'s *comment* (an example, not an active default) and in
  docs/reports (prose, not code). The actual default parameter values are empty strings.
- **Test independence**: tests do not depend on anything outside the sourced ROS workspace + this
  package + `autokart_msgs`; verified by the validator running them from a tarball-only clean
  workspace with the original repo workspace not sourced.
- **QoS**: camera streams use `qos_profile_sensor_data` (best-effort/volatile), matching master
  architecture doc §15. Status uses a small reliable queue (depth 10), appropriate for a low-rate
  diagnostic stream that should not drop silently.

## Not independently re-verified by this reviewer

- The native `hp60c_bridge.cpp` / vendor SDK math (frame geometry, calibration blob interpretation)
  was authored before this session and is unchanged by this node; this review did not re-derive it,
  consistent with "do not rewrite/re-litigate working algorithms unnecessarily." If that native layer
  is wrong, this node inherits the error — that risk already existed in the embedded capture thread
  and is not introduced or hidden by this change.

## Post-integration fix (found by the node 03-05 workstream's review of the same pattern)

4. **Unclean Ctrl-C shutdown.** `main()` called `rclpy.shutdown()` unconditionally (raises `RCLError`
   if SIGINT already shut the context down), and the capture thread could publish into an
   already-invalidated context before `destroy_node()` set `_stop`, dying with a traceback.
   **Fixed**: `main()` guards shutdown with `rclpy.ok()` and catches `ExternalShutdownException`;
   the capture thread swallows exceptions only when `self.context.ok()` is already false (real faults
   while the context is valid still raise). Verified with a real `SIGINT`: 1/1 runs showed a
   traceback before the fix, 0/5 after; full suite still 13/13. The same `main()` fix was applied to
   `autokart_perception_adapter` (0 tracebacks on SIGINT, 11/11).
