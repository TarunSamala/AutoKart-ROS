# Senior reviewer report — autokart_dashboard_gateway

Performed as a distinct, sequential review pass: re-read the actual reused
source (not the developer's summary of it), ran the full test suite
(including the reused package's own, unmodified), and specifically performed
the actuation-authority boundary audit this session's instructions called
out as "the most safety-critical review item in scope."

## PRIMARY FINDING: actuation-authority boundary — no critical issue found, full chain documented

**Question: can the dashboard (any browser connected to this gateway) send
an arm/motion command upstream without going through the actual safety
node?**

**Answer: No. Verified by reading every hop of both possible paths, with
file/line citations, not by trusting a docstring.**

### Path 1: `/autokart/safety/request`

- `kart/ros2_ws/src/autokart_dashboard_gateway/autokart_dashboard_gateway/gateway.py`,
  `DashboardGatewayNode.send_request()` (lines 150–158): publishes a plain
  `std_msgs/String` to `/autokart/safety/request` **only if** `action in
  ALLOWED_SAFETY_REQUESTS` (line 27–28:
  `{"arm", "stop", "reset_fault", "confirm_manual_power_cut",
  "demo_drive_start", "demo_drive_stop"}`); otherwise returns `False` and
  publishes nothing.
- The only consumer of `/autokart/safety/request` is `autokart_safety`
  (this library's node 10; `autokart_ros_nodes/autokart_safety/autokart_safety/node.py`,
  `_on_request()`), which **independently** re-checks against the identical
  whitelist (`ALLOWED_REQUESTS`) before adding anything to
  `pending_requests`, and even then an `"arm"` entry only ever results in
  `vcu_req = "ARM"` if `self.state == "DISARMED"`, `not self.stop_latched`,
  `ready` (every upstream freshness/verification gate — see that node's own
  `CONTRACT.md`), and `vcu.state == "SAFE"` (reused `core.py`,
  `SafetyLogic.step()`, the `elif r == "arm":` branch). The dashboard has no
  way to set any of those gating conditions itself.
- Confirmed this is the SAME whitelist-then-independently-re-validated
  pattern the original (not-yet-wrapped) `autokart_vcu_safety.node` also
  implements (`ALLOWED_REQUESTS` there is textually identical) — the gateway
  cannot even depend on `autokart_safety` specifically to be safe; the
  consumer side independently enforces this regardless of which safety node
  is listening.

### Path 2: `/autokart/demo_drive/control`

- `gateway.py`'s `send_demo_drive_message()` (lines 167–179) additionally
  validates via `validate_demo_drive_request()` (lines 33–43): action must
  be `"stop"` or `"drive"`; `"drive"` requires `direction in
  DEMO_DRIVE_DIRECTIONS` (`{"forward"}` only — line 29, `"reverse"` is
  explicitly excluded with a comment pointing at the firmware reason) and
  `0.0 < duration_s <= DEMO_DRIVE_MAX_DURATION_S` (10.0 s).
- The only consumer is
  `kart/ros2_ws/src/autokart_vehicle_bridge/autokart_vehicle_bridge/node.py`'s
  `_on_demo_drive_control()` (line 92) feeding
  `autokart_vehicle_bridge/demo_drive.py`'s `DemoDriveController`.
  `DemoDriveController.compute()` (demo_drive.py lines 79–91) only ever
  returns nonzero throttle when `self.active and self.driving`, and that
  output still passes through the vehicle bridge's pre-existing
  `CommandGate` in `node.py`'s `_tick()` (line 246:
  `self.gate.compute(now, self.safety_armed, vcu_armed, throttle_v,
  brake_pct, cmd_wall)`) — confirmed by reading `_tick()`, not assumed from
  a comment.
- **The critical chain**: `node.py` line 136,
  `self.demo_drive.set_active(self.safety_armed and
  bool(payload.get("demo_drive_active")))`, where `payload` is the parsed
  `/autokart/safety/status` message and `self.safety_armed` is set at line
  134, `self.safety_armed = valid and str(payload.get("state")) ==
  "ARMED"`. **Both `safety_armed` and `demo_drive_active` are read-only
  derivations of `autokart_safety`'s own published state** — the dashboard
  gateway never publishes to `/autokart/safety/status`, has no subscriber
  wired to override it, and has no other code path into `self.safety_armed`
  or `self.demo_drive.active` in the vehicle bridge. The vehicle bridge's
  own `arm_request_approved()` check at line 217 is a further, independent
  gate on `"ARM"` requests specifically (not demo-drive, but confirms the
  bridge does not blindly trust an upstream `"ARM"` string either).

### Conclusion

**No critical finding.** The dashboard gateway has exactly two publishers
(confirmed empirically, not just by reading source — see
`test/test_node_integration.py::test_gateway_node_has_exactly_two_publishers_both_bounded_requests`,
which enumerates the real node's publishers via rclpy's own graph
introspection and asserts the set is exactly these two topics), both are
bounded request *strings* subject to a fixed whitelist enforced on **both**
the sending and receiving side, and the one topic that actually produces a
throttle value (`demo_drive.compute()`) is gated by a flag that only
`autokart_safety`'s own `ARMED` state can set — never anything the gateway
sends. If a future change to either the gateway or the vehicle bridge adds
a new publisher, service client, or a way to set `safety_armed`/
`demo_drive_active` from something other than `/autokart/safety/status`,
`test_gateway_node_has_exactly_two_publishers_both_bounded_requests` and
`test_gateway_node_has_no_service_clients_either` will fail and must be
re-reviewed before being accepted — this is deliberately a regression test
for the boundary itself, not just a one-time audit note.

## Defect found and fixed (adapter, not the reused gateway)

**`adapter.load_gateway()`'s `perception_package_path` could be silently
ignored in favor of a different, globally-installed copy of
`autokart_perception`.**

While writing this adapter's own tests, a test asserting that a *wrong*
`perception_package_path` fails to import did **not** raise — the real
gateway module imported successfully anyway. Investigated rather than
weakened the test: this development machine has `autokart_perception`
permanently importable via a site-wide editable pip install
(`~/.local/lib/python3.10/site-packages/__editable__.autokart_perception-0.2.0.pth`,
pointing at `autonomy_stack_phase1/phase_01_perception/src`), present on
`sys.path` independent of anything this adapter does. The original
implementation only protected against *stale cached* modules (the same
class of bug already found and fixed in `autokart_safety`'s adapter) — it
did not protect against Python's import system finding a *different*,
legitimately-importable copy of the same package elsewhere on `sys.path`
when the given path doesn't actually contain it. That is a real, if narrow,
risk: a deployer could set the wrong `perception_package_path` and the node
would silently run against whatever `autokart_perception` happens to be
globally installed, with no indication the path they specified was ignored.

**Fixed**: `load_gateway()` now explicitly imports `autokart_perception`
first and verifies the resolved module's `__file__` actually lives under
the given `perception_package_path` (`Path.relative_to`), raising
`PERCEPTION_DEPENDENCY_PATH_MISMATCH` otherwise — before ever attempting the
transitive `gateway.py` import. **Verified** with
`test/test_adapter.py::test_load_gateway_wrong_perception_path_is_rejected_not_silently_substituted`,
which reproduces this exact machine's global-editable-install condition and
confirms the adapter now refuses rather than silently substitutes.

## Checked and found correct

- **Read-only HTTP surface**: `create_app`'s `read_only` middleware (reused,
  unchanged) rejects every non-`GET`/`HEAD` method with `405` — re-confirmed
  by re-running the reused package's own
  `test_live_gateway.py::test_status_without_odometry_and_http_reconnect`
  (asserts a `POST /mission/api/status` returns 405) unmodified.
- **Data-source labeling never defaults to LIVE**: re-ran
  `test_live_gateway.py::test_safety_envelope_payload_never_defaults_missing_data_source_to_live`
  unmodified — still passes.
- **This wrapper's own degraded-dependency path has no ROS node at all**:
  confirmed by reading `node.py`'s exception branch (`rclpy.shutdown()` is
  called, and `_run_degraded_app()` never touches `rclpy`) and by live
  `ros2 topic list` during the clean-workspace portability test showing no
  topic from this node in that state (see VALIDATOR_REPORT.md).
- **No hardcoded project-home paths in source**: grepped the package; only
  occurrences are in `config/dashboard_gateway_live.yaml`'s comments
  (documentation) and in this report's prose.

## Not independently re-verified by this reviewer

- The reused `gateway.py`/`core.py`/`camera_feed.py` logic itself (frame
  joining, cone/path normalization, camera pairing) was authored and already
  reviewed before this session and is unchanged — this review re-ran its
  existing test suite (33 tests, see VALIDATOR_REPORT.md) rather than
  re-deriving the logic from scratch.
- `autokart_vehicle_bridge`'s `CommandGate`/`arm_request_approved()` own
  correctness was read and cited above but not independently re-tested by
  this package (that is `autokart_vehicle_bridge`'s own, separately tested,
  concern, and out of this session's three-node scope).
