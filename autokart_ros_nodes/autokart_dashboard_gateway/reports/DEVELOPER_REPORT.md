# Developer report — autokart_dashboard_gateway

## Audit performed before writing any code

- Read `kart/ros2_ws/src/autokart_dashboard_gateway/autokart_dashboard_gateway/`
  in full: `gateway.py` (516 lines — ROS subscriptions, `create_app`'s
  aiohttp routes/WebSocket, `send_request`/`send_demo_drive_message`/
  `send_demo_drive_ping`), `core.py` (361 lines — `MissionStore`, the
  timestamp-joined `WorldFrame` normalization, `mapping_visualization`),
  `camera_feed.py` (188 lines — paired RGB/depth JPEG preview, status
  matching). Read both existing test files
  (`test/test_core.py` — 12 tests, `test/test_live_gateway.py` — 21 tests)
  to understand exactly what is already covered before deciding what this
  wrapper needs to test itself.
- Traced the **entire** actuation-authority chain by reading, not assuming:
  - `gateway.py`'s `DashboardGatewayNode.__init__` creates exactly two
    publishers: `self.request_pub` (`std_msgs/String` to
    `/autokart/safety/request`) and `self.demo_drive_pub` (`std_msgs/String`
    to `/autokart/demo_drive/control`). No other publisher, no service
    client, anywhere in the file.
  - `send_request()` publishes only if `action in ALLOWED_SAFETY_REQUESTS`
    (`{"arm", "stop", "reset_fault", "confirm_manual_power_cut",
    "demo_drive_start", "demo_drive_stop"}`), else returns `False` and
    publishes nothing.
  - `send_demo_drive_message()` additionally validates via
    `validate_demo_drive_request()` (direction must be `"forward"` —
    `"reverse"` is explicitly refused; duration must be in `(0, 10]` s).
  - `/autokart/safety/request` is consumed **only** by `autokart_safety`
    (node 10, this library) / the original `autokart_vcu_safety.node`, both
    of which independently re-validate against the identical whitelist
    before doing anything, and even an `"arm"` request is refused unless
    every one of that node's own readiness gates holds.
  - `/autokart/demo_drive/control` is consumed by
    `kart/ros2_ws/src/autokart_vehicle_bridge/autokart_vehicle_bridge/node.py`'s
    `DemoDriveController`. Read `demo_drive.py` (92 lines) in full:
    `compute()` only ever produces nonzero throttle when
    `self.active and self.driving` and the output still passes through the
    vehicle bridge's existing `CommandGate` (ARMED + fresh-command + voltage
    ceiling) in `node.py`'s `_tick()` — confirmed by grep, not assumed.
    `self.active` is set via `demo_drive.set_active(self.safety_armed and
    bool(payload.get("demo_drive_active")))` (`node.py` line 136), where
    `self.safety_armed` is itself set via `self.safety_armed = valid and
    str(payload.get("state")) == "ARMED"` (`node.py` line 134) from
    `/autokart/safety/status` — **the dashboard never writes `safety_armed`
    or `demo_drive_active`; both come only from `autokart_safety`'s own
    published state.**
- See `reports/SENIOR_REVIEWER_REPORT.md` for the full write-up of this
  chain as the primary, explicitly-requested review finding.

## Decision: reuse the real gateway via a lazy import, do not rewrite it

`autokart_dashboard_gateway_node/adapter.py` imports
`autokart_dashboard_gateway.{gateway,core}` from
`kart/ros2_ws/src/autokart_dashboard_gateway` at runtime, via
`gateway_package_path` — never hardcoded. `node.py` is new (ROS-parameter
driven entry point replacing the original's `argparse` CLI, a degraded
fail-closed `/health`-only fallback, and the package metadata) but contains
no reimplementation of the gateway's own normalization/HTTP/WebSocket logic.

A naming collision had to be resolved: the dependency IS a Python package
named `autokart_dashboard_gateway`. This wrapper's own importable module is
therefore named `autokart_dashboard_gateway_node` (the ROS package/directory
name is still `autokart_dashboard_gateway`, matching this library's node-11
naming) — see DEPENDENCIES.md.

A second, non-obvious coupling was found and had to be designed around:
`gateway.py` unconditionally imports `camera_feed.py`, which unconditionally
imports `autokart_perception.live_render` — so even a camera-disabled,
replay-only deployment of the real gateway requires the separate
`autokart_perception` package to be resolvable. Documented in full in
DEPENDENCIES.md, with an explicit second path parameter
(`perception_package_path`) and a verified-resolution check (see reviewer
report) rather than silently requiring a global install.

## What was implemented

- `autokart_dashboard_gateway_node/adapter.py`: `GatewayAdapterError`,
  `load_gateway()` (with the perception-path verification described above),
  `load_core()` (the perception-free subset: `MissionStore`/`valid_frame`),
  `load_camera_feed()`.
- `autokart_dashboard_gateway_node/node.py`: a parameter-reading bootstrap
  node, dispatch to `mode: "replay"` (no further ROS) or `mode: "live"`
  (the real `make_ros_node()`), and `_run_degraded_app()` — a bare `/health`
  endpoint with explicitly zero ROS node/publisher when the dependency
  cannot be loaded.
- Full ament_python package skeleton, launch file, two config profiles.
- 13 tests across three files (adapter unit tests, real-rclpy
  actuation-boundary introspection tests, degraded-app fault injection).
- `export_node.sh`, `CONTRACT.md`, `DEPENDENCIES.md`, `README.md`.

## Known gaps (explicit, not hidden)

- No real browser/frontend was exercised against this wrapper's HTTP/
  WebSocket surface in this session — only the underlying `aiohttp` app
  factory, already extensively tested by the reused package's own test
  suite (re-run unmodified, not duplicated here; see VALIDATOR_REPORT.md).
- No real camera hardware; `enable_camera_feed` defaults `false`.
- `mode: "live"` itself (subscribing to real ROS topics end to end with a
  live vehicle bridge/safety node/perception stack all running together)
  was not exercised as a full system in this session — each node in this
  library was validated independently. A follow-up cross-node bring-up test
  (launching nodes 10 and 11 together with synthetic/injected topic traffic)
  would strengthen this further but was out of this session's scope.
