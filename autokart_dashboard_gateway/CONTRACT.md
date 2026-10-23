# autokart_dashboard_gateway — interface contract

## What this node is

A **read-only** ROS-to-WebSocket/HTTP gateway for Mission Control. It wraps
the real, already-tested implementation at
`kart/ros2_ws/src/autokart_dashboard_gateway` (`gateway.py`, `core.py`,
`camera_feed.py`) via a lazy, path-parameterized import — never
reimplemented.

## What this node is NOT

It is not an actuation path. **The only effect any browser connected to this
gateway can have on the vehicle is a bounded, whitelisted request STRING**:

- `/autokart/safety/request` (`std_msgs/String`, one of `arm`, `stop`,
  `reset_fault`, `confirm_manual_power_cut`, `demo_drive_start`,
  `demo_drive_stop`) — evaluated **only** by `autokart_safety` (node 10),
  which may refuse it for any of the reasons in that node's own `CONTRACT.md`.
- `/autokart/demo_drive/control` (`std_msgs/String`, JSON
  `{"action": "drive"|"stop"|"ping", "direction": "forward", "duration_s": <=10}`)
  — acted on only by the vehicle bridge's `DemoDriveController`, which only
  ever drives when its own `active` flag is true, and that flag is derived
  **solely** from `autokart_safety`'s published `demo_drive_active` (itself
  only ever true while `ARMED`) AND the vehicle bridge's own `safety_armed`
  flag (derived solely from `autokart_safety`'s published state, never from
  anything this gateway sends). See `reports/SENIOR_REVIEWER_REPORT.md` for
  the full file/line audit of this chain.

This node has **no other publisher, no service client**, and categorically
cannot send an arm/throttle/brake/steering command directly — confirmed by
this package's own `test/test_node_integration.py`, which enumerates the
node's actual rclpy publishers/clients via graph introspection, not just by
reading source.

## Topics (when `mode: "live"`, dependency loaded)

| Topic | Type | Direction |
|---|---|---|
| `/vehicle/odometry`, `/perception/cones`, `/autokart/mapping/cones`, `/planning/path`, `/autokart/planner/path_candidate`, `/control/motion_command`, `/vehicle/actuator_feedback`, `/vcu/state`, `/safety/envelope`, `/dashboard/debug` | various | sub (display only) |
| `/autokart/safety/request` | `std_msgs/String` | pub (bounded whitelist, see above) |
| `/autokart/demo_drive/control` | `std_msgs/String` | pub (bounded whitelist, see above) |

## HTTP/WebSocket surface

`/health` (process liveness only), `/mission/api/manifest`,
`/mission/api/status`, `/mission/api/readiness` (read-only passthrough of a
saved file, never computed here), `/mission/api/camera*`, `/mission/ws`
(read + the one whitelisted outbound request type). A `read_only` aiohttp
middleware rejects every HTTP method except `GET`/`HEAD` with `405`. See the
real gateway's own `gateway.py` for the exact handler bodies (unchanged by
this wrapper).

## Data-source labeling

`LIVE`/`REPLAY`/`SIMULATED`/`UNVERIFIED` labeling is read verbatim from
upstream messages (e.g. `/safety/envelope`'s `data_source` field) — this
node never defaults an unset/garbage value to `LIVE`; see the reused
`safety_envelope_payload()` and this package's own
`test/test_adapter.py`/upstream `test_live_gateway.py` coverage of that
property (tested in the real, reused package — not duplicated here).

## Dependency-unavailable behavior

If `gateway_package_path` or `perception_package_path` is unset, wrong, or
the resolved import does not match the given path (see DEPENDENCIES.md for
why that last check exists), this node creates **no ROS node and no
publisher at all** — it serves only a degraded `/health` endpoint reporting
`gateway_available: false`, `vehicle_authority: "OFF"`,
`actuator_authority: "NONE"`. See `test/test_fault_injection.py`.

## Modes

- `mode: "replay"` (default) — no ROS node; serves `create_app(...,
  replay=True)` directly from a `MissionStore` optionally pre-loaded from
  `replay_path`.
- `mode: "live"` — real `rclpy` node via the reused `make_ros_node()`,
  subscribes to the topics above.
- `display_only: true` — disables both request sinks even in `mode: "live"`
  (pure display, no forwarding at all).
