# Current takeover behavior

See [CONTRACT.md](CONTRACT.md) for authoritative integration behavior.

The takeover adapter reuses existing MissionStore, CameraFeed,
make_ros_node and create_app. It destroys both upstream application request
publishers and installs no request/demo-drive sinks. display_only is always
enforced regardless of parameter override. `/health` is process liveness;
it is not proof that sensors or the vehicle are healthy.

Canonical `/odom`, `/mapping/cones`, HP60C RGB/depth topics are added to the
existing callbacks with sensor best-effort QoS. CameraFeed owns no device;
its timestamp/frame pairing and invalidity checks remain unchanged. A raw
unpaired/depth-unregistered stream may therefore remain unavailable, which
must be diagnosed upstream rather than relabelled or synthesized.

GET `/integration/api/state` exposes latest published state for canonical
mission, planning, safety, control, vehicle telemetry/status, system health
and sensor metadata topics, with arrival_age_s and unmodified source claims.
The cache retains one latest message per fixed topic. Nonfinite/malformed
JSON becomes explicit UNVERIFIED invalid data. The existing MissionStore
uses legacy REAL_KART/SIMULATION and LIVE/SIMULATED names: new source enums
are preserved by this endpoint and are not silently rewritten for legacy UI
validity gates. The React frontend is reused and not modified here. Complete
canonical overlays therefore require frontend consumption of this endpoint;
no claim is made that legacy UI renders every new field. No physical sensor
collection or browser visual qualification was performed.

## Prior package notes

# autokart_dashboard_gateway

Read-only ROS-to-WebSocket/HTTP Mission Control gateway. Wraps the real,
already-tested `autokart_dashboard_gateway` implementation via a lazy,
path-parameterized import (never reimplemented). See `CONTRACT.md` for the
exact interface and `DEPENDENCIES.md` for what this package needs from
elsewhere.

**This node cannot command the vehicle.** See `CONTRACT.md`'s opening
section and `reports/SENIOR_REVIEWER_REPORT.md` for the full
actuation-authority boundary audit before assuming otherwise.

## Quick start (no dependency required)

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select autokart_dashboard_gateway
source install/setup.bash
ros2 launch autokart_dashboard_gateway dashboard_gateway.launch.py config_file:=$(ros2 pkg prefix autokart_dashboard_gateway)/share/autokart_dashboard_gateway/config/dashboard_gateway_synthetic.yaml
# in another shell:
curl http://127.0.0.1:8765/health
```

With no dependency paths set, this serves only a degraded `/health`
endpoint (`gateway_available: false`) — no ROS node, no publisher, zero
external dependency.

## Running the real gateway

See `DEPENDENCIES.md`. Set `gateway_package_path` AND `perception_package_path`
(required even without camera display) in a config override, then:

```bash
ros2 launch autokart_dashboard_gateway dashboard_gateway.launch.py config_file:=/path/to/dashboard_gateway_live.yaml
```

## Tests

```bash
source /opt/ros/humble/setup.bash
source <this_or_any_workspace>/install/setup.bash
cd autokart_ros_nodes/autokart_dashboard_gateway
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test/ -q
```

- `test/test_adapter.py` — unit tests for the lazy-import adapter (7 tests,
  most run against the real checked-out dependencies, skipped if absent).
- `test/test_node_integration.py` — real rclpy node built from the real
  gateway; directly enumerates its publishers/clients via graph
  introspection to confirm the actuation-authority boundary (4 tests).
- `test/test_fault_injection.py` — degraded-app behavior when the
  dependency is unavailable (2 tests).

All 13 pass as of this writing. See `reports/VALIDATOR_REPORT.md` for the
exact run log, including re-running the REUSED package's own existing test
suites (`test_core.py`, `test_live_gateway.py`) unmodified, and the
clean-workspace portability check.

## Reusing this node elsewhere

```bash
./export_node.sh /path/to/export/dir
```

## Reports

- `reports/DEVELOPER_REPORT.md`
- `reports/SENIOR_REVIEWER_REPORT.md` — **read this one first**: the
  actuation-authority boundary audit this session was specifically asked to
  perform.
- `reports/VALIDATOR_REPORT.md`

## Status

IMPLEMENTED and TESTED (software-only). The reused package's own extensive
test suites (`test_core.py`, `test_live_gateway.py`) are reused unmodified,
not duplicated. No real browser/frontend, no real camera, was exercised
against this wrapper. **This node has no actuation authority and reports,
never decides, vehicle safety state.**
