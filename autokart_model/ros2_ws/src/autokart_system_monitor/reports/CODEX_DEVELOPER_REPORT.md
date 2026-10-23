# Codex developer evidence — 2026-10-22 (Asia/Kolkata)

Role: developer only. Independent reviewer and validator approval pending.
Work executed only in isolated takeover worktree. No commits, hardware access,
flashing, energizing or vehicle motion. Existing source cores were imported,
never edited through dependency symlinks.

## Exact test command

Working directory:
`/home/mark1/Desktop/autonomous_kart/_autokart_integration/takeover_20261022/worktree/autokart_ros_nodes`

```bash
source /opt/ros/humble/setup.bash
source /home/mark1/Desktop/autonomous_kart/_autokart_integration/takeover_20261022/validation_ws/install/local_setup.bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONPATH="$PWD/autokart_safety:$PWD/autokart_dashboard_gateway:$PWD/autokart_system_monitor:${PYTHONPATH:-}" python3 -m pytest autokart_safety/test autokart_dashboard_gateway/test autokart_system_monitor/test -q --import-mode=importlib
```

Final result: **54 passed in 4.15s**, exit 0. Includes real rclpy pub/sub,
real reused core/gateway imports, malformed/replay/synthetic input rejection,
ARM rejection despite true calibration flags, zero application gateway
publishers, HTTP GET serialization and POST rejection, bounded arrival rate
and nonmonotonic timestamp evidence.

Earlier environment failures retained explicitly: default pytest plugin
loading failed with `ModuleNotFoundError: _pytest.scope` from host anyio;
autoload-disabled test run against old kart install failed collection because
installed autokart_msgs lacked MissionDebug. The isolated freshly built msgs
prefix resolved that dependency. These were environment failures, not tests
silently omitted or substituted with mocks.

```bash
python3 -m compileall -q autokart_safety/autokart_safety autokart_dashboard_gateway/autokart_dashboard_gateway_node autokart_system_monitor/autokart_system_monitor
```

Result: exit 0. Full colcon build/integrated launch is root integrator scope;
not independently claimed here. Physical qualification NOT EXECUTED, motion
NO-GO. Historical reviewer reports predate these changes and are not approval.

## Changes and remaining gates

Extends existing monitor; no second monitor subsystem. All canonical critical
status topics configured; best-effort subscriptions receive sensor publishers.
Reports bounded measured arrival rates, source age with unverified clock-domain
flag, stamp monotonicity counters and original reported integrity/hash/latency
fields. Fresh but invalid payloads get ERROR evidence diagnostics. Startup
seen-once latch remains historical and cannot be an arming gate. Queue backlog
and USB reset counts stay unknown unless upstream provides them. Optional
resource sampler unchanged. Bag recording belongs to bringup.

## Export command

```bash
bash autokart_system_monitor/export_node.sh /home/mark1/Desktop/autonomous_kart/_autokart_integration/takeover_20261022/exports
```

Result: exit 0; source archive generated in exports. Export scripts support
AUTOKART_MSGS_SRC for standalone dependency resolution where applicable.
Runtime algorithm dependencies remain explicit in DEPENDENCIES.md.

## Independent reviewer corrections

Reviewer identified an odometry provenance mismatch in the legacy view.
All unpaired legacy pose subscriptions were removed; canonical /odom is
paired with same-stamp, fresh, valid /vehicle/odometry_status and a source
matching the MissionStore container before rendering. Raw unmatched poses
remain diagnostic only. New regression prevents SYNTHETIC-to-REAL_KART
relabeling. Monitor now reads sample_time_ns from String JSON statuses.
Safety consumes canonical planning/control/mapping/perception diagnostics
and canonical String odometry status. Planning speed profile subscriber
uses actual String type. Gateway includes /perception/diagnostics and
/hp60c/diagnostics plus SensorDeliveryMetadata /perception/status.

Source test suite rerun after corrections: **55 passed in 4.47s**, exit 0.
Reviewer re-read source changes; independent validator still pending.
