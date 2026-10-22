# Build ledger (max 60 lines)

GPIO resolved: EN=GPIO1, BRK=GPIO2, F/R=GPIO39 (matches frozen pin budget xlsx +
vcu_firmware/include/config.h, already correct, unchanged). bench/PINOUT.txt's GPIO7-for-F/R is
stale/wrong (GPIO7 is E-stop aux status per budget); not fixed here, flagged only.

Architect decision (revised): gateway.py's own comment says "this gateway has no actuator
publisher and cannot bypass [vcu_safety's] decision" -- preserved. New demo-drive candidate math
(direction-lock-while-moving, timed hold, 500ms browser-heartbeat dead-man) lives in a NEW pure
class `autokart_vehicle_bridge/demo_drive.py` (`DemoDriveController`, no rclpy, unit-testable),
owned by the bridge node that already does `CommandGate`. Gateway gets exactly ONE new narrow
publisher: a heartbeat beacon (`/autokart/demo_drive/heartbeat`, timestamp only) bumped whenever
the browser sends ANY demo-drive websocket message -- it carries no throttle/direction/speed data,
so the gateway still cannot command motion by itself. `demo_drive_start`/`demo_drive_stop` extend
the EXISTING `ALLOWED_SAFETY_REQUESTS` whitelist (autokart_vcu_safety decides, as today; only legal
once already ARMED). Drive parameters (direction, duration) go through a new, separately
whitelisted `/autokart/demo_drive/control` message, validated by the gateway before forwarding.
CommandGate, 2.0V firmware cap, CMD_RANGE fault, 300ms LINK_TIMEOUT: unchanged, reused as-is.

| # | Task | Owner | Files | Status | Validator |
|---|---|---|---|---|---|
| 1 | GPIO/pin verify | A | (none, verify only) | DONE | pending |
| 2 | WIRING_SIGNOFF.md | D | docs/WIRING_SIGNOFF.md | DONE | pending |
| 3 | readiness gateway endpoint | B | gateway.py, readiness_report.py + tests | DONE (29+11 tests pass) | pending |
| 4 | demo_drive controller+safety state+bridge wiring | A/B | demo_drive.py(new), vcu_safety/core.py+node.py, vehicle_bridge/node.py | DONE (17+24 tests) | pending |
| 5 | gateway demo_drive heartbeat+control relay | B | autokart_dashboard_gateway/gateway.py | DONE (33 tests) | pending |
| 6 | dashboard readiness panel | C | RosLivePanel.tsx (/ops) | DONE (reads new /mission/api/readiness) | pending |
| 7 | dashboard DEMO DRIVE panel | C | RosLivePanel.tsx, useMissionDataLive.ts, MissionControlLive.tsx | DONE (18+ frontend tests, tsc+build clean) | pending |
| 8 | fault-injection tests | D | demo_drive.py tests (heartbeat/over-cap/direction/boot/stop); CommandGate 5V-clamp test pre-existing, reused | DONE | pending |
| 9 | DEMO_RUNBOOK.md | D | docs/DEMO_RUNBOOK.md | DONE | pending |
| 10 | run_all_tests.sh green + launch cmd | D | scripts/run_all_tests.sh (+demo_drive_ros_bundle_suite) | DONE | PASS (see verdict below) |

Conflicts: none.

VALIDATOR final verdict: PASS. perception/kart/tools/perception-ROS-adapter/demo-drive-ROS-bundle
all green. phase_sim frontend: green except 2 pre-existing LabSkidpad tests unrelated to this work
(timeout only under heavy external load -- an already-running, external ROS graph process
(perception_node) is consuming ~3.8 CPU cores continuously on this machine, not started by or
related to this task; confirmed those 2 tests pass cleanly in isolation). No verified flag changed
(grepped all four config files post-change). docs/WIRING_SIGNOFF.md unsigned (blank sign-off
section, confirmed by reading). Dashboard readiness/VCU panels show explicit NO DATA/UNKNOWN on
fetch failure, never a fabricated number (tested). Firmware 2.0V cap: pre-existing, triple-checked
(firmware CMD_RANGE fault + belt-and-brace clamp; CommandGate's existing 5V-request test; new
DemoDriveController over-cap test) -- not weakened.
