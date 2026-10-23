# Independent senior review — 2026-10-22

Reviewer: sensors workstream developer reviewing independently authored nodes
10–12. Actual source, diff, reused Mission Control callbacks and source interface
schemas were read; this review did not rely on developer-report approvals.
No safety/dashboard/monitor implementation changes were made by this reviewer.

## Findings raised and resolved by owning developer

1. P1 — dashboard source mix at original readonly.py:84: canonical /odom was
sent directly to MissionStore, allowing a synthetic pose under REAL_KART
configuration. Resolved in autokart_dashboard_gateway_node/readonly.py:43–78
with bounded pose/status pairing, acquisition stamp match, validity, source
normalization and freshness checks. Legacy unpaired odometry subscriptions
are removed before canonical callbacks are installed (lines 99–109). Invalid
or unmatched raw data remains diagnostic cache data; it does not enter pose
rendering through this path. Regression test covers matching/mismatched source.

2. P2 — monitor source age absent for canonical String JSON: extraction read
only ROS headers. Resolved in autokart_system_monitor/node.py:119–120 by reading
integer sample_time_ns from JSON when no message stamp exists. Arrival rate
remains separately labelled; clock relation remains explicitly unverified.

3. P2 — safety observed legacy graph names rather than canonical planner,
controller and odometry status. Resolved in autokart_safety/node.py:26–31,
84–85 with canonical diagnostics plus String odometry-status adapter. All
public authority outputs remain permanently inhibited regardless of core state.

4. P2 — canonical planning speed profile subscribed with incompatible
MissionDebug type. Resolved in readonly.py:140–141 using std_msgs/String,
matching the planner's publisher. No conflicting alias added on this topic.

## Authority and evidence audit

Safety excludes ARM/power-confirm/demo-drive start. Both available-core and
missing-core paths emit vehicle_authorized=false, output_inhibited=true,
valid=false and a zero-speed stop request. Forwarded requests are stop/disarm
only. Diagnostic core state cannot authorize integrated hardware.

Gateway destroys request publishers and passes no HTTP/WebSocket control sinks.
It reuses existing Mission Control and camera subscribers, adding no camera or
serial owner. Integration snapshot preserves source claims and arrival age;
raw messages never gain physical qualification from receipt. Camera rendering
and complete browser presentation were not visually qualified by this review.

Monitor has no command publisher. It reports bounded arrival-rate/source-age
history, explicit missing metrics and invalid upstream evidence. Startup latch
is historical seen-once evidence, documented as unsuitable for current health
or arming. Clock-domain validity, USB resets and backlog are not invented.

## Executed software evidence and disposition

Independent reviewer execution against fresh ROS Humble autokart_msgs prefix:
**55 passed in 4.47s**, no failures/skips, across safety/dashboard/monitor tests.

Disposition: PASS for the reviewed permanent-inhibit, read-only software scope;
no outstanding blocking defect from this review. Full browser visual acceptance,
full ROS deployment graph, measured timing under Pi load and physical release
remain separate evidence gates. This review grants no actuator, calibration,
firmware, braking, independent E-stop or vehicle-motion qualification.
