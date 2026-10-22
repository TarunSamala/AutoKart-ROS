# autokart_nodes_bringup — full-graph integration for the autokart_ros_nodes library

Launch-only package (Phases 5-6 of the original spec). Brings up all 12 `autokart_ros_nodes`
packages together for the first time, plus two small typed-topic adapters this integration pass
added, and a launch test that exercises the live graph. Software-only: no Pi, camera, IMU or ESP32
is attached, and this package never authorizes vehicle motion (see "Stays DISARMED" below).

## What's here

- `launch/disarmed_sim.launch.py` — the main deliverable. All 12 nodes, synthetic/standalone/
  builtin backends, every output inhibited, permanently DISARMED.
- `launch/replay_demo.launch.py` — optional demo, real recorded HP60C frames through the real
  perception algorithm instead of the synthetic layout. See "Replay demo" below for what this
  does and does not show.
- `autokart_nodes_bringup/safety_armed_bridge.py` — bridges node 10's `MissionDebug` arming state
  to node 04's `std_msgs/Bool safety_armed` input (the gap node 04's own CONTRACT.md names
  explicitly).
- `autokart_nodes_bringup/mapping_path_bridge.py` — bridges node 06's `centerline_odom` geometry
  to node 08's `nav_msgs/Path reference_path` input (a gap found while wiring this graph: node 07
  computes a path internally but its own CONTRACT.md says it deliberately never publishes one).
- `autokart_nodes_bringup/replay_publisher.py` — stands in for node 01 in the replay demo only,
  publishing real recorded frames (node 01 itself has no "replay a recording" backend).
- `test/test_graph_integration.py` — launches the real `disarmed_sim.launch.py` as a subprocess and
  checks it from real ROS topics (see "Test results" below).

## The wiring (what was remapped, and why)

| Link | Mechanism |
|---|---|
| 01 hp60c -> 02 perception_adapter | topic names already match; `input_mode` overridden from `standalone_timer` to `topics` so 02 actually consumes 01's output |
| 02 -> 06 mapping | remap: mapping's `cones_topic` default (`/autokart_mapping/cones_in`) -> `/perception/cones` |
| 04 feedback -> 05 odometry | `source` overridden to `actuator_feedback` (default is an internal synthetic generator); `counts_per_m` overridden to `0.0` (speed mode — `ActuatorFeedback` carries no raw axle counts, see odometry's own CONTRACT.md "Displacement semantics"; the synthetic profile's `counts_per_m: 100.0` is a counts-mode test value and is refused, `COUNTS_MODE_UNSUPPORTED`, for this source) |
| 05 -> 06/07/08/09 | remap: odometry's hardcoded `/odom` publisher -> `/autokart/odom`, the `odom_topic` default every downstream node already expects |
| 06 -> 08 planner | **[mapping_path_bridge]** — see "Known, deliberate gap" below; node 07 is not and cannot be this link |
| 08 -> 09 controller | topic names already match, no remap |
| 09 -> 04 vcu_bridge | **found during this integration, not previously documented**: vcu_bridge's `candidate_topic` default (`/autokart/controller/motion_command_candidate`) does **not** match controller's real, hardcoded output topic (`/autokart_controller/motion_command_candidate`) — fixed by overriding `candidate_topic` |
| 10 -> 04 vcu_bridge | **[safety_armed_bridge]** — see node 04's own CONTRACT.md "Relationship to node 10" |
| 10 mapping-status input | remap: safety's hardcoded `/autokart/mapping/status` subscription -> `/autokart_mapping/status`, the one safety input that is genuinely wireable (same `MissionDebug` type on both sides) |
| 03 imu | standalone; CONTRACT.md is explicit that `autokart_odometry` does not fuse IMU data, and no other node in this library subscribes it |

## Known, deliberate gaps this integration does NOT paper over

1. **Node 07 (`autokart_mission`) cannot be node 08's path source.** Its own CONTRACT.md: "This
   node never publishes a path ... publishing `path_odom` here would blur \[node 08's\] boundary."
   The literal "06 -> 07 -> 08" topic chain the mission brief describes has no topic to carry a
   path from 07 to 08 — not something this integration broke, something the node's own contract
   forbids. Node 07 still runs, in parallel, consuming the same node 06 status for its own
   skidpad/braking FSM and publishing its own diagnostic status; `mapping_path_bridge` reads node
   06 directly instead. See that module's docstring for the full reasoning.
2. **Node 10 (`autokart_safety`)'s perception/planner/controller/localization freshness inputs
   can never be wired in this graph.** Its `STATUS_TOPICS` dict hardcodes
   `autokart_msgs/MissionDebug` subscriptions on `/autokart/{perception,planner,controller}/status`
   and `/autokart/localization/status`. In this library: node 02 publishes
   `SensorDeliveryMetadata` (not `MissionDebug`) on `/perception/status`; node 08 publishes
   `std_msgs/String` JSON (not `MissionDebug`) on its own topics; node 09 publishes
   `std_msgs/String` JSON (not `MissionDebug`) on `/autokart_controller/diagnostics`; no node in
   this library plays "localization" at all. A ROS topic remap cannot bridge a type mismatch, and
   this integration will **not** fabricate `MissionDebug` wrappers around those nodes' real output
   just to make safety's gate see "fresh" — that would be exactly the kind of invented data this
   whole library's honesty discipline forbids. The practical effect: these four of safety's five
   upstream gates are permanently `UNAVAILABLE` in this graph, which is one more reason (on top of
   `calibration_verified`/`vehicle_config_verified`/`braking_verified` defaulting `false`) arming
   can never succeed here — confirmed live, see "Test results" below.
3. **Node 10's mapping-status gate additionally requires `source == "LIVE"`** (hardcoded in
   `node.py`, not a parameter). Node 06 in this launch runs `source_mode: "SIMULATED"` (honestly —
   it is simulated), so even the one input that *is* correctly wired by topic and type never
   counts as "fresh" for arming either. This is correct, intended fail-closed behavior for a
   simulated graph, not a bug to fix.
4. **Node 02's synthetic backend output is `ground_truth=true`**, and node 06's own qualification
   logic explicitly rejects ground-truth cone input (`UNQUALIFIED_CONE_FRAME`) — a real sensor
   mapper must never treat a known-fabricated layout as an observation. In `disarmed_sim`
   (synthetic backend throughout) this means node 06's map is **always** `valid=false`, and
   everything downstream of it (`mapping_path_bridge`'s path, node 08's envelope, node 09's
   candidate) is consistently, correctly empty/invalid/zero-speed. This is the expected
   zero-model, disarmed result for an all-synthetic graph — see "Replay demo" for the one path
   that can produce a genuinely qualified map in this environment.
5. **Dashboard gateway (node 11)'s own hardcoded display subscriptions use yet another set of
   topic names** (`/autokart/mapping/cones`, `/planning/path`, `/control/motion_command`, etc. —
   see its CONTRACT.md) that don't match this graph's real topics either. Left unremapped: it is
   read-only/display and `display_only: true` inhibits both of its request sinks regardless, so a
   stale display has no safety consequence, unlike gap 2 above.

## Stays DISARMED

Nothing in either launch file ever publishes to `/autokart/safety/request`. Node 10's own
`calibration_verified`/`vehicle_config_verified`/`braking_verified` parameters are left at their
honest default `false`. Gaps 2 and 3 above mean arming is *also* structurally unreachable on top of
that. Live verification (see below): `/autokart/safety/status` reports `"state": "DISARMED"` with
`arm_allowed: false` and nine blocking reason codes, continuously, for the life of the launch.
`/vcu_bridge/safety_armed` (via `safety_armed_bridge`) reports `data: false` continuously.
`/autokart_controller/motion_command_candidate` reports `valid: false, target_speed_mps: 0.0`.

## Test results (commands and actual output, not just pass/fail)

Built fresh (not using the stale `kart/ros2_ws/install`, which this session confirmed is missing
`MissionDebug`/`BenchVcuStatus`/`VcuState.estop_input_fitted`):

```bash
mkdir -p /tmp/autokart_bringup_ws/src
cp -r kart/ros2_ws/src/autokart_msgs /tmp/autokart_bringup_ws/src/
ln -s <this repo>/autokart_ros_nodes/autokart_{hp60c,perception_adapter,imu,vcu_bridge,odometry,\
  mapping,mission,planner,controller,safety,dashboard_gateway,system_monitor,nodes_bringup} \
  /tmp/autokart_bringup_ws/src/
cd /tmp/autokart_bringup_ws && source /opt/ros/humble/setup.bash && colcon build --symlink-install
```
Result: **14/14 packages built** (`autokart_msgs` + the 12 node packages + this package). PASS.

```bash
source install/setup.bash
ros2 launch autokart_nodes_bringup disarmed_sim.launch.py
```
Result: all 14 processes start and keep running; no exceptions once the one real bug below was
fixed. Manually verified over several runs:
- `/vcu_bridge/status`: `quality: OK, fault_state: NONE`
- `/vehicle/actuator_feedback`: `valid: true` (real reused `autokart_vehicle_bridge`/
  `autokart_localization` control law, not the degenerate fallback)
- `/vcu_bridge/safety_armed`: `data: false`
- `/autokart/safety/status`: `"state": "DISARMED"`, 9 blocking reason codes (see above)
- `/autokart_controller/motion_command_candidate`: `valid: false, target_speed_mps: 0.0`
PASS.

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test/test_graph_integration.py -v
```
Result:
```
test_every_expected_topic_is_flowing PASSED
test_every_node_is_fresh_on_system_health PASSED
test_killing_hp60c_propagates_as_stale_not_silence PASSED
3 passed in 7.31s
```
PASS. The third test kills the real `hp60c_node` OS process mid-run and confirms node 12's
`/system/health` for `/hp60c/status` transitions to `STALE` (level 3) within the configured
2-second heartbeat timeout — the upstream failure becomes visible, not silent.

All `ros2`/node processes were confirmed terminated (`ps aux`) after every run and after the test.

### One real bug found and fixed during this integration

`autokart_odometry`'s `source: actuator_feedback` with the synthetic profile's inherited
`counts_per_m: 100.0` fails closed with `COUNTS_MODE_UNSUPPORTED` (`ActuatorFeedback` carries no
raw axle counts — see that node's own CONTRACT.md). Fixed by overriding `counts_per_m: 0.0` (speed
mode) in both launch files when switching `source` away from the synthetic default. See the
`candidate_topic` mismatch above for the second fix.

## Replay demo (optional, step 4) — NOT RUN to full qualification

```bash
ros2 launch autokart_nodes_bringup replay_demo.launch.py \
  recording_path:=autonomy_stack_phase1/phase_01_perception/recordings/hp60c-bench-20261008-03
```
`replay_publisher` correctly reads and republishes all 20 recorded frames (confirmed: process
stays up, `/hp60c/status` flows, no exceptions) through node 02's real `algorithm` backend (the
actual YOLO/Delaunay pipeline, not the synthetic 4-cone layout). **It does not reach a qualified,
published `/perception/cones`** in this environment: the underlying `autokart_perception`
`Application`/`AlgorithmCore.evaluate()` call that node 02's algorithm backend drives always
receives `pose=None` (node 02's own CONTRACT.md: it subscribes only RGB/depth/camera_info, no
pose/odometry input), so `status.safety.reasons` always includes `INVALID_POSE`/`NO_VALID_PATH`,
which `backends.py::AlgorithmBackend.process()` treats as unconditionally blocking
(`valid = not blocking`) regardless of `source_label`. This is a structural property of how node 02
calls the reused algorithm (confirmed by reading `autokart_perception/application.py`'s own
comment: "No pose provider is currently qualified on the Pi. Geometry still runs"), not an
artifact of this integration's wiring or of the specific recording used. Reaching a genuinely
qualified real-camera cone output would need either a pose feed into node 02 (out of scope — its
CONTRACT.md defines no such input) or a model/manifest that changes this gating (not available in
this environment). Reported honestly as **NOT RUN to a qualified STOP candidate**, not claimed as
passing.
