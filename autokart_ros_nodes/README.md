# autokart_ros_nodes — reusable AutoKart ROS 2 node library

This is a growing library of **standalone, independently portable** ROS 2 Humble packages, built per
`AUTOKART_12_NODE_MASTER_ARCHITECTURE.md` (supplied separately; a copy of the prompt pack that
specified this library is not duplicated here — see the repository root for the original
`AutoKart_Claude_12_Node_Production_Prompt_Pack.zip`). It is additive to this repository: it does not
modify, replace, or depend on the live production workspace at `kart/ros2_ws/`, which remains the
active runtime for the existing simulator/replay/bench work.

See:
- [`AUDIT_PHASE0.md`](AUDIT_PHASE0.md) — repository/interface audit performed before any node was built.
- [`INTERFACE_FREEZE_PHASE1.md`](INTERFACE_FREEZE_PHASE1.md) — which shared interfaces are reused as-is.

## Nodes in this library

| # | Package | Status |
|---|---|---|
| 01 | [`autokart_hp60c`](autokart_hp60c/) | IMPLEMENTED, TESTED (software-only; see its `reports/VALIDATOR_REPORT.md`) |
| 02 | [`autokart_perception_adapter`](autokart_perception_adapter/) | IMPLEMENTED, TESTED (software-only; cones+status only, detections/obstacles deferred — see its `reports/VALIDATOR_REPORT.md`) |
| 03 | [`autokart_imu`](autokart_imu/) | IMPLEMENTED, TESTED (software-only; **new code** — no MPU-6500 driver existed; orientation never fabricated (`orientation_covariance[0]=-1`); live I2C path and static TF NOT RUN; see its `reports/VALIDATOR_REPORT.md`) |
| 04 | [`autokart_vcu_bridge`](autokart_vcu_bridge/) | IMPLEMENTED, TESTED (software-only; reuses `autokart_vehicle_bridge` protocol/core via path parameter; relays only, no motion authority — throttle requires an external `safety_armed` input, fails closed; live USB-serial path NOT RUN; see its `reports/VALIDATOR_REPORT.md`) |
| 05 | [`autokart_odometry`](autokart_odometry/) | IMPLEMENTED, TESTED (software-only; reuses `autokart_localization` `WheelOdometry` via path parameter; **no IMU fusion**; sole `odom->base_link` writer in-package — duplicate-TF hazard with the Gazebo follower plugin; lifted-axle/push tests NOT RUN; see its `reports/VALIDATOR_REPORT.md`) |
| 06 | [`autokart_mapping`](autokart_mapping/) | IMPLEMENTED, TESTED (software-only; see its `reports/VALIDATOR_REPORT.md`) |
| 07 | [`autokart_mission`](autokart_mission/) | IMPLEMENTED, TESTED (software-only; braking FSM has a reported, pre-existing named-state gap — see its CONTRACT.md; see `reports/VALIDATOR_REPORT.md`) |
| 08 | [`autokart_planner`](autokart_planner/) | IMPLEMENTED, TESTED (software-only; curvature/speed-profile/collision/stop-envelope candidates, no mission FSM, no authority; `a_lat_max_mps2` UNVERIFIED, braking params unconfigured ⇒ fails closed by default; see its `reports/VALIDATOR_REPORT.md`) |
| 09 | [`autokart_controller`](autokart_controller/) | IMPLEMENTED, TESTED (software-only; Pure Pursuit command **candidate** only, no authority; reuses `kart_sim` via path parameter; wheelbase/steering limits UNVERIFIED; verified end to end with node 08; see its `reports/VALIDATOR_REPORT.md`) |
| 10 | [`autokart_safety`](autokart_safety/) | IMPLEMENTED, TESTED (software-only arming authority — **NOT the physical E-stop**; reuses `autokart_vcu_safety` core via path parameter; fails closed without it; see its `reports/VALIDATOR_REPORT.md`) |
| 11 | [`autokart_dashboard_gateway`](autokart_dashboard_gateway/) | IMPLEMENTED, TESTED (software-only; read-only, actuation-authority boundary audited — no critical finding; see its `reports/SENIOR_REVIEWER_REPORT.md`) |
| 12 | [`autokart_system_monitor`](autokart_system_monitor/) | IMPLEMENTED, TESTED (software-only; reports only, no safety authority; see its `reports/VALIDATOR_REPORT.md`) |

## Full-graph integration

[`autokart_nodes_bringup`](autokart_nodes_bringup/) brings up all 12 nodes above together for the
first time (`disarmed_sim.launch.py`), with a launch test that verifies every topic flows, every
node reports fresh on node 12's `/system/health`, and killing an upstream node propagates
downstream as staleness rather than silence. Permanently DISARMED by construction — see that
package's own README.md for the exact wiring, the two adapters it adds, the real bug it found and
fixed, and the known, deliberate gaps it does not paper over.

## How to reuse a node (general procedure, demonstrated for node 01)

1. `cd autokart_ros_nodes/<node>/ && ./export_node.sh /some/export/dir`
2. Copy the resulting tarball to the target workspace, extract into its `src/`.
3. `colcon build --packages-up-to <node>`
4. `source install/setup.bash`
5. `ros2 launch <node> <node>.launch.py config_file:=<path to a config/*.yaml>`
6. Read that node's `CONTRACT.md` for topics/types/rates and `DEPENDENCIES.md` for anything it needs
   from outside the bundle (shared interfaces, hardware SDKs).

## Honesty notes carried over from the master architecture doc

- A node here being IMPLEMENTED/TESTED is a **software-only** claim, verified by this session's own
  colcon builds, pytest runs, and live `ros2 launch`/`ros2 topic echo` checks (see each node's
  `reports/VALIDATOR_REPORT.md` for the actual commands and output, not just a pass/fail summary).
- No claim here authorizes or implies physical vehicle motion. `vehicle_authorized`,
  `calibration_verified`, `braking_verified` remain false/unset everywhere in this repository; this
  library does not touch those flags.
- Where a capability could not be exercised in this environment (no Raspberry Pi, no camera, no
  VCU), it is marked `BLOCKED` or `NOT RUN`, not claimed as passing.
