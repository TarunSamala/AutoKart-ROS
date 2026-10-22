# ROS node grouping (not a ROS package)

This directory groups the four ROS 2 adapters specified by `AUTOKART_NODES_AGENT_MASTER_PLAN.md`. It intentionally has no `package.xml`; colcon discovers the child packages recursively.

- `autokart_perception/`: the existing ROS package shell, relocated here without changing its package name or Python algorithm library. The authoritative Python import remains `autokart_perception` from `autonomy_stack_phase1/phase_01_perception/src/autokart_perception`. Its ROS wrapper module uses the distinct name `autokart_perception_ros` to avoid import shadowing.
- `autokart_mapping/`: new adapter package around the tested shared geometry/map APIs.
- `autokart_planner/`: new adapter package around existing mission/path planning APIs.
- `autokart_controller/`: new adapter package around `kart_sim.autonomy.PurePursuitController`.

RC-03–05 now install disarmed `perception_node`, `mapping_node`, and
`planner_node` executables; `controller_node` (Pure Pursuit candidate, engine in
`autokart_controller/engine.py`, 12 pure-logic tests) is added and publishes only
`/autokart/controller/motion_command_candidate`. The new
`autokart_bringup/launch/rc03_rc05_diagnostics.launch.py` runs their read-only
diagnostic graph. It does not supply odometry or brake capability: fixtures or
verified external providers must provide those explicitly. Its optional
gateway serves the existing Mission Control UI; the controller output is a candidate topic with no VCU subscriber. Existing `autokart_planning` and `autokart_control`
remain in place pending staged migration.
