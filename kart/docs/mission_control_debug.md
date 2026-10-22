# Mission Control debug contract

`/dashboard/debug` uses `autokart_msgs/MissionDebug`. It is an optional,
read-only explanation channel. The existing control and VCU messages retain
their original meaning. A publisher sets `measurement_time` to the autonomy
cycle's source time, `cycle` to its planner cycle, and `frame_id` to its planning
frame. `payload_json` is a JSON object. The gateway accepts these stages:

| Stage | Payload key | Required item fields |
| --- | --- | --- |
| `delaunay` | `edges` | `id`, `a`, `b`, `triangle_ids` |
| `gabriel` | `edges` | `id`, `a`, `b`, `accepted`, `reason`, `blocking_cone_id`, `radius_m` |
| `gates` | `gates` | `id`, `midpoint` (`x_m`,`y_m`), `width_m`, `valid`, `reason` |
| `candidates` | `paths` | `id`, `points`, `costs`, `total_cost`, `rank`, `selected`, `reason` |
| `mission` | `skidpad` | `state`, `w_left`, `w_right`, `branch`, `branch_locked`, `reason` |
| `obstacle` | `obstacles`, `safety` | obstacle pose; range, TTC, stopping distance, margin, trigger from the independent safety supervisor |
| `control` | `lookahead`, `predicted_path`, `brake_command` | measured planner/controller output |
| `safety` | `priority`, freshness and heartbeat fields | output of the independent safety supervisor |
| `metrics` | metric names | `cross_track_error_m`, `heading_error_rad`, `planner_runtime_ms`, `controller_runtime_ms` when available |

The optional `mission.skidpad` object can also include `left_center`,
`right_center`, `left_radius_m`, `right_radius_m`, `target_winding`, and
`branches`. Each branch has `name`, `points`, `valid`, optional `selected`,
and optional `reason`. Mission Control draws branch choices and winding only
from published mission debug; its DEMO winding is separately labeled as a
display-only observation of the simulated vehicle trail.

IDs must be stable within a run. Edge endpoint IDs refer to IDs in
`/perception/cones`; each point is in the declared `frame_id`. All debug data
must be derived from autonomy inputs and calculations. The gateway stores
simulator ground truth in `perception.ground_truth_cones` solely for display;
it never copies that field into planner/debug stages or publishes ROS topics.

The currently checked-in ROS planning and perception packages have no runtime
nodes, so there are no publishers for `/dashboard/debug` yet. Mission Control
shows those stages as unavailable until actual autonomy nodes publish them.

## Run

```bash
cd kart/ros2_ws
source /opt/ros/humble/setup.bash
colcon --log-base /tmp/autokart-mc-log build \
  --build-base /tmp/autokart-mc-build --install-base /tmp/autokart-mc-install \
  --packages-select autokart_msgs autokart_dashboard_gateway
source /tmp/autokart-mc-install/setup.bash
ros2 run autokart_dashboard_gateway dashboard_gateway --source SIMULATION \
  --static-root /absolute/path/to/phase_sim/dist
```

For a running FSDS ROS2 bridge, add `--fsds --scenario SKIDPAD`. In this mode
the gateway reads `/fsds/testing_only/odom` for measured simulator pose and
`/fsds/testing_only/track` for display-only physical cones. The FSDS `fs_msgs`
package must be installed in the ROS2 workspace. The track topic is latched;
the gateway retains its cones across odometry frames. These testing-only topics
are unavailable when FSDS competition mode is enabled. Perception detections,
planner stages, control feedback, and safety still come only from this kart's
ROS2 autonomy publishers. The dashboard never sends `/fsds/control_command`.

FSDS topic and message definitions: [official ROS bridge topics](https://fs-driverless.github.io/Formula-Student-Driverless-Simulator/v2.2.0/ros-bridge/),
[Track.msg](https://github.com/FS-Driverless/fs_msgs/blob/master/msg/Track.msg),
[Cone.msg](https://github.com/FS-Driverless/fs_msgs/blob/master/msg/Cone.msg).

The gateway serves the built React UI on port 8765 and has no command endpoint.
Use `--source REAL_KART` for the same downstream schema with real ROS topics.
Use `--replay /path/to/mission-frames.jsonl` to serve previously normalized
frames without ROS. For frontend development, run `npm run dev` from
`phase_sim`; Vite proxies `/mission` to the gateway. The original browser
simulator remains at `/demo` and is explicitly marked DEMO in Mission Control.
