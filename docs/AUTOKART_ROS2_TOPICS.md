# AutoKart ROS 2 Interface Contract (Agent 1)

Real topic names, from source (`autokart_bringup/config/topics.yaml` plus direct grep of publishers/subscribers), not assumed.

## Canonical topic list (`autokart_bringup/config/topics.yaml`)

```
clock:        /clock
transforms:   /tf, /tf_static
joint_states: /joint_states
vehicle:      /vehicle/odometry, /vehicle/state, /vehicle/wheel_states, /vehicle/actuator_feedback
vcu:          /vcu/state, /vcu/safety_state, /vcu/faults, /vcu/validated_command
safety:       /safety/envelope, /safety/estop
sensors:      /sensors/camera/image, /sensors/camera/camera_info, /sensors/depth/image,
              /sensors/depth/points, /sensors/lidar/scan, /sensors/imu
autonomy:     /perception/cones, /perception/obstacles, /localization/odometry,
              /localization/status, /planning/path, /planning/trajectory,
              /control/motion_command
diagnostics:  /diagnostics
command_rule: "/control/motion_command -> VCU only; no integrated actuator command topics"
```

## What's actually wired today (verified by running the combined launch, not just reading config)

| Topic | Type | Publisher | Subscriber(s) |
|---|---|---|---|
| `/vehicle/odometry` | `nav_msgs/Odometry` | external (sim bridge / real odometry — not supplied by `rc03_rc05_diagnostics.launch.py` itself) | `autokart_dashboard_gateway`, `front_safety_envelope_monitor` |
| `/perception/obstacles` | `autokart_msgs/ConeArray` | external/test harness today — **no production node publishes this yet**; see Open Issues | `front_safety_envelope_monitor` |
| `/perception/cones` | `autokart_msgs/ConeArray` | `autokart_perception` (`perception_node`) | `autokart_mapping`, `autokart_dashboard_gateway` |
| `/safety/envelope` | `autokart_msgs/SafetyEnvelope` | `front_safety_envelope_monitor` | `autokart_dashboard_gateway` |
| `/vcu/state` | `autokart_msgs/VcuState` | (sim) `autokart_sim_bridge` | `autokart_dashboard_gateway` |
| `/autokart/perception/rgb`, `/autokart/perception/depth` | `sensor_msgs/Image` | `autokart_perception` (when `camera` feed enabled) | `autokart_dashboard_gateway` (`CameraFeed`) |
| `/dashboard/debug` | `autokart_msgs/MissionDebug` | `autokart_perception`, `autokart_mapping`, `autokart_planner` (stage diagnostics) | `autokart_dashboard_gateway` |
| `/control/motion_command` | `autokart_msgs/MotionCommand` | `autokart_controller` | `autokart_sim_bridge` (`vcu_bridge`, gated `integrated_mode`) |
| `/mission/ws`, `/mission/api/*` | WebSocket/HTTP (not a ROS topic) | `autokart_dashboard_gateway` | browser (`phase_sim`) |

**Note on `/perception/obstacles`:** the canonical topic list declares it, and `front_safety_envelope_monitor` already subscribes to it, but no existing production node publishes generic (non-cone) obstacles to it — only this session's test harness did, for RC-06 integration evidence. This is the real gap behind TASK C9 ("generic obstacle monitoring") — the topic and the consumer exist; the producer doesn't yet. See `AUTOKART_OPEN_ISSUES.md`.

## Coordinate frames (confirmed, not reassigned)

- `base_link`: X forward, Y left, Z up — matches ROS REP-103 convention, used consistently in `autokart_perception`'s `mount.py`/`depth.py` output and in `front_safety_envelope_monitor`'s obstacle handling. Origin: **not yet physically marked/surveyed** (open issue, see `AUTOKART_CAMERA_GEOMETRY.md`).
- `camera_color_optical` / `camera_optical`: X right, Y down, Z forward — standard optical-frame convention, used in `mount.py`'s `optical_to_kart` axis-conversion matrix.
- `odom`: world-fixed, used by `autokart_dashboard_gateway`'s `MissionStore` (requires `frame_id == "odom"`, `child_frame_id == "base_link"` on incoming `Odometry`, rejecting anything else — a real, tested guard, not just documentation).

## Not independently re-verified this session

QoS compatibility matrix (TASK A5) and a full TF-tree validation (`ros2 doctor` style) were not re-run across the complete combined graph — the combined launch requires external odometry input that isn't supplied standalone (see `AUTOKART_DEPLOYMENT.md`). Marked **NOT TESTED**, not assumed clean.
