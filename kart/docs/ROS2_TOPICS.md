# ROS 2 topic contracts

| Topic | Meaning | Safety boundary |
| --- | --- | --- |
| `/clock` | authoritative simulated time and reset epoch | never wall clock |
| `/sensors/imu`, `/sensors/camera/image`, `/sensors/lidar` | operational sensor streams | original measurement timestamp |
| `/autonomy/motion_command` | high-level timed motion command | converted to `MotionCommand`, then VCU validates it |
| `/vcu/state`, `/diagnostics` | safety and health telemetry | read-only for autonomy |
| `/debug/ground_truth/*` | test-only truth | disabled in production SIL |

Safety-relevant command contracts include source, sequence, generation time,
expiry, values in SI units, and the VCU checksum. ROS reliable transport is not
treated as integrity or freshness evidence.
