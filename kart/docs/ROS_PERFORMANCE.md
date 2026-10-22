# ROS Performance

The real-time bridge uses:

- one `SingleThreadedExecutor` for deterministic callback ordering;
- a separate authoritative process to remove Python GIL contention;
- reliable command/authority/metadata paths;
- best-effort sensor payload QoS with bounded depth four;
- metadata depth 128;
- 20 Hz full pose/joint/TF diagnostics and 100 Hz follower application/clock;
- direct local IMU validation without subscribing to its own publisher;
- representative payload hashes capped at eight per sensor.

The bridge remains one way. Its snapshot publisher has no VCU input endpoint.
No duplicate TF, wheel state, steering state, or IMU authority was added.

Loaned messages were not used because support was not verified for this
ROS 2 Humble/Python/Gazebo Classic combination. ROS callback latency is shown
as `NOT MEASURED` in the dashboard rather than fabricated.

Native ROS builds completed for `autokart_msgs`, `autokart_gazebo`, and
`autokart_sim_bridge`. The dependency-closure rebuild completed and package
tests reported 6 tests, 0 errors, 0 failures, and 0 skipped.
