# Sensor Fidelity Presets

| Stream | Real-time SIL | Balanced | Offline |
| --- | --- | --- | --- |
| RGB | 160×90 @ 10 Hz | 640×480 @ 20 Hz | 1280×720 @ 30 Hz |
| Depth | 160×90 @ 10 Hz | 640×480 @ 20 Hz | 1280×720 @ 30 Hz |
| LiDAR | 180 @ 10 Hz | 540 @ 15 Hz | 1080 @ 20 Hz |
| IMU | internal @ 200 Hz | internal @ 200 Hz | internal @ 200 Hz |

Real-time SIL deliberately trades perception fidelity for closed-loop timing.
It must not be used to claim performance at the balanced/offline resolutions.

Every delivery records capture, processing-completion and delivery time.
Capture time is the ROS header timestamp. Drops are separated into warm-up
rejection, intentional rate decimation, queue overflow, processing deadline
miss, transport drop, and stale rejection. Queue depths are bounded in both
Gazebo and ROS.

The qualifying run delivered 165 RGB, 165 depth, 165 LiDAR, and 3,310 IMU
payloads. It reported no empty payload and maximum queue depth one. One initial
RGB/depth stale capture and one LiDAR warm-up capture were explicitly rejected;
they were not published as valid.
