# Gazebo Coordinate Mapping

All operational world frames use right-handed ENU units: metres, seconds and
radians.

| Quantity | Internal | Gazebo / ROS | Mapping |
|---|---|---|---|
| forward | body `+x` | `base_link +x` | identity |
| left | body `+y` | `base_link +y` | identity |
| up | world `+z` | ENU `+z` | identity |
| yaw | positive CCW about `+z` | positive CCW | identity |
| roll | positive about `+x` | positive about `+x` | identity |
| pitch | positive about `+y` | positive about `+y` | identity |
| distance | metres | metres | identity |
| angles | radians | radians | identity |

The world relationship is `map -> odom -> base_link`. `map -> odom` is the
identity static transform for the deterministic scenario; the follower alone
publishes dynamic `odom -> base_link`.

## Quaternion convention

Internal roll/pitch/yaw is converted to a normalized quaternion and published
in ROS `x, y, z, w` order. Gazebo's quaternion constructor receives
roll/pitch/yaw in radians. Unit tests cover identity, 90-degree yaw,
normalization and yaw wrap interpolation.

## Optical and sensor frames

`camera_link` and `depth_camera_link` use the robot convention (`+x` forward,
`+y` left, `+z` up). Their optical children use ROS optical convention:
`+z` forward, `+x` right and `+y` down, implemented by roll `-pi/2`, yaw
`-pi/2`. LiDAR and IMU retain robot-frame axes.

```text
map -> odom -> base_link
  -> chassis_link
  -> front steering and wheel links
  -> rear_axle_link -> rear wheel links
  -> camera_link -> camera_optical_frame
  -> depth_camera_link -> depth_camera_optical_frame
  -> lidar_link
  -> imu_link
```

## Joint signs

Measured left/right road-wheel steering angles map directly to Gazebo steering
joint position. Positive internal forward wheel rotation maps to negative
Gazebo wheel-joint position because the SDF wheel joint axis is `+y`; this sign
conversion is explicit for both front wheels and the common rear axle. Wheel
rotation is never inferred from chassis speed.

The native lap measured zero steering and wheel-position mismatch at snapshot
boundaries. These are simulation checks, not real-kart calibration evidence.
