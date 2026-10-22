# ROS 2 frame tree

The required hierarchy is `map -> odom -> base_link -> chassis_link -> sensor/wheel/linkage`
with camera optical children where applicable. `FrameTree` rejects duplicate
children, cycles, invalid identifiers, and zero-norm quaternions, normalizing
accepted quaternions. Physical mounts and reported calibration stay separate in
the sensor model; a transform publication must not overwrite physical pose.
