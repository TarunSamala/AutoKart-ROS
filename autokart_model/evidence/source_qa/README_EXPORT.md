# AutoKart clean ROS 2 workspace

All twelve packages and autokart_msgs are in src/. Reused pure Python algorithms
are copied unchanged under dependencies/ with their original required package
relationships. SOURCE_SHA256.json records the exported bytes. No original source
path is required. Configure paths from this export location.

    source /opt/ros/humble/setup.bash
    colcon build --executor sequential
    source install/local_setup.bash
    ros2 launch ./integration/disarmed.launch.py dependency_root:=$PWD/dependencies

The default graph is diagnostic synthetic and DISARMED. The legacy mock VCU
intentionally cannot supply qualified clock/session/calibration measurements, so
odometry and downstream motion candidates remain invalid. Explicit synthetic
integration fixtures test the complete data flow separately. No physical rate,
calibration, braking or emergency-stop qualification is included. See package
contracts for dependencies and separate hardware setup. Models and SDK binaries
are not bundled. The nominal geometry file is historical operator input, never
independent survey or physical release evidence.
