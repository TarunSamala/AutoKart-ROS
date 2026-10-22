# Gazebo Integration

The installed runtime is Gazebo Classic 11.10.2. AutoKart uses it initially as
a static/kinematic visual and sensor follower while internal physics owns kart
motion.

```text
internal state + reset epoch + owner ID
        -> validated VisualState
        -> Gazebo pose/joint follower boundary
        -> RGB/depth/LiDAR/IMU sensor interfaces
```

There is no Gazebo drive, Ackermann, differential-drive, steering, brake or
motor command plugin. The model is `static` in the initial follower role so
Gazebo does not apply a second vehicle-motion solution. The transport-neutral
`GazeboVisualFollower` rejects epochs, owners, backwards time and actuator
command fields. A native continuous pose/joint bridge is the leading incomplete
integration item.

Worlds:

- empty engineering pad;
- acceleration course with timing/E-stop zones;
- skidpad preview with infields/timing line;
- cone-track environment;
- dry/wet split and oil patch;
- fault lane with obstacle intrusion and stop boundary.

All use configured physics, lighting, sky color, shadows and non-grey surface
materials. Track-generated worlds are available from `kart-sim track export`.

Native evidence on 2026-07-29:

- model and six worlds: `gz sdf -k` PASS;
- empty world launch: PASS;
- kart spawn: PASS;
- stable static follower pose for two seconds: PASS;
- advertised RGB, depth, LiDAR and IMU Gazebo topics: PASS;
- headless payload capture: `NOT_EXECUTED_HEADLESS_TRANSPORT_CAPTURE`;
- ROS sensor bridge and driving follower: not executed.

Run:

```bash
./scripts/run_gazebo_sim.sh
AUTOKART_HEADLESS=1 ./scripts/run_gazebo_sim.sh
./scripts/test_gazebo_native.sh
```

`physics_backend: gazebo` is reserved for a reviewed custom solid-axle system
plugin and backend parity tests. It is not implemented or silently selected.

