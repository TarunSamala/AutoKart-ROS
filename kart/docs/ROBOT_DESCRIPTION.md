# Kart Robot Description

The canonical robot sources are:

- `ros2_ws/src/autokart_description/urdf/autokart.urdf.xacro`
- dependency-free structural output `autokart.urdf`
- Gazebo visual/sensor follower `simulation/models/autokart/model.sdf`

The description uses REP-style `x` forward, `y` left, `z` up. It contains all
required chassis, wheel, actuator, power, compute, VCU and sensor links. Extra
front steering-knuckle links provide separate steering-pivot and wheel-spin
degrees of freedom.

The Xacro values mirror `configs/karts/prototype_dynamic.yaml`:

| Parameter | Value |
|---|---:|
| total described mass | 145.0 kg |
| wheelbase | 1.15 m |
| front/rear track | 0.95/0.92 m |
| wheel radius/width | 0.14/0.10 m |
| CG | `(0.05, 0, 0.30)` m |
| yaw inertia | 35 kg m² |

These remain **SIMULATION USE ONLY — MEASUREMENT REQUIRED**. Component mass
allocation, roll/pitch inertia, chassis envelope and all sensor poses are
provisional.

The rear wheels are fixed to one rotating `rear_axle_link`. The
`rear_solid_axle_joint` is the common angular state; no differential-drive
plugin exists. The internal plant remains the engineering authority for axle
torsion, bearing loss, chain torque, scrub and inside-wheel unloading.

Visuals and collisions are separate primitive elements. No visual mesh is used
as collision geometry. `kart_sim.robot_description` checks required links,
joints, positive mass/inertia, limits, unique parentage, configuration
wheelbase, and prohibited mesh collisions.

Native checks:

```bash
xacro ros2_ws/src/autokart_description/urdf/autokart.urdf.xacro
check_urdf ros2_ws/src/autokart_description/urdf/autokart.urdf
python3 -m kart_sim.cli robot validate simulation/models/autokart/model.sdf
```

