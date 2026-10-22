# Assumptions Register

All values below are provisional and **must be measured before simulation-to-real comparison or any vehicle use**.

| ID | Assumption | Value / handling | Validation task |
| --- | --- | --- | --- |
| A-001 | Vehicle mass | 145 kg in `configs/karts/prototype.yaml` | weigh complete kart with driver/load state defined |
| A-002 | Geometry and CG | 1.15 m wheelbase, provisional CG | measurement and corner-weight test |
| A-003 | Yaw inertia | 35 kg m² | pendulum/CAD/model identification |
| A-004 | Tire parameters | linear capped model, mu 1.0, provisional stiffnesses | skidpad, braking, and combined-slip tests |
| A-005 | Motor/brake forces | simple maximum tire-contact forces | dynamometer/current/brake-pressure calibration |
| A-006 | Environment | dry level surface, 1.225 kg/m³ air density | ambient sensor and coast-down tests |
| A-007 | Integration | 1 ms semi-implicit Euler step | timestep-convergence study |
| A-008 | Frames | planar z=0 baseline; no roll/pitch/heave | Phase 3 wheel-load and chassis model |
| A-009 | Rear axle | perfect rigid axle, 0.12 kg m² inertia, 0.2 N m loss | shaft inertia, torsional compliance, and bearing-loss tests |
| A-010 | Chassis | quasi-static load-transfer coupling; 1500 N m/rad nominal stiffness | torsion rig and corner-weight/load-cell correlation |
| A-011 | Phase 4 actuator/electrical parameters | default steering, brake, BLDC, battery, and pre-charge constants are provisional | component bench tests listed in Phase 4 model documents |
| A-012 | Track geometry | native reference tracks are sampled polylines, not survey data or splines | survey and spline/resampling validation |
| A-013 | Surface materials | dry/wet friction and rolling values are provisional | friction/roughness measurement per track material |

No component values were inferred from a particular real kart or copied tire dataset.
# ROS 2/SIL provisional assumptions

- `configs/autonomy/ros2_sil.yaml` is simulation use only and not approved for direct vehicle deployment.
- The `DeterministicRosBridge` is a test transport, not evidence of DDS, rosbag, network, or real-time behavior.
- ROS middleware is optional to the core. The presence of a `ros2` CLI does not prove that the Python runtime can execute `rclpy` nodes.

# Phase 7–15 provisional assumptions

- VCU, controlled-stop, E-stop, steering, brake and traction thresholds are
  simulation-only. Real loop time, actuator onset, pressure rise, guaranteed
  deceleration, torque decay and contactor opening remain unmeasured.
- The hydraulic pressure sensor is assumed available for operational brake
  plausibility. Cable force and brake torque are simulator-internal unless the
  physical kart adds corresponding sensors.
- The independent safety model demonstrates logical separation, not physical
  electrical independence or certified behavior.
- CRCs detect transmission errors; they do not authenticate a command source.
- The ROS bridge, Chrono boundary, CARLA/Unreal follower and CAN plant bridge
  are tested local contracts. Mocks are not native integration evidence.
- The CAN protocol values and rates are provisional; no real VCU firmware or
  bus was inspected.
- Calibration optimizer tests use synthetic data. No real-kart parameter,
  uncertainty, or predictive-validity claim is made.
- Real-vehicle readiness is `REJECTED` until the evidence listed in
  `docs/EXTERNAL_INTEGRATION_STATUS.md` and the staged real test plan exists.
