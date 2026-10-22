# Simulation Modes

**SIMULATION USE ONLY — NOT APPROVED FOR DIRECT VEHICLE DEPLOYMENT.**

All modes use the same high-level command and safety path. Presets are in
`configs/presets`; `kart-sim studio validate-preset` rejects unsafe or
double-physics combinations.

| Mode | Preset examples | Implemented execution | External boundary |
|---|---|---|---|
| Fast 2D | `minimal_ci`, `fast_f1tenth_style`, `multi_kart_race` | `AutoKartFastEnv`, vector wrapper, 2D LiDAR, collision/progress/reset | optional ROS/rendering |
| Engineering | `engineering_internal`, `fault_validation` | existing dynamic four-wheel plant, actuators, electrical/thermal, VCU, safety, faults | Chrono remains gated |
| Gazebo robotics | `gazebo_sensor_development` | Xacro/URDF/SDF, six worlds, native spawn and four Gazebo sensor interfaces | native state/ROS sensor bridge incomplete |
| Formula Student-style | skidpad/autocross presets | event configs, cone tracks, evaluator, timing/penalty/race-control contracts | not an official compliance claim |
| SIL/HIL | `sil_jetson`, `hil_vcu` | deterministic mock bridge, native ROS messages, CAN plant bridge | real Jetson/VCU/CAN unavailable |

Fast-mode API:

```python
env = AutoKartFastEnv(track, kart)
observation, info = env.reset(
    seed=7,
    options={"arm": True, "operator_acknowledged": True},
)
observation, reward, terminated, truncated, info = env.step(
    {"target_speed_mps": 3.0, "steering_rad": 0.1}
)
```

Explicit acknowledgement prevents reset from automatically restarting
autonomy. A moving reset requires `physical_stopped_confirmation`; latched
faults require `repair_acknowledged`.

Fidelity tiers are independent of modes. Tier 0–2 run headlessly; Tier 3
selects Gazebo robotics assets; Tier 4 selects unavailable Chrono/offline
research dependencies.

