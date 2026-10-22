# AutoKart Sim Studio Architecture

**SIMULATION USE ONLY — NOT APPROVED FOR DIRECT VEHICLE DEPLOYMENT.**

## Purpose

AutoKart Sim Studio extends the existing deterministic autonomous electric
go-kart simulator. It does not replace the validated plant, actuator, VCU,
independent-safety, fault, telemetry, replay, or dashboard modules. The platform
adds fast racing, robot-description, ROS 2, Gazebo, Formula Student-style event,
multi-kart, and race-control boundaries around that core.

The platform has four independently testable layers:

```text
 Layer 4  Gazebo visual world | RViz | web engineering UI | race control
                                  |
                         read-only state/sensors
                                  |
 Layer 3  ROS 2 bridge | autonomy | localization | perception | planning
                                  |
                     high-level MotionCommand only
                                  v
 Layer 2  VCU validation -> safety envelope -> independent safety override
                                  |
                         validated actuator request
                                  v
 Layer 1  actuators -> tires/electrical/thermal plant -> vehicle physics
                                  |
                         authoritative state
                                  +----> sensors/logging/replay/visual follower
```

Layer 3 and Layer 4 are clients of Layer 2. They do not own motor torque, brake
position, or steering-actuator state. An explicitly selected
`low_level_component_test` launch may exercise one component without the VCU;
it is rejected when autonomy, racing, ROS-integrated, or safety-evaluation mode
is active.

## Mandatory command and state paths

The integrated command path is:

```text
 planner / manual test source
 -> transport-neutral motion command
 -> optional ROS 2 serialization
 -> VCU command freshness, sequence, integrity, mode and plausibility checks
 -> VCU dynamic safety envelope
 -> independent watchdog / controlled-stop / E-stop authority
 -> steering, brake and traction actuator dynamics
 -> selected physics authority
 -> authoritative kart and wheel state
```

State flows outwards:

```text
 authoritative physics
 -> sampled sensor truth
 -> timestamped noisy sensor delivery
 -> ROS 2 / mock transport
 -> perception, localization, planning

 authoritative physics
 -> joint-state and pose follower
 -> Gazebo visuals

 authoritative physics + VCU + safety + faults
 -> telemetry -> dashboard / race control / replay / reports
```

The dashboard, RViz, Gazebo GUI, scenario editor, track editor, and race-control
UI have no actuator interface. Their launch or command requests enter through
the same VCU ingress as other sources.

## Execution modes

| Mode | Default authority | Rendering | Primary use |
|---|---|---|---|
| Fast 2D racing | `internal` | optional 2D | deterministic autonomy/RL, CI, multi-kart |
| Engineering dynamics | `internal` | optional dashboard | four-wheel, actuator, tire, electrical, thermal, safety and fault analysis |
| Gazebo robotics | `internal` with visual follower | Gazebo/RViz | robot model, sensors, TF, controllers and SIL |
| Formula Student race | `internal` | 2D or Gazebo | cone events, timing, penalties, planning and race control |
| SIL/HIL | `internal` simulated plant | optional | Jetson, virtual/physical CAN and VCU integration |

`chrono` and `gazebo` are explicit optional authorities. They never silently
fall back. Real actuator output is disabled by default in every preset.

## Modular boundaries

The existing `kart_sim` package remains the deterministic core. New packages
are added alongside it where moving established code would create needless
regression risk:

```text
kart_sim/
  scheduler.py                       fixed-step authority
  vehicle.py / four_wheel.py
  dynamic_kart.py / actuators.py     existing plant
  vcu.py / watchdog.py               existing Layer 2
  track.py / track_formats.py        source geometry and converters
  fast_env.py / controllers.py       fast racing and autonomy
  sensor_suite.py / cones.py         deterministic sensor and cone models
  multi_kart.py / race_control.py    isolated agents and session rules
  sim_studio.py                      mode/preset/authority orchestration

ros2_ws/src/
  autokart_msgs                      only domain-specific message types
  autokart_description               Xacro, URDF, SDF and ros2_control boundary
  autokart_bringup                   launch composition and presets
  autokart_gazebo                    worlds and visual-follower integration
  autokart_sim_bridge                core/ROS conversions
  autokart_control                   high-level controller output
  autokart_*                         stack interface packages

simulation/
  worlds/ tracks/ models/ plugins/   engine-facing assets
  race_control/ multi_agent/         session inputs

configs/
  presets/ vehicles/ sensors/ tracks/
  scenarios/ controllers/ safety/
```

All new Python core modules remain ROS- and renderer-independent so Tier 0–2
tests work without external middleware.

## Time, reset and determinism

- The fixed-step scheduler advances the physics authority.
- `/clock` mirrors that time and never advances physics.
- Measurement and delivery timestamps are separate.
- Rendering cadence may interpolate authoritative states but may not step them.
- Seeds belong to scenario/preset configuration and are recorded in metadata.
- Reset increments a clock epoch, clears queued commands, pauses the system,
  clears non-latched simulation state, retains required fault latches, and
  requires a new guarded enable sequence.
- A reset never republishes or reactivates a pre-reset command.

## Safety invariants

These are backend-independent acceptance conditions:

1. No expired, repeated, invalid, out-of-order, or pre-reset command controls
   the kart.
2. No integrated ROS node, plugin, GUI, or dashboard writes actuator state.
3. Independent safety overrides every normal command source.
4. Emergency stop removes normal propulsion authority.
5. Critical faults latch and are visible in telemetry and diagnostics.
6. Reset requires measured standstill and explicit operator acknowledgement.
7. Autonomous operation never restarts automatically.
8. Unknown perception cells are occupied/unsafe until a policy explicitly
   resolves them.
9. Exactly one physics authority advances vehicle state.
10. A visual follower reports an error if it receives an actuator command.

## External dependency policy

Core schemas, generated assets, package manifests, mock transport, structural
validation, and deterministic tests run without ROS 2 or Gazebo. Native tests
are executed only when their complete runtime is available. Missing runtime
pieces are reported as `NOT_EXECUTED_EXTERNAL_DEPENDENCY`; a generated world or
message definition is never represented as a successful native simulation.

## Current implementation baseline

The audit on 2026-07-29 found the following reusable components:

- deterministic planar, four-wheel and dynamic four-wheel plants;
- steering, brake, traction, solid-axle-related wheel state, electrical and
  thermal behavior;
- VCU state machine, command expiry, dynamic safety envelope, controlled stop,
  independent watchdog/E-stop, latched faults and reset guards;
- scenario fault campaigns, JSONL telemetry, replay and engineering dashboard;
- track projection, boundary, surface and corner segmentation;
- Pure Pursuit/Stanley controller contracts and deterministic ROS mock clock,
  QoS and TF primitives;
- backend gates for internal and unavailable Chrono integration.

The previous top-level `docs/ARCHITECTURE.md` describes an earlier phase and is
superseded for platform-level decisions by this document.

