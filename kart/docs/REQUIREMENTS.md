# Requirements Baseline

**Safety notice:** all configurations, models, and outputs in this repository are for simulation only and must not be used to command a real vehicle.

## Implemented baseline requirements

The first executable baseline implements a deterministic headless simulator with SI units, a seeded configuration field, fixed physics timestep, validated YAML configuration, structured JSONL telemetry, and a planar bicycle vehicle model. It is deliberately independent of rendering, ROS 2, Project Chrono, CARLA, and hardware.

Required Phase 2 physics equations are implemented in body coordinates:

`m(du/dt - r v) = ΣFx`, `m(dv/dt + r u) = ΣFy`, `Iz dr/dt = ΣMz`.

The model includes bounded linear lateral tire forces, axle-static vertical load used for a friction cap, rolling resistance, aerodynamic drag, grade force, basic motor force, basic braking force, and semi-implicit Euler propagation. It is not a four-wheel, brake-chain, solid-axle, or validated real-kart model.

## Traceability and gates

| Requirement | Current evidence | Status |
| --- | --- | --- |
| Fixed timestep independent of rendering | `FixedStepScheduler` unit test | implemented |
| SI units and one frame convention | coordinate document and typed field names | implemented |
| Reject invalid configuration | Pydantic validation tests | implemented |
| Stationary/constant velocity/force/corner response | physics tests | implemented baseline |
| Four independent wheels and solid axle | Phase 3 | pending |
| Dynamic steering/braking/electrical models | Phase 4 | pending |
| Track, surface and corners | Phase 5 | pending |
| Sensor faults and timing | Phase 6 | pending |
| VCU and independent safety abstraction | Phase 7 | pending |

## Acceptance rule

Passing a software test establishes only the stated numerical or interface property. It does not validate a physical kart, prove safety, establish ISO compliance, or establish digital-twin validity.
