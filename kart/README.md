# AutoKart Sim Studio

**SIMULATION USE ONLY — NOT APPROVED FOR DIRECT VEHICLE DEPLOYMENT.**

This repository contains the deterministic, headless foundation of a configurable autonomous electric go-kart simulator. It intentionally starts with a traceable planar model, configuration validation, fixed-step execution, and safety-oriented engineering records before any rendering or high-fidelity external backend.

It now also contains an original multi-mode autonomous-racing platform
foundation: VCU-routed fast racing/RL APIs, Formula Student-style cone events,
multi-kart race control, one-source track conversion, a complete kart
Xacro/URDF, Gazebo worlds/sensors, ROS 2 packages/messages/launches and RViz
configurations. See [the platform architecture](docs/SIM_PLATFORM_ARCHITECTURE.md)
and [current upgrade report](docs/phase_reports/AUTOKART_SIM_STUDIO_UPGRADE_REPORT.md).

## Quick start

```bash
./scripts/setup.sh
./scripts/test.sh
kart-sim validate configs/karts/prototype.yaml
kart-sim run configs/scenarios/straight_acceleration.yaml
```

Launch the live read-only engineering dashboard:

```bash
PYTHONPATH=. python3 -m kart_sim.cli dashboard launch \
  configs/scenarios/dashboard_research_lap.yaml \
  --host 127.0.0.1 --port 8000
```

Then open `http://localhost:8000/dashboard.html`. The dashboard has no actuator
authority.

The phase status and implemented scope are maintained in [docs/STATUS.md](docs/STATUS.md). See [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) for boundaries and [docs/ASSUMPTIONS.md](docs/ASSUMPTIONS.md) for every provisional value.

The new read-only ROS 2 Mission Control transport is documented in
[docs/mission_control_debug.md](docs/mission_control_debug.md). It streams
timestamped odometry, cones, path, command, actuator, VCU, and optional
algorithm debug messages to the React/Three.js frontend in `../phase_sim`.
