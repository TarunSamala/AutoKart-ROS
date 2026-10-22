# Scenario Editor

The scenario editor is schema-driven and reproducible rather than graphical.
`kart_sim.studio_scenario.StudioScenario` covers track, kart, start/grid,
target speed, controller, sensor suite, surface/weather, obstacles, timed
faults, agents, session rules and logging.

Validation resolves every referenced file, validates the preset and track,
requires VCU and independent safety, rejects real actuator output and unsafe
multi-kart grid spacing, and returns a SHA-256 of normalized content.

Example:

```bash
python3 -m kart_sim.cli studio validate-scenario \
  configs/studio/cone_track_demo.yaml
```

The output records scenario hash, preset, selected physics authority and track
source hash. Pydantic rejects invalid numeric bounds and unsupported fields are
preserved only when explicitly added to the schema.

This backend is suitable for a future web form because validation and file
generation remain server-side and deterministic. Launch controls must call the
normal simulator CLI/ROS launch; an editor must never expose actuator methods.

