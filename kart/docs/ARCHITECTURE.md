# Architecture

The project is layered so the deterministic headless core never depends on rendering or ROS wall-clock execution.

```text
CLI / scenarios
       |
validated YAML configuration --> fixed-step scheduler --> internal analytical vehicle backend --> telemetry
                                                        |
                             future common backend interface: Chrono adapter

future ROS 2 / CAN / HIL <--- VCU + safety layer <--- autonomy commands
future CARLA/Unreal rendering <--- sensor adapters <--- ground-truth environment (debug isolated)
```

## Present module boundaries

- `kart_sim.config`: YAML boundary and Pydantic validation; no hidden physical constants.
- `kart_sim.scheduler`: fixed-step timing only.
- `kart_sim.vehicle`: Phase 2 analytical dynamics only.
- `kart_sim.scenario`: deterministic sequencing and JSONL telemetry.
- `kart_sim.cli`: thin user boundary.

Future backends must implement a common vehicle state/command contract and must not let rendering cadence advance physics. Ground truth and simulated sensor streams will be separate interfaces. Commands will become VCU-validated rather than flowing directly to the vehicle once Phase 7 begins.
