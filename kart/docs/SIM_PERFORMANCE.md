# Simulation Performance

Run measurements with:

```bash
python3 -m kart_sim.cli performance --output-dir runs/performance
```

2026-07-29 local result:

| Tier | Status | Update rate | Real-time factor | Peak RSS | GPU |
|---|---|---:|---:|---:|---|
| 0 planar, 5000 steps | executed | 116,022 Hz | 116.02× | 45.3 MiB | headless |
| 1 VCU + dynamic plant + 108-beam LiDAR | executed | 120.2 control Hz | 2.40× | 45.8 MiB | headless |
| 2 dynamic controlled-stop scenario | executed | 2,522 physics Hz | 2.52× | 60.5 MiB | headless |
| 3 Gazebo robotics | external measurement incomplete | — | — | — | not measured |
| 4 Chrono/offline | dependency unavailable | — | — | — | not measured |

CPU utilization was approximately one saturated core for all executed
workloads. These are single-run process measurements, not portable guarantees.
Tier 1 reports control steps; Tier 0/2 report physics steps. The YAML artifact
contains wall time, process CPU time, CPU/wall percentage, RSS, update rate and
real-time factor.

Native Gazebo world/spawn/topic smoke passed, but no stable payload/GPU benchmark
was collected, so Tier 3 is not reported as executed performance. Chrono Python
bindings were not found.

