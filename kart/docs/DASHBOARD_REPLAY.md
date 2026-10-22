# Dashboard replay

Serve an existing run:

```bash
PYTHONPATH=. python3 -m kart_sim.cli dashboard serve \
  runs/dashboard_research_lap/telemetry.jsonl \
  --track configs/tracks/research_circle.yaml \
  --kart configs/karts/prototype_dynamic.yaml \
  --port 8000
```

Replay supports play, pause, previous/next measurement, 0.25–10x speed, seek,
restart, loop, jump-to-fault, synchronized plots, and timeline selection.
Interpolation occurs only between adjacent valid samples and is tagged as
display interpolation. Fault/state timing and original measurement timestamps
are preserved. Random sensor data is never regenerated.

Two-run comparison accepts another run directory beneath the current runs root
and aligns speed and trajectory by Phase 5 track distance `s`.

Static bundles:

```bash
PYTHONPATH=. python3 -m kart_sim.cli dashboard export \
  runs/dashboard_research_lap/telemetry.jsonl \
  --track configs/tracks/research_circle.yaml \
  --kart configs/karts/prototype_dynamic.yaml \
  --output-dir dashboard-export
cd dashboard-export && python3 -m http.server 8000
```

