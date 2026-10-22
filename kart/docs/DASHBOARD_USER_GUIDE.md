# Dashboard user guide

## Quick start

Run the prepared replay:

```bash
PYTHONPATH=. python3 -m kart_sim.cli run \
  configs/scenarios/dashboard_research_lap.yaml \
  --output-dir runs
PYTHONPATH=. python3 -m kart_sim.cli dashboard serve \
  runs/dashboard_research_lap/telemetry.jsonl \
  --track configs/tracks/research_circle.yaml \
  --kart configs/karts/prototype_dynamic.yaml \
  --host 127.0.0.1 --port 8000
```

Open `http://localhost:8000/dashboard.html`.

For a paced live run, use `dashboard launch` as documented in
`DASHBOARD_LIVE_MODE.md`.

## Controls

- Drag/zoom the scene or select one of seven camera modes.
- Use Overlays to enable forces, velocity, stopping envelope, CG, and faults.
- Select Vehicle, Safety, Actuators, Tires, Power, Sensors, or Corner.
- Use the lower transport bar for replay, step, speed, seek, loop, and faults.
- Click a plot/timeline position to move the shared replay cursor.
- Export the selected plot to PNG or normalized data to CSV.
- Compare Run asks for another sibling run-directory name and aligns by `s`.

The UI is analysis-only. It cannot release E-stop, reset faults, or command the
kart.

