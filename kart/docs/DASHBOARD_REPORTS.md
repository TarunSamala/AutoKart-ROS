# Dashboard reports

The export command now creates the complete modular UI plus immutable normalized
data, CSV, and raw JSON:

```bash
PYTHONPATH=. python3 -m kart_sim.cli dashboard export \
  run/telemetry.jsonl \
  --track configs/tracks/research_circle.yaml \
  --kart configs/karts/prototype_dynamic.yaml \
  --output-dir dashboard-export
cd dashboard-export
python3 -m http.server 8000
```

Open `http://localhost:8000/dashboard.html`. The exported bundle is replay-only;
it contains no mutation or simulator-control endpoint.

