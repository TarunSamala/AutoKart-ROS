# Dashboard live mode

Start a paced simulator plus dashboard:

```bash
PYTHONPATH=. python3 -m kart_sim.cli dashboard launch \
  configs/scenarios/dashboard_research_lap.yaml \
  --host 127.0.0.1 --port 8000 --rate-hz 30
```

Open `http://localhost:8000/dashboard.html`.

The simulator writes complete JSONL measurements on a dedicated thread. The
server tails them without holding a physics object or command interface.
Per-client WebSocket queues are fixed at 64 messages and drop the oldest
display-bound item under backpressure. Vehicle display rate is configurable
from 1–120 Hz; physics can remain 1000 Hz.

Disconnect changes the top status and activates a stale-data overlay. Rendering
freezes at the last valid sample. Reconnect uses bounded exponential backoff.
No false “live” indication persists after socket loss.

