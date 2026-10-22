# Dashboard architecture

The implemented boundary is:

```text
dynamic simulator or immutable replay
  -> normalized DashboardService
  -> bounded HTTP/WebSocket transport
  -> browser DashboardStateStore
  -> scene, panels, plots, replay, and comparison views
```

`kart_sim.dashboard` owns normalization, bounded replay, Phase 5 track export,
kart-dimension export, CSV/JSON output, and offline bundles.
`kart_sim.dashboard_server` owns the optional `aiohttp` GET-only service.
`dashboard/js` contains browser modules. Physics and the VCU never import the
dashboard.

## Safety boundary

There is no route, message, or object reference for steering, braking, motor
torque, VCU transition, fault reset, or E-stop release. HTTP methods other than
GET/HEAD receive status 405. The WebSocket sends telemetry only. Replay controls
alter the browser time cursor, never simulation state.

Every page displays:

```text
ENGINEERING VISUALIZATION ONLY
NO ACTUATOR AUTHORITY
```

## Data modes

- `serve`: immutable replay plus a 30 Hz measured-time WebSocket.
- `launch`: real-time paced simulator thread writing complete JSONL records,
  tailed by the service with bounded queues.
- `export`: modular static UI and immutable `data/*.json` for a generic local
  HTTP server.

The service retains old normalized data if a live writer is between JSONL
records, and retries without exposing malformed data.

