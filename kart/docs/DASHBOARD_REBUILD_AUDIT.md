# Dashboard rebuild audit

Date: 2026-07-29

## Baseline

- Full repository baseline: `98 passed` using `./scripts/test.sh`.
- Previous browser output: a 407-byte static HTML document containing only
  sample count, connection text, final simulation time, VCU state, and faults.
- Previous architecture: `TelemetryStore -> DashboardExporter`; no HTTP
  application, telemetry endpoint, WebSocket, scene renderer, client state
  store, replay controls, or live-tail adapter existed.
- The prior localhost process was a generic `python3 -m http.server` serving an
  exported snapshot. It was not a simulator/dashboard server.

## Available telemetry before rebuild

- Authoritative planar state: time, world `x/y`, yaw, body longitudinal/lateral
  speed, yaw rate, and longitudinal/lateral acceleration.
- Four wheel state: steering angle, angular speed, normal load, slip ratio,
  slip angle, longitudinal/lateral force, utilization, and contact validity.
- Steering feedback: target, shaft/linkage/left/right road-wheel angles,
  velocity, current, supply, following error, temperature, and fault.
- Brake feedback: command/travel, current, cable tension, hydraulic pressure,
  brake torque, temperatures, missed steps, and fault.
- Motor/battery/power-domain feedback and VCU final command, state, active
  faults, controlled-stop phase, independent safety status, and speed limit.

## Missing or incomplete telemetry

- The dynamics backend is planar. Roll, pitch, and height were not explicit
  state channels. They are now explicit and are populated from the selected
  Phase 5 surface frame; no suspension movement is invented.
- Per-sensor Phase 6 messages, camera/depth images, LiDAR points, localization
  covariance, perception obstacles/free space, lap timing, and full corner
  telemetry are absent from current scenario logs. The UI marks these signals
  unavailable instead of fabricating them.
- Requested/validated/safety-limited/final command separation was absent.
  Scenario records now preserve the four command stages.
- VCU speed-limit candidates and stopping-distance components were absent.
  Current track-backed scenarios record the available analytical candidates.

## Track and pose sources

- Track geometry: `kart_sim.track.Track` loaded from
  `configs/tracks/*.yaml`; normalized samples include centreline, independent
  boundaries, grade, bank, curvature, surface material, regions, and automatic
  corners.
- Vehicle pose: authoritative simulation `PlanarKartState`.
- Surface-frame `z/roll/pitch`: selected Phase 5 track sample at the measured
  projection. This does not claim a 6-DOF chassis dynamics model.
- Wheel surfaces: four measured wheel contact locations projected independently
  into the track, with the resulting material/friction also supplied to tire
  calculations in track-backed scenarios.

## Replay and refresh

- Original physics logs are commonly 1000 Hz.
- Browser API replay is deterministically bounded to 20,000 points.
- WebSocket display streaming defaults to 30 Hz with a 64-message drop-oldest
  queue.
- `dashboard launch` runs a paced scenario and tails complete JSONL records.
- Replay interpolation is display-only, bounded between adjacent valid
  measurements, and identifies measured versus displayed time.

## Performance constraints

- Browser rendering uses a lightweight local perspective-projection engine and
  Canvas 2D; there is no external CDN or GPU dependency.
- Track geometry is sampled once, telemetry is bounded, plots reuse canvases,
  and heavy camera/depth assets are not loaded.
- The dashboard has no simulator reference and exposes only GET/HEAD routes.
  Mutation attempts return HTTP 405.

