TASK:
Real kart and track engineering dashboard rebuild

STATUS:
COMPLETE

VERDICT:
APPROVED_WITH_CONDITIONS

BASELINE:
- Previous dashboard: 407-byte text-only static HTML; no HTTP application,
  WebSocket, 3D scene, panels, replay UI, or track payload.
- Previous tests: 98 passed, 0 failed, 0 skipped.

IMPLEMENTED:
- Frontend architecture: Modular HTML/CSS/ES modules with a bounded client
  state store and no framework/CDN dependency.
- 3D engine: Local perspective-projection engineering renderer using ENU world
  coordinates and Canvas 2D.
- Track renderer: Phase 5 centreline, independent boundaries, elevation,
  curvature, grade, bank, material regions, wet patches, start line, corners,
  apexes, trajectory, and fault markers.
- Kart renderer: Configuration-scaled procedural chassis, four steer/rotation
  wheels, axle, seat, steering, battery, motor, VCU/compute/sensor/E-stop
  elements; explicit placeholder label and GLB replacement path.
- Live telemetry: Pacing-capable simulator launch, incremental JSONL tail,
  bounded 64-message drop-oldest WebSocket, reconnect and stale indication.
- Replay: Play/pause/step/seek/restart/loop, 0.25–10x speed, shared cursor,
  measured/display timestamp distinction, fault jump, and static bundle.
- Camera modes: Orbit, chase, top, kart-front, driver-eye, corner, and free.
- Vehicle panel: Pose/motion/track and four-stage command chain.
- Steering panel: Requested/validated/shaft/linkage/left/right/error/current.
- Brake panel: Command → travel → cable force → pressure → torque → measured
  deceleration.
- Traction panel: Requested/actual torque, current, speed, chain torque,
  enable/contactor and temperatures.
- Tire panel: Four wheel selection, load/forces/slip/utilization/contact,
  material/friction and friction circle.
- Electrical/thermal panel: Battery/DC bus/inverter/motor flow, SOC,
  voltage/current, power domains, contactor and temperatures.
- Sensor panel: Explicit unavailable/degraded state and disabled debug-ground-
  truth indication for runs without Phase 6 sensor messages.
- Safety panel: Persistent state/independent/E-stop/fault visibility, command
  age, enable/contactor, limit candidates and stopping-distance composition.
- Fault timeline: State/fault markers synchronized with replay and seek.
- Corner analysis: Active Phase 5 segment, phase, curvature, speed, lateral
  acceleration, steering error and rear-unload evidence.
- Comparison mode: A second sibling run is aligned by track distance and drawn
  as a second trajectory with speed deltas from the API.

DATA SOURCES:
- Track: `configs/tracks/research_circle.yaml` through `kart_sim.track.Track`;
  simulation-only provisional geometry, not survey data.
- Kart configuration: `configs/karts/prototype_dynamic.yaml`.
- Vehicle pose: Authoritative `PlanarKartState`; surface-frame z/roll/pitch
  from the selected track because the physics backend remains planar.
- Actuator feedback: Dynamic steering, brake, BLDC drivetrain, battery, power
  domains and four-wheel tire telemetry.
- Safety: VCU output, independent layer, controlled-stop state, limit
  candidates, stopping distance and command age.
- Faults: Physical subsystem injection trace plus observable VCU DTCs.

PLACEHOLDERS:
- Procedural kart enclosures/components pending CAD-derived `kart.glb`.
- Research circle is an engineering test track, not a surveyed real facility.
- Sensor thumbnails remain unavailable when the run has no Phase 6 payloads.

TEST RESULTS:
- Unit: Dashboard parsing/schema/bounds/decimation/frontend-store tests PASS.
- Integration: Track-backed constant-radius and scheduled slalom movement PASS.
- Visual: Browser assets/API/scene inputs and live changing-pose stream PASS;
  automated screenshot comparison NOT_EXECUTED because Firefox SWGL could not
  map a headless framebuffer. Interactive localhost rendering remains the
  verification path.
- Safety: No mutation routes; POST returns 405 with actuator authority NONE;
  stale/invalid/fault/emergency handling tests PASS.
- Regression: 107 passed, 0 failed, 0 skipped.

PERFORMANCE:
- FPS: Runtime FPS counter implemented; automated headless FPS capture not
  available in this framebuffer environment.
- Telemetry latency: Replay WebSocket emits at configured 30 Hz; measurement
  timestamp is preserved separately from display time.
- Memory: 6,000-record normalized replay measured 136.97 MiB current and
  191.66 MiB peak in Python; storage is bounded to 20,000 records.
- CPU: 6,000-record normalization/load measured 1.819 s; a deterministic
  1,200-point query averaged 1.723 ms.
- Real-time factor: Paced live mode is 1x; replay supports 0.25–10x.

FILES CREATED:
- `dashboard/index.html`
- `dashboard/css/dashboard.css`
- `dashboard/js/app.js`, state, transport, replay, scene, renderer, panel and
  plotting modules
- `dashboard/assets/kart/README.md`
- `kart_sim/dashboard_server.py`
- `configs/scenarios/dashboard_research_lap.yaml`
- `configs/scenarios/dashboard_slalom.yaml`
- `scripts/generate_dashboard_runs.sh`
- Dashboard rebuild unit/frontend/integration tests
- Dashboard audit, architecture, scene, track, kart, live, replay, safety, test
  and user documentation

FILES MODIFIED:
- `kart_sim/dashboard.py`
- `kart_sim/scenario.py`
- `kart_sim/types.py`
- `kart_sim/vcu.py`
- `kart_sim/cli.py`
- `pyproject.toml`
- Phase/status/change documentation

KNOWN LIMITATIONS:
- Analytical vehicle dynamics are planar; displayed surface-frame z/roll/pitch
  do not claim 6-DOF suspension dynamics.
- Track is provisional simulation geometry.
- Procedural kart is not real CAD.
- Phase 6 image/LiDAR/sensor packets are absent from the demonstration log.
- No native ROS feed or automated pixel-diff browser test was executed.
- Long 20,000-record full-detail replays can consume several hundred MiB;
  server/client decimation should be selected for longer runs.

EXTERNAL ASSETS REQUIRED:
- Real kart CAD/GLB: Required for geometric visual fidelity.
- Real track survey: RTK/total-station/LiDAR/photogrammetry data required.
- Texture assets: Optional; engineering rendering does not depend on them.
- Camera calibration: Required before showing calibrated real-camera overlays.

REAL-VEHICLE AUTHORITY:
NONE

NEXT ENGINEERING TASK:
Acquire a surveyed track and CAD-derived kart GLB, then verify their shared
metric coordinate frame against measured landmarks.

