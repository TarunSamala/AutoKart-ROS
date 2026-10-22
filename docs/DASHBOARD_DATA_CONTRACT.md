# AutoKart Mission Control data contract (RC-05/RC-07, 2026-10-16)

The existing `phase_sim` Mission Control is a **read-only diagnostic display**. Its default operating source is `REAL_KART` in `LIVE` mode. `SIMULATION`, `LAB`, and `REPLAY` are explicit modes; samples from one source cannot populate another. The browser publishes no vehicle commands. `vehicle_authorized=false` is a release condition, not a browser control.

## Sources and alignment

| View | Producer | Required evidence | Missing/stale behavior |
|---|---|---|---|
| RGB/YOLO and depth | Existing HP60C/perception ROS publisher through `autokart_dashboard_gateway.camera_feed` | RGB and depth share capture timestamp, image frame and dimensions; perception status has the same nanosecond stamp | No image when pair is absent or older than 1 s; show reason, not a substitute frame |
| Observed bird's-eye geometry | ROS mapping `MissionDebug` in `odom` | matching `source`, valid status, `odom` frame, unexpired measurement; published observed cones, edges, gates, centerline and boundaries | Clear the world geometry when status expires or frame/source disagrees |
| Planned path / mission | ROS planner `MissionDebug` and matched path topic | valid planner status in `odom`, matching path measurement time | Clear selected path and mission target when invalid/stale |
| Controller lookahead | ROS control diagnostic | current selected path and target within 0.75 m of that path | Show `NO CURRENT TARGET`; never keep a ghost lookahead |
| Pose / heading / speed | `/autokart/odom` | actual odometry sample | Show `WAITING FOR ODOMETRY`; no world frame is made from diagnostics alone |
| VCU / safety | ROS feedback/status topics | current published observation | Display `UNKNOWN` or `UNAVAILABLE` rather than inferred health |

The gateway serves `/mission/ws`, `/mission/api/manifest`, `/mission/api/replay`, `/mission/api/status`, `/mission/api/camera`, and `/mission/api/camera/{frame_id}/{rgb|depth}`. It subscribes to ROS and does not open a second HP60C acquisition process. Camera status includes source, capture time, frame age, frame ID, resolution, received FPS, missing sequence count, processing time, capture-to-gateway delay, model hash and detection metadata when ROS publishes them. LIVE camera and error diagnostics use the HP60C monotonic capture clock. RAW16 depth is rendered as a preview with `depth_qualified=false`; no scale, registration or metric 3D is inferred from the image. Detection boxes/classes/confidence may display while depth remains unqualified. Candidate distances in per-detection metadata are not shown as metres unless depth and that detection's validation are qualified.

The viewport draws published observed geometry and path in `odom`, using the same frame and freshness checks as the status panel. LAB draws its own deterministic synthetic course and executed trail. The mission card separately identifies active segment, selected waypoint index, mission winding and target progress when available. REPLAY is labeled as recorded source data; it never becomes LIVE. The legacy `phase_sim/src/live/LiveMissionControl.tsx` and shared-core LAB remain separate existing views and do not supply fallback telemetry to Mission Control.

The browser was checked against the actual local gateway in simulated ROS mode, in REAL mode without hardware, and later with an attached HP60C feeding paired LIVE RGB/depth and ROS perception status through the existing pipeline. The real stream remains unqualified, and no odometry or VCU status was present. These checks do not qualify HP60C calibration, latency on the target Pi, VCU telemetry or physical operation. Evidence and remaining limitations are in `RC05_RC07_VALIDATION_REPORT.md`.
