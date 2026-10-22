# Interface contracts: HP60C perception service and Dashboard status

Owner: Camera/SDK + Perception Backend. Last verified: 2026-10-18.
Implementation: `autonomy_stack_phase1/phase_01_perception/src/autokart_perception/live_mission.py`.
Tests: `phase_01_perception/tests/test_pipeline_health.py`, `tests/test_live_mission.py`.

Items marked **NOT TESTED** were not exercised against an HP60C on the vehicle.

## 1. Service `autokart_perception.live_mission`

```
python3 -m autokart_perception.live_mission --mode {live,replay,sim} [--camera hp60c]
    [--replay DIR] [--detector conenet-v1|conenet-v2|yolo26s|coneslayer]
    [--host 127.0.0.1] [--port 8768] [--token-file FILE]
```

- This is the only process allowed to open the HP60C vendor SDK (`camera.HP60C` over `libhp60c_bridge.so`). Only `--mode live` opens it. `sim` and `replay` never load or start the SDK.
- The service is read-only. Every `POST` returns `405 {"error":"READ_ONLY_DIAGNOSTIC"}`. It has no actuator transport. `vehicle_authorized` is always `false`.
- Binding to anything other than localhost requires `--token-file` (24 characters or more).
- **Auth:** when a token is configured, `/health`, `/ready` and every `/api/*` path (including `/api/v1/*`) need `Authorization: Bearer <token>`. Otherwise the response is `401 {"error":"AUTH_REQUIRED"}`. Static `/live` and `/assets/*` need no token.
- All JSON is encoded with `allow_nan=False`. Values that are not finite are sent as `null`. Every response carries `Cache-Control: no-store`.
- Unknown paths return `404 {"error":"NOT_FOUND"}`.

### Clocks and units

| Field suffix | Meaning |
|---|---|
| `*_ns` (stamps) | Host `CLOCK_MONOTONIC` ns for LIVE (SDK arrival stamp) and SIMULATED. REPLAY stamps are the recorded clock. Exception: `service.started_at_ns` is wall-clock `time.time_ns()`. |
| `*_age_ms` | Milliseconds, measured on host monotonic time when the response is built |
| `*_fps` | Frames per second |
| `*_s` | Seconds |
| `*_m`, `*_xyz_m` | Metres |

### Routes

| Route | Code | Body |
|---|---|---|
| `GET /health` | 200 | `{"server":"RUNNING","source":S,"vehicle_authorized":false}`. This shows the process is alive, not that the camera is healthy. |
| `GET /ready` | 200 / 503 | `{"ready":bool,"status":...,"vehicle_authorized":false}`. Currently always 503 because metric depth, the mount and pose are unqualified. |
| `GET /api/snapshot` | 200 | Latest perception snapshot (section 1.2). |
| `GET /api/image/<frame_id>/<rgb\|depth>` | 200 JPEG / 404 | The JPEG belongs to the same frame as the snapshot `frame_id`. Only the last 32 frames are kept; older ones return `404 {"error":"SNAPSHOT_EXPIRED"}`. |
| `GET /api/v1/diagnostics/pipeline` | 200 | Per-hop pipeline health (section 1.1). Always 200 while the server is up. Health is reported in the body, not the status code. |
| `GET /`, `/live`, `/assets/*` | 302 / 200 | Static UI from `--static-root`. |

`S` (source) is one of `LIVE`, `REPLAY` or `SIMULATED`. It is set once from `--mode` and never changes. In live mode the service never produces synthetic frames. If no camera frame arrives, the snapshot has `frame_id: null` and `status` is `SENSOR_FAULT` or `STALE`.

### 1.1 `GET /api/v1/diagnostics/pipeline` (schema_version 1)

```json
{"schema_version":1,
 "service":{"name":"autokart_perception.live_mission","mode":"live|replay|sim",
            "source":"LIVE|REPLAY|SIMULATED","pid":int,"started_at_ns":int,"uptime_s":float,
            "camera_owner":bool,"detector":str|null,"model_sha256":str|null},
 "hops":[{sdk_capture},{detector},{geometry},{http}],
 "overall":"<state>","vehicle_authorized":false}
```

- `camera_owner` is `true` only while this process holds an open HP60C SDK session (`mode==live`, the source is `HP60C`, and `hp_start` succeeded). It is always `false` in sim and replay. In live mode it is `false` while the SDK cannot start, and it flickers during the 1 s reconnect back-off.
- `hops` always appears in the order shown. Every hop has `name`, `state` and `reason` (a code string or `null`).
- `overall` is the worst state across hops, ignoring `NOT_APPLICABLE`. Severity order: `SDK_ERROR` > `NO_FRAME` > `STALE` > `DEGRADED` > `CALIBRATION_UNVERIFIED` > `STARTING` > `OK`.
- Poll at 1 to 2 Hz. Use a 1 s client timeout. If the request fails, the client should show the service as down.

**State vocabulary (exact):**

| State | Meaning |
|---|---|
| `OK` | Hop is producing current output |
| `NO_FRAME` | Hop has no input or output (never received, upstream absent, or worker exited) |
| `STALE` | Last output is older than `stale_after_ms` |
| `SDK_ERROR` | Live capture: the most recent capture attempt raised an error after the last good frame |
| `CALIBRATION_UNVERIFIED` | Output exists, but it depends on an unverified mount, depth scale or intrinsics |
| `DEGRADED` | Output exists, but with a fault (inference failure, frozen or out-of-order frame, invalid calibration, RGB-D not streaming) |
| `NOT_APPLICABLE` | Hop not used in this mode |
| `STARTING` | Within 5 s of start (`STARTUP_GRACE_S`) and no output yet |

`stale_after_ms` is the config `max_frame_age_ms` (150 ms by default) in LIVE. In SIMULATED and REPLAY it is `max(max_frame_age_ms, 1000)`, because the frame owner paces those modes at about 10 Hz.

#### `sdk_capture`

Covers frames that the source (HP60C SDK, Synthetic or Replay) hands to the frame owner.

| Field | Unit / meaning |
|---|---|
| `source`, `uses_vendor_sdk` | Same as `service.source`. `uses_vendor_sdk` is true only in live mode. |
| `last_error` | CameraError code: the text before the first `:` (for example `CAMERA_DISCONNECTED`, `FRAME_TIMEOUT`, `STALE_FRAME`, `RGB_DEPTH_FRAME_MISMATCH`, `TIMESTAMP_MISMATCH`, `CAMERA_TIMESTAMP_UNAVAILABLE`, `CALIBRATION_UNAVAILABLE`, `SDK_START_FAILED`, `SDK_NOT_PREPARED`, `SDK_ADAPTER_NOT_BUILT`, `CAMERA_OPEN_FAILED`, `FRAME_ERROR`, `UNSUPPORTED_FRAME_FORMAT`). A failure that is not a CameraError is reported as `SOURCE_OPEN_FAILED` or `SOURCE_READ_FAILED`. The field keeps its last value after recovery. |
| `last_error_detail` | Full text, `<ExceptionType>:<message>` |
| `last_error_age_ms`, `error_counts{code:n}`, `consecutive_errors` | `consecutive_errors` resets on the next good frame |
| `source_opens` | How many times the source was created. More than 1 means it reconnected. |
| `frames_received` | Frames returned by `source.read()` in this process |
| `last_frame_stamp_ns`, `last_frame_sequence` | From the last delivered `Frame` |
| `frame_age_ms`, `frame_age_basis` | `CAPTURE_STAMP_MONOTONIC` (LIVE, SIM) or `HOST_INGEST_MONOTONIC` (REPLAY) |
| `stale_after_ms` | Threshold used for `STALE` |
| `measured_fps`, `fps_window_s` | Frames delivered in the last 2 s, measured by this process. `null` before the first frame. |
| `sdk_callback_fps`, `sdk_received_frames` | Native bridge counters. LIVE only, otherwise `null`. |
| `drops.sdk_mailbox_overwrites` | Native mailbox overwrites (capacity 1). LIVE only. |
| `drops.processing_failures` | Frames received but not published (exception in processing) |
| `last_processing_error`, `worker_alive`, `worker_exit_reason` | Exit reason is `REPLAY_COMPLETE`, `FRAME_LIMIT_REACHED`, `STOPPED` or the fault text |

How the state is chosen, in live mode:
- If the current error is transient (`FRAME_TIMEOUT`, `CALIBRATION_UNAVAILABLE` or `STALE_FRAME`) and no frame has arrived yet within the grace period, the state is `STARTING`.
- `STALE_FRAME` gives `STALE`.
- Any other current error gives `SDK_ERROR`.
- If there is no error, the frame age decides between `OK` and `STALE`.
- `DEGRADED` means frames arrive but `camera_streaming` is false.

In sim and replay, an error gives `NO_FRAME`.

Live capture retries for as long as the process runs. On a transient code the open SDK session is kept and the read is retried after 0.1 s. On any other code the source is closed, the service waits 1 s, then reopens the SDK. The HTTP server stays up throughout.

#### `detector`

`detector`, `model_sha256`, `inference_count` (inferences that succeeded), `inference_failures`, `last_latency_ms` (the detector's own `latency_ms` for the last inference), `last_inference_age_ms`, `last_detection_count`, `last_error` (`INFERENCE_FAILURE:<msg>`), `last_error_age_ms`.

- In sim mode the state is `NOT_APPLICABLE` with reason `SIMULATED_GEOMETRY_NO_INFERENCE`. Synthetic cones go straight to the geometry stage.
- In replay without a detector it is `NOT_APPLICABLE` with reason `RECORDED_OBSERVATIONS_NO_INFERENCE`.
- In live mode without a detector it is `DEGRADED` with reason `MODEL_NOT_AVAILABLE`.
- The state is `DEGRADED` when the latest inference failed.

#### `geometry`

Covers metric localisation and qualification, plus Delaunay, Gabriel, gates and centerline.

| Field | Meaning |
|---|---|
| `calibration_status` | Mount status: `MOUNT_VERIFIED`, `MOUNT_CONFIGURED_UNVERIFIED`, `MOUNT_CALIBRATION_INVALID` or `MOUNT_CALIBRATION_UNAVAILABLE` |
| `calibration_applies` | `true` for LIVE, and for REPLAY of a LIVE recording. In those cases cones are localised through the camera mount. `false` for SIM and synthetic replay, where cones are given in base_link. |
| `processed_frames`, `qualified_frames` | `qualified_frames` counts frames with at least one qualified metric observation |
| `last_frame_id`, `last_processed_age_ms` | |
| `perception_status`, `snapshot_status`, `geometry_status`, `path_valid` | |
| `reason_codes` | Reason codes from the snapshot (for example `CALIBRATION_UNVERIFIED`, `DEPTH_SCALE_UNVERIFIED`, `INVALID_POSE`) |
| `counts` | `qualified_observations`, `candidate_observations`, `provisional_3d_observations`, `rejected_candidates`, `delaunay_edges`, `gabriel_edges`, `gates`, `centerline_points` (all from the last frame) |

Qualification rule, which the SDK enforces: LIVE cones are never qualified. `qualified_observations` is always 0 in LIVE until the RAW16 depth scale and the camera-to-base_link transform are surveyed. Their candidates appear only in `candidate_observations` and `provisional_3d_observations`. So in LIVE the hop reports `CALIBRATION_UNVERIFIED` whenever `calibration_status != MOUNT_VERIFIED`.

#### `http`

`snapshots_served`, `image_requests`, `image_not_found`, `diagnostics_served`, `unauthorized_requests`, `last_client_request_age_ms`. The last field is the age of the most recent `/api/snapshot` or `/api/image` request, which shows whether a Dashboard is polling. `state` is `OK` whenever this response is served.

### 1.2 `GET /api/snapshot` (schema_version 1, summary)

These are the existing fields; this change did not alter them.

- **Identity and source:** `frame_id` (null when there is a fault or the frame is stale), `source`, `capture_timestamp_ns`, `processed_timestamp_ns`.
- **Validity:** `status` is `VALID`, `DEGRADED`, `SENSOR_FAULT`, `STALE` or a perception status. `reasons[]`, `last_good_age_ms`, `frame_age_ms`, `camera_streaming`.
- **Geometry and path:**
  - `observations` holds qualified metric observations.
  - `candidate_observations` and `provisional_3d_observations` are diagnostic only. They are never fed to geometry.
  - `geometry` contains delaunay/gabriel edges, gates, centerline and boundaries.
  - `planning.path_valid`, and `control_candidate`, where `published_to_vcu` is always false.
- **Telemetry and provenance:** `stream_metrics`, `latencies_ms`, `model`, `calibration_status`, `depth_scale_qualified=false`, `ready=false`, `vehicle_authorized=false`.

When `status` is `SENSOR_FAULT` or `STALE`, all geometry, planning and observation arrays are empty. The service never shows old geometry as current.

### Verification status

| Item | Status |
|---|---|
| Endpoint shape, vocabulary, NaN-free output and auth in sim and replay | Tested (unit tests and a CLI instance on a spare port) |
| Live mode with a failing source: `SDK_ERROR`, server alive, reconnect, no synthetic source | Tested. Covered by an injected `CAMERA_DISCONNECTED` source, and by the real `HP60C` class with a missing vendor configuration (`SDK_NOT_PREPARED`) through the CLI. |
| Real unplug or hot-plug of the HP60C reporting `CAMERA_DISCONNECTED` through the vendor SDK | **NOT TESTED** |
| `camera_owner=true` and `measured_fps` on the device | Seen once on the bench: a device enumerates as USB `3482:6723`, and the vendor SDK streamed at about 12 fps. **NOT TESTED on the vehicle.** |
| `DEGRADED` caused by `camera_streaming=false` on hardware | **NOT TESTED** |

## 2. ROS gateway `GET /mission/api/status` (as consumed by the Dashboard)

Source: `kart/ros2_ws/src/autokart_dashboard_gateway/autokart_dashboard_gateway/core.py` (`live_status`) and `gateway.py` (route). The perception backend reads this endpoint and does not own it. Its schema is unversioned.

```json
{"source":"SIMULATION|REAL_KART","run_id":str,"scenario":str|null,
 "ros_connected":bool,
 "odometry":Channel,"vcu":Channel,"safety":Channel,"safety_envelope":Channel,
 "stages":{"perception":Channel,"mapping":Channel,"planning":Channel},
 "read_only":true,"vehicle_authorized":false,"mode":"LIVE|REPLAY"}

Channel = {"current":bool,"state":"CURRENT|STALE|MISSING","age_ms":float|null,"payload":object|null}
```

- A channel is `CURRENT` when the last ROS message arrived 1.0 s ago or less (`freshness_s`, measured on host monotonic time). It is `STALE` when a message arrived but is older than that, and `MISSING` when nothing has arrived. `payload` is present only while the channel is `CURRENT`.
- `ros_connected` is true when any of odometry, vcu, perception, mapping or planning is current. The safety channels do not count.
- `mode` is added by the route handler: `REPLAY` when the gateway runs in replay mode, otherwise `LIVE`.
- The gateway's `source` vocabulary (`SIMULATION`/`REAL_KART`) is different from this service's (`SIMULATED`/`LIVE`/`REPLAY`). The Dashboard must not compare the two directly.
- The gateway also serves `/mission/api/camera` and `/mission/api/camera/{frame_id}/{kind}`. When no camera is connected, `/mission/api/camera` returns `{"available":false,"reason":"CAMERA_NOT_CONNECTED"}`. Code reading shows that the gateway's `camera_feed.py` imports no `HP60C`/bridge code; it uses only `live_render` helpers.
- **NOT TESTED** by this team against a running ROS graph. The fields above come from reading the code.
