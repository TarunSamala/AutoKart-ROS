# autokart_perception_adapter — interface contract

## Topics

| Direction | Topic | Type | QoS | Notes |
|---|---|---|---|---|
| sub (topics mode) | `/hp60c/rgb/image_raw` (param `rgb_topic`) | `sensor_msgs/Image` (`rgb8`) | sensor data | default points at this library's node 01 |
| sub (topics mode) | `/hp60c/depth/image_raw` (param `depth_topic`) | `sensor_msgs/Image` (`16UC1`) | sensor data | |
| sub (topics mode) | `/hp60c/rgb/camera_info` (param `camera_info_topic`) | `sensor_msgs/CameraInfo` | sensor data | intrinsics only; distortion ignored (node 01 publishes a placeholder `d`) |
| pub | `/perception/cones` | `autokart_msgs/ConeArray` | sensor data | **only published when the result is qualified** (`valid=true`); never a diagnostic/unqualified cone |
| pub | `/perception/status` | `autokart_msgs/SensorDeliveryMetadata` | reliable, depth 10 | one per processed frame or fault, always published |

Three RGB/depth/CameraInfo subscriptions are time-synchronized with
`message_filters.ApproximateTimeSynchronizer` (`slop=0.05s`, `queue_size=5`) — a frame is only
processed once all three arrive within that window.

## Input modes (`input_mode` parameter)

- `topics` (default): subscribes as above. Requires an upstream publisher (e.g. `autokart_hp60c`).
- `standalone_timer`: ignores the topics entirely and ticks a timer (`standalone_period_s`) feeding
  dummy zero-filled imagery into the backend. This exists so the node is fully buildable, testable
  and launchable **with no other node running** — exactly the same portability bar as
  `autokart_hp60c`. With `backend: synthetic`, the dummy imagery is irrelevant (the synthetic backend
  ignores pixel content and returns a fixed cone layout); this is the mode this node's own test suite
  and `config/synthetic.yaml` use.

## Backends (`backend` parameter)

- `synthetic` (default): zero external dependency, deterministic 4-cone fixed layout
  (`(2,1.5,blue), (2,-1.5,yellow), (5,1.5,blue), (5,-1.5,yellow)`), always `valid=true`,
  `ConeArray.ground_truth=true`. Never claims LIVE provenance.
- `algorithm`: reuses the existing, already-tested `autokart_perception.{application,pipeline,detector}`
  algorithm package (YOLO26s ONNX + depth fusion + the full safety/qualification gating) — see
  DEPENDENCIES.md. Requires `algorithm_package_path`; `model_path`/`model_manifest` are optional (if
  omitted, the underlying `Pipeline` runs in ITS OWN synthetic-geometry mode, not this node's
  dummy-image mode — see the existing package's own `synthetic_cones()`).

## Qualification gating (what makes a cone actually get published)

A result's cones are published only if `valid=true`. For the `algorithm` backend with
`source_label=LIVE` (the default-live config), this additionally and explicitly requires
`diagnostics["calibration_status"] == "MOUNT_VERIFIED"` before any cone counts as metric-qualified —
mirroring the gating already proven in the existing (not-yet-decoupled) ROS perception node in
`kart/ros2_ws`. An unqualified result still publishes `/perception/status` with
`quality=DIAGNOSTIC_ONLY` and the specific reason codes, but **no** `ConeArray` message at all (not
an empty one, not a flagged one — none), so a consumer that only looks at `/perception/cones` without
checking freshness can never be fooled by a stale-but-present message; it simply sees nothing new.

## Deferred (not implemented this session — see DEVELOPER_REPORT.md)

`/perception/detections` (raw, pre-qualification boxes) and `/perception/obstacles` (arbitrary 3D
footprint obstacles, not just cones) from the master architecture doc's node 02 row are **not**
published by this node. Only `/perception/cones` and `/perception/status` are implemented and
tested. This is a scope reduction made explicitly, not a silent gap — see CONTRACT "Deferred" note
duplicated in the developer report.

## Frames

Cones are published in `frame_id = "base_link"` per `autokart_msgs/ConeArray`'s existing convention
(matches the live production node). This node does not publish any TF.

## What this node is not

It does not open a camera (consumes node 01's topics or a dummy standalone feed). It does not do
cone-track association, Delaunay/Gabriel graph construction, gates, or centerline generation — those
are node 06 (`autokart_mapping`), built separately in this library. It does not decide vehicle motion
— `valid`/`quality` are diagnostic inputs for downstream nodes (mapping, safety), not an authorization.
