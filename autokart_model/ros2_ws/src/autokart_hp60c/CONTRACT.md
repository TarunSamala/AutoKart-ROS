# autokart_hp60c — interface contract

## Topics published

| Topic | Type | QoS | Rate |
|---|---|---|---|
| `/hp60c/rgb/image_raw` | `sensor_msgs/Image` (`rgb8`) | sensor data (best-effort, volatile) | observed camera FPS |
| `/hp60c/depth/image_raw` | `sensor_msgs/Image` (`16UC1`, raw SDK units) | sensor data | observed camera FPS |
| `/hp60c/rgb/camera_info` | `sensor_msgs/CameraInfo` | sensor data | one per RGB frame |
| `/hp60c/depth/camera_info` | `sensor_msgs/CameraInfo` | sensor data | one per depth frame |
| `/hp60c/status` | `autokart_msgs/SensorDeliveryMetadata` | reliable, depth 10 | one per capture attempt (success or fault) |

No point cloud is published by this node (optional, deferred — see LIMITATIONS.md in reports/).

## Depth units — read this before consuming `/hp60c/depth/image_raw`

Depth is published **raw**, exactly as the SDK returns it (`uint16`), not converted to metres. The
conversion factor is the `depth_scale_m_per_unit` parameter (default `0.001`, i.e. the vendor's
documented RAW16-millimetre convention) — **not independently verified in this session** (no
hardware available). Any consumer doing 3D projection must multiply raw depth by this factor and
must re-derive it against a surveyed target before trusting it metrically, per
`AUTOKART_12_NODE_MASTER_ARCHITECTURE.md` §3 ("do not assume depth units are millimetres").

## CameraInfo contents

`k = [fx, 0, cx, 0, fy, cy, 0, 0, 1]`, `distortion_model = "plumb_bob"`, `d = [0,0,0,0,0]` (the SDK's
calibration blob does not expose distortion coefficients in the payload this node reads; `d` is a
placeholder, not a verified "no distortion" claim). `p` mirrors `k` into a trivial 3x4 projection
matrix (no stereo baseline — RGB and depth are not a stereo pair here).

## Frames

- `rgb_frame_id` (param, default `hp60c_rgb_optical_frame`) and `depth_frame_id` (param, default
  `hp60c_depth_optical_frame`). Optical convention: +Z forward, +X right, +Y down.
- This node does **not** publish the static `base_link -> hp60c_*_optical_frame` transform. That
  transform depends on the physical mount (height/pitch/roll/yaw) and must come from a separately
  measured and published static TF — never guessed here. See `AUTOKART_12_NODE_MASTER_ARCHITECTURE.md` §3/§20.

## Fault behavior

On any backend read failure (disconnect, stale frame, timestamp mismatch, bad format, unavailable
calibration, missing dependency), this node:
- does **not** publish an Image/CameraInfo for that attempt (no stale/zero-filled frame is ever
  marked valid),
- publishes `/hp60c/status` with `quality=INVALID`, `fault_state=<reason>`, `payload_present=false`,
- keeps retrying (bounded backoff), and recovers to `quality=OK` on the next good frame.

This node never decides to stop the vehicle; it only reports. `autokart_safety` (node 10, not yet
built in this library) is the authority that must treat `INVALID`/stale status as a blocking input.

## Parameters

See `config/hp60c_synthetic.yaml` and `config/hp60c_live.yaml`. `backend` selects `synthetic`
(default, no external dependency) or `hp60c` (reuses the external `autokart_perception.camera.HP60C`
class — see DEPENDENCIES.md).

## What this node is not

It does not run YOLO, does not compute 3D cone positions, and does not fuse RGB/depth — that is
node 02 (`autokart_perception`), not yet built in this library (see ../AUDIT_PHASE0.md "remaining
work"). It also does not replace or modify the camera handling already embedded in
`kart/ros2_ws/src/autokart_nodes/autokart_perception/autokart_perception_ros/node.py`, which remains
the live production path until a separate, reviewed integration step migrates it to consume this
node's topics instead.
