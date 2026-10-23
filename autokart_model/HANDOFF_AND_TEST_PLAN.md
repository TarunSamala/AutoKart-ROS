# AutoKart colleague handoff and staged test plan

## 1. Purpose and safety boundary

This handoff supports the next minimum AI test: transfer the ROS/perception workspace and model, start the HP60C and perception only, place physical left/right cones, record rosbag2 data, and inspect `/perception/cones` and diagnostic topics.

For this stage:

- motor battery disconnected;
- no actuator power;
- no VCU serial owner;
- no controller, steering, brake or BLDC process;
- no vehicle authority;
- no movement.

Software tests, model loading and a visible topic do not prove physical calibration or vehicle readiness.

## 2. Bundle map

| Path | Contents | Evidence status |
|---|---|---|
| `ros2_ws/src` | 13 ROS 2 packages from the clean QA export | source tests/build re-run for this bundle |
| `ros2_ws/dependencies` | reused perception, planner, safety, dashboard and geometry Python code | hashed with bundle |
| `runtime/phase_01_perception` | HP60C/perception Python and native bridge source | software-only validation |
| `runtime/vendor` | prepared vendor SDK and architecture libraries | copied; target-host load still required |
| `models/yolo26s/best.onnx` | 5-class experimental YOLO26s | SHA-256 checked; ONNX Runtime smoke inference |
| `models/yolo26s/best.json` | labels, input/output contract and model hash | checked against ONNX |
| `calibration/perception_kart_unverified.yaml` | reported mount candidate | **UNVERIFIED; not an activation artifact** |
| `reference` | full engineering and calibration background | reference, not release authority |

## 3. Target computer prerequisites

Recommended target: Ubuntu 22.04 with ROS 2 Humble and Python 3.10.

Install ROS build/runtime dependencies appropriate to the target computer, including:

```bash
sudo apt update
sudo apt install -y python3-colcon-common-extensions python3-rosdep python3-venv \
  build-essential cmake libusb-1.0-0-dev python3-opencv python3-numpy \
  ros-humble-cv-bridge ros-humble-image-transport ros-humble-message-filters \
  ros-humble-sensor-msgs ros-humble-geometry-msgs ros-humble-nav-msgs \
  ros-humble-rosbag2 ros-humble-rosbag2-storage-default-plugins
```

Create a Python environment only if needed by the host; do not transfer a `.venv` between machines. The perception code needs PyYAML, NumPy, OpenCV and ONNX Runtime. Example:

```bash
python3 -m venv --system-site-packages .venv
. .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install pyyaml onnx onnxruntime
```

## 4. Integrity and build

From the bundle root:

```bash
bash scripts/verify_bundle.sh
bash scripts/build_and_test.sh
```

The build script uses local `build/`, `install/` and `log/` directories under `ros2_ws`; they are generated on Tarun's computer and are not portable evidence from this computer.

## 5. Physical stationary setup

1. Disconnect the traction/motor battery and make vehicle motion mechanically impossible.
2. Leave ESP32/VCU, steering, brake and BLDC actuator links disconnected for this test.
3. Secure the HP60C mount. Do not treat the bundled height/pitch values as verified.
4. Place blue cones on the physical left and yellow cones on the physical right when facing the travel direction. Include an independently measured survey sheet for any metric-position claim.
5. Connect HP60C USB and confirm `lsusb` includes `3482:6723`.
6. Use only one camera owner.

## 6. Launch camera and perception only

Build the native bridge for the target architecture:

```bash
bash scripts/build_hp60c_bridge.sh
```

Then run:

```bash
AUTOKART_NO_ACTUATION_ACK=YES bash scripts/launch_camera_perception_only.sh
```

That script checks its ROS graph before launch and refuses to start if known control/VCU/actuator node names are already present. It launches exactly `autokart_hp60c` and `autokart_perception_adapter`. This is a useful software guard, not a replacement for physical battery isolation.

## 7. Observe and record

In separate terminals:

```bash
bash scripts/check_perception_topics.sh
bash scripts/record_first_ai_bag.sh tarun-first-ai-001
```

Required observation topics:

- `/hp60c/rgb/image_raw`
- `/hp60c/depth/image_raw`
- `/hp60c/rgb/camera_info`
- `/hp60c/status`
- `/hp60c/diagnostics`
- `/perception/status`
- `/perception/diagnostics`
- `/perception/detections`
- `/perception/cones`

Save operator, computer, Git/provenance hashes, cone layout measurements, camera mounting measurements, start/end time and all diagnostics beside the bag.

## 8. Interpreting `/perception/cones`

The adapter is fail-closed. Live metric cones are published only when the model, RGB/depth pairing, registration, depth scale, frame freshness and calibration conditions are valid. The included mount YAML is a reported candidate with a pitch conflict and is not a signed `BENCH_VERIFIED` JSON artifact. Therefore:

- detections may appear on `/perception/detections` while `/perception/cones` is empty;
- diagnostics should explain why;
- do not alter source, change `LIVE` to `SYNTHETIC`, fabricate a calibration artifact or relax thresholds to force cone publication;
- first resolve the mount/depth survey using the grid and procedures in the reference documents.

## 9. Calibration work still required

Before a metric cone-position pass:

- record the exact HP60C identity and fixed resolution;
- measure optical-center XYZ, pitch, roll and yaw;
- resolve the reported `17.8°` pitch versus `11.4°` ground-plane estimate;
- verify RAW16 scale and RGB/depth alignment physically;
- survey the required left/centre/right grid at 0.5, 1, 2 and 3 m;
- keep held-out stations independent;
- generate and independently review a valid calibration artifact;
- preserve uncertainty and rejection reasons.

The bundled `CAMERA_CALIBRATION.md` contains the detailed procedure.

## 10. Mandatory sequence after the perception-first stage

Do not skip directly to motion. The later stages are:

1. camera only;
2. AI inference with no actuators;
3. metric cone positions;
4. recorded replay;
5. left/right cones and midpoint;
6. middle-obstacle detection;
7. planner dry run with commands displayed but not transmitted;
8. VCU communication with actuator power disconnected;
9. wheels-lifted steering and BLDC test;
10. brake apply/release test;
11. manual low-speed straight driving;
12. autonomous steering at walking speed;
13. smooth obstacle stop at minimum speed;
14. repeat stopping test multiple times;
15. gradually increase speed only after measured stopping margins pass.

Steps 9–15 require a separate reviewed procedure, independent physical E-stop and traction inhibit, qualified personnel, measured feedback and explicit GO decision. They are **not authorized or completed by this bundle**.

## 11. Measured gates for later motion

At minimum, do not advance to vehicle motion until there is current evidence for:

- physical steering direction, centre, limits, backlash, rate and achieved wheel angle;
- resolved DAC requested-versus-measured output fault and BLDC safe-state truth table;
- brake released/applied positions, ≥20 repeat cycles, latency and power-loss behavior;
- independent ground-motion speed evidence;
- worst-case repeated stopping distance/deceleration on the intended surface/load;
- physical E-stop and independent traction inhibit;
- command expiry/session/CRC/serial single-owner behavior;
- target-computer perception latency and visibility envelope;
- full body/bumper clearance margin.

`STOP_VERIFIED` requires independent physical motion evidence. A STOP acknowledgement, zero speed target, brake position or wheel encoder alone is insufficient.

## 12. Known blockers at handoff

- Live metric HP60C calibration: unverified.
- Positive-cone HP60C field metrics and target-Pi timing: incomplete/not current.
- Steering under load: historical stutter/slip failure unresolved.
- GP8630N/BLDC: historical 5 V request measured about 3.3 V; unresolved.
- Mounted brake force, power-loss behavior and vehicle deceleration: unverified.
- Full graph: prior QA reported IMU source/gap and CPU-margin blockers.
- Vehicle motion: NO-GO.

