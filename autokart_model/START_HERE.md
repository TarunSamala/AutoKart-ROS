# AutoKart Tarun handoff — start here

Prepared: **23 October 2026**  
Scope: **stationary HP60C camera + ROS 2 perception/midpoint preparation only**  
Release status: **NO MOTOR BATTERY / NO ACTUATOR AUTHORITY / AUTONOMOUS MOTION NO-GO**

This is a transferable, source-first handoff for continuing the project on another Ubuntu/ROS 2 Humble computer. It intentionally does not include firmware flashing, VCU actuation, an arm command, or a vehicle-motion launch.

## What is included

- `ros2_ws/src/`: 12 reviewed AutoKart ROS 2 nodes plus `autokart_msgs` from the clean 22 October QA export.
- `ros2_ws/dependencies/`: copied pure-Python dependencies used by those nodes.
- `runtime/`: HP60C acquisition/perception source, native bridge source, and prepared vendor SDK for x86_64, aarch64 and armhf.
- `models/yolo26s/`: active ROS-path experimental YOLO26s ONNX model, class manifest, model card and provenance records.
- `calibration/`: reported camera mount and survey material. **It is not a qualified calibration.**
- `reference/`: camera/vehicle documents and the original requested test sequence.
- `scripts/`: integrity, build/test, camera-perception launch, ROS topic checks and rosbag recording helpers.
- `evidence/`: validation results generated for this exact bundle.

## First rule

Do not connect the motor battery and do not run actuator/VCU/controller nodes. Put physical left/right cones in a stationary test area, connect only the ROS computer and HP60C, and keep the kart unable to move.

## Recommended order

1. Transfer the whole `autokart_tarun` folder or `autokart_tarun.tar.gz`.
2. Run `bash scripts/verify_bundle.sh`.
3. Read `HANDOFF_AND_TEST_PLAN.md` and install the listed ROS/Python dependencies.
4. Run `bash scripts/build_and_test.sh`.
5. Connect only the HP60C; confirm USB ID `3482:6723`.
6. In terminal A, explicitly acknowledge the stationary/no-actuation setup and launch:

   ```bash
   AUTOKART_NO_ACTUATION_ACK=YES bash scripts/launch_camera_perception_only.sh
   ```

7. In terminal B, run `bash scripts/check_perception_topics.sh`.
8. In terminal C, record evidence with `bash scripts/record_first_ai_bag.sh <session-name>`.

## What counts as success for this first test

- HP60C RGB, RAW16 depth and CameraInfo topics are live and time-coherent.
- `/perception/status` and `/perception/diagnostics` remain visible.
- The exact model hash is reported.
- `/perception/cones` exists. Qualified cone messages are expected only after all live metric-calibration gates pass.
- The bag and session metadata are saved.
- No control, VCU, brake, steering, BLDC or vehicle-motion node is running.

An empty `/perception/cones` with an explicit `LIVE_METRIC_CALIBRATION_UNVERIFIED`, `CALIBRATION_UNVERIFIED`, alignment, freshness, or depth-scale reason is a **correct fail-closed result**, not permission to bypass a gate.

## Current truth

- ROS source/build/software tests: packaged and revalidated; see `evidence/VALIDATION_REPORT.md`.
- YOLO26s file identity/load/smoke inference: validated on this x86_64 computer.
- HP60C live operation on Tarun's computer: **NOT RUN**.
- Camera mount/depth metric survey: **UNVERIFIED**.
- Raspberry Pi timing: **NOT RUN in this bundle validation**.
- Steering, BLDC, brake, E-stop and stopping distance: **NOT RUN / NO-GO**.
- Vehicle or autonomous motion: **NO-GO**.

