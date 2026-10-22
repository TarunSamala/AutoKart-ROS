# Test and measurement evidence

Audit date: 2026-10-16. Tests below were executed during this audit. All were software/static/simulation only; no command was sent to real traction, steering or braking hardware. October 15 camera artifacts are saved measurements, not reruns.

## Executed this audit

| Result | Exact command | Evidence and limit |
|---|---|---|
| **PASS** | cd autonomy_stack_phase1/phase_01_perception && PYTHONDONTWRITEBYTECODE=1 PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONPATH=src:/home/mark1/Desktop/autonomous_kart/kart python3 -m pytest -p no:cacheprovider -q tests | 139 passed, 7.26 s. Perception/core software only; no camera calibration/Pi/vehicle proof. |
| **PASS** | cd kart && PYTHONDONTWRITEBYTECODE=1 PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONPATH=. python3 -m pytest -p no:cacheprovider -q tests | 149 passed, 56.36 s. Simulator/VCU model/physics/safety software only. |
| **PASS** | cd phase_sim && npm run test -- --run | 8 files, 32 tests passed, 8.10 s. LAB uses simulation. |
| **PASS** | cd phase_sim && npm run build | TypeScript/Vite succeeded, 53 modules. JS 905.32 kB (247.31 kB gzip); advisory only. |
| **PASS** | cd AutoKart_RC_Dynamics_ESP32S3_v1 && PYTHONDONTWRITEBYTECODE=1 PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest -p no:cacheprovider -q tests/test_static.py | 9 passed, 0.03 s. Source assertions only. |
| **PASS** | cd AutoKart_RC_Dynamics_ESP32S3_v1 && node tests/ui_behavior.js | Dashboard hold/release and brake priority checks passed; mock DOM/fetch only. |
| **PASS** | cd AutoKart_RC_Dynamics_ESP32S3_v1 && node --check tests/ui_syntax.js | JavaScript parser only. |
| **PASS** | cd AutoKart_RC_Dynamics_ESP32S3_v1/AutoKart_ESP_IDF_R86mini_WebStop && PYTHONDONTWRITEBYTECODE=1 python3 tests/static_checks.py | Static checks printed PASS for endpoints, pins, pulse constants, stop paths and embedded HTML. |
| **PASS** | cd AutoKart_RC_Dynamics_ESP32S3_v1/AutoKart_ESP_IDF_R86mini_WebStop && node --check tests/dashboard_check.js | JavaScript parser only. |
| **PASS** | cd kart && PYTHONDONTWRITEBYTECODE=1 PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONPATH=.:ros2_ws/src/autokart_dashboard_gateway:ros2_ws/src/autokart_sim_bridge python3 -m pytest -p no:cacheprovider -q ros2_ws/src/autokart_dashboard_gateway/test ros2_ws/src/autokart_sim_bridge/test ros2_ws/src/autokart_gazebo/test | 10 passed, 0.04 s. Local ROS contracts; no live graph/Gazebo/hardware. |
| **PASS (limited)** | cd kart/ros2_ws && colcon --log-base /tmp/autokart_full_audit_ros_log build --base-paths src --build-base /tmp/autokart_full_audit_ros_build --install-base /tmp/autokart_full_audit_ros_install | 13 packages finished, exit 0. ROS Humble build. Warnings include missing pytest for Gazebo hook. |
| **PASS (limited)** | cd kart/ros2_ws && colcon --log-base /tmp/autokart_full_audit_ros_log test --base-paths src --build-base /tmp/autokart_full_audit_ros_build --install-base /tmp/autokart_full_audit_ros_install | 13 packages finished; sim bridge reported 2 tests. Dashboard gateway 0 tests; Gazebo pytest hook unavailable. No full ROS integration. |

First ROS test attempt before build failed (exit 1) because temporary installed package.sh files were absent. After temporary build it was rerun. An earlier command placed --log-base after the test verb and was a CLI usage error (exit 2); no tests ran.

idf.py was not installed (command -v idf.py returned no path). Firmware build is **BLOCKED**, not passed. No flash attempted. No live ROS graph, Pi runtime, micro-ROS Agent or HIL test ran.

## Saved live evidence reviewed (2026-10-15)

### YOLO26s 20-second freshness capture

Source: _autokart_audit/yolo26s-phase6-evidence-20261015.json. 125 polls, 118 unique synchronized frames, zero HTTP/sequence errors, max 7 detections. Inference p50/p95 127.8/139.3 ms; capture-to-publish p50/p95 197.1/238.5 ms. Frame age p50/p95 286.3/354.1 ms vs 250 ms limit. pass=false, 118 SENSOR_FAULT + 7 STALE, authorized_count=0. **FAIL for saved freshness gate.** Acquisition/inference evidence only.

### HP60C 10-minute geometry capture

Source: _autokart_audit/hp60c-geometry-10min-20261015.json. 600.135 s, 2,989 polls, 2,523 unique frames, no HTTP or RGB/depth sequence mismatch, no authorization events; 15,336 detections, 9,827 localization candidates, **0 validated metric cones**, **0 geometry frames**. Status 747 stale, 75 ambiguous, 2,167 sensor fault. Rejections: 4,931 clipped feet, 9,827 unverified depth scale and mount. Capture-to-publish p50/p95/max 206.0/266.7/807.1 ms; frame age 281.4/378.0/821.3 ms; processed rate 4.20 Hz. **FAIL for metric geometry; limited PASS for sustained acquisition/sequence continuity.**

### Survey grid

Source: _autokart_audit/hp60c-calibration-grid-20261015.json. Twenty frames inspected but no surveyed cones; accuracy **NOT EXECUTED**. Invalid depth rate 37.48%; required grid empty.

## Current host checks

The three user services autokart-phase1-live.service, autokart-phase1-synthetic.service and autokart-live-diagnostic.service were **inactive**. No matching app/ROS/micro-ROS process or port was observed. This says nothing about a separate Pi. ros2 and colcon exist; idf.py does not. ROS build output was under /tmp/autokart_full_audit_ros_*.

## Not run / blocked

| Test | Classification | Reason |
|---|---|---|
| Surveyed cone accuracy, depth scale, mount and registration | **NOT RUN / BLOCKED** | No independent survey grid. |
| Pi timing under camera+ROS+VCU load | **NOT RUN** | Pi runtime unavailable. |
| Encoder/IMU/steering fusion and drift | **NOT RUN** | No connected pose source/data. |
| Skidpad end-to-end/lap completion | **NOT RUN** | No validated map, odometry, full-loop path or vehicle link. |
| Braking marker→command→measured stop | **NOT RUN** | No marker pipeline/VCU link or measured brake capability. |
| Pi↔ESP32 USB/micro-ROS round trip/recovery | **NOT RUN** | No Pi/Agent session. |
| ESP-IDF firmware build | **BLOCKED** | idf.py absent; no flash. |
| Physical E-stop, power-loss brake, jam, traction isolation, loaded stop | **NOT RUN** | No physical safety evidence supplied; no hardware action taken. |

PASS applies only to exact software/static commands. Simulation, replay, mocks, source assertions, package build and browser tests do not prove vehicle motion, physical braking, E-stop independence or safety at 10 km/h.

## Saved images: visual review and interpretation

The images below are saved diagnostics from the earlier audit, dated October 14 unless noted. They are not new captures made during the October 16 audit. RGB screenshots show what a dashboard or camera presented; the paired depth images are false-color visualizations, not metric depth proof. In all relevant dashboard captures the UI itself marks calibration, mount, depth scale, pose/path or geometry as unqualified and vehicle authority disabled.

| File | What is visible | What it does and does not establish |
|---|---|---|
| `_autokart_audit/live-dashboard-before-user-20261014.png` | Earlier full desktop/dashboard capture before the later diagnostic refresh; diagnostic view has incomplete/old state and no valid metric geometry. | Historical UI baseline only. It is not evidence of current operation or an end-to-end stack. |
| `_autokart_audit/live-dashboard-after-20261014.png` | Refreshed diagnostic dashboard with RGB/depth panes, frame/timing status and geometry area. | Shows UI wiring and safety/status presentation; no vehicle command authority. |
| `_autokart_audit/live-dashboard-after-final-20261014.png` | Final dashboard screenshot: ConeNet v2 experimental, `LIVE AMBIGUOUS`, 5 depth candidates and 0 validated metric positions; warnings call out unverified depth scale/mount. | Directly supports “diagnostic candidates only,” not validated cone locations or a usable path. |
| `_autokart_audit/live-dashboard-after-full-20261014.png` | Tall full-page dashboard. Five detector boxes are drawn on orange cones but labeled yellow; metric view says unavailable, 5 diagnostic depth candidates and 0 validated metric positions. Trace says path invalid, pose unavailable, no telemetry/VCU publication. | Strong visual confirmation of displayed diagnostic behavior and remaining gates. The class/color mismatch and candidate/metric distinction must not be hidden by the overlay. |
| `_autokart_audit/live-dashboard-after-paused-20261014.png` | Same view with display paused. The frame is labeled historical and 5.7 seconds old; status reads stale/not ready while geometry remains invalid. | Confirms the UI marks a paused stale frame rather than implying fresh live geometry. |
| `_autokart_audit/aut05-live-cone-run-last.jpg` | Single 640×480 RGB camera view of cones; the saved view is rotated/inverted relative to ordinary upright viewing. | A capture exists, but this image alone cannot tell whether the orientation comes from sensor mounting, SDK configuration, or saved-view rendering. It is not enough to claim a physical camera fault. |
| `_autokart_audit/aut05-live-evidence-20261014/rgb.jpg` | Dim indoor scene with shelves/table and no clear cone course in view. | Visual example of a difficult/empty scene; not a surveyed test target. |
| `_autokart_audit/aut05-live-evidence-20261014/depth.jpg` | False-color RAW16 depth visualization with broad invalid/black regions and no legend conveying metric scale. | Illustrates depth availability/holes, not calibrated distances. The associated snapshot JSON is the source for numerical pixel counts/metadata. |
| `_autokart_audit/aut05-live-evidence-20261014-real-graph/rgb.jpg` | Indoor room with orange cones; orange candidate boxes and raw-distance labels; overlay says path projection unavailable because calibration/pose are unverified. | Demonstrates candidate detection in a real camera frame. Candidate boxes are not verified metric positions. |
| `_autokart_audit/aut05-live-evidence-20261014-real-graph/depth.jpg` | False-color depth frame aligned in appearance with the room image, including invalid/black patches. | Does not establish RGB-depth registration accuracy or raw-unit-to-meter scale. |
| `_autokart_audit/aut05-live-evidence-20261014-rejected-detections/rgb.jpg` | Tilted indoor cone image with one orange candidate box; projection warning remains. | Example of a detection/rejection diagnostic context, not a validated localized cone. |
| `_autokart_audit/aut05-live-evidence-20261014-rejected-detections/depth.jpg` | False-color depth frame for the same scene; large near-field surfaces and invalid pixels. | Supports visual review of difficult depth geometry only. Rejection reasons and measurements belong to matching `snapshot.json`. |
| `_autokart_audit/conenet-v1-live-dashboard/rgb.jpg` | Cone room with ConeNet v1 detections/overlay. | Saved model/dashboard sample; cannot independently establish model accuracy or runtime freshness. |
| `_autokart_audit/conenet-v1-live-dashboard/depth.jpg` | False-color depth counterpart for the v1 dashboard sample. | No calibration legend or survey; qualitative visualization only. |
| `_autokart_audit/conenet-v2-live-dashboard/rgb.jpg` | Similar cone-room scene with five yellow-labeled candidate boxes over visibly orange cones. | Shows detections and the observed class/color discrepancy. Does not validate class identity or geometry. |
| `_autokart_audit/conenet-v2-live-dashboard/depth.jpg` | False-color depth counterpart for the v2 scene, with black invalid regions. | Visual-only; no metric validation. |
| `autonomy_stack_phase1/phase_01_perception/recordings/synthetic-preview.png` | Generated synthetic UI preview: colored cone markers at left, a simple triangulation/gate-like diagram at right, and footer labels `SYNTHETIC`, zero detections and model unavailable. | Illustrates the synthetic visualization path; it is not a camera frame, detector result or performance measurement. |

The five full dashboard screenshots have `.png` filenames, although file inspection found several are JPEG-encoded image payloads. Image viewers can open them by content; the extension mismatch is a packaging detail, not validation evidence. The RGB/depth pairs are 640×480 JPEGs.

## Other saved audit files and how to read them

| Artifact group | Contents / interpretation |
|---|---|
| `_autokart_audit/01_REPOSITORY_INVENTORY.md` through `13_TEN_DAY_EXECUTION_PLAN.md` | Prior audit narrative: inventory, architecture, perception, geometry, planning, firmware, FMEA, performance, tests, defects, readiness, decisions and plan. Useful history, but its older test totals are superseded by commands above. |
| `*_evidence*.json`, `*metrics*.json`, `*validation*.json`, `*comparison*.json`, `*check*.json` | Structured snapshots/measurements for model checks, freshness, browser/service status, calibration, geometry and release decisions. Read with its matching date/run and status fields; saved JSON is not a rerun. |
| `*.jsonl` live/soak/replay logs | Time-ordered per-frame or per-event records. They preserve observed status transitions and timing, but may contain repeated frames, stale samples or experimental model runs; use unique frame IDs and timestamps before counting. |
| `_autokart_audit/aut05-live-recording-20261014/000001.json` … `000100.json` | Per-frame metadata sidecars for the recorded diagnostic sequence. Each sidecar describes one capture/result; inspect its sequence/timestamp/status before pairing it to the matching NPZ. |
| `_autokart_audit/aut05-live-recording-20261014/000001.npz` … `000100.npz` | Compressed numeric frame arrays (RGB/depth and related structured arrays). Binary data were not individually opened in this report; the recording loader/tests and JSON summaries are the review path. |
| `*_tests.txt`, `*_build.txt`, `*_check.json`, `preflight.log` | Saved command output from earlier audit runs. These are historical evidence; exact fresh software checks from this audit are listed at the start of this file. |
| `camera_team_backup_20261015/manifest.json` | Backup manifest describing captured files/hashes for the camera team snapshot. It records provenance, not runtime deployment. |
| `AUT05_ABC_RELEASE_EVIDENCE.json` | Saved release-gate evidence object. Treat missing, stale or unmeasured fields as unknown; it cannot replace physical safety evidence. |

The image catalog is descriptive review. It does not transform the pictures into ground-truth labels, surveyed distances, calibration measurements or proof of physical hardware state.

### Image collections not reviewed one image at a time

The workspace scan found 1,783 image files in the scanned extensions: 1,529 under `datasets/`, 24 under `training_runs/`, 214 under Phase 1 and historical camera snapshots, including 202 under Phase 1 `recordings/`, and 16 selected audit images. The dataset images are training/validation examples, not runtime captures; reviewing each individual training image was outside this system audit. The 24 training-run figures are generated diagnostics (labels, training batches, precision/recall/F1 curves, confusion matrices and result plots); they describe model training/evaluation artifacts and are not independent live-camera evidence. The 202 Phase 1 recording images are arranged in run directories: dated HP60C samples, annotated ConeNet v2 examples and related capture frames. They were not individually labeled or independently scored here; matching JSON labels/evaluation files and the aggregate saved measurements are the proper quantitative evidence. Historical `_camera_*` snapshots contain duplicate or divergent images. Therefore, the visual table above itemizes the selected safety/diagnostic images that support conclusions in this audit, while the count and role of bulk image collections are recorded here. “Every image” should not be read as a ground-truth human annotation of all 1,783 images.
