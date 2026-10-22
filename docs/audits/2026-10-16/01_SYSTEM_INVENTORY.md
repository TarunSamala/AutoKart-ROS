# AutoKart system inventory

Audit date: 2026-10-16. I recursively inventoried /home/mark1/Desktop/autonomous_kart, inspected source/config/interfaces and saved evidence, and ran safe software checks. Generated caches, virtual environments, build products, data sets, recordings and model binaries were excluded from line-by-line inspection. No production code, firmware, interface or safety behavior was changed.

## Guide to the five deliverables

| Report | Contents |
|---|---|
| `01_SYSTEM_INVENTORY.md` | Workspace/repository map, what the main source trees contain, what was excluded, and which artifacts are snapshots versus active candidates. |
| `02_END_TO_END_ARCHITECTURE.md` | Signal path from camera/sensors through perception, geometry, mission, control, ROS/VCU and actuators; identifies the implemented links and the missing/unproven links. |
| `03_DEFECT_REGISTER.md` | Prioritized issues with evidence, operational impact, minimum correction and verification needed. Unknown physical facts are kept explicitly unknown. |
| `04_TEST_EVIDENCE.md` | Exact checks run, saved live measurements, what each selected image shows, and blocked/not-run physical validations. |
| `05_COMPLETION_PLAN.md` | Staged work order and exit evidence required to progress from source audit to bench integration and any later controlled operation. |

The reports are complementary: 01 describes the files and boundaries; 02 follows data and commands; 03 records defects; 04 is the evidence ledger; 05 is the closure sequence.

## Verdict

**MORE DATA REQUIRED** for vehicle operation. There is substantial simulation, diagnostic perception, control candidate and safety-oriented source, but no verified end-to-end vehicle stack. Current live diagnostics explicitly set ready=false, vehicle_authorized=false and published_to_vcu=false. The latest saved HP60C geometry run produced zero validated metric cones.

## Repository and ownership

The workspace root, kart/ and phase_sim/ have no Git metadata. Nested Git repositories:

| Path | State | Disposition |
|---|---|---|
| autonomy_stack_phase1/ | main; modified/untracked phase 1 source, docs and evidence | Most complete camera diagnostic candidate; no vehicle command interface. |
| AutoKart_RC_Dynamics_ESP32S3_v1/ | No Git repository | Two ESP-IDF RC/bench firmware variants, MicroPython experiments and dashboard; installed firmware/wiring unidentified. |
| _camera_a1…_camera_a5/ | camera-a1…camera-a5 | Historical development snapshots. |
| _camera_prod_a1…_camera_prod_a5/ | camera-prod-a1…camera-prod-a5; some changed | Qualification snapshots, not deployed runtime. |
| _camera_team_base/ | camera-team-base; changed | Team snapshot with duplicated perception and phase_sim. |

There are 471 Markdown files. _autokart_audit/ is the previous audit and saved evidence ledger. datasets/, training_runs/, models/ and recordings contain excluded data/artifacts; manifests and relevant measurements were inspected. The actual Pi checkout and vehicle were unavailable.

## Projects, entry points and actual scope

| Project | Entry points | Scope |
|---|---|---|
| autonomy_stack_phase1/phase_01_perception/ | python -m autokart_perception; live_mission.py; application.py; shared_core.py | HP60C/replay/synthetic acquisition, detection, depth candidates, per-frame geometry, diagnostic UI, pose-aware map and mission/controller candidates. No VCU interface. |
| autokart_apps/ | run_core.sh, run_lab.sh, run_live.sh, run_tests.sh | Shared Python CORE and local LAB/dashboard diagnostics. No vehicle authority. live_mission defaults to ConeNet v1; YOLO26s is selectable. |
| kart/ | kart-sim CLI; scripts/test.sh | Largest source: simulated vehicle/VCU, controllers, faults, scenarios, docs, tests, dashboard, ROS 2/Gazebo. README says simulation use only. |
| phase_sim/ | npm dev/test/build | React/Three.js dashboard with synthetic LAB and external ROS display modes. |
| AutoKart_RC_Dynamics_ESP32S3_v1/ | main/main.c; nested R86mini WebStop; MicroPython | Manual RC/bench controllers; not a verified autonomy endpoint. idf.py absent on host. |
| XIAO_C3_RA02_ESTOP_PRODUCTION/ | main.py, receiver_esp32s3_rp2350.py | LoRa stop prototype with unimplemented send and stop behavior. |
| bench/ | main.py, validate_bench.py and subsystem folders | Historical BLDC/brake/steering/throttle/E-stop code, not runtime-connected. |
| AUT05_Laptop_First/ | preflight scripts | Laptop setup only. |

kart/ has kart_sim, 12 ROS packages, simulation models/worlds, configuration, tests and docs. ROS packages cover messages, bringup, perception, localization, planning, control, safety bridge, sim bridge, dashboard gateway, diagnostics, scenarios, description and Gazebo. Package presence is not working runtime evidence.

## Implemented versus demonstrated

| Subsystem | Implemented | Verification and limit |
|---|---|---|
| HP60C acquisition | Native SDK/C++ bridge, RGB/depth pair, Python camera, recording/replay | Saved bench captures have matching sequences. Exposure alignment and metric registration residual unmeasured. |
| Detection | ONNX; YOLO26s and ConeNet model artifacts; orange-only ConeSlayer | Saved YOLO26s run: 118 unique frames, inference p95 139.3 ms, but freshness p95 354.1 ms vs 250 ms gate. Colors not qualified. |
| Depth/metric transform | Intrinsics, depth sampling, ground-ray candidates, mount transforms, uncertainty/rejection reasons | Latest 10-minute run: 15,336 detections, 9,827 candidates, zero validated metric cones. Depth scale/mount unverified, many cone feet clipped. |
| Geometry | SciPy Delaunay, Gabriel, boundary/gates/centerline/curvature/topology | Unit tests pass. No live validated inputs; Gabriel computed but not used to filter gates; forward-X local corridor only. |
| Mapping/mission/control | shared_core has bounded pose-aware map, skidpad ring/winding and brake-capability contract; kart_sim has PP/Stanley/PID | Software candidate. No qualified live pose or physical brake capability; authority remains false. |
| Simulation | Python plant/VCU, scenarios, faults, Gazebo plugins | Current software suites pass; not a physical plant/safety proof. |
| ROS | Typed schemas, Gazebo path, read-only dashboard gateway, sim bridge subscriber | Workspace builds. full_stack launch runs only vcu_bridge, which stores command in memory. No live planner or ESP32 transport. |
| ESP32 | Several bounded manual RC/bench firmware candidates and static tests | No selected installed image, complete wiring map, verified feedback or calibrated brake. Root firmware requires brake AS5600; project context says none installed. |
| Safety | Software faults and one firmware auxiliary E-stop input | Independent traction cutoff/power-loss stop unverified. LoRa send() stub returns success; receiver only prints. |

## Duplicates and selection

1. Perception copies in autonomy_stack_phase1, 11 camera/team snapshots and duplicated phase_sim are divergent development worktrees, not deployed versions.
2. Phase 1 CLI selects orange-only ConeSlayer; run_live selects ConeNet v1; YOLO26s is optional. Do not assume the requested model context is the active default.
3. phase_sim LAB, kart/kart_sim, Phase 1 corridor geometry and autokart_apps shared core have different inputs and authority; they are not interchangeable.
4. Root ESP-IDF, nested R86mini, MicroPython and bench firmware differ in pins, pulse ratios, interlocks and transport. Installed version unknown.
5. _autokart_audit and prior reports contain stale test counts. Current reruns in 04_TEST_EVIDENCE.md supersede software test totals.

Inventory covered the full directory structure and first-party source/config/test/docs. Generated caches and binaries were intentionally not read line-by-line. Selected Pi runtime, ROS graph, installed ESP32 binary, as-built wiring and vehicle remain unavailable.

## File-by-file guide to the principal active source trees

This catalog gives each first-party source file in the main implementation trees a purpose-level description. It is not a claim that every module is wired into the same runtime. A file's presence means code or configuration exists; the evidence column in the other reports says whether that behavior was exercised. Paths are relative to the workspace root.

### Phase 1 perception package

| File | What it contains / why it exists |
|---|---|
| `autonomy_stack_phase1/phase_01_perception/src/autokart_perception/__init__.py` | Package exports and public import surface. |
| `.../__main__.py` | `python -m autokart_perception` command entry point; selects and runs the package CLI. |
| `.../application.py` | Assembles the perception application, source selection, processing loop and status outputs. |
| `.../boundaries.py` | Derives left/right track-boundary candidates from cone/point geometry. |
| `.../calibration.py` | Reads and validates camera intrinsics and calibration metadata used by pixel-to-ray calculations. |
| `.../camera.py` | Camera capture abstraction and frame acquisition lifecycle. |
| `.../centerline.py` | Builds a centerline candidate from boundary/gate geometry. |
| `.../curvature.py` | Estimates local path curvature and related geometric quantities. |
| `.../dataset_sources.py` | Resolves dataset, replay and other recorded input sources for evaluation/training. |
| `.../depth.py` | Samples depth within detections, estimates ground intersections and records metric validity/rejection reasons. |
| `.../detector.py` | Detector implementations/adapters and inference result normalization, including model-backed and color-based paths. |
| `.../evaluation.py` | Computes dataset/evaluation metrics and compares detector outputs with annotations. |
| `.../gabriel.py` | Constructs Gabriel graph edges from point sets for geometric-neighborhood analysis. |
| `.../gates.py` | Selects cone-pair gate candidates and orders/filters them for a traversable corridor. |
| `.../geometry.py` | Shared vector, coordinate-frame and planar geometry helpers. |
| `.../interfaces.py` | Typed contracts/protocols between acquisition, detection and downstream consumers. |
| `.../lab_api.py` | Local LAB/diagnostic API endpoints and serialization helpers. |
| `.../live_mission.py` | Live diagnostic mission/controller candidate loop and readiness/authority status. It is explicitly not a VCU command publisher. |
| `.../live_render.py` | Renders/serializes live perception and geometry overlays for the diagnostic view. |
| `.../mount.py` | Camera-to-vehicle mount transform parsing and validation state. |
| `.../pipeline.py` | Orchestrates a frame through detection, depth, geometry and result/status assembly. |
| `.../recording.py` | Writes and reads frame-level diagnostic recordings and metadata for replay. |
| `.../release_audit.py` | Evaluates release evidence and produces a readiness/audit result with unmet gates. |
| `.../safety.py` | Fail-safe status, stale/fault handling, readiness and authorization gates for perception outputs. |
| `.../sdk.py` | Native camera SDK bindings/adapters and capture metadata normalization. |
| `.../shared_core.py` | Shared state/map, pose-aware observation accumulation and controller-candidate snapshot. |
| `.../synthetic.py` | Synthetic scenes/frames and deterministic non-hardware source used by development and tests. |
| `.../temporal.py` | Temporal association/tracking and age/continuity handling across frames. |
| `.../training.py` | Dataset/model training and artifact-resolution utilities; not a live runtime model proof. |
| `.../types.py` | Dataclasses/enums and common typed records for frames, detections, geometry and statuses. |
| `.../visualization.py` | Image/geometry drawing utilities shared by diagnostics and saved artifacts. |
| `.../webui.py` | Web server and dashboard wiring for live/synthetic/replay diagnostics. |
| `.../data/coneslayer_manifest.json` | Model/artifact identity and expected metadata for the ConeSlayer path. |
| `.../data/perception.yaml` | Perception defaults/thresholds and configuration for detector/depth processing. |

The corresponding `autonomy_stack_phase1/phase_01_perception/tests/test_*.py` files are named for the module or contract they exercise: application integration, calibration, camera, dataset source resolution, depth, detector, evaluation, Gabriel graph, gates, geometry, live mission status, mount, pipeline, recording, release audit, safety, SDK API, shared core, temporal behavior, track geometry, training/resolver, and web UI. The exact executed count and command are recorded in `04_TEST_EVIDENCE.md`.

Supporting Phase 1 files in that same package tree have these roles: `README.md` is the setup/usage overview; `SDK.md` documents camera SDK integration; `PHASE_01.txt` and `PHASE_01_TEST_REPORT.md` record phase scope and prior test notes; `A1_REPORT.md` … `A5_REPORT.md` are milestone reports; `CAMERA_TEAM_CONTRACT.md` and `CAMERA_TEAM_INTEGRATION_REPORT.md` specify the team interface and integration state; `pyproject.toml` and `requirements.txt` declare package/build dependencies; `build_native.sh` builds the native bridge; `native/CMakeLists.txt` and `native/hp60c_bridge.cpp` define/build the C++ SDK bridge; `deployment/autokart-core-diagnostic.service.example` is an example service unit; `examples/geometry_sdk.py` and `examples/replay_sdk.py` demonstrate SDK-backed geometry and replay; `benchmarks/benchmark_detector.py`, `benchmark_pipeline.py`, and `benchmark_sdk.py` run component benchmarks; `benchmarks/monitor_live_geometry.py` monitors live geometry; the neighboring dated `.json` benchmark files are saved results rather than executable source. `models/README.md` and download scripts document or fetch model artifacts; `coneslayer-simplified.onnx`, `yolo11n.pt`, the top-level `yolo11n.pt`, and the wheel under `dist/` are binary/model/package outputs, not inspectable source. `recordings/*labels*.json` and `*evaluation*.json` hold annotations/evaluation metadata; `synthetic-preview.png` is a generated preview. `.venv*`, `.pytest_cache`, `UNKNOWN.egg-info` and `dist/` are environment/package outputs, not runtime source of record.

### Python simulator (`kart/kart_sim`)

| File | What it contains / why it exists |
|---|---|
| `__init__.py` | Package-level exports. |
| `actuators.py` | Simulated actuator interfaces and limits. |
| `autonomy.py` | Autonomy orchestration for simulated missions. |
| `backends.py` | Simulation/backend selection and backend interfaces. |
| `benchmark.py` | Benchmark execution and summary metrics. |
| `calibration.py` | Simulator calibration values and transforms. |
| `can_hil.py` | CAN/HIL integration helpers; code presence does not mean hardware connection. |
| `cli.py` | `kart-sim` command-line entry points and argument routing. |
| `communication.py` | Simulated communication links, delays and message handling. |
| `cones.py` | Cone/track marker models and course geometry. |
| `config.py` | Loading, validation and composition of simulator configuration. |
| `dashboard.py` | Dashboard state and view-model generation. |
| `dashboard_server.py` | HTTP/WebSocket dashboard server. |
| `diagnostics_bridge.py` | Diagnostic/status bridge between simulation and UI/API. |
| `dynamic_kart.py` | Higher-fidelity dynamic vehicle model. |
| `electrical.py` | Electrical system/energy model. |
| `fast_env.py` | Fast simulation environment for batch runs or RL-style stepping. |
| `fault_manager.py` | Fault injection, fault state and recovery logic. |
| `formula_student.py` | Formula Student event/vehicle behavior helpers. |
| `four_wheel.py` | Four-wheel vehicle dynamics/model utilities. |
| `gazebo_follower.py` | Gazebo follower behavior and state contract. |
| `gazebo_full_lap.py` | Full-lap Gazebo scenario/runner support. |
| `gazebo_preset.py` | Named Gazebo preset construction. |
| `multi_kart.py` | Multi-vehicle simulation coordination. |
| `phase16.py` | Phase 16 compatibility/track scenario definitions. |
| `powertrain.py` | Powertrain, motor and torque-response model. |
| `race_control.py` | Start/stop and event-level race control logic. |
| `racing_autonomy.py` | Racing-oriented planning/control loop in simulation. |
| `realtime.py` | Real-time stepping and timing coordination. |
| `robot_description.py` | Robot description/model generation helpers. |
| `ros2_sil.py` | Software-in-the-loop ROS integration support. |
| `ros2_wire.py` | ROS message/topic serialization and wire-contract helpers. |
| `safety_config.py` | Safety-related simulator limits and configuration. |
| `safety_diagnostics.py` | Safety event/status diagnostics. |
| `safety_health.py` | Health-state and degraded-mode calculations. |
| `safety_metrics.py` | Safety/performance metric collection. |
| `scenario.py` | Scenario schema, setup and execution support. |
| `scheduler.py` | Deterministic update scheduling for simulation components. |
| `sensor_suite.py` | Composed simulated sensor suite. |
| `sensors.py` | Individual sensor models and noise/fault behavior. |
| `sim_studio.py` | Simulation studio/session orchestration. |
| `studio_performance.py` | Studio performance instrumentation and reporting. |
| `studio_scenario.py` | Studio-specific scenario handling. |
| `thermal.py` | Thermal behavior model. |
| `tires.py` | Tire force/slip model. |
| `track.py` | Track representation and sampling. |
| `track_formats.py` | Track import/export format converters. |
| `types.py` | Shared simulator records/enums. |
| `vcu.py` | Simulated vehicle-control-unit state machine and command limits. |
| `vehicle.py` | Vehicle composition/state/update model. |
| `watchdog.py` | Simulated watchdog timers and timeout behavior. |
| `world_adapter.py` | Adapts simulator worlds to backend/rendering interfaces. |

### ROS 2 workspace (`kart/ros2_ws/src`)

| Path / file group | Contents and role |
|---|---|
| `autokart_bringup/package.xml`, `CMakeLists.txt` | ROS package metadata/build setup. |
| `autokart_bringup/config/topics.yaml` | Topic names and bring-up wiring/configuration. |
| `autokart_bringup/launch/cone_track.launch.py` | Cone-track simulation launch. |
| `.../fast_sim.launch.py` | Lightweight simulation launch. |
| `.../full_stack.launch.py` | Intended integrated launch; inspection found it currently starts only the VCU bridge, so its name overstates integration. |
| `.../gazebo_sim.launch.py` | Gazebo simulation launch. |
| `.../skidpad.launch.py` | Skidpad scenario launch. |
| `autokart_control/config/controllers.yaml` | ROS controller parameters; package has metadata/config but no complete deployed control node. |
| `autokart_dashboard_gateway/autokart_dashboard_gateway/core.py` | Dashboard gateway state/contract logic. |
| `.../gateway.py` | ROS-facing gateway node and API/read-only telemetry bridge. |
| `.../test/test_core.py` | Gateway-core contract tests. |
| `autokart_description/urdf/autokart.urdf` | Expanded robot description. |
| `.../autokart.urdf.xacro` | Parameterized robot description source. |
| `autokart_description/rviz/camera_depth.rviz` | RViz display preset for camera/depth streams. |
| `.../cone_perception.rviz` | RViz preset for cone detections. |
| `.../full_autonomy.rviz` | Combined autonomy visualization layout. |
| `.../lidar.rviz` | Lidar visualization layout. |
| `.../localization.rviz` | Localization/pose visualization layout. |
| `.../planning.rviz` | Planning/path visualization layout. |
| `.../safety.rviz` | Safety/status visualization layout. |
| `.../tf.rviz` | Transform-tree visualization layout. |
| `.../vehicle_state.rviz` | Vehicle state visualization layout. |
| `autokart_gazebo/config/bridge.yaml` | Gazebo↔ROS topic bridge configuration. |
| `.../src/gazebo_follower_plugin.cpp` | Gazebo plugin implementing follower behavior. |
| `.../src/gazebo_sensor_delivery_plugin.cpp` | Gazebo plugin publishing sensor delivery metadata/behavior. |
| `.../test/test_gazebo_follower_contract.py` | Source/contract test for follower behavior. |
| `autokart_gazebo/models/README.md`, `worlds/README.md` | Model/world asset notes; referenced full assets are not established by these notes alone. |
| `autokart_msgs/msg/*.msg` | Typed ROS contracts: `ActuatorFeedback`, `AuthoritativeVehicleSnapshot`, `Cone`, `ConeArray`, `GazeboFollowerState`, `MissionDebug`, `MotionCommand`, `SafetyEnvelope`, `SensorDeliveryMetadata`, `VcuState`, `VehicleState`, `WheelState`, and `WheelStates`. These define message shapes, not publishers/subscribers or a functioning stack. |
| `autokart_sim_bridge/autokart_sim_bridge/gazebo_full_lap.py` | ROS executable/helper for a Gazebo full-lap scenario. |
| `.../gazebo_sync_probe.py` | Synchronization/availability probe for the Gazebo path. |
| `.../vcu_bridge.py` | VCU command/state bridge; current implementation stores command in memory and does not forward to an ESP32. |
| `.../test/test_bridge_contract.py` | Sim-bridge contract tests. |
| `autokart_diagnostics`, `autokart_localization`, `autokart_perception`, `autokart_planning`, `autokart_safety_bridge`, `autokart_scenarios` package manifests/build files | Package shells and metadata. The inspected workspace did not contain complete production nodes for these named functions. `autokart_scenarios/config/scenario_contract.yaml` specifies scenario interface expectations. |

### Simulator configuration files (`kart/configs`)

These YAML files are inputs to simulation, test, and HIL tooling. They document intended settings and test scenarios; they do not prove hardware configuration or test execution.

| File(s) | Purpose |
|---|---|
| `autonomy/ros2_sil.yaml` | ROS software-in-the-loop setup. |
| `backends/chrono.yaml` | Chrono physics backend selection/settings. |
| `benchmarks/critical_smoke.yaml` | Small critical-path simulator smoke benchmark. |
| `calibration/dataset_schema.yaml` | Dataset/calibration record schema. |
| `controllers/baseline_racing.yaml` | Baseline racing controller parameters. |
| `events/acceleration.yaml`, `autocross.yaml`, `emergency_brake_test.yaml`, `skidpad.yaml`, `trackdrive.yaml` | Event-level scenario and rule/config inputs for acceleration, autocross, emergency braking, skidpad and trackdrive. |
| `hardware/ci_headless.yaml`, `cpu_only_software_render.yaml`, `development_workstation.yaml`, `jetson_orin_nano_super.yaml` | Hardware/rendering profiles for CI, CPU-only rendering, workstation and Jetson-targeted simulation. |
| `hil/can_protocol.yaml` | CAN HIL protocol/config contract. |
| `karts/prototype.yaml`, `prototype_dynamic.yaml`, `prototype_four_wheel.yaml` | Vehicle parameter sets for basic, dynamic and four-wheel prototype models. |
| `presets/engineering_chrono.yaml`, `engineering_internal.yaml` | Engineering configurations for Chrono and internal simulator use. |
| `presets/fast_f1tenth_style.yaml` | Faster simplified F1TENTH-style simulation preset. |
| `presets/fault_validation.yaml` | Fault-injection/validation preset. |
| `presets/formula_student_autocross.yaml`, `formula_student_skidpad.yaml` | Formula Student event presets. |
| `presets/gazebo_engineering_balanced.yaml`, `gazebo_high_fidelity_offline.yaml`, `gazebo_realtime_sil.yaml`, `gazebo_sensor_development.yaml` | Gazebo presets for balanced engineering, offline fidelity, real-time SIL and sensor development. |
| `presets/hil_vcu.yaml` | Intended VCU hardware-in-the-loop preset; not proof of a live connection. |
| `presets/minimal_ci.yaml` | Minimal CI simulation preset. |
| `presets/multi_kart_race.yaml` | Multi-kart scenario preset. |
| `presets/sil_jetson.yaml` | Jetson-oriented software-in-the-loop preset. |
| `safety/gazebo_follower_acceptance.yaml`, `prototype_safety.yaml` | Safety limits/acceptance inputs for Gazebo follower and prototype model. |
| `scenarios/dashboard_research_lap.yaml`, `dashboard_slalom.yaml` | Dashboard demo/research lap and slalom scenes. |
| `scenarios/dynamic_braking.yaml`, `empty.yaml`, `four_wheel_skidpad.yaml`, `gazebo_follower_full_lap.yaml`, `gazebo_realtime_sil_full_lap.yaml`, `straight_acceleration.yaml` | Dynamic braking, empty fixture, four-wheel skidpad, Gazebo follower/full-lap, real-time SIL full-lap and straight acceleration scenarios. |
| `scenarios/vcu_battery_undervoltage.yaml`, `vcu_bms_shutdown.yaml` | Battery undervoltage and BMS shutdown fault scenarios. |
| `scenarios/vcu_brake_cable_break.yaml`, `vcu_brake_fade.yaml`, `vcu_brake_partial_cable.yaml`, `vcu_brake_partial_nominal.yaml`, `vcu_brake_position_sensor_fault.yaml`, `vcu_brake_power_loss.yaml`, `vcu_brake_propulsion_conflict.yaml`, `vcu_brake_release_failure.yaml`, `vcu_brake_stall.yaml`, `vcu_hydraulic_leak.yaml`, `vcu_pressure_sensor_fault.yaml` | Brake system degradation, sensor, power and conflicting propulsion scenarios. |
| `scenarios/vcu_chain_break.yaml`, `vcu_command_timeout.yaml`, `vcu_controller_reset.yaml`, `vcu_controller_overtemperature.yaml`, `vcu_safety_task_freeze.yaml`, `vcu_stop_timeout.yaml`, `vcu_task_freeze.yaml` | Communication, timeout, reset, thermal and frozen-task scenarios. |
| `scenarios/vcu_controlled_stop_corner.yaml`, `vcu_controlled_stop_nominal.yaml`, `vcu_dry_surface_stop.yaml`, `vcu_split_friction_stop.yaml`, `vcu_wet_surface_stop.yaml` | Controlled-stop behavior under nominal, cornering and surface/friction conditions. |
| `scenarios/vcu_current_sensor_bias.yaml`, `vcu_false_zero_stop.yaml` | Current-sensor bias and false-zero fault scenarios. |
| `scenarios/vcu_estop_stuck_torque.yaml`, `vcu_physical_estop.yaml`, `vcu_physical_estop_acceleration.yaml`, `vcu_physical_estop_at_rest.yaml`, `vcu_physical_estop_corner.yaml`, `vcu_remote_estop.yaml` | Simulated E-stop cases across operating states, plus stuck-torque and remote-stop cases. These YAMLs are not physical E-stop test evidence. |
| `scenarios/vcu_grade_hold.yaml` | Grade holding scenario. |
| `scenarios/vcu_high_speed_steering_jam.yaml`, `vcu_road_wheel_jam.yaml`, `vcu_steering_jam.yaml`, `vcu_steering_mechanical_limit.yaml`, `vcu_steering_offset_jump.yaml`, `vcu_steering_power_loss.yaml`, `vcu_steering_primary_freeze.yaml`, `vcu_steering_runaway.yaml`, `vcu_steering_sensor_drift.yaml` | Steering/road-wheel jam, runaway, sensor, power and mechanical-limit faults. |
| `scenarios/vcu_motor_overtemperature.yaml`, `vcu_motor_speed_sensor_loss.yaml`, `vcu_motor_stuck_torque.yaml`, `vcu_stuck_torque_welded_contactor.yaml`, `vcu_stuck_torque_wet.yaml`, `vcu_stuck_torque_wheel_spin.yaml`, `vcu_torque_loss.yaml`, `vcu_wrong_torque_sign.yaml` | Motor/torque faults and dangerous stuck-torque conditions. |
| `sensors/planned_kart.yaml` | Planned sensor suite parameters. |
| `studio/cone_track_demo.yaml` | Simulation Studio cone-track demo. |
| `tracks/acceleration_event.yaml`, `cone_track.yaml`, `phase16_soak_circle.yaml`, `research_circle.yaml`, `skidpad_event.yaml`, `straight_split_friction.yaml` | Track geometry/environment inputs for acceleration, cone course, Phase 16 circle, research circle, skidpad and split-friction straight. |
| `world/carla_unreal.yaml` | CARLA/Unreal world backend settings. |

### Browser simulator (`phase_sim/src`)

| File | What it contains / why it exists |
|---|---|
| `App.tsx`, `main.tsx`, `styles.css` | React app root, browser mount point and global styles. |
| `engine/avoidance.ts` | Obstacle avoidance candidate generation/scoring. |
| `engine/controller.ts` | Simulated steering/speed control calculations. |
| `engine/course.ts` | Course/track and cone generation. |
| `engine/engine.ts` | Simulation tick and state update orchestration. |
| `engine/math.ts` | Shared geometry/math utilities. |
| `engine/planner.ts` | Simulated route/path planning. |
| `engine/sensor.ts` | Synthetic sensor observation generation. |
| `engine/types.ts` | Engine state and typed contracts. |
| `engine/vehicle.ts` | Simulated vehicle state/dynamics. |
| `live/LiveMissionControl.tsx`, `live/liveMissionControl.css` | Separate live telemetry display view and its styling; display capability is not evidence of a live ROS/planner chain. |
| `mission/LabSkidpad.ts` | Skidpad geometry and simulation logic. |
| `mission/MissionControl.tsx`, `MissionScene.tsx`, `mission.css` | Mission UI, scene composition and styling. |
| `mission/demo.ts` | Demo data/fixtures. |
| `mission/labAlgorithms.ts` | LAB algorithm implementations. |
| `mission/labScenarios.ts` | LAB scenario definitions. |
| `mission/replay.ts` | Replay input/state handling. |
| `mission/schema.ts` | Mission/replay data schema. |
| `mission/useMissionData.ts` | React hook that loads/selects mission data. |
| `shared/SharedLab.tsx`, `shared/sharedLab.css` | Shared LAB view components and styles. |
| `ui/LivePanel.tsx` | Live-status panel. |
| `ui/MethodViews.tsx` | Algorithm/method comparison view. |
| `ui/SimulationCanvas.tsx` | Canvas/Three.js rendering surface for the simulator. |
| `ui/format.ts` | Display formatting helpers. |
| `test/*.test.ts`, `*.test.tsx`, `setup.ts`, `testUtils.ts` | Unit/integration checks for engine, LAB, algorithms, live UI, mission, replay, skidpad geometry and UI consistency; these run against software/synthetic state. |
| `vite-env.d.ts` | Vite/TypeScript environment declarations. |

## What “every file” means in this report

The workspace also contains historical camera/team snapshots, dozens of event/config files, documentation, test fixtures and large non-text datasets. Snapshot trees repeat much of the code above with divergent revisions; they are listed as snapshots in the repository table rather than falsely treated as live modules. Binary model weights, camera recordings, venvs, `__pycache__`, `.pytest_cache`, `node_modules`, ROS build/install/log trees and generated dashboard bundles were not individually reverse-engineered. The named active source trees and configuration families are cataloged here; data, model, image and report artifacts relevant to the audit are inventoried in `04_TEST_EVIDENCE.md`. This boundary is deliberate and is not a claim that every binary artifact was opened or validated.
