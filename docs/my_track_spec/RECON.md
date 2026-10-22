# RECON — my_track feasibility audit

Branch: `feat/my-track`. This is a read-only audit; no simulation/control/firmware code was changed to produce it. All paths are absolute from the repo root (`/home/mark1/Desktop/autonomous_kart`) unless noted.

## TL;DR

The repo has **two parallel simulation/control stacks**, not one, plus a **third, already-written but unintegrated draft** that appears to anticipate this exact task:

- **Layer A — `kart/kart_sim/`**: a generic Formula-Student-style engineering simulator. This is where `skidpad`/`braking`/`autocross` scenario *registration* and the actual vehicle dynamics model live.
- **Layer B — `autokart_ros_nodes/` + `kart/ros2_ws/src/autokart_nodes/` + `autonomy_stack_phase1/phase_01_perception/`**: the real ROS2 mission/perception/planner/controller stack that would drive the physical vehicle. Only `SKIDPAD` and `BRAKING_TEST` exist here as named missions — **there is no `autocross` mission in Layer B**.
- **Layer C — `docs/my_track_spec/MY_TRACK_SIM.py`**: a standalone 454-line draft, already written, that implements a full my_track SIL simulation (kart profiles, HP60C `CamCfg`, gates, centreline, obstacle classifier, detour logic, mission FSM). It imports nothing from `kart/` or `autokart_*` — it is not wired into either real layer. **This needs a decision from you**: is it the intended starting point to port behaviors out of (as Prompt 4 assumes), or a scratch reference to discard?

Also found and worth flagging before any further work:
- The `~/Desktop/kart stl` folder (note: literal trailing space in the real path, `kart stl `) contains an open-source **1:5-scale RC hobby "OpenRC GoKart" model**, not the real vehicle's CAD. Its chassis bounding box is roughly 330×160×330 mm — about 1/7 the real kart's 2.400 m length. **Do not use these STLs as the real vehicle's visual/collision model** without telling you first.
- There is **no dimension-sheet PDF** in that STL folder — the one PDF present is a Printables/Thingiverse project listing for the RC model, unrelated to the real kart's measurements.
- The real production geometry file (`kart/ros2_ws/src/autokart_bringup/config/vehicle_geometry.yaml`) has **zero** wheelbase/track/mass/length/width/height/clearance fields — only camera and bumper geometry. Vehicle dynamics parameters exist only in Layer A, and they're placeholder values that match **neither** dimension sheet (see §8).

---

## 1. Scenario registration

**Layer A (`kart/kart_sim`)** — no single registry; three places must be touched together:
- Scenario config: Pydantic `ScenarioConfig` at [kart/kart_sim/scenario.py:29](../../kart/kart_sim/scenario.py#L29) (name, kart_config, duration_s, command, initial_state, control, track_config, command_schedule). Run by `run_scenario()` at [scenario.py:42](../../kart/kart_sim/scenario.py#L42), which dispatches to `PlanarBicycleKart`/`FourWheelKart`/`DynamicFourWheelKart` based on `configuration.simulation.vehicle_backend`, and writes `telemetry.jsonl` ([scenario.py:287](../../kart/kart_sim/scenario.py#L287)) and `metadata.json` ([scenario.py:300](../../kart/kart_sim/scenario.py#L300)).
- "Event" format (higher level, different shape): flat YAML under `kart/configs/events/{skidpad,autocross,emergency_brake_test,acceleration,trackdrive}.yaml`. Parsed by `EventType` enum + `load_event()` in [kart/kart_sim/formula_student.py:16-45](../../kart/kart_sim/formula_student.py#L16). A fifth `EventType` member (`MY_TRACK`) would follow this pattern.
- Launch script: [kart/scripts/run_formula_student_event.sh](../../kart/scripts/run_formula_student_event.sh) — bash `case` mapping scenario name to a track config + `kart_sim.cli fast` invocation. Needs a new `case` arm.
- CLI: [kart/kart_sim/cli.py:33-71](../../kart/kart_sim/cli.py#L33) — argparse subcommands (validate/run/test-model/vcu/ros2/dashboard); scenarios are referenced by YAML path, not a registry object.

**Layer B (real ROS2 mission stack)** — only `SKIDPAD` and `BRAKING_TEST` exist:
- Mission type is a plain ROS string param, validated in [kart/ros2_ws/src/autokart_nodes/autokart_planner/autokart_planner/mission.py:120-121](../../kart/ros2_ws/src/autokart_nodes/autokart_planner/autokart_planner/mission.py#L120) — `if mission not in {"SKIDPAD", "BRAKING_TEST"}: raise`.
- Dispatch in `PlannerEngine.evaluate()` ([mission.py:165-181](../../kart/ros2_ws/src/autokart_nodes/autokart_planner/autokart_planner/mission.py#L165)) to `_skidpad()` ([mission.py:183](../../kart/ros2_ws/src/autokart_nodes/autokart_planner/autokart_planner/mission.py#L183)) or `_braking()` ([mission.py:347](../../kart/ros2_ws/src/autokart_nodes/autokart_planner/autokart_planner/mission.py#L347)).
- Config files: `autokart_ros_nodes/autokart_mission/config/mission_{skidpad,braking_test}.yaml`.
- Launch files per scenario: `kart/ros2_ws/src/autokart_bringup/launch/{skidpad,cone_track,fast_sim,gazebo_sim,full_stack,phase1_vehicle}.launch.py`. `skidpad.launch.py` is literally a 1-line shell-out to Layer A's script — the two layers are coupled only via that shell script, not a shared registry.
- A `my_track` mission here = add a `_my_track()` method parallel to `_skidpad`/`_braking`, extend the mission set at mission.py:120, add `mission_my_track.yaml`, add `my_track.launch.py`.

**Implication for Prompt 3**: "register my_track next to the others" means touching Layer A (enum + shell case + event YAML) at minimum. Whether it also needs to exist in Layer B depends on whether you want it runnable against the real mission FSM, not just the generic SIL simulator — recommend confirming scope before starting.

## 2. Vehicle dynamics model

Real production code (`autokart_vehicle_geometry`) has **no dynamics model** — only static camera/bumper geometry (see §8). All actual dynamics/integration code is Layer A:

- [kart/kart_sim/vehicle.py:27](../../kart/kart_sim/vehicle.py#L27) — `PlanarBicycleKart`: single-track bicycle model, force saturation, **semi-implicit Euler** integration ([vehicle.py:74-80](../../kart/kart_sim/vehicle.py#L74)). `dt` guard: `0 < dt <= 0.02` (vehicle.py:41-44).
- State vector: `PlanarKartState` ([kart/kart_sim/types.py:32](../../kart/kart_sim/types.py#L32)) — time_s, x_m, y_m, yaw_rad, u_mps, v_mps, yaw_rate_radps.
- CG geometry derived from wheelbase: `a_m = wheelbase/2 - cg.x_m`, `b_m = wheelbase - a_m` (vehicle.py:35-38); static axle loads vehicle.py:54-55.
- Two other backends with the same config shape exist but weren't read in depth: `kart/kart_sim/four_wheel.py` (`FourWheelKart`), `kart/kart_sim/dynamic_kart.py` (`DynamicFourWheelKart`), selected by `configuration.simulation.vehicle_backend`.
- Tyre model: `linear_friction_circle` (name only, in config) — cornering stiffness clipped by a friction circle, not full Pacejka/Fiala. Implementation in `kart/kart_sim/tires.py`, not read in detail.
- **All current parameters** (`kart/configs/karts/{prototype,prototype_dynamic,prototype_four_wheel}.yaml` — near-identical, each explicitly headed "SIMULATION USE ONLY — NOT APPROVED FOR DIRECT VEHICLE DEPLOYMENT... provisional pending physical measurement"):
  - mass_kg 145.0, wheelbase_m **1.15**, front_track_m **0.95**, rear_track_m **0.92**
  - cg {x_m 0.05, y_m 0.0, z_m 0.30}, inertia.yaw_kg_m2 35.0
  - max_road_wheel_angle_rad 0.5585 (32°)
  - tyre: radius_m 0.14, width_m 0.10, friction_peak 1.0, cornering_stiffness_n_per_rad 8500.0, longitudinal_stiffness_n 9000.0
  - motor.max_wheel_force_n 850.0 — **no steering actuator (rate/lag) model**
  - brake.max_brake_force_n 1300.0 — **no brake dead-time model**
  - rolling_resistance_coefficient 0.015, air_density 1.225, drag_coefficient_area_m2 0.55
  - simulation.physics_dt_s 0.001, seed 20260729

**Conclusion**: this is a dynamic bicycle model with tyre cornering stiffness + friction-circle saturation (not purely kinematic), but Layer A has no steering-actuator or brake-actuator dynamics — commands act as forces/angles directly. The only place a steering-rate/lag + brake-deadtime model exists at all is the unintegrated Layer C draft (`steer_rate`, `steer_tau`, `brake_deadtime`, `brake_rate`, MY_TRACK_SIM.py lines ~70-76).

**No multi-profile support exists in either real layer.** See §8.

## 3. HP60C perception model vs. datasheet (FOV 64.4°×51.7°, range 0.2–4 m, ~4.2 Hz, ~0.29 s latency)

**Real codebase has no simulated camera model at all.** `autonomy_stack_phase1/phase_01_perception/src/autokart_perception/camera.py:18-102` (`class HP60C`) is a ctypes wrapper over a native SDK bridge (`hp_start`/`hp_connect`/`hp_read`/`hp_stop`) talking to real hardware — FOV/range/rate/latency are all *measured at runtime*, not configured constants. `synthetic.py:1-23` (`class Synthetic`) is a trivial interface stub (fixed 4000 mm depth plane, two painted cone silhouettes), no FOV/noise/recall model.

Measured-metric locations (not sim parameters): `live_mission.py:158` (`NON_LIVE_STALE_MS = 1000.0`), `live_mission.py:266/354` (measured_fps from real timestamps), `live_mission.py:452` (`last_latency_ms`).

**Discrepancy worth flagging**: [CHANGELOG.md:40-42](../../CHANGELOG.md#L40) states the best isolated-capture ceiling measured so far is ~7.2 Hz (target 9 Hz), and that the live after-fix paired RGB+depth rate hasn't been re-measured. That conflicts with the task's "~4.2 Hz" figure — likely two different measurement conditions (full pipeline vs. isolated capture), but worth confirming which number is current before building a sim perception model around it.

`kart/ros2_ws/src/autokart_bringup/config/vehicle_geometry.yaml:18-19` records `camera_height_m: 0.8382` (33 in), `camera_pitch_down_deg: 17.8` (marked **DISPUTED** per its own comment pointing at `docs/HANDOVER_STATUS.md`) — mount geometry only, not FOV/range.

**The only place FOV/range/rate/latency/noise/recall are modeled as sim parameters** is the Layer C draft's `CamCfg` dataclass (MY_TRACK_SIM.py lines 96-107):
- hfov_deg 64.4 — **matches** datasheet. No independent vfov parameter — vertical FOV is implicit via fx/aspect ratio, not an explicit 51.7° value. Gap vs. datasheet's explicit V-FOV.
- zmin/zmax 0.20/4.0 — **matches** datasheet.
- rate_hz 4.2 ("measured full-pipeline output, 10-min diagnostic") — matches the task's figure, conflicts with CHANGELOG's ~7.2 Hz isolated figure as above.
- latency 0.29 ("measured frame age p50 286 ms") — matches task figure.
- height/pitch_deg **0.30 m / 4.0°** (explicit comment: "do not mix with 0.18 m lab value") — **mismatches** the real `vehicle_geometry.yaml` (0.8382 m / 17.8°). If this draft is reused, its mount geometry needs correcting to the real file, not left as placeholder.
- recall_peak/px_full/px_min, label_swap_p, sig_r0/r1/l0/l1 — all marked ASSUMED, matches the task's intent to keep unmeasured values tagged.

## 4. Algorithm chain (cone map, Delaunay, Gabriel, gates, centreline, obstacles, finish line)

Canonical source per the code's own docstrings (and `docs/decisions/CANONICAL_COMPONENTS.md`): `autonomy_stack_phase1/phase_01_perception/src/autokart_perception/{application,shared_core,geometry,gabriel,gates,boundaries,centerline,curvature,safety,types,sdk}.py`.

The ROS adapter `autokart_ros_nodes/autokart_mapping/autokart_mapping/algorithm.py:1-13` explicitly documents that it "never vendors or reimplements Delaunay triangulation, the Gabriel graph filter, gate selection, centerline/curvature fitting" — it imports `application.CoreRuntime`, `shared_core.AlgorithmCore`/`fit_skidpad_topology`/`PoseEstimate`, `types.Cone` from the path above.

**Not yet verified** (follow-up before any reuse decision): the actual math inside `geometry.py`, `gabriel.py`, `gates.py`, `boundaries.py`, `centerline.py`, `curvature.py`, `shared_core.py` — existence confirmed, bodies not read. Finish-line detection specifically was not confirmed present or absent in this canonical chain (search budget ran out before a targeted grep for "finish").

**Layer C draft implements a full standalone version** (MY_TRACK_SIM.py ~lines 340-480), not reusing `autokart_perception`:
- Obstacle classifier: flood-fill connected components on orange cones, ≤0.85 m neighbor link, cluster is "obstacle" only if ≥4 cones *and* lateral range ≥0.28 m, else "boundary" (lines 367-376).
- Delaunay via `scipy.spatial.Delaunay` (line 382); gates from edges between opposite-side boundary cones filtered by width (2.0–3.8 m) and skew (≤45°) (lines 384-393).
- Single-side fallback synthetic gates when only one boundary is visible (lines 394-405).
- Centreline via IRLS polynomial fit, degree 1–3 by point count, 5 iterations, Tukey-like weighting (lines 408-417).
- Finish line: same-colour (orange) pair 0.2–0.6 m apart, gated on >40 m odometry progress (lines 355-366).
- Detour: ramped lateral offset with swept-footprint collision check (lines 441-462).
- **Gabriel graph**: mentioned only in the module docstring ("Gabriel = evidence only," line 11) — no actual Gabriel-graph computation found in this draft. Worth confirming with you whether it's elsewhere or was dropped from this file.

**This is the single highest-priority read before Prompt 4** — need to actually read the canonical perception module bodies to know what to reuse vs. port from the draft.

## 5. Controller (Pure Pursuit, speed control, braking, mission FSM)

Fully implemented and cross-consistent between two copies:
- `autokart_ros_nodes/autokart_controller/autokart_controller/pursuit_backends.py` — `BuiltinPursuitBackend` (lines 78-156), `ReusedPursuitBackend` (lines 159-241, wraps `kart_sim.autonomy.PurePursuitController`/`LongitudinalPidController`).
- Canonical original: `kart/kart_sim/autonomy.py` — `PurePursuitController` (line 199), `LongitudinalPidController` (line 270).
- **Lookahead**: `lookahead = max(0.1, lookahead_m + speed_lookahead_gain_s * max(0, u_mps))` (pursuit_backends.py:118 / autonomy.py:216) — **lower-clamped only, no upper clamp**, and the code's own docstring (pursuit_backends.py:28-41) flags this as a discrepancy against `CONTRACT.md`, which allegedly documents a `clamp(L0+kv|v|, Lmin, Lmax)` form. Worth reconciling if `my_track`'s controller needs an upper lookahead bound (tight bends).
- **Steering**: `requested = atan2(2*wheelbase_m*y_local, distance**2)` (pursuit_backends.py:124-131 / autonomy.py:219-229) — curvature form, algebraically equivalent to the textbook `atan(2L sinα / Ld)` when `distance == Ld`. Rate-limited by `maximum_steering_rate_radps * dt_s`, clamped to `±maximum_steering_rad`.
- Default gains: `lookahead_m=1.5`, `speed_lookahead_gain_s=0.15`, `maximum_steering_rad=0.6` (≈34.4°), `maximum_steering_rate_radps=3.0` (pursuit_backends.py:80-88). Note the task's software steering clamp of 20° is *not* the current default (34.4°) — needs an explicit override for my_track if 20° is the intended limit.
- **Speed control**: full PID (`kp=0.35, ki=0.12, kd=0.0`), conditional/anti-windup integration (pursuit_backends.py:136-146), output clamped [-1,1] then split into drive=max(0,out)/brake=max(0,-out) (lines 147-148).
- **Braking distance formula**: `v*t_latency + v^2/(2a) + margin`, referenced via `BrakeCapability.stopping_distance()` import from `shared_core.py` (used at mission.py:246, mission.py:352 docstring) — formula confirmed by docstring reference only, body not read directly.
- **Mission FSM states found**:
  - SKIDPAD (`mission.py:31-111`): ENTRY→RIGHT_1→RIGHT_2→TRANSITION→LEFT_1→LEFT_2→EXIT→STOP_PENDING→STOP, with a HOLD fault-sink reachable from WRONG_LOBE_OR_OFF_TRACK / WRONG_DIRECTION_OR_POSE_JUMP / PREMATURE_OR_DUPLICATE_GATE / NONMONOTONE_MISSION_POSE (mission.py:72-96).
  - BRAKING_TEST (mission.py:146, 326-345): WAITING→TRACK_VALID→READY→RUNNING→STOP_TRIGGERED→STOPPING→STOP_VERIFIED (only reachable with real measured-stop feedback, never from zero commanded speed alone), plus FAULT sink (mission.py:159).
  - Layer C draft's FSM (MY_TRACK_SIM.py:220): READY→RUN→{OBSTACLE_ASSESS, DETOUR, RECENTER, FINISH_BRAKING, CONTROLLED_STOP, PATH_LOST}→STOPPED, PATH_LOST>2 s forces CONTROLLED_STOP — this already matches the states Prompt 4 asks for almost exactly.

## 6. Existing logging/telemetry and UI

- **Layer A**: `run_scenario()` writes `telemetry.jsonl` (one JSON record/line, scenario.py:287-289) and `metadata.json` (scenario.py:300) into `runs/`. Exact per-record field list not enumerated (follow-up).
- `kart_sim dashboard` CLI (cli.py:55-71): `inspect` / `export` / `serve` / `launch` subcommands — Layer A already has its own dashboard tooling, separate from `phase_sim`.
- `kart/dashboard/` — plain JS/CSS/HTML dashboard (index.html, css/dashboard.css, js/{app,websocket,state-store}.js), presumably serving the `dashboard export/serve` CLI output.
- **`phase_sim/`** — a Vite/React/TypeScript app, package name `live-delaunay-kart-lab` (phase_sim/package.json:2), separate dashboard/lab build targets. Key files: `src/mission/{MissionControl.tsx, MissionControlLive.tsx, MissionScene.tsx, RosLivePanel.tsx, LabSkidpad.ts, labAlgorithms.ts, labScenarios.ts, replay.ts, schema.ts, useMissionData*.ts, demo.ts}`, `src/live/LiveMissionControl.tsx`, `src/ui/{LivePanel.tsx, MethodViews.tsx, SimulationCanvas.tsx}`. Extensive test coverage under `src/test/`.
- **Data contract**: [docs/DASHBOARD_DATA_CONTRACT.md:1-8](../DASHBOARD_DATA_CONTRACT.md#L1) — explicitly a read-only diagnostic display; SIMULATION/LAB/REPLAY modes never cross-populate; browser publishes no vehicle commands. Backend is `autokart_dashboard_gateway`, serving `/mission/ws` and `/mission/api/{manifest,replay,status,camera,camera/{frame_id}/{rgb|depth}}`. Camera status fields include source, capture time, frame age, received FPS, missing-sequence count, capture-to-gateway delay, detection metadata. Depth preview is explicitly unqualified (`depth_qualified=false`).

**Implication for Prompt 4/7**: a 4th scenario plugs into this existing dashboard contract the same way SKIDPAD/BRAKING_TEST do — the contract is schema-driven, not scenario-specific, so a new telemetry pipe is likely unnecessary. `phase_sim` is the natural home for "one viewer for all four scenarios" rather than building something new.

## 7. Kart STL files

**Path has a literal trailing space**: `/home/mark1/Desktop/kart stl ` (confirmed via `ls -la ~/Desktop/`). All files binary STL.

- `Rear Axle/` — 9 files (OpenRC-GoKart-* parts, axle holders, motor spacer), 456 KB–5.6 MB.
- `Chassis/` — 11 files. `OpenRC-GoKart_Chassis_ONEPIECE_085.stl`: 2.2 MB, 43,748 triangles, bounding box (treating units as mm) ≈ 160×4×330 mm.
- `Body/` — 5 files, 43 KB–5.4 MB.
- `Steering/` — 12 files, 16 KB–651 KB.
- `Engine/` — 7 files, several very large (Vergaser.stl 125 MB, grundkorper_2.stl 145 MB, Zylinder.stl 54.5 MB, krummer-v4.stl 15.7 MB, Zündkerzenstecker.stl 25 MB) — a nitro engine model.
- One PDF: `56062-openrc-gokart-...pdf` (316 KB) — **not a dimension sheet**. `pdftotext` shows it's a Printables/Thingiverse project listing for "OpenRC GoKart" by Christian Erazo, a hobbyist RC project page.

**Identification**: filenames and German part names (Zündkerze=spark plug, Zylinderkopf=cylinder head, Vergaser=carburetor) identify this as the open-source **"OpenRC GoKart" 1:5-scale 3D-printable RC hobby model** with a nitro engine — not CAD of the real full-size vehicle. The one computed bounding box (~330×160×330 mm) is roughly 1/7 the real kart's 2.400 m length.

**Action needed from you before Prompt 6**: these STLs cannot stand in for the real vehicle's visual/collision model. Per the ground rules ("report any mismatch instead of rescaling"), I'm stopping here rather than rescaling a 1:5 RC model to fake the real dimensions. If no real CAD exists, Prompt 7's "exact kart STL silhouette" will need to fall back to the plain footprint-rectangle option it already allows.

## 8. Dimension mismatches

- The **only** place the real dimension-sheet numbers (wheelbase 1.422 m, track F 1.0648 m, track R 1.1811 m, length 2.400 m, width 1.181 m, height 1.118 m, ground clearance 0.381 m) appear verbatim is the Layer C draft's `PROFILES["shared"]` (MY_TRACK_SIM.py ~line 85). The same draft **already has the "built" profile too**: wheelbase 1.1176 m, front track 1.0668 m, rear track 1.1811 m (same L/W/H/clearance as shared) — i.e. someone already encoded both profiles from this task into that draft before this audit started.
- The real production geometry file (`kart/ros2_ws/src/autokart_bringup/config/vehicle_geometry.yaml`, enforced as single-source-of-truth by its own header comment and `kart/ros2_ws/src/autokart_vehicle_geometry/test/test_vehicle_geometry.py`) has **no wheelbase, track width, mass, length/width/height, or ground clearance at all** — only camera_height_m (0.8382), camera_pitch_down_deg (17.8, disputed), camera_to_front_bumper_horizontal_m (1.0668), bumper_width_m (1.016), stop_gap_from_bumper_m (0.15), lateral_margin_m (0.15).
  - `bumper_width_m: 1.016` (40 in) vs. dimension-sheet width 1.181 m — a ~0.165 m (6.5 in) discrepancy. The line's own comment ("never 39 in / 0.9906 m") shows this value has already been revised once and still may not match the sheet.
- Layer A's only dynamics parameters (`kart/configs/karts/prototype*.yaml`) are explicitly "provisional pending physical measurement" and match **neither** dimension sheet: wheelbase_m 1.15 (vs. 1.422 shared / 1.1176 built), front_track_m 0.95 (vs. 1.0648 / 1.0668), rear_track_m 0.92 (vs. 1.1811 for both).
- **Multi-profile support is confirmed absent from both real layers.** The three `kart/configs/karts/*.yaml` files are three different *simulation backends* for the *same single* placeholder geometry (all three repeat mass_kg 145.0 / wheelbase_m 1.15 / front_track_m 0.95 / rear_track_m 0.92 verbatim) — there's no kart-A-vs-kart-B selector anywhere in Layer A or B. Confirms Prompt 1's premise that this needs to be added.
- Pure Pursuit's `wheelbase_m` in Layer B (pursuit_backends.py:80) is a required constructor argument with no default — where it's actually supplied at runtime (ROS launch param?) was not found within this pass. Follow-up needed before wiring profile switching into the real controller.

---

## What exists / what's missing / risks

| Area | Exists | Missing | Risk |
|---|---|---|---|
| Scenario registration (Layer A) | EventType enum + YAML event configs + shell dispatch | No single registry — 3 places to touch (enum, shell case, event YAML) | Easy to add my_track in one place and miss another |
| Scenario registration (Layer B) | SKIDPAD + BRAKING_TEST only | **No autocross mission in the real stack at all** | The task's "3 existing scenarios" premise only half-holds for Layer B |
| Vehicle dynamics | Dynamic bicycle model, tyre cornering stiffness + friction circle, semi-implicit Euler (Layer A only) | No steering-actuator or brake dead-time model in Layer A; Layer B has no dynamics/wheelbase concept at all | Pure Pursuit's `wheelbase_m` source at runtime still unconfirmed |
| HP60C camera model | Real hardware SDK wrapper (measured) + one complete `CamCfg` in the unintegrated draft | No simulated camera model anywhere in integrated code | Any SIL my_track work must adopt/adapt the draft's CamCfg — none exists in production code |
| Delaunay/Gabriel/gates/centreline | Canonical implementation referenced in `autokart_perception`; full second implementation in the draft | Canonical module bodies not yet read; Gabriel step not confirmed in the draft | High-priority read before any reuse decision for Prompt 4 |
| Pure Pursuit / speed PID / mission FSM | Implemented twice, consistently, in both layers | Braking-distance formula body not read directly | Low risk — well documented already |
| Telemetry/dashboard | JSONL (Layer A) + React/TS `phase_sim` app + ROS dashboard_gateway with a documented contract | Exact telemetry JSONL field list not enumerated | Likely no new telemetry plumbing needed — contract is schema-driven |
| Kart STL files | Present | **Wrong kart** — 1:5-scale RC hobby model, not the real vehicle; no dimension-sheet PDF present | Must not be used for real-vehicle visuals/collision without flagging (done here) |
| Vehicle dimensions | Both profiles already encoded in the unintegrated draft | Real production code has zero vehicle-dimension fields; Layer A placeholders match neither sheet | If my_track needs vehicle-geometry-aware real-stack logic, `vehicle_geometry.yaml`'s schema and its test need extending |

---

## Open questions for you

1. **Is `docs/my_track_spec/MY_TRACK_SIM.py` the intended starting point** to port behaviors from (as Prompt 4 assumes), or just a scratch/reference draft to discard? It already contains both kart profiles, the HP60C `CamCfg`, the full algorithm chain, and a mission FSM that closely matches what Prompts 1-4 ask for — but it's completely disconnected from the rest of the repo.
2. **Scope of "my_track" registration**: Layer A only (generic SIL simulator, like autocross today), or also Layer B (real ROS mission FSM, like skidpad/braking today)? This changes how much needs to change.
3. Where does Layer B's Pure Pursuit `wheelbase_m` actually come from at runtime? Not found within this pass — needed before multi-profile switching can reach the real controller.
4. Does the canonical `autokart_perception` package (`gabriel.py`, `gates.py`, `centerline.py`, `boundaries.py`, `shared_core.py`) actually implement a Gabriel graph and finish-line detection? Existence confirmed, bodies not read yet.
5. Real STL/CAD for the actual vehicle wasn't found anywhere in this pass — confirm there's no other source before Prompt 6 falls back to a plain footprint rectangle for the viewer.
6. `kart/kart_sim/tires.py`, `four_wheel.py`, `dynamic_kart.py` — not read in detail; their force models are unverified beyond config field names.
7. `autokart_bldc test/` (literal space in the directory name) showed up in several greps referencing skidpad/autocross/braking/camera — possibly a third parallel implementation, not explored in this pass.
8. Several other candidate directories (`_autokart_audit`, `_autokart_full_audit`, `_autokart_integration`, `_camera_team_base`, `autokart/`, `autokart_cone_detection_model/`) look like historical snapshots/worktrees per `PROJECT_INDEX.md` and were deliberately skipped as likely redundant — that's an assumption, not verified.

**Spec files still missing**: of the five files you asked to be placed under `docs/my_track_spec/` before Prompt 1, only `MY_TRACK_SIM.py` was found on disk (at `~/Desktop/sim/MY_TRACK_SIM.py`, now copied in). `my_track_layout.json`, `MY_TRACK_shared_10Hz.csv`, `MY_TRACK_built_10Hz.csv`, and `MY_TRACK_SCENARIO_MATRIX.csv` were not found anywhere searched. Prompt 0 didn't need them, but Prompts 1, 3, and 9 do — please locate or regenerate them before we go further.
