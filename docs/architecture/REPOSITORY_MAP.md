# AutoKart repository map (RC-01)

## Repository tree and navigation

```text
autonomous_kart/
├── .gitignore / README.txt / PROJECT_INDEX.md
├── .venv/                                # local Python environment; preserve
├── AutoKart_Camera_Algorithms_Codex_Master_Prompt.md
├── AutoKart_Camera_Quick_Start.txt
├── AutoKart_HP60C_Camera_Geometry_Production_Spec.md
├── AutoKart_S3_Pin_and_Power_Budget_RevA.xlsx
├── AutoKart_RC_Dynamics_ESP32S3_v1.zip / XIAO_C3_RA02_ESTOP_PRODUCTION.zip
├── ESP32S3_RA02_ESTOP_RECEIVER_PRODUCTION.zip
├── AutoKart_RC_Dynamics_ESP32S3_v1/       # firmware candidates; unresolved
├── XIAO_C3_RA02_ESTOP_PRODUCTION/         # E-stop prototype; unresolved
├── autokart_apps/                         # cross-project launch and smoke-test wrappers
├── autonomy_stack_phase1/
│   ├── phase_01_perception/               # active local perception implementation
│   └── _camera_*/                         # linked worktrees registered in its .git metadata
├── kart/
│   ├── kart_sim/                          # Python kart simulation and CLI
│   ├── configs/ tests/ docs/ artifacts/   # scenarios, regression coverage, evidence
│   └── ros2_ws/src/                       # ROS 2 messages, launch, bridge and sim packages
├── phase_sim/                             # React/TypeScript Mission Control and LAB UI
├── bench/                                 # bench procedures and historical firmware
├── models/ datasets/ training_runs/       # model artifacts, data and training evidence
├── _autokart_audit/ _autokart_full_audit/ # dated source audit material
├── _camera_a1/ … _camera_a5/             # linked historical worktrees; approval gated
├── _camera_prod_a1/ … _camera_prod_a5/   # linked historical worktrees; approval gated
├── _camera_team_base/                    # linked historical worktree; approval gated
├── docs/                                  # RC-01 index, map, manifest, audits and runbook
└── AUT05_Laptop_First/                    # laptop preflight/regression kit
```

This is a navigation tree, not a claim that every package is active at runtime. The complete row-by-row inventory and proposed dispositions are in [`../decisions/manifest.csv`](../decisions/manifest.csv); symlink targets are in [`../audits/2026-10-16/SYMLINK_INVENTORY.csv`](../audits/2026-10-16/SYMLINK_INVENTORY.csv).

### Minimal proposed layout

Keep all runtime directories at their present paths. The workspace has no root Git repository, and moving the Python package, ROS packages, firmware projects, model files or dashboard would risk imports, launch paths, service examples and model loaders. Consolidate navigation through `PROJECT_INDEX.md` and this map. If approved, move only the 11 linked `_camera_*` worktree checkouts to the proposed external archive location using Git-aware worktree operations; preserve their branches and working-tree contents. Keep generated TypeScript cache files in place until their individual removal is approved. No source implementation or algorithm rewrite is proposed.

## Authoritative component candidates

“Authoritative” here means the current local implementation to use as the repository reference for this freeze; it does not imply field qualification or production approval.

| Subsystem | Current local reference | Evidence and limit |
|---|---|---|
| Perception | `autonomy_stack_phase1/phase_01_perception/` | Current 139-test Python package and diagnostic entry points. Its source is untracked in its owning Git checkout; preserve the working tree and capture it in reviewed source control separately. |
| Camera SDK / depth / geometry | `autonomy_stack_phase1/phase_01_perception/` and its pinned vendor SDK subtree | Current local bridge, depth and geometry candidates. Pi runtime, camera mount, surveyed depth accuracy and physical calibration remain unverified. |
| Shared CORE / mapping / mission | `autonomy_stack_phase1/phase_01_perception/` shared-core modules | Local shared computation and tests are present. They do not establish live vehicle localization or completed physical mission behavior. |
| Kart simulation / Pure Pursuit | `kart/kart_sim/` | Current simulation implementation and tests; simulation is not vehicle validation. |
| ROS 2 interfaces and bring-up | `kart/ros2_ws/src/` | Current package names, message contracts, launch files and simulator bridges. Some packages are metadata/config shells; no complete verified end-to-end vehicle path is established. |
| Mission Control / LAB dashboard | `phase_sim/` with Python API in `autonomy_stack_phase1/phase_01_perception/` | Current frontend and LAB UI, with build/test evidence. LAB values are simulated and `dist/` is consumed by the diagnostic launcher. |
| VCU / actuator firmware | `AutoKart_RC_Dynamics_ESP32S3_v1/` and bench candidates | No single installed image or verified command/feedback path is identified. Keep classified `UNKNOWN_DO_NOT_TOUCH`; no canonical deployed firmware can be named from current evidence. |
| Independent E-stop | `XIAO_C3_RA02_ESTOP_PRODUCTION/` prototype | No verified wiring or physical stop-path evidence. Keep classified `UNKNOWN_DO_NOT_TOUCH`; no production authority is implied by directory name. |

## Source ownership and status

| Path | Role / entry point | Status for integration |
|---|---|---|
| `autokart_apps/` | Workspace launch wrappers. `run_core.sh` starts shared-core evaluation; `run_lab.sh` starts the Python LAB API with `phase_sim`; `run_live.sh` builds the frontend if needed and starts `autokart_perception.live_mission`; `run_tests.sh` is the short cross-project smoke suite. | Active wrapper path; disarmed diagnostics only. |
| `autonomy_stack_phase1/phase_01_perception/` | Main local Phase 1 perception source, SDK bridge, camera, model adapters, depth, geometry, shared core, mission candidate, diagnostic web UI and 139-test suite. `python -m autokart_perception` and `python -m autokart_perception.live_mission` are entry points. | Keep at this path. The source tree is mostly untracked in its Git checkout; the commit alone does not capture the working source. |
| `autonomy_stack_phase1/.git` | Git common directory for `main` plus 11 `_camera_*` linked worktrees/branches. HEAD `87f2e288cb1505a9eb233e077603f19689cc3114`; modified `.gitignore`, `README.md`, `setup.sh`; several untracked source/docs/runs directories. | Preserve, including all branches and untracked files. |
| `kart/kart_sim/` | Python simulation models and `kart-sim` CLI. | Active simulator, not physical vehicle evidence. |
| `kart/ros2_ws/src/` | ROS messages, launch/config, Gazebo plugins, dashboard gateway and simulator bridge. `full_stack.launch.py` currently starts only the VCU bridge; bridge command handling is not a verified ESP32 transport. Several intended node packages contain metadata/config shells rather than complete runtime nodes. | Preserve contracts and tests; do not treat package names as completed node implementations. |
| `kart/ros2_ws/src/autokart_nodes/` | Grouping directory, not a ROS package. It contains `autokart_perception`, `autokart_mapping`, `autokart_planner` and `autokart_controller` adapter packages. | New adapters are package scaffolds only; no executables/topics or command publishers are installed. The old `autokart_planning` and `autokart_control` shells remain for compatibility. |
| `kart/configs/`, `kart/tests/`, `kart/docs/` | Simulation scenarios, fault injection, safety parameters, tests and engineering evidence. | Keep for tests/evidence. |
| `phase_sim/` | React/TypeScript Mission Control and LAB. `npm run dev`, `npm run test -- --run`, `npm run build`. | Active UI; LAB is simulated. Build output is generated and is referenced by the live wrapper for serving. |
| `AutoKart_RC_Dynamics_ESP32S3_v1/` | ESP-IDF RC/bench firmware candidate, nested `AutoKart_ESP_IDF_R86mini_WebStop/`, MicroPython experiments and build binaries. | `UNKNOWN_KEEP`; competing images, backup binaries and installed firmware identity are not reconciled. The `.git` directory is empty/nonfunctional here. |
| `XIAO_C3_RA02_ESTOP_PRODUCTION/` | LoRa E-stop prototype plus checklist and receiver/transmitter code. | `UNKNOWN_KEEP`; name does not establish production safety. Code contains unimplemented/stub behavior per earlier audit. |
| `bench/` | Hardware bench procedures and historical subsystem firmware, with prior versions and pinout/recovery notes in `archive/`. | Keep as test/evidence; compare wiring and firmware before use. |
| `models/`, `training_runs/`, `datasets/`, perception `recordings/` | Model checkpoints/manifests, training output, dataset material, labels and saved frames. | Keep test/evidence and preserve hashes/provenance. |
| `_autokart_audit/` | Earlier 13-report audit and saved logs/measurements/images. | Keep test/evidence; dated history is not overwritten. |
| `_autokart_full_audit/` | Five 2026-10-16 audit reports. | Keep test/evidence; copied to `docs/audits/2026-10-16/`. |
| `_camera_a1`…`_camera_a5`, `_camera_prod_a1`…`_camera_prod_a5`, `_camera_team_base` | Perception Git linked worktrees and branches. Source is duplicated but differs by milestone; some worktrees contain local modifications/untracked tests, reports, caches, model links and UI changes. | Archive candidates only. They remain at their current paths pending explicit path-list approval. |
| `AUT05_Laptop_First/` | Read-only laptop preflight and regression scripts plus implementation notes. | Keep test/evidence; useful operational context. |
| `AutoKart_Camera_Algorithms_Codex_Master_Prompt.md`, `AutoKart_Camera_Quick_Start.txt`, `AutoKart_HP60C_Camera_Geometry_Production_Spec.md`, `AutoKart_S3_Pin_and_Power_Budget_RevA.xlsx`, root ZIPs and `README.txt` | Supplied reference/specification/archive files. | Keep test/evidence; do not replace by derived documentation. |

## Frozen integration responsibilities

These are intended responsibilities from the supplied RC-01 brief; they do not all exist as verified, working ROS nodes.

| Responsibility | Current local evidence | Unresolved integration gap |
|---|---|---|
| Perception | Python HP60C SDK path, replay/synthetic sources, model adapters, depth and geometry candidates. | Pi runtime and surveyed depth/mount accuracy unverified; current live model default differs from the frozen YOLO26s evaluation request. |
| IMU | Proposed MPU-6500 on Pi. | No current verified driver/connection/calibration or live topic identified. |
| VCU bridge | ROS message definitions and simulation bridge; firmware candidates. | No verified bridge to installed ESP32 or measured telemetry path. |
| Odometry | Simulator and shared-core pose interfaces. | Real sensor fusion/pose is not established. |
| Mapping | Shared core stores pose-aware cone observations; simulator has map/course models. | Live map is invalid without qualified pose and metric detections. |
| Mission | Shared-core mission candidates and `phase_sim` LAB workflows. | FKDC right-two-then-left-two winding/gate completion is not proven on the physical skidpad. |
| Planner | Simulator and diagnostic path candidates. | No validated full-loop physical planner output or obstruction stop path. |
| Controller | Simulator Pure Pursuit/other controllers and firmware command candidate. | No end-to-end command authority, measured feedback, speed control or physical brake evidence. |

## Models and hashes observed locally

| Artifact | SHA-256 / status | Interpretation |
|---|---|---|
| `models/yolo26s/best.onnx` | `425a89cef2eda9db9c84fbf30cad187e510510b95d7b7e9904003fd22cec1158`; matches `best.json`. | Frozen YOLO26s evaluation artifact, manifest marks `EXPERIMENTAL_HP60C_VALIDATION_PENDING` and vehicle authorization disabled. |
| `training_runs/conenet_v1_20261014_114447/fit/weights/best.onnx` | `0554f1fb28d19a4defac6b65b33e790ac2398e2f577fe353d01cbe28743aa103`. | `run_live.sh`/`live_mission.py` default model (ConeNet v1), not the frozen YOLO26s selection. |
| `models/conenet_v2/model.onnx` | `80d8c6e84dc312d5e47d8983618253ea6d1cab6f8fb2dd73af1cb00c33f07224`. | Experimental alternative, not the default. |
| `autonomy_stack_phase1/phase_01_perception/models/coneslayer-simplified.onnx` | `67c8dd8922fa793a0577297c8936de1fc5decaebfb7c79c1f9a7aa1542f93faf` (also pinned in source). | Orange-only rollback checkpoint. |

The default of the example systemd unit is also ConeNet v1 because it invokes `run_live.sh` without a `--detector` override. The unit is an example file, was not active on the laptop, and does not establish Pi deployment. Firmware/model production selection remains unresolved.

## Path and Git metadata detail

The full inventory found 55 symlinks; their paths and stored link targets are in [`SYMLINK_INVENTORY.csv`](../audits/2026-10-16/SYMLINK_INVENTORY.csv). Important groups include `phase_01_perception/config/perception.yaml` pointing to package data, shared `phase_sim/node_modules` links from `_camera_a5`, `_camera_prod_a4` and `_camera_team_base`, a model link in `_camera_team_base`, ROS `latest*` links into dated logs, and ESP-IDF-generated source links into `/tmp` SDK trees. They are links, not additional source copies.

There are 12 linked worktrees under `autonomy_stack_phase1/.git`: `main` plus the 11 snapshot branches listed in the disposition report. Snapshot `.git` entries are pointer files into that one Git common directory. `AutoKart_RC_Dynamics_ESP32S3_v1/.git` is an empty directory that Git does not recognize as a repository. The workspace root, `kart/` and `phase_sim/` have no Git metadata.

The example systemd units found are `autokart_apps/autokart-live-diagnostic.service.example` and `autonomy_stack_phase1/phase_01_perception/deployment/autokart-core-diagnostic.service.example`. No matching AutoKart user service was active during the host check. No service or launch file was started during RC-01.

The inspected startup/script groups are `autokart_apps/run_core.sh`, `run_lab.sh`, `run_live.sh`, `run_tests.sh`; `autonomy_stack_phase1/setup.sh` and `run_viewer.sh` plus vendor `run_ascamera*.sh` scripts; `kart/scripts/run_*.sh` for dashboard, fast simulation, fault demo, Formula Student event, Gazebo profiles/follower, multi-kart, Phase 16, realtime soak and ROS SIL; `kart/scripts/setup.sh` and `test.sh`; `kart/ros2_ws/src/autokart_bringup/launch/{cone_track,fast_sim,full_stack,gazebo_sim,skidpad}.launch.py`; and `AUT05_Laptop_First/{laptop_preflight,baseline_tests}.sh`. Their presence identifies entry points, not that they are all active services. The ROS build/test commands used external `/tmp` build paths.
