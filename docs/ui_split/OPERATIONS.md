# AutoKart UI split — Operations

Two separate applications, each with its own backends, started/stopped with one command.
Everything here is a **read-only diagnostic**: vehicle authority is OFF everywhere, and no
process started by these scripts publishes a command/brake/throttle/steer topic
(`scripts/audit_no_actuation.sh` proves it on the running graph).

| App | One command | URL |
|---|---|---|
| Vehicle Dashboard | `scripts/run_dashboard.sh start` | `http://<host>:5180/overview` |
| Algorithm Lab | `scripts/run_lab.sh start` | `http://<host>:5181/home` |

Machine: Jarvis workstation (Ubuntu 22.04 x86_64, ROS 2 Humble). Not validated on the Pi.

## 1. Ports and processes

| Process (pid-file name) | Command (as started by the scripts) | Port | Env override | Health |
|---|---|---|---|---|
| `dash-perception` — HP60C single SDK owner | `python3 -m autokart_perception.live_mission --mode live --camera hp60c --host 127.0.0.1 --port 8768` | 8768 | `AUTOKART_PERCEPTION_PORT` | `GET /health` 200; hops at `/api/v1/diagnostics/pipeline` |
| `dash-gateway` — live ROS gateway | `ros2 run autokart_dashboard_gateway dashboard_gateway --source REAL_KART --host 127.0.0.1 --port 8765` | 8765 | `AUTOKART_LIVE_GATEWAY_PORT` | `GET /mission/api/status` → `vehicle_authorized:false` |
| `dash-safety` — front safety envelope monitor | `ros2 run autokart_safety_bridge front_safety_envelope_monitor --ros-args --params-file <install>/.../front_safety_envelope_monitor.yaml -p data_source:=LIVE` | — | — | `ros2 param get` (audit script) |
| `dash-ui` — Vehicle Dashboard | `npm run dev:dashboard` (or `--prod`: `build:dashboard` + `preview:dashboard`) | 5180 | `AUTOKART_DASHBOARD_PORT` | HTTP 200 |
| `lab-core` — Lab CORE API | `python3 -m autokart_perception.lab_api --host 127.0.0.1 --port 8767` | 8767 | `AUTOKART_LAB_CORE_PORT` | `GET /api/core/status` 200 |
| `lab-gateway` — Lab ROS SIMULATION gateway | `ROS_DOMAIN_ID=42 ros2 run autokart_dashboard_gateway dashboard_gateway --source SIMULATION --port 8775` | 8775 | `AUTOKART_LAB_GATEWAY_PORT` | `GET /mission/api/status` → `source:SIMULATION` |
| `lab-ui` — Algorithm Lab | `npm run dev:lab` (or `--prod`: `build:lab` + `preview:lab`) | 5181 | `AUTOKART_LAB_PORT` | HTTP 200 |

ROS domains: live stack uses `AUTOKART_LIVE_ROS_DOMAIN_ID` (default: inherited `ROS_DOMAIN_ID`, else 0).
Lab uses `AUTOKART_LAB_ROS_DOMAIN_ID` (default 42). `run_lab.sh` refuses if the two are equal or if any
Lab port is 8765/8768 or equals a live port.

UI proxies (in `phase_sim/vite.dashboard.config.ts` / `vite.lab.config.ts`, also used by `preview`):
- Dashboard: `/api/snapshot`, `/api/image`, `/api/v1`, `/health`, `/ready` → perception; `/mission` (+ws) → live gateway.
- Lab: `/api/core` → Lab CORE; `/mission` (+ws) → Lab SIMULATION gateway. **No** `/api/snapshot` — the Lab never reaches the camera service.

The scripts export `AUTOKART_PERCEPTION_URL`, `AUTOKART_ROS_GATEWAY_URL`, `AUTOKART_LAB_CORE_URL`,
`AUTOKART_LAB_ROS_GATEWAY_URL` from the port variables so the Vite proxies follow any override.

## 2. Command cookbook

```bash
cd /home/mark1/Desktop/autonomous_kart

# one-time (and after any change under kart/ros2_ws/src): ~50 s, builds into .ros_build/
scripts/build_ros.sh                       # extra args go to colcon, e.g. --packages-select autokart_dashboard_gateway

# Vehicle Dashboard
scripts/run_dashboard.sh start             # live HP60C, dev UI
scripts/run_dashboard.sh start --prod      # build:dashboard + preview (AUTOKART_SKIP_BUILD=1 to reuse dist-dashboard)
scripts/run_dashboard.sh start --dev-sim   # BENCH ONLY: perception --mode sim, loud SIMULATED banner
scripts/run_dashboard.sh status | stop | restart
scripts/smoke_dashboard.sh                 # PASS/FAIL/BLOCKED, exit 1 on any FAIL
scripts/audit_no_actuation.sh              # with the live stack running

# Algorithm Lab
scripts/run_lab.sh start [--prod]
scripts/run_lab.sh status | stop | restart
scripts/smoke_lab.sh

# everything
scripts/status.sh                          # table: pid, alive, port, listen, HTTP, detail
```

All start commands are idempotent (a second `start` reports "already running" and starts nothing).
`stop` is idempotent and kills each process's whole session (`ros2 run` → node, `npm` → `sh` → `node`),
SIGTERM then SIGKILL after 10 s.

Extra perception flags: `AUTOKART_PERCEPTION_EXTRA_ARGS="--detector conenet-v2"` (may not contain `--mode`;
the mode is owned by the script so it can never silently become sim).

### Running on non-default ports (e.g. alongside services you must not disturb)

```bash
export AUTOKART_DASHBOARD_PORT=15180 AUTOKART_PERCEPTION_PORT=18768 AUTOKART_LIVE_GATEWAY_PORT=18765
export AUTOKART_LAB_PORT=15181 AUTOKART_LAB_CORE_PORT=18767 AUTOKART_LAB_GATEWAY_PORT=18775
export AUTOKART_LIVE_ROS_DOMAIN_ID=77 AUTOKART_LAB_ROS_DOMAIN_ID=78   # keep test ROS nodes off domain 0/42
scripts/run_dashboard.sh start && scripts/smoke_dashboard.sh && scripts/run_dashboard.sh stop
```
Every script (status/smoke/audit too) reads the same variables, so export them once per shell.

## 3. Logs and state

- Pid files: `.run/pids/<name>.pid` — contains `pid starttime`; starttime guards against pid reuse.
- Logs: `.run/logs/<name>.log` (appended; each start writes a `===== <name> start <time> =====` header and the exact command).
- Build logs: `.run/logs/dash-ui-build.log`, `.run/logs/lab-ui-build.log`.
- ROS build: `.ros_build/{build,install,log}` (`AUTOKART_ROS_BUILD_DIR` to move it). The in-tree
  `kart/ros2_ws/{build,install}` have stale absolute-path caches and are **not used**.
- Override the state dir with `AUTOKART_RUN_DIR`.

## 4. Safety properties enforced by the scripts

- **Single camera owner.** `run_dashboard.sh` refuses to start perception if (a) our pid file is alive
  (idempotent no-op), (b) the perception port is held by any foreign process, or (c) any other
  `autokart_perception.live_mission --mode live` is running on *any* port (e.g. legacy `autokart_apps/run_live.sh`).
  If perception cannot start, the whole dashboard start aborts and nothing else is started — the UI is
  never left proxying to some other (possibly SIMULATED) perception service.
- **Live by default, never auto-sim.** Only `--dev-sim` selects `--mode sim`, and it prints a banner before
  and after start. A missing camera is reported, not hidden: perception stays up with
  `sdk_capture=SDK_ERROR` and `smoke_dashboard.sh` prints `BLOCKED ... BLOCKED-no-hardware` (not FAIL).
- **Safety monitor** runs with the package's own DIAGNOSTIC_ONLY config + `data_source:=LIVE`; no
  authority parameter is ever passed (the node also refuses to start unless `vehicle_authority=OFF`).
- **Lab isolation**: separate ports, separate ROS domain, no `/api/snapshot` proxy (smoke asserts this).
- **Foreign ports**: every start checks the port first and exits with
  `port N is already in use by a process NOT started by these scripts (pid(s): ...)`.

## 5. Troubleshooting

| Symptom | Cause / fix |
|---|---|
| Dashboard shows **"Failed to fetch"** / `Unexpected token '<'` | Diagnosed in `AS_IS_MAP.md` §4: `/api/snapshot` was not proxied to 8768 under `npm run dev`, so Vite's SPA fallback returned `index.html` (200 text/html). Fixed in `vite.dashboard.config.ts` (and legacy `vite.config.ts`). If it recurs: `curl -si localhost:5180/api/snapshot` — `text/html` means the proxy rule is missing or the UI was started with the wrong config; `502` means perception (8768) is down → `scripts/status.sh`. `smoke_dashboard.sh` checks exactly this. |
| `port N is already in use by a process NOT started by these scripts` | Something else owns the port (another engineer, legacy `autokart_apps/run_*.sh`, the 5173 legacy Vite). `ss -ltnp 'sport = :N'` to identify; stop it or use the env overrides. The scripts never kill foreign processes. |
| `another HP60C owner (live_mission --mode live) is already running` | Only one SDK owner is allowed. Stop the other one (`scripts/run_dashboard.sh stop`, or the terminal running `autokart_apps/run_live.sh`). |
| Smoke: `BLOCKED camera hop sdk_capture=SDK_ERROR reason=SOURCE_OPEN_FAILED` | Camera not attached / SDK cannot open it. Expected without hardware; the rest of the dashboard is healthy. Check `lsusb`, `.run/logs/dash-perception.log`. |
| Smoke: `camera hop sdk_capture=OK source=LIVE`, overall `CALIBRATION_UNVERIFIED` | Camera streams; mount calibration is not yet verified (by design, not a script fault). |
| `ROS workspace not built at .../.ros_build/install` | Run `scripts/build_ros.sh`. |
| `ros2 run` says package not found | Stale or partial `.ros_build`; rerun `scripts/build_ros.sh`. Do not source `kart/ros2_ws/install`. |
| Gateway `ros_connected=false` | No publishers on that ROS domain (normal on the bench). Check `ROS_DOMAIN_ID` matches the vehicle's (`AUTOKART_LIVE_ROS_DOMAIN_ID`). |
| `dash-perception did not open port … within 90s` | Log tail is printed; common causes: `phase_sim/dist` missing (`live_mission` requires it: `cd phase_sim && npm run build`), model/ONNX load failure. |
| `--prod` fails with `build:dashboard failed` | `tsc -b` type-checks the whole project; a teammate's in-progress edit can break it. See `.run/logs/dash-ui-build.log`; use `AUTOKART_SKIP_BUILD=1` to serve the last good `dist-dashboard`. |
| `status.sh` shows `LISTEN=FOREIGN` | The port is held by something whose session is not the one in our pid file (foreign process, or ours was restarted by hand). |
| Audit FAIL `topic X has N publisher(s)` | Some process (possibly hidden, e.g. a `ros2 topic pub`) publishes an actuation-like topic on the live domain. Investigate before anything else. |

## 6. systemd (example only)

`deploy/systemd/autokart-{perception,dashboard,lab}.service.example` call
`scripts/run_foreground.sh {perception|dashboard|lab}` (foreground group; if any child exits the group is
torn down and the unit restarts, at most 5 times per 5 min). They are **NOT installed and NOT validated on
the Pi**; `systemd-analyze verify` passed on Jarvis. UIs under systemd use `preview` on prebuilt
`dist-dashboard` / `dist-lab` (`cd phase_sim && npm run build:apps` first).

## 7. Legacy

`autokart_apps/run_live.sh`, `run_lab.sh`, `run_core.sh`, `run_tests.sh` are untouched and still work
(single-port legacy layout: `run_live.sh` serves `/live` from the perception process on 8768;
`run_lab.sh` runs `lab_api` + `npm run dev` on 5173). Do not run `run_live.sh` and
`scripts/run_dashboard.sh` at the same time — the camera-owner guard will refuse the second one.
