# AutoKart Dashboard/Lab Split — STEP 0 Inventory (NO EDITS)

> **CORRECTION (2026-10-18, later):** statements in this document that no HP60C is attached are WRONG or STALE. An HP60C (USB `3482:6723`) is attached and streams RGB+depth at ~11-13 fps via the vendor SDK; the live Vehicle Dashboard was verified against real frames (see `ui_split/VALIDATION_MATRIX.md`). Items below marked NOT TESTED because "no camera" are now testable; those not re-run in `ui_split/VALIDATION_MATRIX.md` remain NOT TESTED. Calibration is still UNVERIFIED and no qualified metric geometry exists.

**Per the spec's own instruction: this is Step 0 only — inventory and a minimal change plan. No Milestone 1-5 code changes have been made. Stopping here for review before any rewrite begins.**

## 1. Nested git repositories

```
./autonomy_stack_phase1/.git     — real repo, branch main, DIRTY:
    M .gitignore, M README.md, M setup.sh
    ?? docs/, ?? phase_01_perception/, ?? runs/
    (phase_01_perception/ — where ALL camera/YOLO26s/geometry code lives — is entirely UNTRACKED)
./AutoKart_RC_Dynamics_ESP32S3_v1/.git  — empty/stub, "not a git repository" when queried; not real
./_camera_a1..a5, _camera_prod_a1..a5, _camera_team_base/.git  — real repos, ARCHIVED SNAPSHOTS (constraint #6: do not touch)
```

**`kart/` and `phase_sim/` — where the ROS2 workspace, the safety-envelope monitor, the dashboard gateway, and essentially all of this project's current frontend live — have NO git repository at all.** No commit-based rollback exists for most of this codebase; `ROLLBACK.md` must be file-copy-based (dated backup directories), not `git revert`.

## 2. CRITICAL FINDING: two independent, parallel application stacks already exist

This contradicts both the spec's and my own prior assumption of a single Mission Control to split. There are **two unrelated, non-interoperating backend architectures**, both real, both tested, both currently reachable through the same `phase_sim` frontend shell on different routes:

### Stack A — ROS 2 (`kart/ros2_ws`), newer, actively touched through 2026-10-18

- `autokart_perception_ros`, `autokart_mapping`, `autokart_planner`, `autokart_controller` (ROS2 nodes)
- `autokart_safety_bridge` → `front_safety_envelope_monitor` (DIAGNOSTIC_ONLY, this session's RC-05/06 work)
- `autokart_dashboard_gateway` (`gateway.py`) — ROS→WebSocket/HTTP bridge, **port 8765**, serves `/mission/ws` + `/mission/api/*`
- Frontend consumers: `MissionControl.tsx` (default route `/`, the LAB/SIMULATION/REPLAY/REAL_KART switcher) and `MissionControlLive.tsx` (route `/ops`, this session's earlier work — hard-locked to `REAL_KART`)
- Has **no production path to the physical HP60C at all** — subscribes to `/autokart/perception/rgb`/`depth` Image topics, which only `autokart_perception` (ROS adapter) would publish if run in `live` mode; never exercised against real hardware this session.

### Stack B — `autokart_perception.shared_core` / CORE+LAB (`autokart_apps/`, `autonomy_stack_phase1/`), dated 2026-10-13, **not ROS at all**

- `autokart_perception.shared_core.AlgorithmCore` — single Python implementation of geometry/mapping/corridor/mission/Pure-Pursuit, calling the same SciPy Delaunay/Gabriel/gates/centerline functions and `kart_sim.autonomy` for the controller (the same `kart_sim` package I installed editable earlier this session for the ROS-side tests — **it is shared by both stacks**, the one genuine point of code reuse between them).
- `autokart_perception.application.CoreRuntime` — CLI entrypoint, no HTTP.
- `autokart_perception.lab_api.LabAdapter` — HTTP API, **port 8767** (`run_lab.sh`), serves `/api/core/*`. Feeds route `/shared` (`SharedLab.tsx`).
- `autokart_perception.live_mission` / `webui.py` — the actual **single HP60C SDK owner** process, **port 8768** (`run_live.sh`), serves both the built static frontend AND `/api/snapshot` + `/api/image/*`. Feeds route `/live` (`LiveMissionControl.tsx`) — **this is the real camera-owning dashboard backend**, not the ROS `autokart_dashboard_gateway`.
- Per `autokart_apps/VALIDATION.md` (2026-10-13): CORE/LAB software-tested (49 Python + 23 browser-engine tests), explicitly **BLOCKED** on "ROS 2 and VCU... typed read-only ROS publication and feedback adapter not connected," and **NOT TESTED** on real HP60C or Pi.

**Both stacks independently implement "LAB."** `MissionControl.tsx`'s LAB dropdown option is a third thing again — a client-side TypeScript physics sandbox (`LabSkidpad`, no backend at all) — unrelated to either Stack A or Stack B's LAB. Three different systems answer to the name "LAB." This needs a naming/ownership decision before any further work, not a guess.

## 3. Current frontend routes, confirmed by reading `main.tsx` and each component, with actual backend/port

| Route | Component | Backend | Port | Status this session |
|---|---|---|---|---|
| `/` (default) | `MissionControl.tsx` | `autokart_dashboard_gateway` (Stack A) | 8765 | Full LAB/SIM/REPLAY/REAL_KART switcher — exactly what the new spec's constraint forbids for a "Dashboard" |
| `/ops` | `MissionControlLive.tsx` | `autokart_dashboard_gateway` (Stack A) | 8765 | Added this session; hard-locked REAL_KART, verified in a live browser — closest existing thing to the spec's "Vehicle Dashboard," but still Stack A, not Stack B/live_mission (no actual camera path wired) |
| `/live` | `LiveMissionControl.tsx` | `autokart_perception.live_mission` (Stack B, the real camera owner) | 8768 | Confirmed broken under `npm run dev` — see §4 |
| `/shared` | `SharedLab.tsx` | `autokart_perception.lab_api` (Stack B) | 8767 | Per `VALIDATION.md`, software-tested 2026-10-13; not re-verified this session |
| `/demo` | `App.tsx` | none (client-side `AutonomyEngine`) | — | Legacy, per `autokart_apps/README.md` |

Vite dev-server proxy (`phase_sim/vite.config.ts`) maps **only** `/mission → 8765` and `/api/core → 8767`. **There is no proxy rule for plain `/api/*` → 8768.**

## 4. CAMERA_DEBUG_REPORT — "Failed to fetch" root cause, diagnosed with evidence, not guessed

Reproduced live in this session's browser pane, `npm run dev` (port 5173), route `/live`:

```
$ fetch('/api/snapshot') from the browser, on /live:
  status: 200
  content-type: text/html
  body: "<!doctype html>...<script type=\"module\">import { injectIntoGlobalHook }..."
```

**Root cause: `/api/snapshot` is not proxied to port 8768 at all under `npm run dev`. Vite's dev server SPA-fallback serves `index.html` (HTTP 200, `text/html`) for the unmatched path instead of erroring.** `LiveMissionControl.tsx`'s `await response.json()` then throws `SyntaxError: Unexpected token '<'`, which its `catch` block turns into the displayed error banner. This is **not** a camera, SDK, ROS, or gateway failure — it is a missing one-line dev-server proxy rule. `run_live.sh` avoids this entirely by building `phase_sim` to static `dist/` and serving everything (static files + `/api/*`) from `live_mission.py`'s own port-8768 process — `/live` only works when launched that way, never under plain `npm run dev`.

Separately observed in the same browser session (from earlier `/ops` testing, same long-lived tab, not re-diagnosed as part of this finding): repeated `502 Bad Gateway` on `/mission/*` and failed `ws://localhost:5173/mission/ws` — expected and correct, since no `autokart_dashboard_gateway` process was running at the time; this is Stack A's gateway being absent, a separate, already-understood, correctly-surfaced condition (RC-06 work this session already verified the dashboard fails closed/shows UNKNOWN in exactly this situation).

No HP60C hardware is attached in this sandbox (`lsusb` shows only a laptop webcam, `/dev/video0,1`) — the `/live` → 8768 hop cannot be tested end-to-end against real frames here regardless of the proxy fix. Fixing the proxy rule would let `/live` reach `live_mission.py`'s synthetic/replay modes for a software-level test, which has not been done yet.

## 5. Camera/USB/SDK status (re-confirmed from earlier this session, unchanged)

- `lsusb`: HP Wide Vision HD Camera (laptop webcam, irrelevant), no HP60C or depth-sensor device.
- `/dev/video0`, `/dev/video1` present (the webcam), nothing HP60C-specific.
- Native SDK bridge **is built**: `autonomy_stack_phase1/phase_01_perception/native/build-x86_64-linux-gnu/libhp60c_bridge.so` exists, so `camera.py`'s `HP60C.__init__` would not fail on `SDK_ADAPTER_NOT_BUILT` — but `hp_connect()`/`hp_read()` would correctly raise `CAMERA_DISCONNECTED` since no device is physically present (verified this behavior via `camera.py`'s own test suite, 7/7 passing, mocked SDK calls).

## 6. ROS 2 state (from this session's RC-06 work, re-confirmed)

16 packages build clean. `/safety/envelope`, `/perception/cones`, `/perception/obstacles` (no production publisher — see `docs/AUTOKART_OPEN_ISSUES.md` #8), `/vehicle/odometry`, `/vcu/state` are the live-wired topics. No topic publishes to any actuator interface — independently confirmed via `ros2 node info` on every node touched this session.

## 7. Minimal change plan — proposed, not yet executed

Given the two-stack discovery in §2, the spec's 2-app model (Dashboard = real-only, Lab = everything else) maps **cleanest onto Stack B** (`live_mission`/`lab_api`, already separate processes/ports, already has the single real camera-SDK owner) rather than onto Stack A (the ROS gateway, which has no camera path at all yet). Rebuilding Stack A's `/ops` into a second "Dashboard" would create a **third** dashboard-shaped thing, contradicting constraint #1 ("do not clone algorithms or create a new parallel autonomy project") — except here the two parallel stacks already pre-date this request.

**This is the fork in the road the spec's own Step 0 rule says to surface, not guess past.** Two honest options, not resolved by this document:

1. **Treat Stack B (`/live` + `/shared`, ports 8768/8767) as the real split-target** — it already has independent processes, independent ports, a single real-camera owner, and documented NOT-TESTED/BLOCKED honesty in `VALIDATION.md`. The work would be: fix the Vite proxy gap (§4, a one-line config fix), rename/reskin `/live` as the "Dashboard" (strip anything LAB-shaped from it — it currently has none to strip, per its README), harden `/shared` as the "Lab," and leave Stack A (`/ops`, `/`) alone as a separate, already-working ROS-telemetry view, documented as out of scope for this split rather than silently abandoned.
2. **Treat Stack A (the ROS gateway, `/ops`) as the real split-target**, since it's the actively-developed, newer stack this session has spent the most effort hardening (RC-05/06 safety envelope, provenance labeling) — but it has zero camera path today, so "Vehicle Dashboard" would ship with `RGB CAMERA · NO CURRENT FRAME` permanently until someone wires `autokart_perception`'s live mode into a ROS Image publisher the gateway already expects (`/autokart/perception/rgb`/`depth` — the subscription exists, nothing publishes to it in production).

Recommend option 1 as lower-risk (camera path already exists and is closer to real), but this is exactly the kind of decision the spec's Step 0 asks to be surfaced, not resolved unilaterally.

## 8. Explicit non-edits this pass

No code was changed in this document's preparation. `_camera_a*`/`_camera_prod_a*`/`_camera_team_base`/`AutoKart_RC_Dynamics_ESP32S3_v1` archives were not touched or further inspected beyond confirming they are separate git repos (constraint #6).
