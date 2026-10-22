# AutoKart Mission Control: Live/Simulation Separation (Agent 6)

Scope of this document: `phase_sim/` (the Vite + React + TypeScript frontend) only.
No ROS2/Python code, no `autokart_dashboard_gateway` code, and no actuator/vehicle-command
code path was touched.

## 1. The problem this addresses

Before this change, the default route (`/`) served `MissionControl.tsx`, a single
component/bundle whose `useMissionData()` hook let an operator switch at runtime,
via a dropdown, between `LAB` (local deterministic physics sandbox), `SIMULATION`
(ROS-connected but simulated), `REPLAY` (recorded `.jsonl` files), and `REAL_KART`
(the real ROS2 gateway). A production/operational deployment of Mission Control
therefore had a reachable UI path into simulated or replayed data, and a developer
could in principle confuse which mode was active.

## 2. Approach taken

Per the master plan, this was done as an **additional isolated entry point**, not a
full `apps/mission_control` + `apps/simulation` directory rewrite. The existing
`MissionControl.tsx` (full LAB/SIMULATION/REPLAY/REAL_KART switcher) is **unchanged**
and remains the development/simulation tool, served at the default route. A new,
separate, hard-locked component and data hook were added for the live-only
operational view.

### Entry points / routes (`src/main.tsx`)

| Route      | Component                                   | Purpose                                                                 |
|------------|----------------------------------------------|--------------------------------------------------------------------------|
| `/live`    | `src/live/LiveMissionControl.tsx` (pre-existing, unchanged) | HP60C camera + perception diagnostic dashboard. Real gateway only (`/api/snapshot`, `/api/image/...`). Already single-purpose; not touched by this task. |
| **`/ops`** | **`src/mission/MissionControlLive.tsx` (new)** | **Live-only operational Mission Control: the full world/ROS view, hard-locked to `REAL_KART`. No simulation/lab/replay code path.** |
| `/shared`  | `src/shared/SharedLab.tsx` (unchanged)       | Shared lab tooling.                                                      |
| `/demo`    | `src/App.tsx` (unchanged)                    | Pure simulation demo.                                                    |
| (default)  | `src/mission/MissionControl.tsx` (unchanged) | Development/simulation tool: LAB / SIMULATION / REPLAY / REAL_KART switcher, for engineers debugging the pipeline. Not for production/operational use. |

`/ops` was chosen (rather than renaming `/live`, which is already a distinct,
narrower HP60C-specific diagnostic tool) to avoid colliding with that existing
route's meaning.

### Files added

- **`src/mission/useMissionDataLive.ts`** — a new, separate, minimal data hook.
  It is hard-locked to `source: 'REAL_KART'`: there is no `setSource`, no mode
  other than `LIVE`/`PAUSE`, no file loader, and no gateway-replay loader. It
  imports only `react` and `./schema` (types + the `frameAt` helper) — nothing
  from `./LabSkidpad`, `../engine/engine`, `./demo`, or `./labScenarios`. The
  WebSocket message handler rejects any frame whose own `frame.source !==
  'REAL_KART'` before it ever reaches `setFrames`, so even a manifest/stream
  mismatch from the gateway cannot get a non-REAL_KART frame into the buffer.
- **`src/mission/MissionControlLive.tsx`** — the new live-only UI. It calls
  `useMissionDataLive()` (not `useMissionData()`), reuses `MissionScene` (the
  3D world renderer) and `RosLivePanel` (the ROS gateway status / camera /
  safety-envelope panel) unmodified, and renders no `<select>`, button, or
  file input that can change the data source or load replay/demo data. The
  operating-mode indicator is a plain `<span>`, not a control.
- **`src/test/missionControlLive.test.tsx`** — new tests (see §4).

### Files changed

- **`src/main.tsx`** — added the `/ops` branch and route-map comment. No other
  route's behavior changed.

### Files explicitly NOT changed

`src/mission/MissionControl.tsx`, `src/mission/useMissionData.ts`,
`src/mission/RosLivePanel.tsx`, `src/mission/schema.ts`,
`src/live/LiveMissionControl.tsx` — all read for context, none modified.
`RosLivePanel.tsx`'s RC-05/RC-06 safety-envelope provenance warnings (the
`data_source` / `mc-alert` banner logic, the "never show CLEAR on stale data"
behavior, the non-LIVE `data_source` warning banner) are untouched and are
rendered as-is inside `MissionControlLive.tsx` via the unmodified
`<RosLivePanel source="REAL_KART" frame={frame} />`.

## 3. How the hard requirements are satisfied

1. **No build-time path from the live-only entry to simulated/lab/demo data.**
   Verified by inspecting the full import graph of `MissionControlLive.tsx` →
   `useMissionDataLive.ts` / `MissionScene.tsx` / `RosLivePanel.tsx` / `schema.ts`:
   none of these files import `./LabSkidpad`, `../engine/engine` (the
   `AutonomyEngine`), `./demo`, or `./labScenarios`, directly or transitively
   (confirmed by `grep` across that file set — only this document's own prose
   matches those names, not an `import`).
   **Caveat, stated plainly:** this project's Vite config does not code-split
   by route — `npm run build` emits a single JS bundle (`dist/assets/index-*.js`)
   that still contains `MissionControl.tsx`'s LAB/SIMULATION/REPLAY code
   *for the other routes*, because `main.tsx` statically imports all of
   `App`, `MissionControl`, `MissionControlLive`, `SharedLab`, and
   `LiveMissionControl` up front and switches on `location.pathname` at
   runtime. This was already true before this change (e.g. `/demo` and
   `/live` share the same bundle as the default route today). So: **the
   `/ops` React component tree itself has zero reachable code path into
   synthetic-data generators**, but a single physical file served to the
   browser still contains that code for the *other* routes in the same
   bundle. If a stricter guarantee (a physically separate JS artifact with
   no byte of lab/demo code in it) is required for the production live
   deployment, that needs either Vite route-based code-splitting/dynamic
   `import()` for `MissionControl`/`App`, or a separate Vite entry point —
   neither was done here, to keep this change minimal and low-risk as
   instructed. This is a known limitation, not an oversight.
2. **No silent fallback to fabricated data on disconnect.** `useMissionDataLive`
   mirrors the existing, already-hardened `useMissionData` logic for the
   `REAL_KART` + `LIVE` case: `current` is `null` whenever `liveFrameFresh` is
   false (socket closed, or no frame within 1000 ms), never a held or
   synthesized frame. `MissionControlLive` renders an explicit `NO LIVE
   CONNECTION` / `DISCONNECTED FROM GATEWAY · NO LIVE FRAME DISPLAYED` banner
   in that state instead of the 3D world view.
3. **`vehicle_authority` OFF / read-only everywhere.** Not touched. Both apps
   render `RosLivePanel`'s existing "VEHICLE AUTHORITY: OFF · DIAGNOSTIC_ONLY"
   / "VEHICLE AUTHORITY DISABLED" lines unmodified, and `useMissionDataLive`
   has no vehicle-command code path at all (no outgoing WS writes, no POST to
   any gateway command endpoint).
4. **RosLivePanel safety-envelope provenance warnings kept intact.**
   `RosLivePanel.tsx` is imported and rendered unmodified in
   `MissionControlLive.tsx`; its `mc-alert` / `data_source` mismatch banner
   logic is exercised by the pre-existing `rosLivePanel.test.tsx` suite,
   which still passes unchanged (see §4).

## 4. Tests

New file: `src/test/missionControlLive.test.tsx`, 4 tests, using the same
`vi.stubGlobal`/`act()`/`createRoot` pattern as `rosLivePanel.test.tsx` and
`liveMissionControl.test.tsx`, with a small hand-rolled controllable
`MockWebSocket` class (none of the existing tests exercised `useMissionData`'s
WebSocket path, so there was no existing WS mock to reuse):

- **Renders locked to `REAL_KART` with no reachable control to change the
  source.** Asserts there is no `<select>` element anywhere in the tree, no
  file input, no "GATEWAY RUN"/"OPEN FILE" button, and that `LAB`, `SIMULATION`,
  `REPLAY`, `DEMO` do not appear as text anywhere in the rendered output.
- **Shows a disconnected/stale state instead of fabricated data when the
  gateway is unavailable.** With the mock WS never opened and `fetch` mocked
  to fail, asserts `NO LIVE CONNECTION` / `DISCONNECTED FROM GATEWAY · NO LIVE
  FRAME DISPLAYED` is shown and vehicle speed reads `—`, not a fabricated
  number.
- **Drops a live connection and shows disconnected rather than keeping stale
  data as current.** Opens the mock WS, delivers one real `REAL_KART` frame
  (`CURRENT ROS FRAME` appears), then closes the socket and asserts the view
  flips to `DISCONNECTED` rather than continuing to display the last frame as
  current.
- **Never renders a frame payload tagged `SIMULATED` even if delivered over
  the `/mission/ws` the live app connects to.** Sends a frame with
  `source: 'SIMULATION'` and a conspicuous `speed_mps: 99`; asserts the hook
  rejects it (text `99.00 m/s` never appears) and the gateway-mismatch state
  (`SOURCE MISMATCH`) is surfaced instead.

Because `MissionScene.tsx` constructs a real `THREE.WebGLRenderer` and a
`ResizeObserver` on mount, and jsdom (vitest's test environment here) has
neither, the test file stubs `THREE.WebGLRenderer` with a no-op class and
stubs `ResizeObserver` — both are pre-existing jsdom gaps unrelated to this
change (no other test in the suite renders `MissionScene` with a live frame,
which is why this had not surfaced before).

### Before / after counts

| | Test files | Tests |
|---|---|---|
| Before this change | 10 | 43 |
| After this change | 11 | 47 |

Run with `npx vitest run`. No existing test was modified or weakened; all 43
pre-existing tests still pass unchanged, plus the 4 new ones.

### `npx tsc --noEmit -p .`
PASS — no errors.

### `npm run build`
PASS — build succeeds (`tsc -b && vite build`, 56 modules, single
`dist/assets/index-*.js` bundle as before). The pre-existing "chunk larger
than 500 kB" warning is unrelated to this change (the app was already a
single unsplit bundle before `/ops` was added).

## 5. What remains NOT separated or NOT tested — stated explicitly

- **NOT a physically separate production artifact.** As noted in §3.1, this
  is a single-bundle app; `/ops` is isolated at the React component/import
  level, not at the level of "ship a different .js file to the kart's
  on-vehicle browser with zero bytes of simulation code in it." If that
  stronger guarantee is required, it needs Vite-level route code-splitting
  or a second build entry, which was intentionally not done here (the task
  explicitly allowed "isolated entry points" instead of a full app split, to
  minimize risk to a working app).
- **NOT verified in a real browser against a real gateway.** All verification
  here is `npx vitest run` under jsdom with a hand-written `MockWebSocket`
  and mocked `fetch`/`THREE.WebGLRenderer`/`ResizeObserver`. This was never
  run against an actual `autokart_dashboard_gateway` process, a real
  `/mission/ws` WebSocket server, or real HP60C hardware. The WebSocket
  message/manifest-mismatch handling, the 1000 ms staleness timer, and the
  3D scene's actual WebGL rendering of a real frame are therefore
  **NOT TESTED** end-to-end — only their logic in isolation under mocks.
- **NOT tested: `/ops` under `npm run dev` / an actual browser.** No
  `preview_start`/browser-pane manual check was performed for this route in
  this session; only automated `vitest` + `tsc` + `vite build` checks were run.
- **NOT changed: PAUSE-mode scrubbing UI.** `MissionControlLive` exposes only
  LIVE/PAUSE buttons (no STEP, no timeline scrubber, no telemetry plots) to
  keep the live-only surface minimal; this is a deliberate reduction in
  functionality versus `MissionControl.tsx`, not a bug, but it means the
  live-only view currently has less inspection tooling than the dev tool.
- **NOT touched:** ROS2/Python code, `autokart_dashboard_gateway`, any
  actuator/vehicle-command path, `vehicle_authority` wiring — per the task's
  explicit constraints.
