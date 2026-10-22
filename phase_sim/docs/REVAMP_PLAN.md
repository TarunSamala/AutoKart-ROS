# phase_sim revamp plan

Written per the master prompt's Step 0, before any UI/algorithm changes. Based on a full read-only pass over `phase_sim/` (see the commit history for the exploration). This plan is honest about scope: a full 16-page design-system rebuild, a TypeScript algorithm port with Python conformance testing, and an Ackermann-steered STL-to-three.js pipeline is realistically a multi-week effort. This document scopes what's actually buildable in this pass and says plainly what isn't, rather than padding the plan to look complete.

## Current architecture

**Three separate Vite builds** share one `phase_sim/` source tree:
- Legacy/demo app (`vite.config.ts`, `src/App.tsx`) — the original `AutonomyEngine`-driven sandbox.
- **Lab app** (`vite.lab.config.ts`, port 5181, `src/apps/lab/`) — this is "the Lab" the master prompt means: a hand-rolled History-API router (`src/apps/lab/router.tsx`), a route table (`src/apps/lab/routes.ts`) with explicit `AVAILABLE`/`PARTIAL`/`NOT IMPLEMENTED` status per route, and pages composed in `LabApp.tsx`.
- Dashboard app (`vite.dashboard.config.ts`, port 5180, `src/apps/dashboard/`) — the live-vehicle-facing app; **out of scope for the Lab work below**, touched only where explicitly noted.

**Algorithm code is implemented three times, independently**, not stubbed but genuinely duplicated:
1. `src/engine/planner.ts` + `src/engine/engine.ts` (`AutonomyEngine`, multi-rate: physics 60 Hz / controller 50 Hz / sensor 20 Hz / planner 15 Hz) — drives the legacy/demo app via `requestAnimationFrame`.
2. `src/mission/labAlgorithms.ts` (`delaunayFromCones`, `gabrielFilter`, `gatesFromGabriel`, `candidatePaths`, `fitCircle`, `observeCones`) — drives the Lab's `/mission` page.
3. `src/mission/demo.ts` (`gabrielForDemo`) — a third, simplified re-implementation just to adapt `AutonomyEngine` snapshots into the shared `MissionFrame` schema.

**Scenarios**: `src/mission/labScenarios.ts` — `SKIDPAD`, `AUTOCROSS`, `BRAKE`, all real geometry generators (SDF + marching squares for skidpad, seeded-PRNG route segments for autocross), dispatched by `generateScenario(name, settings)`.

**3D rendering**: `src/mission/MissionScene.tsx` uses real three.js (`THREE.WebGLRenderer`, instanced meshes, orbit/top-down cameras, raycaster picking) — not a 2D canvas. **Cones already render as real cone geometry** (`THREE.ConeGeometry` + a base cylinder, `MissionScene.tsx:79-80`), colored by class. **The kart is a placeholder**: a `THREE.BoxGeometry` body + a 3-sided `ConeGeometry` "nose" + 4 wheel boxes (`MissionScene.tsx:98-102`) — this is the one piece matching the master prompt's "replace the dummy kart" ask; the cone-rendering ask is already mostly satisfied in 3D (the HP60C 2D camera panel's detection-box rendering was not located in this pass — flagged as an open item).

**Schema**: `src/mission/schema.ts` already defines exactly the shared frame format the master prompt asks for — `MissionFrame`/`WorldFrame` (`schema_version: 1`, `vehicle`, `perception`, `planning`, `safety` with a `data_source` provenance tag, `metrics`). `src/mission/replay.ts` already encodes/decodes it as JSONL. **This means "define the schema once" is largely done** — the real work is (a) getting the Python Layer A runner to emit it, and (b) getting a MY_TRACK scenario's TS engine to populate it with the same shape as SKIDPAD/AUTOCROSS/BRAKE do today.

**Styling**: no design system exists. Eight independent `.css` files with different, non-overlapping CSS variable names (`src/styles.css` uses `--ink`/`--panel`/`--teal`; `src/apps/dashboard/dashboard.css` uses `--bg`/`--text`/`--muted` with different values; five more). **Zero light-theme support anywhere** (`prefers-color-scheme`/`data-theme` greps returned nothing). The master prompt's "one design system, light and dark" is a genuine from-scratch build, not a migration of an existing token set.

**Route status, self-reported in code** (`src/apps/lab/routes.ts`): 4 routes are literally `NOT IMPLEMENTED` (`/perception`, `/depth-calibration`, `/benchmark`, `/faults`), 4 are `PARTIAL` (`/paths`, `/scenario`, `/dataset`, `/logs`). The master prompt says keep every feature and don't delete a tab silently — this plan does not change route *status*, only (eventually) their visual chrome.

**Safety banner**: no single reusable component. In the Lab app it's structurally guaranteed on every route (`LabApp.tsx:81`, inside the layout header, outside the per-route switch) — confirmed safe. It's duplicated with different wording in 8+ other files across the Dashboard/legacy apps; a real design-system pass would consolidate these into one `<SafetyBanner>` component, but — critically — **must not change the asserted text**, since tests pin exact substrings (`src/test/dashboard.test.tsx`, `rosLivePanel.test.tsx`, `liveMissionControl.test.tsx` all assert specific banner text). Any consolidation keeps the per-surface wording identical, just deduplicates the component.

**Clutter found, not part of this plan to fix but noted**: `src/mission/MissionControlLive.tsx.bak-20261019` and `RosLivePanel.tsx.bak-20261019` are dead backup files sitting in `src/`; `src/mission/MissionControl.tsx` + `src/shared/LabNav.tsx` are confirmed dead code in the Lab app (not imported by `LabApp.tsx` — `MissionPage.tsx:14-16` says so explicitly).

## What's reused vs. rewritten

| Area | Decision | Why |
|---|---|---|
| `schema.ts` / `replay.ts` (MissionFrame, JSONL encode/decode) | **Reused as-is** | Already exactly what the master prompt asks for; MY_TRACK and the Python Layer A exporter both target this existing format, not a new one. |
| `labAlgorithms.ts` (Delaunay/Gabriel/gates/paths) | **Extended, not forked** | The master prompt explicitly says "reuse... extend, do not fork." MY_TRACK's extra behaviors (single-side fallback gates, obstacle classifier, detour admission with locking, finish-line latch) are added as new exported functions in this file, called from a new scenario-specific planner step, not a parallel algorithm file. |
| `labScenarios.ts` | **Extended** | Add `'MY_TRACK'` to `ScenarioName`, a `myTrack(settings)` generator that loads the shared layout JSON instead of procedurally generating cones (the other three scenarios generate; MY_TRACK's cone positions are fixed ground truth from `docs/my_track_spec/my_track_layout.json`). |
| `MissionScene.tsx` cone rendering | **Reused as-is** | Already real cone geometry, already correct. |
| `MissionScene.tsx` kart rendering | **Rewritten** | Box+cone+boxes placeholder replaced by loading the glTF assets already produced for the Python-side viewer (`kart/viewer/assets/kart/*.glb`) via `GLTFLoader` (not currently a dependency — `three-stdlib` or `three/examples/jsm` needs adding), with the dimension-sheet footprint rectangle as an explicit fallback if a part fails to load (matches the master prompt's own fallback instruction). Front wheels rotate by the live steering angle using the existing proxy-wheel glTF files, swapped per profile (shared/built). |
| CSS (8 independent files) | **New token layer added, not a full migration** | A `src/styles/tokens.css` is introduced with light+dark CSS custom properties and applied to the Lab app's shell (header/nav) as a working proof of the design system. Migrating all 16 pages' existing per-component styles onto these tokens is the long pole of this entire master prompt — not completed in this pass; see "What is NOT done" below. |
| Legacy `src/engine/` (`AutonomyEngine`) | **Untouched** | Still backs the legacy/demo app; the master prompt's scope is the Lab, not the legacy app. Its duplication with `labAlgorithms.ts` is flagged as pre-existing tech debt, not fixed here (fixing it would mean changing the legacy app's behavior, outside this task's stated scope). |

## New design system (what actually gets built this pass)

A single `src/styles/tokens.css`: CSS custom properties for background/panel/border/text/muted/accent + semantic good/warn/critical, defined under `:root` (dark default) and `:root[data-theme="light"]`, following the same pattern already used successfully for the Python-side `kart/viewer/index.html` built earlier this session. Cone data-colors (blue/yellow/orange) are kept as fixed, theme-independent values since they're real-world colors, not UI chrome. This is the foundation a full migration would build on; this pass wires it into the Lab shell only (header, nav, page background) as a visible, testable proof, not into all 16 pages' internal layouts.

## File-by-file change list (this pass)

- `phase_sim/docs/REVAMP_PLAN.md` — this file.
- `phase_sim/src/styles/tokens.css` — new design tokens (light/dark).
- `phase_sim/src/apps/lab/LabApp.tsx` — import tokens.css, apply theme toggle + `data-theme` attribute, wire token variables into the header/nav.
- `phase_sim/src/mission/labScenarios.ts` — add `MY_TRACK` to `ScenarioName`, add `myTrackLayout` loader, add `myTrack()` generator.
- `phase_sim/src/mission/myTrackLayout.ts` (new) — loads/validates the shared `docs/my_track_spec/my_track_layout.json` (fetched at runtime, not duplicated by hand, per the master prompt's explicit instruction never to hand-duplicate cone coordinates).
- `phase_sim/src/mission/labAlgorithms.ts` — add the MY_TRACK-specific extensions (obstacle classifier, single-side fallback gates, detour admission+lock, finish-line latch) as new exported functions, reusing `delaunayFromCones`/`gabrielFilter`/`gatesFromGabriel` unchanged.
- `phase_sim/src/mission/MissionScene.tsx` — replace the placeholder kart mesh with a `GLTFLoader`-based loader for the prepared glb assets, footprint-rectangle fallback.
- `phase_sim/package.json` — add `three-stdlib` (or use `three/examples/jsm/loaders/GLTFLoader.js` directly, avoiding a new dependency if the existing `three` package ships it — to confirm during implementation).
- Test files: `src/test/myTrackLayout.test.ts`, `src/test/myTrackAlgorithms.test.ts` (new), extending `src/test/labAlgorithms.test.ts` coverage.

## What is explicitly NOT done in this pass (said plainly, not hidden)

- **Not** a full migration of all 16 pages onto the new token system — only the Lab shell (header/nav/background) is wired in. Core Lab, Delaunay, Gabriel, Gates, Paths, Scenario, Demo, Dataset, Replay, Perception, Depth & Calib, Benchmark, Faults, Logs & Config keep their existing per-page CSS. This is the single largest remaining item.
- **Not** a redesign of the nav grouping, Panel/Stat/Gauge/Chip/Toggle/Segmented/Table/StripChart/Legend component library the master prompt describes — those are new components that would need building from scratch and wiring into every page; out of scope for this pass.
- **Not** a TypeScript port of the full MY_TRACK mission FSM/Pure-Pursuit controller — only the algorithm extensions needed to classify obstacles and build gates/paths against the MY_TRACK layout in the Lab's existing scenario-selection and pipeline-visualization flow. A full interactive drive-through-MY_TRACK experience matching the Python mission controller's behavior (detour locking across frames, finish braking, PATH_LOST handling) is not built.
- **Not** a TS-vs-Python conformance test suite (comparing gates/path/state sequence between the two engines on identical seeds) — this needs the above FSM port to exist first.
- **Not** the HP60C camera panel redesign (cone silhouettes with solid/faded/missed states, depth labels, frame age) — this pass did not locate/inspect the existing camera panel implementation in enough depth to redesign it responsibly within budget.
- **Not** screenshot tests across desktop/tablet/phone × light/dark × 16 pages.
- **Not** a 3D Ackermann steering rig (inner wheel turns more than outer) — the kart swap targets getting a real mesh on screen with front wheels rotating by a single shared steering angle (matching what the Python-side prepared assets already support), not differential Ackermann geometry between the two front wheels.
- **Not** a cleanup of the three-way algorithm duplication, the `.bak-20261019` files, or the dead `MissionControl.tsx`/`LabNav.tsx` code paths — flagged, not removed, since removal wasn't asked for and risks breaking something not fully traced in this pass.

## Test plan (for what is built)

- `npm run test` (vitest) must still pass unmodified for all existing suites — this is the regression gate.
- New: `myTrackLayout.test.ts` — loads the shared JSON, asserts 120 cones / 80-20-20 colour split / 1.0 m spacing, mirroring the Python-side `test_my_track_layout.py` assertions exactly (same source file, two consumers).
- New: tests for each added `labAlgorithms.ts` extension (obstacle classifier accepts a compact cluster and rejects a straight wall; single-side fallback gate synthesis; detour lock reuse) — same behavioral assertions as the Python `test_my_track_algorithm.py`/`test_my_track_mission.py` suites, not byte-identical code, so a divergence here is a real finding worth reporting, not a bug to hide.
- No automated TS-vs-Python conformance test in this pass (see above) — any comparison done is manual/reported in prose, not a CI gate.

## Risks

- **Duplicating the algorithm a fourth time instead of truly reusing `labAlgorithms.ts`** is the main risk the master prompt explicitly warns about; mitigated by extending that file's exports rather than writing new standalone functions elsewhere.
- **GLTFLoader availability**: `three@^0.186.1` is already a dependency, but `GLTFLoader` lives under `three/examples/jsm/` or the separate `three-stdlib` package — neither was confirmed usable within this recon pass; first implementation step is confirming which import path works with this Vite setup before writing the loader code.
- **Partial design-system rollout risk**: applying tokens to only the Lab shell while 15 pages keep old styles could look inconsistent (a half-themed app) rather than better than before. Mitigated by only toggling `data-theme` at the shell level and keeping it additive (old CSS keeps working, tokens layer on top) rather than ripping out existing styles.
- **Safety banner text must not change** — any consolidation is verified against the exact test-asserted substrings before landing.
