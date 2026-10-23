# phase_sim MY_TRACK architecture (developer round 2)

Scope: the TS port of `kart/kart_sim/my_track_algorithm.py` and
`my_track_mission.py` in `phase_sim/src/mission/labAlgorithms.ts`, wired into
`LabSkidpad.ts`, plus the 3D kart model (`kartModel.ts`/`MissionScene.tsx`).
Read-only Python references cited below; never edited from this side.

## Persistent cone map (Task 1 fix)

Round 1 (`ccef9a8`) refit the centreline fresh every frame from only that
frame's raw `observeCones()` output inside the forward/lateral body window
(`myTrackBodyFramePoints`). That lagged through the two lateral bend-shifts
on the real course and measured ~0.60 m max lateral deviation.

`MyTrackConeMap` (new, in `labAlgorithms.ts`) ports the persistent,
pose-aware map pattern from `kart/kart_sim/hp60c.py`'s `ConeMap`: detections
accumulate in the WORLD frame across frames; a new detection within 0.6 m of
an existing map cone is associated and averaged into it; a cone is
"confirmed" only once seen >= 2 times; an unseen cone expires after 10 s;
class is decided by majority vote. `LabSkidpad.stepMyTrack()` now updates
the map every frame and builds gates from `map.confirmed()`, not from the
frame's raw detections. See the doc comment on `MyTrackConeMap` for the
full rationale.

Result (12 seeds x 2 profiles, outside the two detour zones -- see below):
max lateral deviation improved from ~0.50-0.72 m (round 1) to ~0.11-0.17 m
(round 2), against the 0.25 m target. Full numbers in the round-2 developer
handback.

**Detour-zone exclusion used for the benchmark:** the two obstacle clusters
in `src/mission/data/my_track_layout.json` sit at world x in
[17.775, 19.175] and [37.551, 38.951]. Excluded: any frame with
`planning.skidpad.state === 'DETOUR'`, OR whose vehicle x falls in
[12.775, 24.175] or [32.551, 43.951] (a 5 m buffer around each cluster, to
also exclude the ramp-in/ramp-out approach and recovery either side of the
detour itself).

**Note on the "2 profiles" axis:** under pure pursuit, the commanded front
steering ANGLE differs by profile (smaller wheelbase -> smaller angle for
the same curvature -- see `computeAckermannAngles`/`MyTrackMission.step`'s
`steeringRad`), but the resulting path CURVATURE, and therefore the
resulting x/y trajectory and lateral deviation, is wheelbase-invariant by
construction of the pure-pursuit formula
(`atan2(2*wheelbase*sin(alpha), ld)` fed into `v/wheelbase*tan(delta)`: the
wheelbase cancels). So shared/built produce numerically identical lateral
deviation traces in this benchmark -- that is the expected, correct
behaviour of the control law, not a wiring bug. Verified by direct
comparison (see developer handback).

## FSM state mapping (Task 4 / documentation)

Python's `MISSION_STATES` (`my_track_mission.py:37`):
`READY, RUN, OBSTACLE_ASSESS, DETOUR, RECENTER, FINISH_BRAKING, CONTROLLED_STOP, PATH_LOST, STOPPED`
(9 states). TS's `MyTrackState` (`labAlgorithms.ts`): `READY, RUN, DETOUR,
FINISH, STOPPED` (5 states) -- an intentional, already-approved
simplification (per the task brief), not a conformance gap:

| Python state | TS equivalent |
|---|---|
| READY | READY |
| RUN | RUN |
| OBSTACLE_ASSESS | RUN (capped target speed while a cluster can't be admitted for detour) |
| DETOUR | DETOUR |
| RECENTER | RUN (folded in; no separate recentring phase, the pure-pursuit aim recentres implicitly) |
| FINISH_BRAKING | FINISH |
| CONTROLLED_STOP | RUN (capped target speed via `maxSpeedForStopDistance`, no obstacle-stop state) |
| PATH_LOST | RUN with `blind: true` (blind creep) |
| STOPPED | STOPPED |

## Completion label (Task 2 fix)

COMPLETE rule: stopped (`speed ~= 0`), front bumper within `[0, 1.5]` m
before the finish line (never beyond), no fault -> TS `state === 'STOPPED'`.
Round 1's condition (`stopGapM < 0.1 && speedMps < 0.06`) left a kart that
braked to a genuine, full stop with the bumper anywhere in `(0.1, 1.5]` m
stuck in `'FINISH'` forever. Fixed in `MyTrackMission.step()`: widened to
`stopGapM in [0, 1.5]`. A gap `< 0` (bumper already past the line) is never
COMPLETE regardless of speed. No enum/name change, so the designer's
existing Chip rendering of `MyTrackState` is unaffected. See
`myTrackMission.test.ts`'s "MY_TRACK completion label" tests.

## Kart model placement and scaling (Task 3)

- `kartModel.ts`'s `computeFrontWheelLocalPosition(profile, side)` is the one
  true source for front-wheel hub placement (rear-axle-referenced,
  `x = wheelbaseM`, `z = +/- frontTrackM/2`), applied explicitly via
  `wheel.position.set(...)` in `loadKartModel()` rather than trusting the
  glb's own baked transform.
- `computeBodyForwardScale(profile)` returns
  `wheelbaseM(profile) / wheelbaseM('shared')` (~0.786 for 'built'), applied
  as `part.scale.set(scale, 1, 1)` to each of `body/chassis/rear_axle/steering`
  BEFORE the Z-up-to-Y-up quaternion rotation -- Three.js applies an
  Object3D's scale in its own pre-rotation local axes (matrix = T*R*S), and
  the STL prep's native output X is forward (see `STL_FIT.md`), so this
  compresses length only; width/height are untouched.
- `getKartInfoPanelData(profile)` exports the "visual model; physics uses
  the dimension sheet" info-panel content (STL scale factor 5.438,
  wheelbase, tracks, overall size) as data only -- no UI built here; a
  future/existing designer info panel calls this directly.
- `LabSkidpad`'s constructor now takes an optional `kartProfile` (default
  `'shared'`) and drives the MY_TRACK mission's own geometry
  (`myTrackGeometryFor`, reusing `kartModel.ts`'s `PROFILE_GEOMETRY` instead
  of a second hand-copied constant) from it, instead of always hardcoding
  the 'shared' wheelbase regardless of which profile was asked for.
- `runContext.tsx` gained a `kartProfile: KartProfile` field on `RunConfig`
  (default `'shared'`) as the shared value a future UI control sets via
  `setRun({ kartProfile: 'built' })`; a page then reads `run.kartProfile`
  and passes it to `<MissionScene kartProfile={...} />` (prop already
  existed from round 1) and to `new LabSkidpad(name, settings, run.kartProfile)`.
  No page wires a control to it yet -- this is deliberately minimal
  plumbing, not a UI change (`runContext.tsx` and pages/* are outside this
  agent's file ownership).

Placement is tested in `src/test/kartModel.test.ts` against the pure
position-computation functions (GLTFLoader cannot run in this project's
vitest/jsdom environment -- it needs `fetch` against a real dev-server path
for binary .glb assets, and nothing in the test setup serves or parses glb
binaries in Node).
