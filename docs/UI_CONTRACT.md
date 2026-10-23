# UI_CONTRACT.md

Written by the developer agent during the MY_TRACK overnight parallel run,
because this file did not exist yet when the `lateralDeviationM` field was
needed and the designer had already added a field with this name (with a
comment pointing here) to `phase_sim/src/mission/schema.ts` before this
file existed. This doc matches the designer's placement; if anything below
needs to change, edit this file and `schema.ts` together rather than
re-defining the field differently in a second place.

## `MissionFrame.lateralDeviationM` (optional field, top-level on `MissionFrame`)

- **Type:** `number | undefined`, metres, signed.
- **Reference point:** the kart's rear axle (the same body-frame origin
  used throughout `kart/kart_sim/my_track_algorithm.py` and this repo's
  `labAlgorithms.ts`).
- **Reference line:** the perpendicular distance from the rear axle to the
  **currently planned path** (the fitted/evaluated centreline, including
  any active detour offset) -- not the raw unmodified centreline fit, and
  not the course's nominal straight guideline.
- **Sign:** positive = kart is to the **left** of the direction of travel
  (y-left, x-forward body frame convention used everywhere else in this
  codebase); negative = right.
- **Scope:** only populated for the `MY_TRACK` scenario in the Lab
  (`source: 'LAB'`, `scenario: 'MY_TRACK'`). Other scenarios/frames leave
  it `undefined` ("no centreline fit available for this frame" per the
  field's own doc comment in `schema.ts`), including MY_TRACK frames
  during blind creep or PATH_LOST, where there is no fitted path yet.
- **Population:** emitted once per frame by `LabSkidpad.getFrame()` when
  driving MY_TRACK, computed by `lateralDeviationM()` in
  `phase_sim/src/mission/labAlgorithms.ts`.

## Helper functions (pure, exported from `labAlgorithms.ts`)

```ts
maxAbsLateralDeviationM(frames: MissionFrame[]): number
rmsLateralDeviationM(frames: MissionFrame[]): number
```

Both read `frame.lateralDeviationM` (top-level, not `frame.vehicle...`),
treating missing/undefined values as absent (not zero: a frame with no
fit does not count as "zero deviation"), and return `0` if no frame in
the array has the field set.
