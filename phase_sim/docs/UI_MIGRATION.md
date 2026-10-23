# UI_MIGRATION.md

Written by the designer agent, round 2, closing the gap `REVAMP_PLAN.md`
left open under "What is explicitly NOT done in this pass" (full 16-page
token migration, nav regrouping, theme toggle, HP60C camera panel, cone
silhouettes). This file says plainly what is real vs not, per page.

## Design tokens: `.ui-root`-scoped -> `:root`-scoped

`phase_sim/src/ui/tokens.css` previously defined `--ui-*` custom
properties only under `.ui-root` (so only `/mytrack`, which wraps its
content in a `.ui-root` element, picked them up). They are now defined at
`:root`, imported once by `LabApp.tsx`, so every page under the Lab shell
sees the same variables regardless of whether it wraps itself in
`.ui-root`. `.ui-root` itself is kept as a harmless alias for any
component that still expects to find the vars on that specific ancestor.

Theme resolution: `<html data-theme="light|dark">` (manual choice,
persisted to `localStorage`) > `prefers-color-scheme` media query (system
default, before any manual choice) > the bare `:root` light fallback. See
`src/ui/ThemeToggle.tsx`.

## Per-page migration status (16 routes)

| Route | Label | Backing CSS | Migrated? | Notes |
|---|---|---|---|---|
| `/home` | Home | `lab.css` (`.lab-card`/`.lab-doc`/`.lab-table`/`.lab-form`) | **Yes** | Shares `lab.css`'s chrome classes; migrated via that file. |
| `/core` | Core Lab | `sharedLab.css` | **Partial** | Page background, panel background, borders, primary/muted text migrated to `--ui-*`. The stage-ribbon state colors, cone-dot colors and per-stage figure stroke colors (triangle/gabriel/gate/path/lookahead/target) are left as literals -- same "fixed diagram accent" convention this repo already uses for cone colors, not full chrome. |
| `/delaunay` | Delaunay | `lab.css` (`CoreStagePage`, `.lab-figure`/`.fig-*`) | **Partial** | Card/table/figure *background* migrated. The `.fig-*` SVG diagram stroke colors (triangle, delaunay edges, gabriel accept/reject, gate colors) are literals, intentionally -- diagram accents, not chrome. |
| `/gabriel` | Gabriel | same as Delaunay (`CoreStagePage`) | **Partial** | Same as above. |
| `/gates` | Gates | same as Delaunay (`CoreStagePage`) | **Partial** | Same as above. |
| `/paths` | Paths | same as Delaunay (`CoreStagePage`) | **Partial** | Same as above. |
| `/mission` | Mission | `mission.css` | **Partial** | Top bar, world background, borders, primary/muted text migrated. The 3D-HUD overlay's decorative per-state tints (legend swatches, floating-panel accent colors, obstacle/braking tints) are left as literals -- same convention. |
| `/mytrack` | MY_TRACK | `myTrackPage.css` | **Yes** | Already token-based since round 1; the HP60C camera panel's dark "lens" background is deliberately fixed regardless of theme (a real camera frame isn't themeable), documented inline in the CSS. |
| `/scenario` | Scenario | `lab.css` | **Yes** | Shares `lab.css`'s chrome classes. |
| `/demo` | Demo | `lab.css` (frame only; the embedded workbench's own internal styles are untouched, out of scope -- it's a reused desktop component, not rebuilt) | **Partial** | The Lab-shell frame around it is themed; the embedded `App.tsx` workbench's own `styles.css` is NOT migrated (different app, outside this page's or this round's file ownership risk; flagged, not touched). |
| `/dataset` | Dataset | `lab.css` | **Yes** | Shares `lab.css`'s chrome classes. |
| `/replay` | Replay | `mission.css` (same `MissionPage` component as `/mission`) | **Partial** | Same as `/mission`. |
| `/perception` | Perception | `lab.css` (`NotImplementedPage`) | **Yes** | Placeholder chrome (`.lab-unavailable-state`) migrated. |
| `/depth-calibration` | Depth & Calib. | `lab.css` (`NotImplementedPage`) | **Yes** | Same as Perception. |
| `/benchmark` | Benchmark | `lab.css` (`NotImplementedPage`) | **Yes** | Same as Perception. |
| `/faults` | Faults | `lab.css` (`NotImplementedPage`) | **Yes** | Same as Perception. |
| `/logs` | Logs & Config | `lab.css` | **Yes** | Shares `lab.css`'s chrome classes. |

**Honest summary:** 9 of 16 pages (Home, Scenario, Dataset, Logs, MY_TRACK,
and the 4 NOT IMPLEMENTED placeholders) have their full visual chrome on
tokens. 6 more (Core, Delaunay, Gabriel, Gates, Paths, Mission, Replay --
7 actually, counting Replay) have their page-level chrome (backgrounds,
borders, primary/muted text) migrated but keep literal, documented
diagram/HUD accent colors by design. `/demo`'s embedded workbench is the
one page whose *internal* styling was not touched at all, because it's a
reused third-party-style component (`App.tsx`), not part of the page
chrome this task scoped.

## Nav regrouping (Task 3)

`LabApp.tsx`'s `LabNavBar` now renders one cluster per `route.group` (the
`RUN` / `ALGORITHM CORE` / `MISSION & SIM` / `DATA` / `QA & OPS` metadata
already on `LAB_ROUTES` in `routes.ts`) instead of one flat 17-link strip.
The 4 `NOT IMPLEMENTED` routes (Perception, Depth & Calib., Benchmark,
Faults) are pulled out of their groups into a single "Unavailable in this
build" dropdown (`UnavailableMenu`) instead of showing as individual
disabled inline tabs.

## HP60C camera panel (Task 4)

`MyTrackPage.tsx`'s `Hp60cFrame` now does a real pinhole projection
(`projectToCamera`, HFOV 64.4° / VFOV 51.7° / 4° downward pitch, ported
from `kart/kart_sim/hp60c.py`'s `_project()` as a read-only reference, not
imported) instead of drawing hand-placed boxes. Cones are drawn as
`ConeGlyph` silhouettes in their true color; a cone present in
`frame.perception.cones` (the actual detected/tracked set) gets a
tracking ring + depth label, one absent from that set but still in the
ground-truth/course set and geometrically in view gets a faded silhouette
+ tick mark. Cones beyond 4m or outside the HFOV are not drawn; cones
beyond 3.5m fade linearly to ~25% opacity by 4m (a documented, not
reference-sourced, choice). Hovering a cone in either the camera panel or
Follow View sets a shared `hoveredConeId` that highlights it in both.

## Cone silhouettes (Task 5)

`src/ui/ConeGlyph.tsx` adds `ConeGlyph` (teardrop/triangle-with-base
silhouette), `TrackingRing`, and `MissTick`. Track overview and Follow
View on `/mytrack` now use `ConeGlyph` instead of `<circle>`; the camera
panel reuses the same component for Task 4.

## Telemetry export UI (Task 6)

`src/ui/telemetryExport.ts` adds `downloadText`, `downsampleByRate` and
`exportCsvAtRate`/`exportJsonSummary`, built on the developer's existing
`exportTelemetryCsv`/`exportTelemetryJson` (`labAlgorithms.ts`, untouched)
and the same `Blob` + anchor-click download pattern `useMissionData.ts`'s
`exportReplay` already uses. Buttons added to both the Mission page
footer and the MY_TRACK toolbar: "Export 1Hz CSV", "Export 10Hz CSV",
"Export JSON summary", plus "Export replay .jsonl" on MY_TRACK (reusing
`data.exportReplay` directly, the same function behind Mission's existing
"SAVE RUN (.jsonl)" button). Works for all four scenarios since they all
produce the same `MissionFrame` shape.

## A note on this round's working conditions

Partway through this round, several in-flight edits to shared files
(`lab.css`, `LabApp.tsx`, `tokens.css`, `mission.css`, `sharedLab.css`,
`MissionPage.tsx`, part of `MyTrackPage.tsx`) were reverted to their
pre-edit `git HEAD` state by what appears to be a concurrent git
operation from another agent in this same (non-worktree) checkout. All of
the above was redone and committed in small, immediate commits afterward
to reduce the window for it to happen again. If any of the above looks
less complete than this document claims, that reversion is the likely
cause -- flagging it here rather than hiding it.
