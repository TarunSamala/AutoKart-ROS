<!--
No GitHub remote is configured for this repo yet, so this was written as a
file instead of an actual PR (per Step 10: "open a pull request description
... do not merge"). Once a remote exists, use this as the PR body, e.g.:

  gh pr create --base main --head feat/my-track --title "my_track scenario (Layer A)" --body-file docs/my_track_spec/PR_DESCRIPTION.md
-->

## Summary

Adds a fourth SIL scenario, `my_track`, next to skidpad/braking/autocross in the generic simulator (`kart/kart_sim/`) only — the real ROS mission stack (Layer B) was explicitly out of scope for this pass; see [RECON.md](../my_track_spec/RECON.md).

- Two vehicle profiles (`shared` wheelbase 1.422 m, `built` wheelbase 1.1176 m) added to the existing `PlanarBicycleKart` dynamics, with a new actuator-lag model (steering rate/lag, brake dead-time/ramp) that defaults to instantaneous so every other existing scenario is unaffected.
- A simulated HP60C pinhole camera model and persistent cone map — nothing like this existed anywhere in the repo before (the real camera driver is a hardware SDK wrapper with no simulation).
- The perception-to-path algorithm chain (Delaunay gates, single-side fallback, robust centreline, obstacle classifier, swept-footprint detour admission with locking, finish-line detection) and mission FSM, ported from the pre-existing `docs/my_track_spec/MY_TRACK_SIM.py` draft into the repo's real module structure, reusing the existing `PurePursuitController`/`LongitudinalPidController` rather than the draft's own inline controller math.
- A telemetry writer (10 Hz/1 Hz CSV + JSON summary) and a Step 9 scenario matrix (`python -m kart_sim.my_track_matrix`).

## What this branch is NOT

- Not a viewer. Steps 6-8 (STL assembly, the real-time 2D/3D viewer, wiring all four scenarios into it) are not done — see `docs/MY_TRACK.md` "What was scoped out" for why, and what's usable now (the telemetry CSVs) for whoever picks that up.
- Not proof the real vehicle is safe. Every dynamics/actuator/perception parameter not on the measured dimension sheet is ASSUMED and tagged as such; see `docs/MY_TRACK.md` for the full list and what's still owed.
- Not collision-free. The nominal matrix shows a real residual collision rate (see the Test plan below for the actual number from this run) traced to Pure Pursuit tracking error against a locked detour offset. Raising the planning margin was tried and made it worse, not better — documented in `docs/MY_TRACK.md` rather than silently left at a "fixed" margin.

## Test plan

- [x] `python3 -m pytest -p no:anyio tests/ --ignore=tests/gazebo_follower` — 180+ tests passing, including all new ones (profile dynamics, HP60C, layout loader, algorithm primitives, mission controller, runner, telemetry writer, matrix).
- [x] Three existing prototype kart configs still validate and their own tests still pass unchanged (actuator-lag defaults are instantaneous).
- [ ] Full Step 9 matrix result (`results/my_track/matrix/matrix.csv` and `SUMMARY.md`) — **paste the actual completion/collision numbers here once the background run this PR's author kicked off finishes; do not state a pass rate that run doesn't show.**
