# my_track — handoff

Branch: `feat/my-track`. This covers the Layer A (generic SIL simulator) implementation only, per the agreed scope — see [docs/my_track_spec/RECON.md](my_track_spec/RECON.md) for why Layer B (the real ROS mission stack) was out of scope, and the open question of feeding my_track's cone layout into the real planner/controller as a separate follow-up step.

## Running it

From `kart/`:

```bash
# Single run, writes telemetry to the console only (no files):
python3 -c "
from kart_sim.my_track_runner import default_run_config, run_my_track_scenario
result = run_my_track_scenario(default_run_config('shared', seed=1, cruise_speed_mps=0.5))
print(result.metrics)
"

# Single run with telemetry written to results/my_track/<timestamp>_<profile>_<seed>/:
python3 -c "
from pathlib import Path
from kart_sim.my_track_runner import default_run_config, run_my_track_scenario
from kart_sim.telemetry_writer import write_my_track_run
result = run_my_track_scenario(default_run_config('built', seed=3))
print(write_my_track_run(result, results_root=Path('results'), profile='built', seed=3))
"

# Full Step 9 matrix (nominal x 12 seeds + 8 perturbation variants x 6 seeds, both profiles):
python3 -m kart_sim.my_track_matrix --out results/my_track/matrix
```

Tests: `python3 -m pytest -p no:anyio tests/ --ignore=tests/gazebo_follower` (180+ passing as of this branch; `gazebo_follower` needs ROS and wasn't touched).

## Kart profiles

Two profiles in `kart/configs/karts/{shared,built}.yaml`, selected by filename to `load_kart_config()`. Both add a `shared`/`built` wheelbase/front-track pair from the dimension sheet to the existing `PlanarBicycleKart` dynamics (see [kart_sim/vehicle.py](../kart/kart_sim/vehicle.py)), plus a new `ActuatorLagConfig` (steering rate+lag, brake dead-time+ramp) that's ASSUMED and defaults to instantaneous for every *other* existing kart config, so autocross/skidpad/braking are unaffected.

| Field | shared | built | Status |
|---|---|---|---|
| wheelbase_m | 1.422 | 1.1176 | MEASURED |
| front_track_m | 1.0648 | 1.0668 | MEASURED |
| rear_track_m | 1.1811 | 1.1811 | MEASURED |
| length/width/height_m | 2.400 / 1.181 / 1.118 | same | MEASURED |
| ground_clearance_m | 0.381 | same | MEASURED |
| rear_overhang_m / front_extent_m | 0.40 / 2.00 | same | ASSUMED split of the measured length |
| mass_kg | 160.0 | 160.0 | ASSUMED |
| tyre cornering stiffness, friction, drive/brake force limits | — | — | ASSUMED (carried over from the provisional `prototype.yaml`) |
| steering rate / lag | 40 deg/s / 0.12 s | same | ASSUMED |
| brake dead-time / ramp | 0.15 s / 50 %/s | same | ASSUMED |
| software steering limit | 20 deg | same | task-stated |
| mechanical steering limit | 25 deg | same | task-stated |

## HP60C camera config

`kart/configs/sensors/hp60c.yaml`. MEASURED (datasheet): 640x480, HFOV 64.4°/VFOV 51.7°, depth 0.2–4.0 m, 4.2 Hz, 0.29 s p50 latency. ASSUMED: cone height (0.325 m), recall curve (peak 0.90, ramps 10–35 px), colour-swap rate (2%), depth/lateral noise coefficients, sunlight-degradation magnitudes (switch off by default).

## Algorithm config and its one deliberate deviation from the task's literal numbers

`AlgorithmConfig` in [kart_sim/my_track_algorithm.py](../kart/kart_sim/my_track_algorithm.py) matches the task's stated gate width (2.0–3.8 m), skew (≤45°), obstacle classifier (≥4 cones, lateral range ≥0.28 m), and footprint margin (0.20 m) verbatim — **except** the finish-line pair thresholds, which were recalibrated after running the actual matrix (not just unit tests):

- `finish_pair_min_m`: 0.1 (task said 0.2)
- `finish_pair_max_forward_gap_m`: 0.5 (the draft used 0.7; an intermediate 0.3 was tried and also wrong)

**Why**: the real finish cones sit only 0.5 m from the last boundary-row cones, inside the cone map's 0.6 m association radius. The map partially blends a finish cone with its nearest boundary neighbour, smearing the true 0.35 m lateral gap down to an observed ~0.12–0.16 m, and the true ~0 m forward gap up to ~0.33–0.39 m. The task's literal 0.2 m lower bound and the draft's 0.7 m forward tolerance are each wrong in opposite directions for this specific layout once the map's own blending is accounted for. These values were tuned against actual logged cone-map positions near the finish (not guessed) — see the commit history on this file for the worked numbers.

## What was found and fixed by actually running the matrix (not just unit tests)

Eight real bugs surfaced only by running the matrix and tracing real collisions with added diagnostic telemetry (nearest cone/corner, cross-track error, steering command vs. actual and saturation, detour lock age, cone map age — see `results/my_track/collision_*/trace_10hz.csv`), not by margin tuning:

1. **Detour admission had no lock-reuse.** The draft locks a detour once admitted and reuses it verbatim; the first port re-searched from scratch every 0.05 s, so admission that succeeded while an obstacle was still far away could fail once closer, snapping the path back toward the obstacle mid-maneuver.
2. **A dead `world_cluster_centroid` callback** always returned the kart's own position instead of the obstacle's actual world position, defeating the lock lookup above before it could even work.
3. **Finish-line detection flickered** frame to frame right at the finish, swinging the kart between braking and cruising. Fixed by latching the finish's world position on first detection.
4. **A pre-existing bug in the shared `PlanarBicycleKart` model** (`vehicle.py`, not introduced by this branch): the `direction` sign gating both brake force and rolling resistance was tied to `LOW_SPEED_MPS` (0.05 m/s), so with zero drive, any speed below that floor leaves only negligible aerodynamic drag — the kart coasted indefinitely at its residual speed instead of decelerating to zero. Traced via a 10 Hz trace showing speed pinned at exactly 0.05 m/s for 3 full seconds while "stopped" in `CONTROLLED_STOP`, creeping into the cone it was braking for. Fixed by gating `direction` on a much smaller epsilon (1e-4); verified against the full existing kart_sim suite (182/183 tests unaffected).
5. **`classify_obstacles()` has no temporal persistence**: a flood-fill link distance right at the 0.85 m threshold flickered a real 4-5 cone obstacle cluster from detected to an empty set between two consecutive 0.05 s cycles, and it never recovered before contact. Fixed with the same latching principle as the detour/finish locks: a **track-id-keyed** obstacle memory (not a snapshotted world position, which drifts as the map keeps refining a freshly-confirmed track's estimate) re-includes a cone for 3.0 s after it was last confirmed.
6. **`CONTROLLED_STOP`'s termination check required speed < 0.02 m/s**, which the floor-coast bug (#4) made unreachable — a run could never end early and always ran the full 400 s cap. Raised to 0.06 m/s, gated on an actual `mission.fault` (not merely being in `CONTROLLED_STOP`, which can also mean "temporarily stopped while a detour retries" and can resolve a few seconds later).
7. **Once latched, the finish wasn't actually used for the final approach**: if the lane-following gates thinned out in the last few metres, the mission fell back to blind/PATH_LOST and lost the latched braking target. Fixed by committing to a straight final approach once the finish is latched.
8. **COMPLETE was self-reported, not ground-truth-checked**: the mission's own `STOPPED` state could be reached via a stale/mis-latched finish belief well short of the real line. Fixed (see "completion criterion" below) by checking the front bumper's actual position against the real finish cones' world position, independent of what the mission believes.

All fixes verified against the full existing kart_sim test suite (183 tests) and re-run against the matrix.

## Completion criterion (ground-truth, not self-reported)

A run is **COMPLETE** iff, at the moment it stops: speed < 0.06 m/s, the front bumper is within **[0, 1.5] m before** the real finish line position (never beyond it), zero collisions, and no mission fault. Everything else is **ABORTED** with a specific reason (`collision (N cone(s))`, `overshot finish by X m`, `stopped X m short of the finish`, the fault string, or `did not stop`). This is checked against the ground-truth world position of the real `FINISH-*` cones, not the mission's own (fallible) finish-detection belief — several earlier runs had reported `COMPLETE` via a false finish latch 6-7 m short of the real line.

## Final nominal matrix (12 seeds × 2 profiles, both root-cause fixes applied, strict completion criterion)

**6/24 COMPLETE, 2/24 have a real collision** (`shared` seed 2: 1 cone; `built` seed 6: 3 cones — both `OB1`/boundary contacts near an obstacle, down from 5/24 before the fixes above). Full per-seed output: `results/my_track/final_nominal_matrix.csv`. The dominant remaining failure mode is **not** collision but **stopping short**: 16/24 runs stop somewhere between 1.8 m and 7.4 m before the real finish line.

**Two more issues found, traced, but NOT fixed** (time-boxed; reporting rather than claiming a 0 this branch does not show):

- **Residual collision rate (2/24).** Traced in depth for both remaining cases: confirmed NOT a classification flicker (the new hysteresis correctly and continuously recognizes the obstacle cluster through the collision — verified via its own diagnostic output). The admitted detour's lateral offset is simply too small for the obstacle's actual position in this specific geometry — a distinct bug in detour *sizing* (the `ramp_height_m` calculation), not detection or locking. Not root-caused further; traces in `results/my_track/collision_shared_2/`.
- **"Stopped short" (16/24, the dominant pattern now that it's no longer masked by a lenient completion check).** At least one traced case (`shared` seed 3) shows the mission reaching a clean, genuine `STOPPED` state (speed exactly 0.0, confirming fix #4 works) at odometry ≈42 m — a **false finish-line latch** around the 40 m gate threshold, the same failure family as the forward/lateral-gap recalibration earlier in this document, evidently not fully eliminated. Not root-caused further.

**Do not treat either "0 collisions" or "reaches the finish" as proven for my_track.** Re-run `python -m kart_sim.my_track_matrix` and read the actual CSV for the current numbers, not this paragraph.

## What was scoped out

- **Odometry/pose estimation**: the runner uses the kart's *true* pose and speed for planning, not a separately noisy encoder/gyro dead-reckoning estimate. The task's explicit noise asks (Step 2) were about the HP60C camera specifically, not inertial/wheel-speed sensing, so this was a deliberate scope trim, not an oversight. `gyro_bias_x3` in the Step 9 matrix is a documented no-op for the same reason.
- **Layer B integration**: my_track exists only in the generic SIL simulator. Feeding its cone layout into the real ROS mission/planner/controller nodes, to compare their output against this implementation, was explicitly deferred to a follow-up step by your own decision 2.
- **STL kart model (D2): done**, within the constraint that the source is the wrong vehicle (see below). `kart/tools/prepare_kart_models.py` assembles Body+Chassis+RearAxle+Steering, scales uniformly on length (5.438x), decimates to 149,998 triangles (4.1 MB), and synthesizes proxy front wheels per profile. Residuals vs. the dimension sheet logged in `docs/my_track_spec/STL_FIT.md`: width +1.18 m, height -0.39 m — this model's proportions are genuinely different from the real kart's. Engine excluded (8.1M raw triangles, disclosed not hidden).
- **Viewer (D3): a real, working, but deliberately scoped-down 2D replay viewer exists** at `kart/viewer/index.html` — top bar, top-down world view (cones, kart footprint, trail, camera wedge, detour offset), strip chart, gauges, state chip, per-second table, light/dark theme, verified in-browser at phone width with no console errors. **Not done**: live WebSocket mode (replay only), Delaunay/Gabriel/gates/centreline/swept-footprint overlays (`MissionCommand` doesn't expose that internal `PlanResult` state out to the runner yet — the data exists inside `my_track_algorithm.py`, just not plumbed through), a 3D/STL view in the viewer itself (the D2 assets exist but aren't wired in), the HP60C camera detection panel (no per-frame image/detection data is stored), and automated screenshot tests (verified manually in-session instead).
- **Wiring all four scenarios (D4): partial.** `kart_sim.viewer_export_generic` adapts any existing `kart_sim.scenario` telemetry.jsonl run to the viewer's replay schema, and this was verified end-to-end in the browser against a pre-recorded run — proving the viewer isn't my_track-specific. **Running skidpad/braking/autocross fresh currently fails** with a pre-existing bug unrelated to this branch: `SimulationError("kart is not autonomously armed after reset")` in `kart_sim/fast_env.py` (reproduce: `bash scripts/run_formula_student_event.sh skidpad`). Reported here rather than fixed, per the ground rule against touching VCU/arming behaviour. The full "4 scenarios × 2 profiles run end to end" matrix was not completed as a result.
- As noted in [RECON.md](my_track_spec/RECON.md), the real STL folder you have (`~/Desktop/kart stl `, trailing space is part of the real name) is a 1:5-scale RC hobby model of a different vehicle, not the real kart's CAD.

## Measurements still owed, before any of this touches the real vehicle

- Real mass, CG position, and yaw inertia (currently ASSUMED 160 kg, CG at the wheelbase midpoint).
- Real brake deceleration curve and dead-time (currently ASSUMED 2.5 m/s² peak, 0.15 s dead time, matching the task's own stated illustration figures, not a measurement).
- Real steering actuator rate and lag (currently ASSUMED 40 deg/s, 0.12 s).
- Real tyre behaviour (cornering stiffness, friction peak — currently ASSUMED, carried over from the unrelated provisional `prototype.yaml`).
- **Which wheelbase is the real vehicle**: 1.422 m (`shared`) or 1.1176 m (`built`) — both profiles exist and are selectable, but this needs confirming before either is trusted for a real run.
- Real cone height (ASSUMED 0.325 m, used only for the simulated camera's pixel-height projection).
- A real sunlight test of the HP60C: it is a structured-light depth sensor, and direct sunlight is a known failure mode for that technology. The simulated `sunlight_degradation` switch (off by default) is a crude range/dropout approximation, not validated against a real outdoor test.

**None of the above has been measured. Nothing in this branch is evidence the real kart is safe to drive autonomously.** An independent hardware E-stop is required for any powered test regardless of what this simulation reports, and the first powered test of the real vehicle must be a supervised creep at very low speed — not a run of this scenario at cruise speed.
