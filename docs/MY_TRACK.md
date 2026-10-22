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

## What was found and fixed while actually running the matrix (not just unit tests)

Five real bugs surfaced only by running the 12-seed×2-profile matrix, each fixed and verified:

1. **Detour admission had no lock-reuse.** The draft locks a detour once admitted and reuses it verbatim; the first port re-searched from scratch every 0.05 s, so admission that succeeded while an obstacle was still far away could fail once closer, snapping the path back toward the obstacle mid-maneuver. Caused real collisions.
2. **A dead `world_cluster_centroid` callback** always returned the kart's own position instead of the obstacle's actual world position, defeating the lock lookup above before it could even work. Fixed by computing the centroid directly where the body-to-world transform already exists.
3. **Finish-line detection flickered** frame to frame right at the finish (see the recalibration above), swinging the kart between braking and cruising. Fixed by latching the finish's world position on first detection, same anti-flicker principle as the detour lock.
4. **`CONTROLLED_STOP`'s termination check required speed < 0.02 m/s**, but `PlanarBicycleKart` disables brake force below `LOW_SPEED_MPS` (0.05 m/s) and coasts there indefinitely rather than reaching exactly zero — so a run could never end early and always ran the full 400 s cap. Raised to 0.06 m/s, and gated on an actual `mission.fault` (not merely being in `CONTROLLED_STOP`, which can also mean "temporarily stopped while a detour retries" and can still resolve a few seconds later).
5. **Once latched, the finish wasn't actually used for the final approach**: if the lane-following gates thinned out in the last few metres (fewer boundary cones remain that close to the track's end), the mission fell back to blind/PATH_LOST and lost the latched braking target. Fixed by committing to a straight final approach once the finish is latched, instead of depending on fresh gates for the last stretch.

## Known failure mode not fixed: residual collision rate

The nominal 12-seed×2-profile matrix (`footprint_margin_m = 0.20`, the task's stated value) showed **0/24 to 3/24 real collisions** depending on exact run (noise-seed-sensitive — see the actual numbers below). Raising the planning margin to 0.35 m was tried as a fix and made things *worse* on both axes in a full re-run: it did not reduce the total collision count, it only reshuffled which seeds collide vs. abort into `PATH_LOST`, because the whole run is sensitive to the earliest detour decision (a classic chaotic-sensitivity symptom, not evidence the margin itself is sized wrong).

**Root cause, as far as this investigation got**: the swept-footprint check validates the *planned* detour path has ≥ margin clearance, but Pure Pursuit tracks that path via a lookahead point, not exact adherence — real tracking error can eat into a static margin that was only ever validated against the open-loop plan. A real fix needs **closed-loop re-validation of the executing detour** against the kart's actual tracked position (not a bigger static planning margin), which is out of scope for this pass.

**This means: do not treat "0 collisions" as proven for my_track.** The actual matrix numbers (run via `python -m kart_sim.my_track_matrix`) are in `results/my_track/matrix/matrix.csv` and `SUMMARY.md` — read those for the real count, not this document.

## What was scoped out

- **Odometry/pose estimation**: the runner uses the kart's *true* pose and speed for planning, not a separately noisy encoder/gyro dead-reckoning estimate. The task's explicit noise asks (Step 2) were about the HP60C camera specifically, not inertial/wheel-speed sensing, so this was a deliberate scope trim, not an oversight. `gyro_bias_x3` in the Step 9 matrix is a documented no-op for the same reason.
- **Layer B integration**: my_track exists only in the generic SIL simulator. Feeding its cone layout into the real ROS mission/planner/controller nodes, to compare their output against this implementation, was explicitly deferred to a follow-up step by your own decision 2.
- **STL kart model / 3D viewer (Steps 6–7) and wiring all four scenarios into one viewer (Step 8): not done.** This turned out to be a large, separate frontend/3D-asset undertaking (STL decimation, three.js assembly, a 60fps canvas viewer with a dozen toggleable overlays, light/dark theming, phone-width layout, live WebSocket + replay modes) that this pass did not have room for alongside getting the simulation itself actually correct. The 10 Hz/1 Hz telemetry CSVs and `summary.json` this branch does produce are a usable data source for that viewer whenever it's built; the 5 Hz viewer-frame JSON-lines stream (gates, Delaunay edges, swept footprint, etc.) is not yet implemented — the raw data exists inside `my_track_algorithm.py`'s `PlanResult` but isn't currently plumbed out through the runner.
- As noted in [RECON.md](my_track_spec/RECON.md), the real STL folder you have (`~/Desktop/kart stl`) is a 1:5-scale RC hobby model of a different vehicle, not the real kart's CAD — it was never going to be usable for Step 6 as given, regardless of the viewer work above.

## Measurements still owed, before any of this touches the real vehicle

- Real mass, CG position, and yaw inertia (currently ASSUMED 160 kg, CG at the wheelbase midpoint).
- Real brake deceleration curve and dead-time (currently ASSUMED 2.5 m/s² peak, 0.15 s dead time, matching the task's own stated illustration figures, not a measurement).
- Real steering actuator rate and lag (currently ASSUMED 40 deg/s, 0.12 s).
- Real tyre behaviour (cornering stiffness, friction peak — currently ASSUMED, carried over from the unrelated provisional `prototype.yaml`).
- **Which wheelbase is the real vehicle**: 1.422 m (`shared`) or 1.1176 m (`built`) — both profiles exist and are selectable, but this needs confirming before either is trusted for a real run.
- Real cone height (ASSUMED 0.325 m, used only for the simulated camera's pixel-height projection).
- A real sunlight test of the HP60C: it is a structured-light depth sensor, and direct sunlight is a known failure mode for that technology. The simulated `sunlight_degradation` switch (off by default) is a crude range/dropout approximation, not validated against a real outdoor test.

**None of the above has been measured. Nothing in this branch is evidence the real kart is safe to drive autonomously.** An independent hardware E-stop is required for any powered test regardless of what this simulation reports, and the first powered test of the real vehicle must be a supervised creep at very low speed — not a run of this scenario at cruise speed.
