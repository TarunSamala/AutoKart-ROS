# Senior reviewer report — autokart_planner

Performed as a distinct, sequential adversarial review pass over the developer's actual source and
tests (this repository has no multi-agent runtime wired up; per the mission statement's own
instruction, this is a genuine sequential review, not a simulated second agent). Three real defects
were found and fixed, each with a regression test, two of them confirmed crashing/failing on the
pre-fix code before the fix was trusted.

## Defects found and their resolution

1. **Misconfigured `curvature_eps_m_inv<=0` crashes the node with an uncaught
   `ZeroDivisionError`** — `PlannerCore.evaluate`'s speed-profile comprehension computes
   `a_lat_max_mps2 / max(abs(k), self.curvature_eps_m_inv)`. On a perfectly straight path segment
   (`kappa=0`, the common case) with `curvature_eps_m_inv` set to `0.0` (or negative), this becomes
   division by exactly zero. **Reproduced before fixing**: constructed a `PlannerCore` with
   `curvature_eps_m_inv=0.0` and a 5-point straight path and confirmed
   `ZeroDivisionError: float division by zero` was raised out of `evaluate()` — this would surface
   inside a ROS subscription callback (`PlannerNode.on_path` → `_process` → `evaluate`), which
   `rclpy` does not restart automatically; depending on the executor, this can silently stop
   further processing on that callback while the process otherwise looks alive. **Fixed**: added a
   configuration-qualification check (`CURVATURE_EPSILON_UNQUALIFIED`) alongside the existing
   `LATERAL_LIMIT_UNQUALIFIED`/`FOOTPRINT_WIDTH_UNQUALIFIED` checks, rejecting instead of crashing.
   **Verified** with `test_zero_curvature_epsilon_rejects_instead_of_dividing_by_zero` and
   `test_negative_curvature_epsilon_also_rejects` in `test_planner_core.py`.

2. **A planner that never receives any reference path stays silent forever, indistinguishable
   from "not started yet"** — `PlannerNode.last_path_wall_s` started as `None`, and
   `on_timer`'s staleness check only fires once `last_path_wall_s is not None`, i.e. only after the
   *first* path message has ever arrived. If the upstream mission/mapping node (06/07, not built in
   this library) never starts, or its first message is delayed past the configured timeout, this
   node would never publish anything on `/autokart_planner/safety_envelope` at all — not even a
   `PATH_MISSING_OR_STALE` report. This is exactly the "silent instead of reporting a fault"
   failure mode the mission statement prohibits, and the same category of bug
   `autokart_hp60c`'s own reviewer pass found in its capture thread (a persistent fault reported
   only once, or never). **Fixed**: `last_path_wall_s` is now initialized to the construction-time
   wall clock, so the staleness timer starts counting from node startup, not from the first
   message. **Verified** with `test_never_receiving_any_path_still_reports_stale_after_startup` in
   `test_fault_injection.py`, which constructs the node, publishes nothing to the reference-path
   topic at all, and asserts a `valid=false` envelope is still reported within
   `path_max_age_s`.

3. **`ReusedAlgorithmBackend`'s dependency-path resolution could silently bind to the wrong
   directory** — the original `if src_str not in sys.path: sys.path.insert(0, src_str)` guard
   (copied from `autokart_hp60c/backends.py`'s identical pattern) only inserts the configured path
   when it is *completely absent* from `sys.path`. This dev environment's own baked-in search path
   already contains the real `autokart_perception` algorithm package's directory (confirmed:
   `python3 -c "import sys; print(sys.path)"` lists
   `autonomy_stack_phase1/phase_01_perception/src` without any explicit insertion). Combined with
   an earlier test that deliberately creates a *fake*, incomplete `autokart_perception` package
   under a different `tmp_path` directory and inserts **that** path at the front of `sys.path`,
   this produced a real, reproduced failure: a later `ReusedAlgorithmBackend` construction with the
   *correct* real path still failed with
   `ModuleNotFoundError: No module named 'autokart_perception.curvature'`, because Python's package
   finder resolved the top-level `autokart_perception` name against the fake directory (still at
   the front of `sys.path` from the earlier test) rather than the real one, and the "insert if
   absent" guard did nothing since the real path technically was already present, just farther
   back in the list. **Reproduced and observed directly** during this node's own test development
   (not a hypothetical — see the actual pytest failure this review started from, captured in git
   history of this session's tool transcript). **Fixed**: `ReusedAlgorithmBackend.__init__` now
   removes any existing occurrence of the configured path and re-inserts it at the front every
   time, guaranteeing the intended dependency always takes precedence regardless of what else is on
   `sys.path`. **Verified**: `test_builtin_and_reused_curvature_agree_on_a_circular_arc` and
   `test_builtin_and_reused_stopping_distance_agree` (which construct a `ReusedAlgorithmBackend`
   *after* `test_reused_backend_rejects_path_without_autokart_perception` has already polluted
   `sys.path` with a fake package, by file ordering) now pass; reverting the fix locally reproduces
   the failure. Also added an autouse fixture (`_clean_autokart_perception_imports`) to
   `test_algorithms.py` restoring `sys.path`/`sys.modules` after every test, as defense in depth —
   this does not substitute for the production fix, since the production fix is what makes the
   *correct* path win even when the test hygiene fixture is absent.
   `autokart_hp60c/backends.py::HP60CBackend` carries the identical latent pattern; it was not
   modified (out of scope for this session, flagged here for that node's own next review) since its
   own test suite never constructs two different dependency paths in one process and so never
   triggers it.

## Checked and found correct

- **Curvature formula**: `BuiltinAlgorithmBackend`'s centered finite-difference implementation of
  `kappa = (x'y''-x''y')/(x'^2+y'^2)^1.5` matches the standard definition; verified numerically
  against a known circular arc (`kappa == 1/radius` within 2%) and against the reused, already-
  tested `profile_centerline` implementation (within 10%, looser since the two use different
  discretizations — a three-point circle fit vs. finite differences — and are expected to agree
  only approximately, not bit-for-bit).
- **Stop-envelope formula**: both backends compute exactly
  `v*t_total_latency + v^2/(2*a_brake_min) + d_margin`; cross-checked bit-for-bit (`abs=1e-9`)
  against `autokart_perception.shared_core.BrakeCapability.stopping_distance` for five speeds.
- **Fail-closed defaults**: `a_brake_min_mps2=0.0` and `t_total_latency_s=0.0` are the shipped
  config defaults (not just code defaults), confirmed in both `config/planner_builtin.yaml` and
  `config/planner_reused.yaml`; `test_unqualified_brake_capability_fails_closed_by_default`
  confirms the engine actually rejects rather than computing a hidden/implicit braking assumption.
- **No hidden/hardcoded path**: grepped the package for `/home/mark1` and absolute paths; the only
  occurrences are in `config/planner_reused.yaml`'s *comment* (an example, not an active default)
  and in docs/reports (prose). Active default parameter values are empty strings.
- **Empty-path-on-invalid convention**: matches `docs/NODE_CONTRACTS.md`'s documented behavior for
  the existing `kart/ros2_ws` planner, confirmed by `test_obstacle_within_stopping_distance_blocks_
  and_zeroes_speed_profile` and the integration test's envelope/path pairing.

## Not independently re-verified by this reviewer

- The native curvature/stopping-distance math inside `autokart_perception.curvature`/
  `shared_core` (authored before this session, unchanged by this node) was not re-derived from
  first principles — consistent with "do not rewrite/re-litigate working algorithms
  unnecessarily." If that implementation has a latent defect, this node's `reused` backend
  inherits it; that risk is not introduced or hidden by this change, and the `builtin` backend
  (independently implemented from the master architecture doc's own formula) provides a basis for
  future divergence detection if one ever appears.
- `a_lat_max_mps2`'s numeric value (`2.0`) was not independently chosen or justified by this
  reviewer beyond confirming it is clearly labeled UNVERIFIED in every place it appears
  (config comments, CONTRACT.md, node.py parameter declaration comment) and is never silently
  treated as measured.
