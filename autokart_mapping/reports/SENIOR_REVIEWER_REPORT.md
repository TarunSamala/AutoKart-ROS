# Senior reviewer report — autokart_mapping

Performed as a distinct, sequential review pass over the developer's actual source (this repository
has no multi-agent runtime wired up; per the mission statement's own instruction, this is a genuine
sequential review, not a simulated second agent).

## Defect found and fixed

**A never-configured `algorithm_package_path` was silently accepted instead of failing closed.**
`Path(algorithm_package_path).expanduser()` on an empty string resolves to `.` (the current working
directory), and `.is_dir()` on that is `True` — so `MappingAlgorithm.__init__`'s own
`if not src.is_dir(): raise ...DEPENDENCY_PATH_NOT_FOUND` check never fired for a blank parameter
value (the parameter's own documented, shipped default). On this specific dev box the defect was
further masked: `autokart_perception` is **globally** importable regardless of any path configuration
(confirmed via `pip show autokart-perception` → an editable install pointing straight at the
canonical source), so a `MappingNode()` constructed with no `algorithm_package_path` override at all
loaded a fully working `MappingAlgorithm` and behaved exactly as if it had been correctly configured.

**Reproduced before trusting the fix**: constructed `MappingNode()` directly (no parameter override)
in a scratch workspace with only `/opt/ros/humble` + this node's own install sourced (not
`kart/ros2_ws`) and confirmed `node.algorithm` was a live `MappingAlgorithm` object with
`node.startup_error is None`, despite the declared parameter default being `""`:
```
algorithm: <autokart_mapping.algorithm.MappingAlgorithm object at 0x7f74...> startup_error: None
```
This is exactly the silent-success-on-unconfigured-input failure mode the mission statement's own
"never silently insert nominal/guessed... constants" and the architecture doc's fail-closed
conventions prohibit — on a machine *without* that editable pip install (any normal deployment target
per DEPENDENCIES.md), the same blank path would still resolve to `.` and only fail later, with a less
specific `ALGORITHM_IMPORT_FAILED` reason instead of the intended, clearer
`ALGORITHM_PACKAGE_PATH_NOT_CONFIGURED` fail-closed signal — but on *this* box it would not fail at
all, silently masking a misconfigured deployment.

**Fixed**: `MappingAlgorithm.__init__` now explicitly rejects an empty/whitespace-only
`algorithm_package_path` with `ALGORITHM_PACKAGE_PATH_NOT_CONFIGURED` before ever touching
`Path(...).is_dir()`. **Verified**: rebuilt the package in the scratch workspace and re-ran the same
direct construction — `algorithm: None startup_error: ALGORITHM_PACKAGE_PATH_NOT_CONFIGURED` — and
added a regression test, `test/test_algorithm.py::test_blank_path_fails_closed_instead_of_resolving_to_cwd`,
covering both `""` and whitespace-only input.

## Checked and found correct

- **Skidpad two-ring fit and crossover-margin exclusion band**: confirmed `geometry.py`'s exclusion
  logic (`crossover_margin_m = 1.6`) and the per-ring residual/angular-coverage gates in
  `fit_skidpad_topology`/`_fit_observed_ring` are called unmodified via `self.algorithm.fit_skidpad_topology`
  — grepped this package for any reimplementation of circle-fitting or crossover logic; found none.
  Did **not** re-derive or second-guess that geometry's correctness beyond what the existing, reused
  adapter's own test suite already establishes (ported into `test/test_engine.py`'s exit-corridor
  test) — per the mission statement's instruction not to "fix" algorithm math without surveyed
  ground truth to validate against.
- **Validation/sequencing glue** (`MappingEngine.update`'s frame/timestamp/odometry checks,
  duplicate-cone-id rejection): line-by-line comparison against the existing, reused adapter's own
  `MappingEngine.update` confirms identical logic, re-expressed over `ConeObservation` instead of the
  ROS `Cone`/`ConeArray` message types. No behavioral divergence found.
- **No hidden/hardcoded path**: grepped the package for `/home/mark1` and absolute paths; none in
  source (only in `config/mapping.yaml`'s comment, an example, and in prose docs/reports).
- **Fault behavior**: on a missing/failed algorithm dependency, the node never fabricates a map — the
  empty-map/fault-status path was confirmed by `test/test_fault_injection.py` to republish every
  ~100 ms (timer period), not report once and go silent, matching the convention `autokart_hp60c`
  already established and the mission statement's own repeated-fault-reporting requirement.
- **Test isolation**: `test/conftest.py`'s `sys.modules`/`sys.path` reset fixture was added after this
  reviewer observed `test/test_algorithm.py`'s own "rejects path without autokart_perception" case
  (which deliberately points at a broken fake package) poisoning the Python import cache for every
  later test in the same pytest process that needed the REAL `autokart_perception` — reproduced the
  failure (`ModuleNotFoundError: No module named 'autokart_perception.application'` on a subsequent,
  otherwise-correct test) before adding the fixture, then confirmed the full suite (11 tests) passes
  in any file order afterward.

## Not independently re-verified by this reviewer

- The reused `autokart_perception` algorithm's own internal correctness (Delaunay/Gabriel/gate
  selection/centerline math, the skidpad ring-fit geometry) was authored before this session and is
  unchanged by this node; this review did not re-derive it. If that layer has a latent geometric or
  associativity bug, this node inherits it unmodified — that risk is not introduced or hidden here,
  and is explicitly out of scope per the mission statement ("note it in your senior-reviewer report
  rather than silently 'fixing' math you don't have surveyed ground-truth to validate against"). No
  such bug was found in this review's own exercise of the reused algorithm (all ported/parity tests
  pass against its real, unmodified output), but that is not the same claim as "verified correct
  against a surveyed course," which remains NOT RUN.
