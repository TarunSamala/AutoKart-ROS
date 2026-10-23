# Developer report — autokart_mapping

## Audit performed before writing any code

- Read `kart/ros2_ws/src/autokart_nodes/autokart_mapping/autokart_mapping/node.py` (326 lines) in
  full: `MappingEngine` (validation/sequencing: frame/ground-truth/source/timestamp/odometry
  qualification, duplicate-cone-id rejection), `observed_exit_centerline` (blue/yellow exit-gate
  pairing over the odom-frame tracked-cone map), and `MappingNode` (ROS pub/sub wiring, perception
  status cross-check, timeouts).
- Read `kart/ros2_ws/src/autokart_nodes/autokart_mapping/test/test_mapping_adapter.py` in full to
  confirm expected behavior before porting any of it.
- Grepped the repo for "delaunay"/"gabriel" (case-insensitive) and confirmed via
  `docs/decisions/CANONICAL_COMPONENTS.md` that the canonical, authoritative algorithm source is
  `autonomy_stack_phase1/phase_01_perception/src/autokart_perception/{application,shared_core,
  geometry,gabriel,gates,boundaries,centerline,curvature,safety,types,sdk}.py` — read all of these in
  full except `sdk.py`'s `process_frame` path (RGB-D pipeline, out of scope for a cone-array-input
  node) and `safety.py`/`gates.py`/`boundaries.py`/`centerline.py`/`curvature.py`/`gabriel.py`/
  `geometry.py` at the level needed to confirm what `AlgorithmCore.evaluate()` returns, since this
  node calls that one entry point rather than any of those modules directly.
- Confirmed `autokart_msgs` already has `Cone`, `ConeArray`, `MissionDebug` — sufficient for this
  node's contract; no `Graph`/`Gate`/`Boundaries` message exists (also independently confirmed by this
  session's own `AUDIT_PHASE0.md`/`INTERFACE_FREEZE_PHASE1.md` and by `docs/NODE_CONTRACTS.md`'s own
  "a persistent map message/schema is not present yet" statement for `autokart_mapping`).
- Confirmed `autokart_perception` is reachable from this dev box two ways: (a) via
  `kart/ros2_ws/install/setup.bash` (ament_python-installed from the canonical source, per that
  package's `CMakeLists.txt`), and (b) as a global editable `pip install` pointing at the same
  canonical source (`pip show autokart-perception` → `Editable project location:
  autonomy_stack_phase1/phase_01_perception`). Neither is relied upon by this node's own
  `algorithm_package_path` parameter — see "Defect found" below for why that distinction mattered.

## Decision: new standalone node, reusing the algorithm, not the existing ROS adapter

Per the mission statement and the same reasoning `autokart_hp60c`'s own developer report already
established: the existing `kart/ros2_ws` mapping adapter is a working, tested ROS package for a
different workspace, not a reusable pure-Python library on its own (no independent `setup.py`
distribution). This package:
- reuses the exact same `autokart_perception.application.CoreRuntime`/`shared_core.AlgorithmCore`/
  `shared_core.fit_skidpad_topology` (imported, lazily, via the `algorithm_package_path` parameter —
  see `autokart_mapping/algorithm.py`), never reimplementing Delaunay/Gabriel/gates/centerline/the
  skidpad ring fit,
- re-implements only the thin ROS-adapter validation/bookkeeping glue (`MappingEngine.update`'s
  qualification checks, `observed_exit_centerline`) that the existing adapter also defines locally
  rather than importing from a shared library — ported in behavior (not copy-pasted verbatim, since
  the input types differ: our own `ConeObservation` dataclass instead of a ROS `ConeArray` message,
  so the engine itself stays ROS-free and unit-testable without `rclpy`),
- is independently buildable/launchable with zero algorithm dependency present (fails closed with a
  clear, repeated fault status rather than crashing or fabricating a map).

## What was implemented

- `autokart_mapping/algorithm.py`: `MappingAlgorithm`, lazy-imports `autokart_perception.{application,
  shared_core,types}` from a runtime parameter, never hardcoded.
- `autokart_mapping/engine.py`: `MappingEngine` (ROS-free), `ConeObservation`, `observed_exit_centerline`.
- `autokart_mapping/node.py`: `MappingNode`, ROS pub/sub wiring, timeouts, fault heartbeat.
- Full ament_python package (`package.xml`, `setup.py`, `setup.cfg`, `resource/`), launch file, one
  config profile (`algorithm_package_path` blank, documented placeholder).
- 11 tests: 3 unit (lazy-import wrapper, no ROS/algorithm needed) + 4 unit (engine, against the REAL
  reused algorithm, ported from the existing adapter's own test suite) + 1 ROS integration (real
  rclpy node, real algorithm) + 1 ROS fault injection (missing dependency, always runs) + 1 ROS fault
  injection (unqualified cone frame, real algorithm) + a `conftest.py` fixing a real test-isolation
  hazard (see below) = 11 total, with the dependency-requiring ones correctly skipping (not failing)
  when `autonomy_stack_phase1` is absent.
- `export_node.sh`, `CONTRACT.md`, `DEPENDENCIES.md`, `README.md` (this directory).

## Known gaps (explicit, not hidden)

- No separate node-02 perception-status companion topic exists yet in this library, so this node
  synthesizes a minimal status dict from the `ConeArray` itself rather than cross-checking a real
  companion message the way the existing, reused adapter does. `PERCEPTION_STATUS_MISMATCH` is
  therefore currently dead code in this node's wiring (retained for a future real integration, not
  deleted) — see CONTRACT.md's own section on this.
- No physically surveyed cone course was available to validate the skidpad ring fit's real-world
  residual accuracy (no hardware/track in this environment) — see `reports/VALIDATOR_REPORT.md`.
- `source_mode=LIVE`'s rejection paths are unit-tested; there is no LIVE sensor pipeline in this
  environment to exercise the accepted path end to end.
