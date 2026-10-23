# External dependencies — autokart_planner

## ROS package dependencies (ordinary colcon deps)

- `rclpy`, `nav_msgs`, `geometry_msgs`, `builtin_interfaces`, `std_msgs` — ROS 2 Humble core.
- `autokart_msgs` — this repository's shared interface package (`kart/ros2_ws/src/autokart_msgs`).
  Only `Cone`/`ConeArray` and `SafetyEnvelope` are used. To reuse this node elsewhere, copy
  `autokart_msgs` alongside it (interface-only, no further dependency chain beyond `std_msgs`,
  `builtin_interfaces`, `geometry_msgs`). See CONTRACT.md for an important caveat on
  `SafetyEnvelope.data_source` in this exact dev environment.
- `python3-yaml` (PyYAML) — only exercised when `vehicle_geometry_yaml_path` is set; otherwise
  this package never imports it (see `geometry_params.py`).

## Algorithm dependency (`algorithm_backend: reused` only)

With the default `algorithm_backend: builtin`, there is no algorithm dependency at all — the
package builds and all tests pass on a plain Ubuntu 22.04 + ROS 2 Humble machine with nothing else
installed (same portability bar as `autokart_hp60c`).

With `algorithm_backend: reused`, at runtime (not build time) this node imports and reuses:

- `autokart_perception.curvature.profile_centerline` (discrete three-point signed-circle-fit
  curvature) and `autokart_perception.shared_core.BrakeCapability` (stopping-distance formula),
  from `autonomy_stack_phase1/phase_01_perception/src/autokart_perception/{curvature,shared_core}.py`
  in the main `autonomous_kart` repository — the same package `autokart_hp60c` reuses for its
  `hp60c` backend, and the same algorithm `kart/ros2_ws`'s own `autokart_mapping`/`autokart_planner`
  nodes already depend on (see `docs/NODE_CONTRACTS.md`'s dependency table).

**To reuse this node in a clean workspace with the real algorithm package:**
1. Copy the `autokart_perception` algorithm package's `src/` directory onto the target machine.
2. Set `algorithm_package_path` (see `config/planner_reused.yaml`) to its actual path on that
   machine — never hardcoded into source.
3. Set `algorithm_backend: reused`.

No path in this package's source code is hardcoded to this repository's location. If the
dependency path is wrong or missing, the node fails closed at startup
(`DEPENDENCY_PATH_NOT_FOUND`/`SDK_ADAPTER_IMPORT_FAILED` in `/autokart_planner/safety_envelope`),
not silently, and keeps reporting the fault (see CONTRACT.md "Fault behavior").

### Import-precedence fix (found during this node's own development, not a hypothetical)

`ReusedAlgorithmBackend.__init__` always moves its configured `algorithm_package_path` to the
**front** of `sys.path` (removing any prior occurrence first), rather than only inserting it when
"not already present" (the pattern `autokart_hp60c/backends.py::HP60CBackend` uses). Testing this
node's cross-validation (`test_algorithms.py`) surfaced a real failure: a *different* directory
also containing a (fake, test-only) `autokart_perception` package had been placed earlier in
`sys.path` by an earlier test, and because the real dependency path was *already* present
somewhere else in `sys.path` (via this dev environment's own baked-in search path), the
"insert-if-absent" guard skipped re-adding it at the front — so Python's package finder bound the
`autokart_perception` name to the wrong (fake, incomplete) directory and
`autokart_perception.curvature` failed to import even though the real dependency path was valid
and configured correctly. Fixed here; `autokart_hp60c`'s identical `backends.py` pattern was not
touched (out of scope for this session, flagged for that node's own next review) but carries the
same latent risk if its test suite or a future caller ever constructs two different
`algorithm_package_path`/`hp60c_library_path` configurations within one process.

## What was NOT verified in this session

No physical vehicle or track was available. `algorithm_backend: reused`'s dependency-failure paths
are unit-tested; its *successful* curvature/stopping-distance computation is cross-validated
against the zero-dependency `builtin` backend on synthetic paths (circular arcs, straight lines),
not against real sensor data or a real track survey. `a_lat_max_mps2` is an UNVERIFIED nominal
placeholder (see CONTRACT.md) — no claim here authorizes trusting the curvature speed cap as a
measured vehicle limit.
