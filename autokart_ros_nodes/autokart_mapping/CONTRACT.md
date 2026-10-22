# autokart_mapping — interface contract

## Topics subscribed

| Topic (param) | Type | QoS | Notes |
|---|---|---|---|
| `odom_topic` (default `/autokart/odom`) | `nav_msgs/Odometry` | sensor data | `child_frame_id` must be `base_link`; used to transform cone observations into `odom`. |
| `cones_topic` (default `/autokart_mapping/cones_in`) | `autokart_msgs/ConeArray` | sensor data | Must be `frame_id=base_link`, `ground_truth=false`. This is this package's own default input topic name — see "What this node is not" below for why it is not wired directly to a frozen upstream perception topic. |

## Topics published

| Topic (param) | Type | QoS | Rate |
|---|---|---|---|
| `map_topic` (default `/autokart_mapping/cones`) | `autokart_msgs/ConeArray` | sensor data | one per processed cone-array input (success or fault; empty on fault) |
| `status_topic` (default `/autokart_mapping/status`) | `autokart_msgs/MissionDebug`, `stage="MAPPING"` | reliable, depth 10 | one per processed input, plus a fault heartbeat every 100 ms when the algorithm dependency failed to load |

`map_topic` carries `frame_id=odom`; cones are emitted only when `valid=true` in the paired status
(on any fault the array is empty, never a stale/fabricated map).

## Status payload (`MissionDebug.payload_json`)

JSON, `schema_version=1`: `source`, `valid`, `reason_codes`, `frame_id=odom`, `sample_time_ns`,
`expires_ns` (`sample_time_ns + 150ms`), `sequence`, `tracked_count`, `geometry`, `vehicle_authorized`
(always `false`), `published_to_vcu` (always `false`).

`geometry` (only populated when `valid=true`) reuses the **unmodified** output of
`autokart_perception.shared_core.AlgorithmCore.evaluate(..., mission="STRAIGHT_TRACK")`:
`status`, `delaunay_edges`, `delaunay_triangles`, `gabriel_edges`, `boundaries`, `gates`,
`rejected_candidates`, `centerline`, `centerline_status`, `curvature_status`, `path_profile`,
`topology_reasons`, plus this node's own additions: `observation_cones_odom`, `centerline_odom`,
`skidpad_topology` (from the reused `fit_skidpad_topology`, fit over the tracked-cone map, not a
surveyed course), and `exit_centerline_odom` (an observed blue/yellow exit-gate pairing, empty list
when ambiguous or insufficiently observed).

## Reason codes (non-exhaustive, fail-closed)

`UNQUALIFIED_CONE_FRAME`, `PERCEPTION_STATUS_MISMATCH`, `SOURCE_UNQUALIFIED`, `NONMONOTONE_CONES`,
`NONMONOTONE_ODOM`, `ODOM_DISCONTINUITY`, `INVALID_OR_STALE_ODOM`, `NO_OBSERVED_CONES`,
`INVALID_CONE`, `LIVE_ODOM_UNVERIFIED`, `LIVE_ODOM_COVARIANCE_UNQUALIFIED`,
`STATUS_OR_ODOM_TIMEOUT`, `MAP_INPUT_TIMEOUT`, `BACKEND_UNAVAILABLE` /
`DEPENDENCY_PATH_NOT_FOUND:<path>` / `ALGORITHM_IMPORT_FAILED:<...>` (startup dependency fault).

## Parameters

See `config/mapping.yaml`. `algorithm_package_path` is required for any valid output (see
DEPENDENCIES.md); `source_mode` is one of `SIMULATED`/`REPLAY`/`LIVE` (matching the convention already
frozen in `../../docs/NODE_CONTRACTS.md` for the existing, reused mapping adapter); `LIVE` additionally
requires `live_odom_verified=true` and nonzero pose covariance, exactly mirroring that existing
contract.

## No typed Graph/Gate/Boundary/Map ROS message exists yet

`autokart_msgs` has no `Graph`/`Gate`/`Boundaries` message type (confirmed by this session's own
`AUDIT_PHASE0.md`/`INTERFACE_FREEZE_PHASE1.md`, and independently by
`../../docs/NODE_CONTRACTS.md`'s own statement that "a persistent map message/schema is not present
yet" for `autokart_mapping`). This is a genuine, pre-existing gap, not introduced by this node. Per
the mission statement's options, this node reuses the existing `MissionDebug` diagnostic message for
the full geometry payload (same choice the existing, reused `kart/ros2_ws` mapping adapter already
makes) rather than defining a new message type in this package. The primary cone-map output
(`autokart_msgs/ConeArray`) is the one typed, already-existing message that fits; the raw graph/gate
geometry remains JSON-only, diagnostic, and not control-input-qualified, matching
`docs/NODE_CONTRACTS.md`'s existing, accepted contract exactly.

## Skidpad two-closed-loop and crossover handling

The observed **ring fit** (`fit_skidpad_topology`, in `geometry.skidpad_topology`) and its
crossover-margin exclusion band are reused **unmodified** from
`autonomy_stack_phase1/phase_01_perception/src/autokart_perception/shared_core.py`. This node does
not re-derive or re-litigate that geometry. The winding-number/gate-crossing **progress** state
machine that consumes this topology to confirm "right lap, right lap, left lap, left lap" lives in
`autokart_mission` (node 07, this library), not here — see that package's CONTRACT.md.

## Simplification versus the existing production contract: no separate perception-status companion

`docs/NODE_CONTRACTS.md`'s existing, frozen RC-03–05 contract requires `ConeArray` consumers to
cross-check a **separate** companion `MissionDebug` perception-status message (matching
`measurement_time`/`cycle`/`source`) before accepting a `ConeArray` as valid — because node 02
(perception) is not yet built in this library, there is no such companion topic to subscribe to here.
This node instead synthesizes a minimal status dict directly from the incoming `ConeArray` itself
(`frame_id`, `ground_truth`, and the node's own `source_mode` parameter) and passes `valid=true`
unconditionally into `MappingEngine.update()`. Consequently the engine's own
`PERCEPTION_STATUS_MISMATCH` reason code (ported from the existing, reused adapter, which does have a
real companion status to compare against) can never actually trigger in this node's current wiring —
it is retained, not deleted, so that a future integration with a real node 02 perception status topic
can wire it up without re-deriving the mapping engine's validation logic, but it is dead in the
current configuration. This is a known, reported simplification, not a silent gap.

## What this node is not

It does not run perception/cone detection (node 02, not yet built in this library). It does not track
lap progress or a braking FSM (node 07, `autokart_mission`, this library). It does not replace or
modify `kart/ros2_ws/src/autokart_nodes/autokart_mapping/`, which remains the live production path.
Its default topic names are this package's own, not asserted as the master architecture's final frozen
cross-node topic names (that remains a separate, reviewed integration decision, out of scope here —
same limitation `autokart_hp60c`'s CONTRACT.md already states for its own topics).
