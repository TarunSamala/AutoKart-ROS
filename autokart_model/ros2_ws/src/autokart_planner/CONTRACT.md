# Planner contract (2026-10-22 integration)

Inputs: `/mission/state` MissionDebug JSON with `sample_time_ns`, `expires_ns`,
`frame_id=odom`, `source`, `valid`, `path_odom:[[x,y],...]`, `target_speed_mps`;
`/odom` Odometry (odom/base_link, fresh source timestamp); and configurable
`obstacles_topic` (default `/autokart_planner/obstacles`) ConeArray in **odom**.
The obstacle stream must have capture-aligned transformation performed upstream.
The planner does not transform the latest pose into an older obstacle capture.

`/perception/obstacle_status` MissionDebug must have matching ConeArray measurement
stamp, `frame_id=odom`, same source as this node, `valid=true`,
`coverage_complete=true`, and unexpired `expires_ns`. Empty ConeArray alone never
establishes free space. Missing arbitrary-obstacle extraction remains unqualified.
Cone radius is supplied by this interim interface's position_std_m; it must bound
physical extent plus uncertainty, not merely statistical center uncertainty.

Outputs: `/planning/path` nav_msgs/Path, `/planning/status` MissionDebug,
`/planning/speed_profile` std_msgs/String JSON and `/planning/stop_reason` String JSON.
Legacy `/autokart_planner/path`, `/autokart_planner/safety_envelope` and
`/autokart_planner/speed_profile` remain compatibility outputs. Path/profile/status
share original mission sample stamp; status expiry is bounded by mission expiry
and maximum path age. Invalid output carries empty path and explicit reasons.
The selected mission path is input; mission branching has one upstream authority.

Status fields include source, frame_id, sample_time_ns, expires_ns, sequence,
valid, decision, reason_codes, path_odom, speed_profile_mps,
curvature_profile_m_inv, clear_distance_m, stopping_distance_m,
vehicle_authorized=false and published_to_vcu=false.

Clearance continuously sweeps a disk enclosing body width and max front/rear
extent along every segment. This conservatively includes corners and turns;
it can reject narrow corridors a rectangular swept-volume method would accept.
First contact, not obstacle center, limits stopping distance. Speed is bounded
by curvature, mission cap, latency-aware stopping envelope, forward acceleration
and backward braking passes. Terminal and post-contact samples have zero speed.
No jerk bound is claimed. The reference geometry may include the stop obstacle;
the speed profile cannot authorize reaching its contact point.

Defaults leave brake/acceleration/curvature qualification absent. LIVE_PHYSICAL
requires physical_limits_verified explicitly; setting parameters is not evidence
of measurement. Measured wheelbase, mechanical steering, body extents, lateral
acceleration, latency and braking remain physical release gates. No actuation.

Both planner and controller additionally require `/vehicle/odometry_status`
std_msgs/String JSON with valid=true, source matching the configured stream,
sample_time_ns exactly matching the latest accepted `/odom`, unexpired expires_ns,
and continuity_lost=false. LIVE_PHYSICAL maps to estimator source LIVE and also
requires physical_qualified=true. A fresh standard odometry message alone cannot
bypass an invalid estimator status. Status and pose may arrive in either order;
unmatched pairs fail closed until both are available for a new mission cycle.

Mapped cones are collision obstacles too. `/mapping/status` MissionDebug must be
valid, unexpired, source/frame qualified and share the exact mission capture stamp.
Its `geometry.observation_cones_odom` supplies finite `x_m,y_m` centers. Configured
`cone_radius_m>0` and `cone_position_uncertainty_m>=0` bound cone extent and center
uncertainty; their sum expands swept clearance. Neither value has a physical
default. LIVE additionally requires cone_footprint_verified=true. Missing companion
or qualification fails closed, including a reference path that cuts cone boundaries.
LIVE/LIVE_PHYSICAL requires algorithm_backend=reused; builtin is diagnostic only.
SIMULATED and SYNTHETIC odometry statuses denote the same nonphysical regime;
this normalization never admits LIVE or REPLAY into a simulation stream.

Synthetic obstacle provenance is preserved: ground_truth=true is accepted only
when this node expects SIMULATED/SYNTHETIC and a qualified matching coverage
companion is present. LIVE/LIVE_PHYSICAL always rejects ground-truth input.
