# Controller contract (2026-10-22 integration)

Canonical input `/planning/status` MissionDebug provides atomic path/profile/expiry
and provenance. `/odom` requires fresh source timestamp, odom/base_link frames;
`/vehicle/actuator_feedback` requires fresh, valid measured steering. Missing
feedback starts invalid. Legacy Path/envelope/profile subscriptions cannot bypass
the canonical status expiry and validity. Profiles with wrong length, negative or
nonfinite values fail closed.

At 50 Hz, `/control/request` and `/control/status` publish MissionDebug JSON.
Fields: schema_version, source, sample_time_ns (original path capture), issued_ns,
expires_ns (min issuance+candidate_ttl_s and planner expiry), sequence, frame_id,
valid, target_speed_mps, target_steering_rad, stop_requested, reason_codes,
path_index, path_arc_m, cross_track_error_m, heading_error_rad,
speed_tracking_error_mps and steering_tracking_error_rad.
Candidate TTL must be in (0, 0.1] seconds. Invalid candidates request STOP and zero
speed. `vehicle_authorized=false`, `output_inhibited=true`,
`published_to_vcu=false` always. Legacy MotionCommand candidates remain on
`/autokart_controller/motion_command_candidate`, never an authorized topic.

Pure Pursuit can directly reuse kart_sim.autonomy classes via the configured
reused backend. Builtin backend is portable diagnostic math with numerical
cross-validation. The reused longitudinal PID is diagnostic only; the actual
candidate requests speed and road-wheel steering, leaving traction/braking loops
to a separately qualified VCU. No PWM, voltage or motor transport is emitted.

Cross-track error is signed nearest-segment projection (positive left); heading
error is path tangent minus pose yaw. Steering tracking uses actual road-wheel
feedback. Yaw-rate error is null with explicit unavailable reason; no fake zero
is substituted. No physical tracking quality is claimed.

LIVE_PHYSICAL additionally requires explicit physical_limits_verified. Nominal
lookahead/geometry values remain commissioning inputs and are not measured
qualification. Source must match planner source. Safety is sole authorization
owner and the integration profile always inhibits outputs.

`/vehicle/odometry_status` String JSON must affirm the exact /odom sample/source,
expiry and continuity. LIVE/LIVE_PHYSICAL additionally requires its
physical_qualified=true. Fresh diagnostic poses with invalid status are vetoed.

LIVE/LIVE_PHYSICAL requires pursuit_backend=reused, so a standalone builtin
implementation cannot become a production controller by changing the source flag.
SIMULATED and SYNTHETIC are equivalent nonphysical odometry status provenance;
LIVE and REPLAY remain separate.
