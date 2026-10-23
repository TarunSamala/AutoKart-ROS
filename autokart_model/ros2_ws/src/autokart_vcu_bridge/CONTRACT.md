# Vehicle diagnostic JSON v1

All String envelopes contain schema_version=1, source, sample_time_ns (aligned
ROS acquisition timestamp; 0 means unavailable), arrival_time_ns, sequence,
boot_id, clock_valid, valid and faults. SYNTHETIC source remains synthetic.

`/vehicle/wheel_encoder` also carries sample_time_us (VCU monotonic),
axle_count_total signed integer, steering_actual_rad, calibration_verified,
calibration_hash and steering_valid. All present legacy records are invalid,
boot_id null, calibration/clock flags false. Arrival time is never substituted
for unavailable acquisition time. Odometry accepts this schema only when all
calibration, clock and measurement gates pass; legacy output never does.

`/vehicle/stop_evidence` also carries wheel_speed_mps/wheel_valid,
ground_speed_mps/ground_valid/ground_independent/ground_speed_source,
brake_instrumented/brake_valid/brake_applied, vcu_valid, stop_active,
estop_active and disarmed. Independent ground speed unavailable => null value,
false validity, false independence, UNAVAILABLE source. No stationarity, applied
brake, disarm or physical stop condition is inferred. Legacy brake booleans
represent an output request and do not prove instrumented brake application.
All qualification booleans stay false, sample_time_ns=0, explicit faults.

`/vehicle/telemetry` additionally exposes ACK and firmware command age for
human inspection, transport_read_only=true and outputs_inhibited=true; those
fields do not attest hardware state. Missing or stale input reports faults.
