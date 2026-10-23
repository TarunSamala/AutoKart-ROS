# Odometry contract

Input `/vehicle/wheel_encoder`: String JSON v1 per autokart_vcu_bridge/CONTRACT.md.
Positive encoder scale and measured sign, circumference, wheelbase,
calibration_verified=true and matching calibration_hash are mandatory.
`valid`, `clock_valid`, `steering_valid` and calibration_verified require actual
JSON booleans true. sample_time_ns is aligned ROS acquisition time; sample_time_us
is original monotonic VCU time. seq and boot_id uint32, counts int64, time uint64.

Input `/imu/data_raw` sensor_msgs/Imu and `/imu/status`
SensorDeliveryMetadata. Status must be fresh (<100 ms), quality OK and encoding
SYNTHETIC/SIMULATED/REPLAY/LIVE[_PHYSICAL] consistent with the wheel source. Gyro
is bias corrected once by the driver, imu_frame_id must match and
imu_axes_aligned must explicitly attest measured base axes. Orientation ignored.

Output `/odom` nav_msgs/Odometry and `/tf` only upon valid intervals; odom/base_link
frames, current wheel acquisition stamp, SI pose and measured-rate twist.
`/vehicle/odometry_status` String v1 reports validity, reason, provenance,
sample_time_ns, continuity_lost and physical_qualified=false. Stale state is
invalid; frozen pose is never republished as a fresh estimate. physical_qualified
is intentionally never inferred from process health.

Per-sample SensorDeliveryMetadata.capture_time must equal Imu.header.stamp.
The wrapper buffers and pairs either callback arrival order; a prior status
cannot qualify a new sample. Invalid status/rate clears integration history and
latches continuity loss after baseline. Status adds rpm, speed_mps,
signed_distance_m, distance_travelled_total, x_m, y_m and yaw_rad. These are
held historical values when valid=false, never fresh qualified measurements.
