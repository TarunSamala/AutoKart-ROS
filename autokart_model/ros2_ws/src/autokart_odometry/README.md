# autokart_odometry

Sole `/odom` and `odom -> base_link` TF writer using signed cumulative axle
counts and corrected IMU z angular rate. The existing localization module uses
steering bicycle yaw; it cannot satisfy the requested inertial fusion, so this
small estimator implements the specified midpoint integration. Disable the
existing localization publisher before launching this replacement. A per-user
advisory lock blocks duplicate instances of this package; deployment must also
verify DDS publisher ownership across other packages/hosts.

Unknown counts/rev, rolling circumference, wheelbase and calibration identity
default to zero/false. No output pose is published with defaults. The estimator
uses VCU sample intervals, checks their consistency against aligned ROS times,
requires bracketing IMU samples and integrates by trapezoids with interpolated
endpoints. The IMU is already bias corrected; no second bias correction occurs.
Measured frame axes must be aligned to base_link explicitly (`imu_axes_aligned`).
No guessed TF is installed.

Steering predicts yaw only for a slip consistency check. A disagreement rejects
the estimate and latches lost continuity. Boot/source changes, impossible counts,
clock discontinuity, malformed/stale stream, IMU loss and queue overflow cannot
silently resume a trusted path. Reinitialize this node and reconcile downstream
map after continuity loss. Dropped count packets within the bounded interval
recover displacement from cumulative counts. Signed displacement and unsigned
path length are distinct in core output.

Pose covariance grows from configured distance/yaw noise. Defaults are synthetic
development values; measured covariance characterization is part of physical
calibration before any real use. No software test qualifies physical slip,
encoder scale/sign, steering mapping, IMU axes/bias or vehicle readiness.

Run `ros2 launch autokart_odometry odometry.launch.py`. Current legacy bridge
cannot satisfy these input gates, so integrated default output is invalid by
design. Test fixtures demonstrate qualified synthetic integration only.
