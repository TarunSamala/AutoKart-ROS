# autokart_imu

MPU-6500 adapter and synthetic ROS 2 Humble source. Publishes bias corrected
`sensor_msgs/Imu` `/imu/data_raw` and `SensorDeliveryMetadata` `/imu/status`.
Orientation is unavailable (`orientation_covariance[0] = -1`). Acquisition
monotonic time is converted to the current ROS clock using measured sample age.
Nonfinite samples, saturation, reversed sample timestamps/sequences, and
acquisition age outside the 100 ms window are rejected.

Default backend is synthetic. Live config also defaults `hardware_enabled:false`.
Enabling physical I2C requires deliberate configuration; advisory owner lock
prevents another instance of this package, but cannot exclude unrelated drivers.
Actual axes, mounting, stationary condition, temperature bias and noise remain
unqualified. `stationary_verified`, `calibration_verified` and
`axis_alignment_verified` default false. Live status cannot become OK before
those gates. Physical stationary calibration also rejects gross gyro/accel motion;
a constant slow yaw cannot be distinguished from bias by this sensor alone.

The I2C backend now writes and reads back gyro/accelerometer range registers,
matching the physical-unit conversion; prior code used a configured sensitivity
without configuring the device. No hardware was opened during development.

Launch: `ros2 launch autokart_imu imu.launch.py`. This package does not publish
static mounting TF; measured `imu_link` to `base_link` remains deployment input.
Metadata `encoding` carries SYNTHETIC or LIVE_PHYSICAL provenance. Default
covariances are development parameters and require measured characterization.
The current backend polls registers; hardware FIFO/interrupt/dead-stream behavior
still requires commissioning and qualified acquisition support.

Reports predating CODEX_DEVELOPER_REPORT are preserved historical snapshot reports.
