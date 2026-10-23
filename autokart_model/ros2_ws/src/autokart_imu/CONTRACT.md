# IMU interface

`/imu/data_raw`: sensor_msgs/Imu, best effort sensor QoS; imu_link, SI gyro
and acceleration, orientation unavailable. Bias correction occurs once here.
`/imu/status`: autokart_msgs/SensorDeliveryMetadata, reliable; quality OK,
CALIBRATING or INVALID. encoding identifies source, calibration_version labels
UNVERIFIED or OPERATOR_CONFIGURED. Consumers must check quality, freshness and
source before using rate samples. Physical calibration is not established by a
software parameter or synthetic test. No actuator output or hardware enable.

Metadata.capture_time exactly matches the published rate acquisition stamp; consumers pair status and sample by this stamp, not arrival order.
