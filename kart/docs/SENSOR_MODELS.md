# Sensor Models

Every sensor uses separate capture/measurement and delivery timestamps.
`AsyncSensor` models acquisition, processing, transport, jitter, sensor-clock
offset/drift, queueing, dropout, power loss and frozen output.

Tier-1/2 models:

- planar LiDAR: configurable beams/FOV/range/noise/dropout, corridor/actor
  intersection, intensity and fog attenuation;
- camera: intrinsics, distortion, exposure/rolling-shutter configuration and
  semantic features; it explicitly reports rendered pixels unavailable;
- depth sampler: min/max, invalid samples, quantization and range error;
- IMU: acceleration/gyro bias and noise with gravity;
- wheel encoder: CPR/count/speed;
- detailed plant feedback: steering/road-wheel angles, brake travel/pressure,
  torque, current, voltage, speed, battery and temperatures.

Tier-3 Gazebo model advertises native RGB image, depth image, ray scan and IMU
interfaces. On 2026-07-29 the interfaces registered successfully after native
spawn; headless payload capture was not completed. ROS camera info, point-cloud
and rendered noise/fault bridge remain integration items.

All sensor poses and planned hardware parameters in
`configs/sensors/planned_kart.yaml` are **SIMULATION USE ONLY — MEASUREMENT
REQUIRED**.

Ground-truth state is a simulator input to sampling. Production ROS mode
rejects debug ground-truth topics.

