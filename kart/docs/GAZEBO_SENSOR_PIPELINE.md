# Gazebo Sensor Delivery Pipeline

RGB, depth and LiDAR are native Gazebo payloads. The active operational IMU is
the established internal sensor model (`imu.source: internal`); no second IMU
publishes on the same topic.

| Sensor | Frame | Native payload | Configured rate | Processing + transport |
|---|---|---|---:|---:|
| RGB | `camera_optical_frame` | `320x240 rgb8` | 30 Hz | 8 + 2 ms |
| depth | `depth_camera_optical_frame` | `320x240 32FC1` | 30 Hz | 10 + 2 ms |
| LiDAR | `lidar_link` | 1080 float ranges, 270 degrees | 20 Hz | 4 + 2 ms |
| IMU | `imu_link` | ROS IMU float64 values | 200 Hz | 1 + 2 ms |

The Tier-3 acceptance stream is 320x240 so Gazebo Classic 11 can meet the
20 ms state-age policy under llvmpipe. Higher resolution belongs to the
offline research tier and does not relax stale-frame limits.

## Metadata contract

Every delivered payload has sensor ID, frame, sequence, reset epoch, capture,
processing-completion and delivery timestamps, calibration version, quality,
fault state, payload presence/size, encoding/dimensions, cumulative
capture/delivery/drop counts and current/maximum queue depth. ROS header time
equals capture time.

Queues are per-sensor, bounded and reorder-free. Overflow drops the oldest
queued frame; configured dropout and stale render completion are counted.
Reset clears the queue and sequence. Callbacks copy payloads and enqueue them;
they never call the internal physics integrator.

## Native result

The nominal lap captured 496 RGB, 496 depth, 330 LiDAR and 3312 IMU samples.
Delivered counts were 471, 470, 329 and 3312. Warm-up/state-age rejection
accounted for 25 RGB, 26 depth and one LiDAR drop. Maximum queue depth was one,
there were no empty payloads and no timestamp violations. Maximum recorded
latencies were 21 ms RGB/depth, 6 ms LiDAR and 3 ms IMU.

Representative RGB PPMs, depth statistics, LiDAR CSVs, complete metadata and
checksums are in `artifacts/gazebo_follower/full_lap`. Raw full-resolution
bags are deliberately not committed; use `scripts/record_gazebo_full_lap.sh`
when local storage policy permits.
