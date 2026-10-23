# Codex developer report — 2026-10-22

Role: DEVELOPER. No independent review or validation is claimed by this report.
Changes apply only to isolated takeover worktree nodes 03/04/05.

## Audit and reuse

Read the complete captured master architecture. Inspected existing
kart/ros2_ws/src/autokart_vehicle_bridge/{protocol.py,core.py} and
kart/ros2_ws/src/autokart_localization/odometry.py. Legacy protocol v2 supplies
counts/t_ms/raw angle/ACK but no qualified boot/session/clock/calibration contract.
Existing odometry uses bicycle model yaw; gyro interval integration required a
new focused estimator. Firmware, physical device paths and source dependencies
were not modified. No sensor/USB/I2C endpoint was opened by this work.

## Changes

IMU: writes and reads back physical range registers before unit conversion;
checks short read, finite values, timestamps/sequences and acquisition age;
stationary window rejects gross motion; explicit physical enable/calibration/
axis/stationary gates and advisory ownership; publishes corrected rate once,
unavailable orientation sentinel and source metadata. Noise parameters remain
unmeasured development configuration; no physical qualification claimed.

VCU: removed raw throttle/brake candidate conversion and ARM/RST command path
from ROS wrapper. Legacy serial transport is read only, direct send APIs disabled;
hardware disabled by default and exclusive owner serial open. Publishes versioned
JSON diagnostic envelopes for wheel telemetry and stop evidence; missing boot,
clock, calibration, independent ground velocity and instrumented brake are
explicitly invalid. No invented actual steering/metres/radps conversions.

Odometry: packaged sole odom+TF writer; validates typed JSON flags, provenance,
clock interval, signed cumulative count and calibration identity; interpolates
corrected gyro to each encoder interval, trapezoid integration plus midpoint
position, slip consistency rejection and covariance growth. Lost continuity
latches invalid; fresh invalid status never repackages a frozen pose as valid.
Default configuration emits no pose. Added README/contracts/dependencies/launch/
config and export helpers; exports include autokart_msgs sources.

## Executed evidence

Combined developer test run: **62 passed in 5.96s** (ROS 2 Humble, fresh
validation_ws autokart_msgs prefix, /usr/bin/python3, pytest plugin autoload
explicitly disabled due to preexisting user-site anyio/pytest incompatibility).
Cases include real ROS synthetic pub/sub, valid synthetic odometry, invalid IMU
veto, exclusive odom owner, default no-pose, serial output inhibition, missing
telemetry/independent ground signal, forward/reverse/yaw, packet loss, boot reset,
nonmonotonic time/sequence, internal IMU gap, slip, kinematic bounds, NaN config,
truthy-string rejection and existing backend fault injection.

Command from worktree/autokart_ros_nodes (shell source paths adjusted as needed):
```
source /opt/ros/humble/setup.bash
source ../../validation_ws/install/local_setup.bash
export PYTHONPATH="$PWD/autokart_imu:$PWD/autokart_vcu_bridge:$PWD/autokart_odometry:$PYTHONPATH"
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /usr/bin/python3 -m pytest -q autokart_imu/test autokart_vcu_bridge/test autokart_odometry/test --import-mode=importlib
```

All three export helpers executed successfully to /tmp/autokart-sensor-exports;
archives were generated with source plus autokart_msgs. Destination ROS build
and actual hardware tests are NOT EXECUTED by this developer report.

## Remaining gates

BLOCKED: qualified physical acquisition, measured encoder scale/sign and rolling
circumference, steering mapping, IMU axes/noise/temperature, live clock relation,
firmware identity/session support, physical brake/ground speed/E-stop evidence.
The legacy transport intentionally cannot satisfy odometry or STOP_VERIFIED.
Single-writer locks cover only cooperating local instances; integration must
verify the actual launch graph and DDS publishers across packages/hosts.
Physical motion remains NO-GO; no traction or flashing work was performed.

## Independent-review corrections applied by developer

Root reviewer found rejected IMU data could be bridged by later interpolation.
Fixed: invalid/nonmonotonic IMU clears the rate buffer and permanently latches
lost continuity after encoder initialization. A regression covers both paths.
IMU status now carries exact capture_time; odometry pairs rate and status by
that timestamp in bounded buffers, supporting either arrival order and rejecting
unpaired metadata. ROS pairing regression added. Canonical odometry status also
exposes RPM, speed, signed distance, unsigned path length and x/y/yaw for gateway
inspection, all subordinate to its valid/stale/continuity fields.
