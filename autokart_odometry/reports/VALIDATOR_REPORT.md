# Independent validator report — autokart_odometry

## Environment

ROS 2 Humble and colcon. No VCU, no AS5047P/AS5600 encoders, no kart. **`kart/ros2_ws/install` is stale**
relative to its own `autokart_msgs` source (for example, the installed `VcuState` lacks
`estop_input_fitted`). It was not used and not touched. All in-repo ROS runs used a fresh `autokart_msgs`
built from source into `/tmp/ak_nodes_scratch/msgs_ws`.

## Runs and actual output

1. **Unit tests with no ROS environment at all** (`env -i`, nothing sourced):
   `python3 -m pytest test/test_backends.py -q` → `9 passed`.
2. **Full suite, in repo:** `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest test/ -q` → `23 passed`
   (9 unit + 2 integration + 12 fault injection). **Soak:** 10 consecutive runs → 10 × `23 passed`, 0
   failures, 0 teardown warnings (after the spin-thread-join fix; see SENIOR_REVIEWER_REPORT.md).
3. **Reviewer's covariance fix, reproduced pre-fix:** with the check disabled,
   `-k covariance` → `3 failed`; restored → passes.
4. **Clean-workspace portability** (`/tmp/ak_nodes_scratch/portability.sh` under `env -i`, with only
   `/opt/ros/humble` sourced; `AMENT_PREFIX_PATH=/opt/ros/humble` was printed before the build to prove it):
   - `./export_node.sh --with-algorithm` → bundle with `src/{autokart_msgs,autokart_odometry}` and
     `third_party/autokart_localization`. The reused algorithm is outside `src/`; the install tree
     contains only `autokart_msgs` and `autokart_odometry`.
   - `colcon build --packages-up-to autokart_odometry` → `Summary: 2 packages finished`.
   - pytest inside the bundle, with `AUTOKART_LOCALIZATION_PATH=<bundle>/third_party/autokart_localization`
     → `23 passed`.
   - Live: `ros2 run autokart_odometry odometry_node --ros-args --params-file .../odometry_synthetic.yaml
     -p algorithm_package_path:=<bundle>/third_party/autokart_localization`. `ros2 topic list` showed
     `/odom`, `/odom/status`, `/tf`. `ros2 topic echo`:
     - `/odom/status`: `quality: OK`, `fault_state: NONE`, `calibration_version: UNVERIFIED`,
       `sensor_id: wheel_odometry`.
     - `/odom`: `frame_id: odom`, `child_frame_id: base_link`, position `x: 5.677 y: 1.775` (a curving
       path from the synthetic 0.1 rad steering), orientation `z: 0.298 w: 0.954`, pose covariance
       `0.25 / 0.25 / 1e6 / 1e6 / ...`.
     - `/tf`: `odom -> base_link`, translation `x: 6.885 y: 2.760`.
   - SIGINT to the process group: no traceback in the log. Afterwards, `pgrep` found no
     `odometry_node`/launch/echo processes, and the `ros2` CLI daemon was stopped with `ros2 daemon stop`.

## Acceptance matrix (software-only)

| Capability | Status | Evidence |
|---|---|---|
| Builds standalone (ament_python) | VALIDATED | in-repo and clean `env -i` workspace |
| Reuses `WheelOdometry` without reimplementation | VALIDATED | lazy import; unit test drives the real class; no kinematics in package |
| `/odom` + `/odom/status` + `odom->base_link` TF | VALIDATED | integration test with `tf2_ros.Buffer.lookup_transform`; live `ros2 topic echo /tf` |
| `actuator_feedback` source (node 04 contract) | VALIDATED (synthetic publisher) | integration test feeding `ActuatorFeedback` at 50 Hz |
| Fail closed: missing algorithm / unconfigured wheelbase / bad covariance / counts+feedback | VALIDATED | fault-injection tests; no `/odom` published |
| Steering loss → DEGRADED, yaw frozen, translation continues | VALIDATED | fault-injection test |
| Speed loss / invalid feedback → no odometry, no TF | VALIDATED | fault-injection tests |
| Gap > 0.2 s and non-monotonic stamp rejected, then recovery | VALIDATED | fault-injection tests |
| Stale/absent input reported repeatedly | VALIDATED | fault-injection test |
| Clean SIGINT shutdown | VALIDATED | subprocess test plus live run |
| Portability (export → clean build → test → launch) | VALIDATED | run 4 above |
| No hardcoded paths in source | VALIDATED | grep; `/home/mark1` appears only in YAML comments/docs |
| `colcon test` hook | BLOCKED | same setuptools `tests_require` tooling gap as `autokart_hp60c`; pytest was run directly |
| IMU fusion | NOT RUN | out of scope by design; not implemented, not claimed |
| Unsigned path-length (`distance_travelled_total`) | NOT RUN | does not exist upstream (DEVELOPER_REPORT finding 1) |
| Reverse motion in speed mode | NOT RUN / known-unsupported | no direction signal (CONTRACT.md) |
| Lifted-axle test, push test, wheelbase measurement, `counts_per_m` and encoder-sign calibration | NOT RUN | no hardware |
| Live `/vehicle/actuator_feedback` from a real VCU | NOT RUN | no VCU |
| Duplicate-TF-writer avoidance vs the Gazebo follower plugin | NOT RUN | operator responsibility; documented, not automatable here |

**Software-only acceptance is not a vehicle-readiness claim.** No safety or calibration flag is set by this
package.
