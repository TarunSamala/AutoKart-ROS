# Real-Time Fault Handling

Timing severity follows function:

- camera/depth/LiDAR miss: perception warning or degradation;
- Gazebo follower miss: visualization/sensor warning;
- VCU miss: error and controlled-stop policy;
- independent-safety miss: critical;
- bridge disconnect: follower freezes and native run is invalid.

The Phase 16 bounded-load campaign reuses the existing physical scenarios and
routes every action through VCU and independent safety. It passed command loss,
VCU freeze, physical E-stop, steering jam, brake cable break, stuck motor
torque, and bounded compute overload.

Evidence is in `artifacts/phase16/faults_under_load.json`. It records the
detected state/fault code, brake pressure, final propulsion torque, and task
timing. This campaign is headless authoritative-plant evidence; a full native
Gazebo fault-under-load matrix is still not executed.

Stale rendered captures are rejected and categorized. Gazebo process loss
cannot construct an actuator command, reset a critical fault, or restore
autonomous authority.
