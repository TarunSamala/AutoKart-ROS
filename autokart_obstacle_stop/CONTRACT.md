# autokart_obstacle_stop — interface contract

This node is a command guard between the controller candidate and the existing
VCU bridge. It does not own serial, GPIO, throttle, brake, steering, arming or
the physical E-stop.

| Direction | Topic | Type | Purpose |
|---|---|---|---|
| Subscribe | `/perception/cones` | `autokart_msgs/ConeArray` | Qualified cones in `base_link` |
| Subscribe | `/autokart_controller/motion_command_candidate` | `autokart_msgs/MotionCommand` | Upstream candidate |
| Publish | `/autokart/obstacle_stop/motion_command_candidate` | `autokart_msgs/MotionCommand` | Filtered candidate for VCU bridge |
| Publish | `/autokart/obstacle_stop/status` | `std_msgs/String` JSON | Decision evidence |

The VCU bridge must be configured with:

```yaml
candidate_topic: "/autokart/obstacle_stop/motion_command_candidate"
```

## Decision

A cone requests STOP when its uncertainty-expanded footprint intersects the
configured forward corridor. Defaults are `x=0.10..3.0 m` and lateral half
width `0.75 m`. Track-edge cones outside that corridor do not stop the kart.

- Qualified obstacle: publish a valid zero-speed candidate.
- Missing/stale/unqualified perception: publish an invalid zero-speed candidate.
- Missing/stale upstream command: publish an invalid zero-speed candidate.
- Clear corridor: copy the fresh upstream candidate with a short new expiry.
- Synthetic `ground_truth=true` data is rejected for actuation by default.

Zero speed relies on the existing VCU bridge/VCU policy to command throttle
zero and braking. This node cannot prove that the physical BLDC stopped.

