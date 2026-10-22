# ROS 2 QoS

QoS is explicit in `kart_sim.ros2_sil.TOPIC_QOS`: sensor streams are bounded,
volatile, best-effort; commands are reliable, depth one, with a 0.1 s deadline;
diagnostics are reliable; and VCU state is reliable/transient-local. A reliable
but stale command is still rejected by the VCU. Queue overflow and liveliness
remain diagnostics, not permission to retain an old actuator command.
