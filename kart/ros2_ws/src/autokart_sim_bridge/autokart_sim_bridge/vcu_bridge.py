"""Native ROS entry point. ROS commands are converted only to VCU inputs."""

from __future__ import annotations


def main() -> None:
    try:
        import rclpy
        from rclpy.node import Node
        from autokart_msgs.msg import MotionCommand as RosMotionCommand
    except ImportError as exc:
        raise SystemExit(f"NOT_EXECUTED_EXTERNAL_DEPENDENCY: {exc}") from exc

    class VcuBridgeNode(Node):
        def __init__(self) -> None:
            super().__init__("vcu_bridge")
            self.declare_parameter("integrated_mode", True)
            self.declare_parameter("direct_actuator_commands", False)
            if not self.get_parameter("integrated_mode").value:
                raise RuntimeError("this node supports integrated VCU mode only")
            if self.get_parameter("direct_actuator_commands").value:
                raise RuntimeError("direct ROS actuator commands are prohibited")
            # Runtime binding to kart_sim.vcu is intentionally completed by the
            # selected plant launcher; this subscriber never owns an actuator.
            self.subscription = self.create_subscription(
                RosMotionCommand,
                "control/motion_command",
                self._receive,
                1,
            )
            self.latest_high_level_request = None

        def _receive(self, message: RosMotionCommand) -> None:
            self.latest_high_level_request = message

    rclpy.init()
    node = VcuBridgeNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
