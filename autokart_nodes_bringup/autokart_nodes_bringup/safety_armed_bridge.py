"""safety_armed_bridge: the one adapter this integration adds to close a documented, pre-existing
gap between node 10 (``autokart_safety``) and node 04 (``autokart_vcu_bridge``).

``autokart_vcu_bridge`` CONTRACT.md ("Relationship to node 10") states the gap plainly: node 10
publishes its arming decision as JSON on ``autokart_msgs/MissionDebug`` (``/autokart/safety/status``),
and node 04 gates all nonzero throttle on a plain ``std_msgs/Bool`` on ``safety_armed_topic``
(default ``/vcu_bridge/safety_armed``) that nothing in the library publishes. This node is exactly
that bridge, and nothing more:

- It does not grant authority. It is a pure republish of a field node 10 already computed
  (``state == "ARMED"``, the same literal the reused ``autokart_vcu_safety`` core uses -- see
  ``kart/ros2_ws/src/autokart_vcu_safety/autokart_vcu_safety/core.py``), never its own decision.
- It fails closed: until the first ``/autokart/safety/status`` message arrives, and on any
  malformed payload, it publishes/holds ``False``. It never defaults to ``True``.
- It republishes on every received status message (node 10 ticks its status at 10 Hz, armed or
  not), so ``autokart_vcu_bridge``'s own ``safety_timeout_s`` freshness check keeps seeing a live,
  current ``False`` rather than going stale -- stale would also gate throttle off, but this node's
  job is to prove the wiring is live, not to rely on absence.
"""
from __future__ import annotations

import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from autokart_msgs.msg import MissionDebug


class SafetyArmedBridge(Node):
    def __init__(self, **kwargs):
        super().__init__("safety_armed_bridge", **kwargs)
        self.declare_parameter("safety_status_topic", "/autokart/safety/status")
        self.declare_parameter("armed_bool_topic", "/vcu_bridge/safety_armed")

        armed_topic = str(self.get_parameter("armed_bool_topic").value)
        status_topic = str(self.get_parameter("safety_status_topic").value)

        self.armed_pub = self.create_publisher(Bool, armed_topic, 10)
        self.create_subscription(MissionDebug, status_topic, self._on_status, 10)

        self.last_state = "NO_SAFETY_STATUS_RECEIVED"

    def _on_status(self, message: MissionDebug) -> None:
        armed = False
        try:
            payload = json.loads(message.payload_json)
            self.last_state = str(payload.get("state", "UNKNOWN"))
            armed = self.last_state == "ARMED"
        except (json.JSONDecodeError, TypeError, ValueError) as exc:
            self.get_logger().warn(f"safety_armed_bridge: malformed /autokart/safety/status payload, "
                                    f"holding False: {exc}")
            armed = False
        out = Bool()
        out.data = armed
        self.armed_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = SafetyArmedBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
