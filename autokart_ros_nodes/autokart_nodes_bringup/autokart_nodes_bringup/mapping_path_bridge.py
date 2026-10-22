"""mapping_path_bridge: the second adapter this integration adds, for a gap this session's audit
found while wiring the graph (not previously documented in any single node's CONTRACT.md, because
no two of these nodes were ever run together before).

``autokart_mission`` (node 07) CONTRACT.md says plainly that it computes a candidate path internally
and then *discards* it: "This node never publishes a path ... publishing path_odom here would blur
[node 08's] boundary." So the literal "06 -> 07 -> 08" topic chain the mission brief describes has
no topic to carry a path from 07 to 08 -- mission's own contract forbids it from ever emitting one.

``autokart_planner`` (node 08) CONTRACT.md says it requires a ``nav_msgs/Path`` on
``reference_path_topic`` and explicitly expects it to come from "the mission/mapping node (06/07),
not yet built in this library" -- i.e. node 08 was written anticipating exactly this adapter, not a
direct 07 publisher.

The one node that already computes reusable centerline geometry is node 06 (``autokart_mapping``):
its ``MissionDebug`` status payload's ``geometry.centerline_odom`` field (a list of ``[x, y]`` pairs,
already expressed in the ``odom`` frame -- see ``autokart_mapping/engine.py::MappingEngine.update``)
is exactly node 08's required shape, modulo the message envelope. This node does only that
envelope conversion:

- Subscribes node 06's status topic (default ``/autokart_mapping/status``).
- If ``valid`` is false, or ``geometry.centerline_odom`` has fewer than 2 points, publishes an
  **empty** path -- matching node 08's own documented convention ("Invalid input publishes an empty
  path and reason; no path persists after expiry"). It never re-publishes a stale previous path.
  It never fabricates points mapping did not report.
- Otherwise builds a ``nav_msgs/Path`` (``frame_id="odom"``) with one pose per centerline point,
  orientation taken from the bearing to the next point (last pose repeats the previous bearing),
  and publishes it on node 08's ``reference_path_topic`` (default ``/autokart_planner/reference_path``).

Node 07 (``autokart_mission``) still runs in this integration, in parallel, consuming the same
node 06 status for its own skidpad/braking FSM and publishing its own diagnostic status -- it is not
bypassed, it simply never was the path source node 08 reads from a serial 07->08 topic, because no
such topic exists in this library. That is a property of the already-reviewed node contracts, not
something this adapter changes.
"""
from __future__ import annotations

import json
from math import atan2, cos, sin

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from autokart_msgs.msg import MissionDebug


class MappingPathBridge(Node):
    def __init__(self, **kwargs):
        super().__init__("mapping_path_bridge", **kwargs)
        self.declare_parameter("mapping_status_topic", "/autokart_mapping/status")
        self.declare_parameter("reference_path_topic", "/autokart_planner/reference_path")

        status_topic = str(self.get_parameter("mapping_status_topic").value)
        path_topic = str(self.get_parameter("reference_path_topic").value)

        self.path_pub = self.create_publisher(Path, path_topic, 10)
        self.create_subscription(MissionDebug, status_topic, self._on_status, 10)

    def _on_status(self, message: MissionDebug) -> None:
        points: list[tuple[float, float]] = []
        try:
            payload = json.loads(message.payload_json)
            if payload.get("valid"):
                raw_points = payload.get("geometry", {}).get("centerline_odom", [])
                points = [(float(p[0]), float(p[1])) for p in raw_points
                          if isinstance(p, (list, tuple)) and len(p) >= 2]
        except (json.JSONDecodeError, TypeError, ValueError, IndexError) as exc:
            self.get_logger().warn(f"mapping_path_bridge: malformed /autokart_mapping/status "
                                    f"payload, publishing empty path: {exc}")
            points = []

        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = "odom"

        if len(points) >= 2:
            for index, (x, y) in enumerate(points):
                pose = PoseStamped()
                pose.header = path.header
                pose.pose.position.x = x
                pose.pose.position.y = y
                nxt = points[index + 1] if index + 1 < len(points) else points[index - 1]
                dx, dy = (nxt[0] - x, nxt[1] - y) if index + 1 < len(points) else (x - nxt[0], y - nxt[1])
                yaw = atan2(dy, dx)
                pose.pose.orientation.z = sin(yaw / 2.0)
                pose.pose.orientation.w = cos(yaw / 2.0)
                path.poses.append(pose)

        self.path_pub.publish(path)


def main(args=None):
    rclpy.init(args=args)
    node = MappingPathBridge()
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
