"""replay_publisher: feeds real, recorded HP60C frames into this graph for the optional replay
demo (phase 6, step 4 of the integration mission). This is NOT a wrapper around node 01
(``autokart_hp60c``) -- that node's own CONTRACT.md and DEPENDENCIES.md only define ``synthetic``
and ``hp60c`` (live SDK) backends; neither reads a pre-recorded session. So for this one demo this
node stands in for node 01 directly on node 01's own topics, publishing real
``autokart_perception.recording.Replay`` frames (rgb/depth arrays + intrinsics, read from a
recording directory under ``autonomy_stack_phase1/phase_01_perception/recordings/``) instead of a
camera.

Every status/metadata this node publishes says ``REPLAY`` explicitly (``SensorDeliveryMetadata``
has no dedicated provenance field, so ``calibration_version`` carries the label, matching the
"never claims LIVE provenance" convention the review of node 01/02 already established). It is
``algorithm_package_path``-parameterized the same way every other node in this library is (lazy
import, never vendored), and it publishes nothing once the recording is exhausted (no looping, no
fabricated extra frames).
"""
from __future__ import annotations

import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
from autokart_msgs.msg import SensorDeliveryMetadata


class ReplayPublisher(Node):
    def __init__(self, **kwargs):
        super().__init__("replay_publisher", **kwargs)
        self.declare_parameter("algorithm_package_path", "")
        self.declare_parameter("recording_path", "")
        self.declare_parameter("rate_hz", 2.0)

        package_path = str(self.get_parameter("algorithm_package_path").value)
        recording_path = str(self.get_parameter("recording_path").value)
        if not package_path or not recording_path:
            raise ValueError("replay_publisher requires algorithm_package_path and recording_path")
        if package_path not in sys.path:
            sys.path.insert(0, package_path)
        from autokart_perception.recording import Replay  # noqa: WPS433 (intentional lazy import)

        self.replay = Replay(recording_path)
        self.sequence = 0
        self.rgb_pub = self.create_publisher(Image, "/hp60c/rgb/image_raw", qos_profile_sensor_data)
        self.depth_pub = self.create_publisher(Image, "/hp60c/depth/image_raw", qos_profile_sensor_data)
        self.rgb_info_pub = self.create_publisher(CameraInfo, "/hp60c/rgb/camera_info",
                                                    qos_profile_sensor_data)
        self.depth_info_pub = self.create_publisher(CameraInfo, "/hp60c/depth/camera_info",
                                                      qos_profile_sensor_data)
        self.status_pub = self.create_publisher(SensorDeliveryMetadata, "/hp60c/status", 10)

        rate_hz = float(self.get_parameter("rate_hz").value)
        self.create_timer(1.0 / max(rate_hz, 0.1), self._tick)
        self.get_logger().info(f"replay_publisher: replaying {len(self.replay.files)} frames from "
                                f"{recording_path} (REPLAY, not LIVE)")

    def _camera_info(self, width, height, intrinsics, frame_id):
        info = CameraInfo()
        info.header.frame_id = frame_id
        info.width, info.height = int(width), int(height)
        info.distortion_model = "plumb_bob"
        info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        info.k = [intrinsics.fx, 0.0, intrinsics.cx, 0.0, intrinsics.fy, intrinsics.cy, 0.0, 0.0, 1.0]
        info.p = [intrinsics.fx, 0.0, intrinsics.cx, 0.0,
                  0.0, intrinsics.fy, intrinsics.cy, 0.0,
                  0.0, 0.0, 1.0, 0.0]
        return info

    def _image(self, array: np.ndarray, encoding: str, frame_id: str) -> Image:
        msg = Image()
        msg.header.frame_id = frame_id
        msg.height, msg.width = array.shape[0], array.shape[1]
        msg.encoding = encoding
        msg.is_bigendian = 0
        msg.step = array.strides[0]
        msg.data = array.tobytes()
        return msg

    def _tick(self):
        frame = self.replay.read()
        if frame is None:
            self.get_logger().info("replay_publisher: recording exhausted, no more frames published")
            return
        self.sequence += 1
        now = self.get_clock().now().to_msg()

        rgb_msg = self._image(frame.rgb.astype(np.uint8), "rgb8", "hp60c_rgb_optical_frame")
        rgb_msg.header.stamp = now
        depth_msg = self._image(frame.depth.astype(np.uint16), "16UC1", "hp60c_depth_optical_frame")
        depth_msg.header.stamp = now

        rgb_info = self._camera_info(frame.rgb.shape[1], frame.rgb.shape[0], frame.intrinsics,
                                      "hp60c_rgb_optical_frame")
        rgb_info.header.stamp = now
        depth_info = self._camera_info(frame.depth.shape[1], frame.depth.shape[0], frame.intrinsics,
                                        "hp60c_depth_optical_frame")
        depth_info.header.stamp = now

        self.rgb_pub.publish(rgb_msg)
        self.depth_pub.publish(depth_msg)
        self.rgb_info_pub.publish(rgb_info)
        self.depth_info_pub.publish(depth_info)

        status = SensorDeliveryMetadata()
        status.header.stamp = now
        status.sensor_id = "hp60c_replay"
        status.sequence = self.sequence
        status.capture_time = now
        status.delivery_time = now
        status.calibration_version = "REPLAY_RECORDING"
        status.quality = "OK"
        status.fault_state = "NONE"
        status.payload_present = True
        status.width = int(frame.rgb.shape[1])
        status.height = int(frame.rgb.shape[0])
        status.cumulative_captured = self.sequence
        status.cumulative_delivered = self.sequence
        self.status_pub.publish(status)


def main(args=None):
    rclpy.init(args=args)
    node = ReplayPublisher()
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
