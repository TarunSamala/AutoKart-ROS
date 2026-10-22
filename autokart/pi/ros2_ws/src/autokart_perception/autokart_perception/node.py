"""ROS 2 node: HP60C RGB + depth -> YOLO26s -> cone positions (vehicle frame).

The camera is owned by ONE existing process (the HP60C SDK owner) that publishes ROS images.
This node only subscribes; it never opens the camera.
"""
import time

import numpy as np
import rclpy
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image

from autokart_msgs.msg import ConeDetection, ConeDetectionArray

from .core import Intrinsics, Mount, cone_position, decode_output, letterbox


def image_to_numpy(msg: Image) -> np.ndarray:
    enc = msg.encoding
    if enc in ("rgb8", "bgr8"):
        a = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)[:, :msg.width * 3]
        a = a.reshape(msg.height, msg.width, 3)
        return a if enc == "rgb8" else a[:, :, ::-1]
    if enc in ("16UC1", "mono16"):
        return np.frombuffer(msg.data, np.uint16).reshape(msg.height, msg.step // 2)[:, :msg.width]
    if enc == "32FC1":
        return np.frombuffer(msg.data, np.float32).reshape(msg.height, msg.step // 4)[:, :msg.width]
    raise ValueError(f"unsupported encoding {enc}")


class Perception(Node):
    def __init__(self):
        super().__init__("autokart_perception")
        p = self.declare_parameter
        self.model_path = p("model_path", "models/yolo26s/yolo26s.onnx").value
        self.class_names = list(p("class_names", ["blue", "yellow", "orange"]).value)
        self.conf = float(p("conf_threshold", 0.4).value)
        self.iou = float(p("iou_threshold", 0.5).value)
        self.imgsz = int(p("input_size", 640).value)
        self.depth_scale = float(p("depth_scale", 0.001).value)       # raw units -> metres (mm -> m)
        self.depth_radial = bool(p("depth_is_radial", False).value)
        self.min_range = float(p("min_range_m", 0.2).value)
        self.max_range = float(p("max_range_m", 8.0).value)
        self.k = Intrinsics(float(p("fx", 600.0).value), float(p("fy", 600.0).value),
                            float(p("cx", 320.0).value), float(p("cy", 240.0).value))
        self.mount = Mount(float(p("mount_x", 0.0).value), float(p("mount_y", 0.0).value),
                           float(p("mount_z", 0.5).value), float(p("mount_pitch_deg", 0.0).value),
                           float(p("mount_yaw_deg", 0.0).value))
        self.calibration_verified = bool(p("calibration_verified", False).value)
        self.jpeg_quality = int(p("jpeg_quality", 70).value)
        self.stream_hz = float(p("stream_hz", 10.0).value)
        color_topic = p("color_topic", "/hp60c/color").value
        depth_topic = p("depth_topic", "/hp60c/depth").value

        import onnxruntime as ort  # imported here so the rest of the package imports without it
        providers = [pr for pr in ("CUDAExecutionProvider", "CPUExecutionProvider") if pr in ort.get_available_providers()]
        self.sess = ort.InferenceSession(self.model_path, providers=providers)
        self.inp_name = self.sess.get_inputs()[0].name
        self.get_logger().info(f"model {self.model_path} loaded, providers {providers}")

        self.pub = self.create_publisher(ConeDetectionArray, "/autokart/detections", 1)
        self.pub_img = self.create_publisher(CompressedImage, "/autokart/camera/compressed", 1)
        self.frame_id = 0
        self.last_stream = 0.0
        sub_c = Subscriber(self, Image, color_topic)
        sub_d = Subscriber(self, Image, depth_topic)
        self.sync = ApproximateTimeSynchronizer([sub_c, sub_d], queue_size=2, slop=0.05)
        self.sync.registerCallback(self.on_pair)

    def on_pair(self, color: Image, depth: Image):
        import cv2
        t0 = time.monotonic()
        rgb = np.ascontiguousarray(image_to_numpy(color))
        if color.encoding == "bgr8":
            rgb = np.ascontiguousarray(rgb[:, :, ::-1])
        depth_m = image_to_numpy(depth).astype(np.float32) * (self.depth_scale if depth.encoding != "32FC1" else 1.0)
        h, w = rgb.shape[:2]
        depth_valid = depth_m.shape[0] == h and depth_m.shape[1] == w   # must be aligned to RGB

        x, scale, px, py = letterbox(rgb, self.imgsz)
        out = self.sess.run(None, {self.inp_name: x})[0]
        dets = decode_output(out, self.conf, self.iou, scale, px, py, w, h)
        infer_ms = (time.monotonic() - t0) * 1000.0

        self.frame_id += 1
        msg = ConeDetectionArray()
        msg.header = color.header
        msg.header.frame_id = "vehicle"
        msg.frame_id = self.frame_id
        msg.image_width, msg.image_height = w, h
        msg.inference_ms = float(infer_ms)
        msg.depth_valid = bool(depth_valid)
        msg.calibration_verified = self.calibration_verified
        for (x1, y1, x2, y2, s, c) in dets:
            d = ConeDetection()
            d.cls = self.class_names[c] if 0 <= c < len(self.class_names) else str(c)
            d.conf = s
            d.x1, d.y1, d.x2, d.y2 = x1, y1, x2, y2
            pos = cone_position((x1, y1, x2, y2), depth_m, self.k, self.mount, self.min_range, self.max_range,
                                self.depth_radial) if depth_valid else None
            if pos is None:
                d.depth_ok = False
                d.dist_m = float("nan")
                d.x = d.y = d.z = float("nan")
            else:
                d.depth_ok = True
                d.x, d.y, d.z, d.dist_m = (float(v) for v in pos)
            msg.cones.append(d)
        self.pub.publish(msg)

        now = time.monotonic()
        if now - self.last_stream >= 1.0 / self.stream_hz:
            self.last_stream = now
            ok, buf = cv2.imencode(".jpg", rgb[:, :, ::-1], [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality])
            if ok:
                ci = CompressedImage()
                ci.header.stamp = color.header.stamp
                ci.header.frame_id = str(self.frame_id)   # lets the gateway pair image and detections
                ci.format = "jpeg"
                ci.data = buf.tobytes()
                self.pub_img.publish(ci)


def main():
    rclpy.init()
    node = Perception()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
