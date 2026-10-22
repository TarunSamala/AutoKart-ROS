"""Disarmed ROS transport for the authoritative Phase 1 RGB-D pipeline."""

from __future__ import annotations

from array import array as ByteArray
import json
import threading
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data
from autokart_msgs.msg import Cone as RosCone, ConeArray, MissionDebug, SensorDeliveryMetadata
from sensor_msgs.msg import Image

from autokart_perception.application import CoreRuntime
from autokart_perception.camera import CameraError, HP60C
from autokart_perception.calibration_store import artifact_summary
from std_srvs.srv import Trigger
from autokart_perception.detector import Yolo26OnnxDetector
from autokart_perception.pipeline import Pipeline
from autokart_perception.recording import Replay
from autokart_perception.synthetic import Synthetic


LABELS = {"blue": 1, "yellow": 2, "orange": 3, "large_orange": 4}


def ros_time(ns: int):
    from builtin_interfaces.msg import Time

    result = Time()
    result.sec, result.nanosec = divmod(int(ns), 1_000_000_000)
    return result


LIVE_CONE_BLOCKERS = {
    "INVALID_SOURCE", "INFERENCE_FAILURE", "MODEL_UNVERIFIED", "MODEL_NOT_AVAILABLE",
    "METRIC_LOCALIZATION_FAILED", "TIMESTAMP_MISMATCH", "DEPTH_UNAVAILABLE", "FRAME_FROZEN",
    "FRAME_OUT_OF_ORDER", "INFERENCE_OVERLOAD", "PROCESSING_OVERLOAD", "FRAME_TOO_OLD",
    "INVALID_FRAME_FORMAT", "DEPTH_ALIGNMENT_UNAVAILABLE",
}


def qualified(result, frame) -> tuple[bool, list[str]]:
    """Synthetic/recorded-synthetic samples, or LIVE samples under a verified calibration artifact, cross this boundary."""
    perception = result.perception_result
    if perception is None:
        return False, ["PERCEPTION_RESULT_MISSING"]
    reasons = list(perception.safety.reasons)
    diagnostics = perception.diagnostics
    if frame.source == "LIVE":
        blocking = [r for r in reasons if r in LIVE_CONE_BLOCKERS or r.startswith(("CALIBRATION", "MOUNT"))]
        if diagnostics.get("calibration_status") != "MOUNT_VERIFIED":
            blocking.insert(0, "LIVE_METRIC_CALIBRATION_UNVERIFIED")
        if diagnostics.get("frame_frozen") or diagnostics.get("frame_out_of_order"):
            blocking.append("NONMONOTONE_FRAME")
        return (False, blocking) if blocking else (True, [])
    if diagnostics.get("frame_frozen") or diagnostics.get("frame_out_of_order"):
        return False, ["NONMONOTONE_FRAME", *reasons]
    if frame.source == "SYNTHETIC" and perception.status == "SYNTHETIC_GEOMETRY":
        return True, []
    if (frame.source == "REPLAY" and perception.status == "RECORDED_OBSERVATIONS" and
            frame.calibration.get("recorded_source") in {"SYNTHETIC", "SIMULATED"} and
            not diagnostics.get("calibration_mismatch") and
            not diagnostics.get("replay_provenance_missing")):
        return True, []
    return False, ["REPLAY_METRIC_PROVENANCE_UNQUALIFIED", *reasons]


class PerceptionNode(Node):
    def __init__(self):
        super().__init__("autokart_perception")
        self.declare_parameter("mode", "disabled")
        self.declare_parameter("replay_dir", "")
        self.declare_parameter("model_path", "")
        self.declare_parameter("model_manifest", "")
        self.declare_parameter("camera_library", "")
        self.declare_parameter("camera_configuration", "")
        self.declare_parameter("calibration_artifact", "")
        # The HP60C read blocks until the next frame (~12.4 Hz), so a short period runs back to back.
        self.declare_parameter("period_s", 0.02)
        self.mode = str(self.get_parameter("mode").value).lower()
        if self.mode not in {"disabled", "synthetic", "replay", "live"}:
            raise ValueError("unsupported perception mode")
        self.core = CoreRuntime()
        self.source = None
        self.done = False
        self.received_live_frame = False
        self.startup_error = None
        self.sequence = 0
        # Live mode only: a dedicated capture thread owns self.source.read() and publishes
        # RGB/depth as soon as each frame lands, decoupled from inference. tick() (on the
        # executor/timer thread) takes whatever frame is newest and runs it through the
        # pipeline; there is exactly one mutable "latest" slot, never a growing queue, so
        # inference always sees the most recent frame and never falls behind on old ones.
        self._frame_lock = threading.Lock()
        self._latest_frame = None
        self._capture_error = None
        self._last_reported_capture_error = None
        self._capture_stop = threading.Event()
        self._capture_thread = None
        detector = None
        if self.mode == "synthetic":
            self.source = Synthetic()
        elif self.mode == "replay":
            directory = str(self.get_parameter("replay_dir").value)
            if not directory:
                self.startup_error = "REPLAY_DIR_REQUIRED"
            else:
                try:
                    self.source = Replay(directory)
                except (OSError, ValueError) as exc:
                    self.startup_error = f"REPLAY_UNAVAILABLE:{type(exc).__name__}"
        elif self.mode == "live":
            model = str(self.get_parameter("model_path").value)
            manifest = str(self.get_parameter("model_manifest").value)
            library = str(self.get_parameter("camera_library").value)
            configuration = str(self.get_parameter("camera_configuration").value)
            if not all((model, manifest, library, configuration)):
                self.startup_error = "LIVE_CONFIGURATION_INCOMPLETE"
            else:
                try:
                    detector = Yolo26OnnxDetector(Path(model), manifest_path=Path(manifest))
                    self.get_logger().info(
                        f"Live detector YOLO26s ONNX: path={Path(model).resolve()} "
                        f"sha256={detector.model_sha256} classes={','.join(detector.classes)} "
                        f"stage={detector.stage}")
                    self.source = HP60C(library=Path(library), configuration=Path(configuration),
                                        max_age_ms=self.core.config["max_frame_age_ms"])
                except Exception as exc:
                    self.startup_error = f"LIVE_SOURCE_UNAVAILABLE:{type(exc).__name__}"
        config = dict(self.core.config)
        artifact = str(self.get_parameter("calibration_artifact").value)
        if artifact:
            config["calibration_artifact"] = artifact
        self.pipeline = Pipeline(detector, config, synthetic=self.mode == "synthetic")
        self.cones_pub = self.create_publisher(ConeArray, "/perception/cones", qos_profile_sensor_data)
        self.status_pub = self.create_publisher(MissionDebug, "/autokart/perception/status", 10)
        self.dashboard_pub = self.create_publisher(MissionDebug, "/dashboard/debug", 10)
        self.metadata_pub = self.create_publisher(SensorDeliveryMetadata, "/autokart/perception/delivery", 10)
        self.rgb_pub = self.create_publisher(Image, "/autokart/perception/rgb", qos_profile_sensor_data)
        self.depth_pub = self.create_publisher(Image, "/autokart/perception/depth", qos_profile_sensor_data)
        self.timer = self.create_timer(float(self.get_parameter("period_s").value), self.tick)
        self.create_service(Trigger, "~/reset_sensor_fault", self.reset_sensor_fault)
        self.calibration_artifact = artifact or None
        if self.mode == "live" and self.source is not None:
            self._capture_thread = threading.Thread(
                target=self._capture_loop, name="hp60c_capture", daemon=True)
            self._capture_thread.start()

    def _capture_loop(self):
        """Runs on its own thread for the lifetime of live mode: repeats _capture_once(),
        backing off between attempts that fail without blocking (see _capture_once)."""
        while not self._capture_stop.is_set():
            if not self._capture_once():
                self._capture_stop.wait(0.1)

    def _capture_once(self) -> bool:
        """One capture attempt: owns the only calls to self.source.read() so tick() never
        blocks on the camera, and publishes images the moment a frame is captured so
        RGB/depth delivery is not gated behind inference. Returns True on a captured frame,
        False on an error (so the caller can back off). Split out from _capture_loop so a
        test can drive a single attempt without starting the thread."""
        try:
            frame = self.source.read()
        except CameraError as exc:
            if self.received_live_frame:
                self.pipeline.note_sensor_fault(type(exc).__name__)
            with self._frame_lock:
                self._capture_error = (type(exc).__name__, str(exc))
            return False
        if frame is None:
            with self._frame_lock:
                self._capture_error = ("LIVE_SOURCE_EXHAUSTED", "")
            return False
        if frame.source == "LIVE":
            self.received_live_frame = True
        self._images(frame)
        with self._frame_lock:
            self._latest_frame = frame
            self._capture_error = None
        return True

    def _take_latest_live_frame(self):
        """Consumes the newest captured frame (or the newest unreported capture error), if any,
        without blocking. A frame is returned at most once; an error is reported at most once
        per occurrence so a persistent fault does not spam a status message every tick."""
        with self._frame_lock:
            frame = self._latest_frame
            self._latest_frame = None
            error = self._capture_error
            if frame is not None:
                self._last_reported_capture_error = None
                return frame, None
            if error is not None and error == self._last_reported_capture_error:
                return None, None
            if error is not None:
                self._last_reported_capture_error = error
            return None, error

    def reset_sensor_fault(self, _request, response):
        reason = self.pipeline.clear_sensor_fault()
        response.success = reason is not None
        response.message = f"cleared {reason}" if reason else "no latched fault"
        if reason:
            self.get_logger().warning(f"Operator cleared latched perception fault: {reason}")
        return response

    def _diagnostic_extras(self, frame, status, result):
        detector = self.pipeline.detector
        diagnostics = status.diagnostics
        extras = {"rgb_timestamp_ms": frame.rgb_timestamp, "depth_timestamp_ms": frame.depth_timestamp,
                  "rgb_sequence": frame.rgb_sequence, "depth_sequence": frame.depth_sequence,
                  "frame_age_ms": diagnostics.get("frame_age_ms"), "inference_ms": diagnostics.get("inference_ms"),
                  "model_version": getattr(detector, "version", "NONE"), "model_backend": getattr(detector, "backend", "NONE"),
                  "model_stage": getattr(detector, "stage", "NONE"), "model_classes": list(getattr(detector, "classes", []) or []),
                  "safety_state": status.safety.state if status.safety else None,
                  "latched_fault": self.pipeline.latched_fault_reason,
                  "calibration_reasons": diagnostics.get("calibration_reasons", []),
                  "depth_scale_factor": diagnostics.get("depth_scale_factor"),
                  "depth_scale_verified": diagnostics.get("calibration_status") == "MOUNT_VERIFIED",
                  "mount_verified": diagnostics.get("calibration_status") == "MOUNT_VERIFIED",
                  "calibration_artifact": artifact_summary(self.calibration_artifact) if self.calibration_artifact else None}
        return extras

    def _status(self, sample_ns: int, source: str, valid: bool, reasons: list[str], **extra):
        payload = {"schema_version": 1, "source": source, "valid": valid,
                   "processed_at_monotonic_ns": time.monotonic_ns(),
                   "reason_codes": sorted(set(reasons)), "frame_id": "base_link",
                   "sample_time_ns": sample_ns, "expires_ns": sample_ns + 150_000_000,
                   "sequence": self.sequence, "vehicle_authorized": False,
                   "published_to_vcu": False, **extra}
        message = MissionDebug()
        message.measurement_time = ros_time(sample_ns)
        message.cycle = self.sequence
        message.frame_id = "base_link"
        message.stage = "PERCEPTION"
        message.payload_json = json.dumps(payload, sort_keys=True, allow_nan=False)
        self.status_pub.publish(message)
        message.stage = "perception"
        self.dashboard_pub.publish(message)

    def _images(self, frame):
        for array, encoding, publisher in ((frame.rgb, "bgr8", self.rgb_pub),
                                           (frame.depth, "mono16", self.depth_pub)):
            message = Image()
            message.header.stamp = ros_time(frame.timestamp_ns)
            message.header.frame_id = "camera_optical"
            message.height, message.width = array.shape[:2]
            message.encoding = encoding
            message.is_bigendian = False
            message.step = int(array.strides[0])
            # array('B') takes rclpy's typed fast path; bytes is validated element by element (~170 ms/frame).
            payload = ByteArray("B")
            payload.frombytes(array.tobytes())
            message.data = payload
            publisher.publish(message)

    def tick(self):
        self.sequence += 1
        if self.source is None:
            self._status(self.get_clock().now().nanoseconds, "UNVERIFIED", False,
                         [self.startup_error or "SOURCE_DISABLED"])
            return
        try:
            if self.mode == "live":
                # The capture thread owns self.source.read() and has already published
                # RGB/depth for this frame (see _capture_loop). tick() only ever sees the
                # single newest frame - never a queue - so inference cannot fall behind.
                frame, cam_error = self._take_latest_live_frame()
                if frame is None:
                    if cam_error is not None:
                        # HP60C capture stamps use the host monotonic clock. Keep
                        # warmup/fault diagnostics in that epoch as well so an early
                        # wall-clock status cannot outrank subsequent camera frames.
                        self._status(time.monotonic_ns(), "LIVE", False,
                                     [cam_error[0], cam_error[1]])
                    return
            else:
                frame = self.source.read()
                if frame is None:
                    self._status(self.get_clock().now().nanoseconds, "REPLAY", False, ["REPLAY_FINISHED"])
                    self.timer.cancel()
                    self.done = True
                    return
                if frame.source == "SYNTHETIC":
                    frame.timestamp_ns = self.get_clock().now().nanoseconds
                # Publish RGB/depth BEFORE inference, not after: inference (ONNX on CPU) is the slow step
                # and previously gated image delivery behind it. Image delivery timing/content is
                # unaffected either way (frame.timestamp_ns is set at capture, before either call);
                # nothing downstream correlates images with detections by publish order, only by
                # stamp/frame_id (see autokart_dashboard_gateway/camera_feed.py).
                self._images(frame)
            result = self.core.sdk.process_frame(frame, pipeline=self.pipeline)
            valid, reasons = qualified(result, frame)
            status = result.perception_result
            metadata = SensorDeliveryMetadata()
            metadata.header.stamp = ros_time(frame.timestamp_ns)
            metadata.header.frame_id = "camera_optical"
            metadata.sensor_id = "HP60C" if frame.source == "LIVE" else frame.source
            metadata.sequence = self.sequence
            metadata.capture_time = ros_time(frame.timestamp_ns)
            metadata.processing_completion_time = ros_time(self.get_clock().now().nanoseconds)
            metadata.delivery_time = metadata.processing_completion_time
            metadata.calibration_version = str(status.diagnostics.get("calibration_sha256", "UNKNOWN"))
            metadata.quality = ("QUALIFIED_LIVE" if frame.source == "LIVE" else "QUALIFIED_SIMULATION") if valid else "DIAGNOSTIC_ONLY"
            metadata.fault_state = "OK" if valid else ",".join(reasons)
            metadata.payload_present = bool(valid)
            metadata.encoding = "bgr8+mono16"
            metadata.width, metadata.height = frame.rgb.shape[1], frame.rgb.shape[0]
            self.metadata_pub.publish(metadata)
            origin = "SIMULATED" if frame.source == "SYNTHETIC" else frame.source
            self._status(frame.timestamp_ns, origin, valid, reasons,
                         model_sha256=getattr(self.pipeline.detector, "model_sha256", "NONE"),
                         calibration_sha256=status.diagnostics.get("calibration_sha256"),
                         calibration_status=status.diagnostics.get("calibration_status"),
                         **self._diagnostic_extras(frame, status, result),
                         processing_ms=result.processing_ms,
                         detections=[item.__dict__ for item in status.detections],
                         cone_details=status.diagnostics.get("cone_details", []))
            if valid:
                message = ConeArray()
                message.measurement_time = ros_time(frame.timestamp_ns)
                message.delivery_time = ros_time(self.get_clock().now().nanoseconds)
                message.frame_id = "base_link"
                message.ground_truth = False
                for item in status.cones:
                    if item.label not in LABELS:
                        continue
                    cone = RosCone()
                    cone.id = item.id
                    cone.semantic_class = LABELS[item.label]
                    cone.position.x, cone.position.y = map(float, item.xy)
                    cone.confidence = float(item.confidence)
                    cone.position_std_m = float(item.sigma_m)
                    message.cones.append(cone)
                self.cones_pub.publish(message)
        except Exception as exc:
            self.pipeline.note_sensor_fault(type(exc).__name__)
            self._status(time.monotonic_ns() if self.mode == "live" else self.get_clock().now().nanoseconds,
                         self.mode.upper(), False, ["PERCEPTION_EXCEPTION", type(exc).__name__])
            self.get_logger().error(f"Perception halted: {exc}")
            self.timer.cancel()
            self.done = True

    def destroy_node(self):
        if self._capture_thread is not None:
            self._capture_stop.set()
            self._capture_thread.join(timeout=2.0)
        if self.source is not None and hasattr(self.source, "close"):
            self.source.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = PerceptionNode()
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
