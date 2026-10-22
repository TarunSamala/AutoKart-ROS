from autokart_perception.application import CoreRuntime
from autokart_perception.pipeline import Pipeline
from autokart_perception.synthetic import Synthetic
from autokart_perception_ros.node import PerceptionNode, qualified


def test_synthetic_is_explicit_and_live_is_never_qualified():
    core = CoreRuntime()
    frame = Synthetic().read()
    result = core.sdk.process_frame(frame, pipeline=Pipeline(None, core.config, synthetic=True))
    valid, reasons = qualified(result, frame)
    assert valid and not reasons and len(result.perception_result.cones) == 6
    frame.source = "LIVE"
    valid, reasons = qualified(result, frame)
    assert not valid and "LIVE_METRIC_CALIBRATION_UNVERIFIED" in reasons


def test_ros_wrapper_does_not_shadow_phase1_algorithm_package():
    import autokart_perception
    import autokart_perception_ros

    assert autokart_perception.__name__ != autokart_perception_ros.__name__
    assert "autokart_perception_ros" not in autokart_perception.__file__


def test_missing_live_model_starts_fail_closed_without_opening_camera():
    import rclpy

    rclpy.init(args=["--ros-args", "-p", "mode:=live",
                     "-p", "model_path:=/definitely/missing/best.onnx",
                     "-p", "model_manifest:=/definitely/missing/best.json",
                     "-p", "camera_library:=/definitely/missing/lib.so",
                     "-p", "camera_configuration:=/definitely/missing/config.json"])
    node = None
    try:
        node = PerceptionNode()
        assert node.source is None
        assert node.startup_error == "LIVE_SOURCE_UNAVAILABLE:DetectorError"
        node.tick()
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


def test_live_camera_warmup_retries_and_later_loss_latches_fault():
    """Live mode normally runs _capture_once() on a dedicated thread (see node.py's
    _capture_loop) and tick() only consumes the result via _take_latest_live_frame().
    This test drives _capture_once() directly, one attempt at a time, instead of starting
    that thread, so the warmup-retry / later-fault-latch behaviour stays deterministic."""
    import rclpy
    import time
    from autokart_perception.camera import CameraError

    class MissingFrame:
        def read(self):
            raise CameraError("CALIBRATION_UNAVAILABLE")

        def close(self):
            pass

    rclpy.init(args=["--ros-args", "-p", "mode:=synthetic"])
    node = None
    try:
        node = PerceptionNode()
        node.mode = "live"
        node.source = MissingFrame()
        status_stamps = []
        node._status = lambda sample_ns, *_args, **_kwargs: status_stamps.append(sample_ns)
        before = time.monotonic_ns()
        assert node._capture_once() is False
        node.tick()
        after = time.monotonic_ns()
        assert before <= status_stamps[-1] <= after
        assert not node.done
        assert node.pipeline.latched_fault_reason is None
        node.received_live_frame = True
        assert node._capture_once() is False
        node.tick()
        assert status_stamps[-1] >= status_stamps[0]
        assert not node.done
        assert node.pipeline.latched_fault_reason == "CameraError"
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


def test_live_tick_consumes_latest_frame_once_and_does_not_queue():
    """The capture thread keeps only the single newest frame (see node.py's _frame_lock /
    _latest_frame). tick() must consume it exactly once; a second tick() with no new capture
    in between must see nothing (no stale re-processing, no growing backlog)."""
    import rclpy
    from autokart_perception.synthetic import Synthetic

    rclpy.init(args=["--ros-args", "-p", "mode:=synthetic"])
    node = None
    try:
        node = PerceptionNode()
        node.mode = "live"
        frame = Synthetic().read()
        frame.source = "LIVE"
        node._images = lambda _frame: None
        with node._frame_lock:
            node._latest_frame = frame

        seen = []
        real_process_frame = type(node.core.sdk).process_frame
        node.core.sdk.process_frame = lambda f, pipeline: (seen.append(f), real_process_frame(node.core.sdk, f, pipeline=pipeline))[1]

        node.tick()
        assert len(seen) == 1 and seen[0] is frame
        with node._frame_lock:
            assert node._latest_frame is None

        node.tick()
        assert len(seen) == 1
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


def test_live_capture_error_status_reported_once_not_every_tick():
    """A persistent capture error (e.g. the camera dropping off USB) must not be republished
    on every tick while nothing about it has changed -- only once per distinct occurrence."""
    import rclpy

    rclpy.init(args=["--ros-args", "-p", "mode:=synthetic"])
    node = None
    try:
        node = PerceptionNode()
        node.mode = "live"
        node._capture_error = ("CAMERA_DISCONNECTED", "No such device")
        reported = []
        node._status = lambda *_args, **_kwargs: reported.append(1)
        node.tick()
        node.tick()
        node.tick()
        assert len(reported) == 1
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
