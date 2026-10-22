from pathlib import Path


PACKAGE = Path(__file__).resolve().parents[1]
FOLLOWER = (PACKAGE / "src/gazebo_follower_plugin.cpp").read_text(encoding="utf-8")
SENSORS = (PACKAGE / "src/gazebo_sensor_delivery_plugin.cpp").read_text(encoding="utf-8")
MODEL = (PACKAGE.parents[2] / "simulation/models/autokart/model.sdf").read_text(
    encoding="utf-8"
)


def test_follower_is_read_only_and_internal_authority_only() -> None:
    assert "visual_sensor_follower" in FOLLOWER
    assert "model_->SetWorldPose" in FOLLOWER
    assert "SetPosition(0, positions[index], false)" in FOLLOWER
    assert "MotionCommand" not in FOLLOWER
    assert "SetForce(" not in FOLLOWER
    assert "<static>true</static>" in MODEL
    for prohibited in ("diff_drive", "gazebo_ros2_control", "motor_controller"):
        assert prohibited not in MODEL.lower()


def test_sensor_headers_preserve_capture_time_and_publish_queue_counters() -> None:
    assert "message->header.stamp = SensorStamp(capture_time)" in SENSORS
    assert "metadata.capture_time = SensorStamp(frame.capture_time_s)" in SENSORS
    assert "metadata.delivery_time = SensorStamp(now)" in SENSORS
    assert "metadata.cumulative_captured" in SENSORS
    assert "queue_.size() >= static_cast<std::size_t>(capacity_)" in SENSORS


def test_single_clock_and_dynamic_base_transform_source() -> None:
    assert FOLLOWER.count('create_publisher<rosgraph_msgs::msg::Clock>') == 1
    assert FOLLOWER.count('"odom", "base_link"') == 1
    assert "maximum_clock_offset_s_" in FOLLOWER
    assert "OnWallWatchdog" in FOLLOWER
    assert "snapshot bridge disconnected; follower frozen and invalid" in FOLLOWER
    assert "double_physics_detected_.load()" in FOLLOWER
    assert "state.follower_valid = false" in FOLLOWER
    assert "state.follower_frozen = true" in FOLLOWER
