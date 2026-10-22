from pathlib import Path

import pytest

from kart_sim.backends import load_backend_configuration
from kart_sim.calibration import (
    CalibrationSplitRegistry,
    DatasetMetadata,
    DatasetSplit,
    Parameter,
    ParameterRegistry,
    bounded_grid_optimize,
    calibration_report_template,
)
from kart_sim.can_hil import (
    CanTimeoutMonitor,
    CounterMonitor,
    MotionCanMessage,
    SimulatedCanPlantBridge,
    crc8_j1850,
)
from kart_sim.ros2_wire import motion_command_from_wire, motion_command_to_wire
from kart_sim.types import SimulationError
from kart_sim.vcu import MotionCommand
from kart_sim.world_adapter import (
    HeadlessSensorCallbackBridge,
    RenderSensorSample,
    TrackVisualPoint,
    WorldPose,
    export_track_payload,
)

ROOT = Path(__file__).parents[2]


def test_ros_wire_round_trip_preserves_measurement_expiry_and_crc():
    command = MotionCommand("jetson", 7, 1.25, 1.35, 3.0, .2, .1).signed()
    assert motion_command_from_wire(motion_command_to_wire(command)) == command


def test_chrono_configuration_is_explicit_and_prohibits_fallback():
    config = load_backend_configuration(ROOT / "configs/backends/chrono.yaml")
    assert config.backend == "chrono_multibody" and not config.allow_fallback


def test_headless_world_track_export_and_sensor_callback_timing():
    payload = export_track_payload(
        (
            TrackVisualPoint(0.0, WorldPose(0, 1, 0, .1), 2, 1.5),
            TrackVisualPoint(1.0, WorldPose(1, 1, .1, .1), 2, 1.5),
        )
    )
    assert payload["points"][0]["pose"]["y_m"] == -1
    received = []
    bridge = HeadlessSensorCallbackBridge()
    bridge.register("camera", received.append)
    bridge.deliver(RenderSensorSample("camera", 1.0, 1.02, "camera_optical", b"fallback"))
    assert received[0].measurement_time_s == 1.0
    with pytest.raises(SimulationError):
        bridge.deliver(RenderSensorSample("camera", .9, 1.03, "camera_optical", b"old"))


def test_can_crc_counter_timeout_and_simulated_plant_bridge():
    assert crc8_j1850(b"123456789") == 0x4B
    counter = CounterMonitor(last=255)
    counter.accept(0)
    with pytest.raises(SimulationError):
        counter.accept(0)
    timeout = CanTimeoutMonitor(.1)
    timeout.observe(1.0)
    assert not timeout.stale(1.1) and timeout.stale(1.101)
    bridge = SimulatedCanPlantBridge(.1)
    command = bridge.receive(MotionCanMessage(1, 400, 100, 0, 100).encode(1.0), 1.001)
    assert command.drive == .5 and command.steering_rad == .1
    assert bridge.command_at(1.2).drive == 0 and bridge.command_at(1.2).brake > 0


def test_calibration_split_registry_parameter_registry_optimizer_and_report():
    metadata = DatasetMetadata("synthetic-001", "2026-07-29", "prototype", "dry", "0.1", "none", "abc")
    splits = CalibrationSplitRegistry()
    splits.assign(metadata, DatasetSplit.CALIBRATION)
    with pytest.raises(SimulationError):
        splits.assign(metadata, DatasetSplit.VALIDATION)
    registry = ParameterRegistry()
    registry.add(
        Parameter(
            "drag_gain",
            "1",
            1.0,
            .5,
            1.5,
            "synthetic",
            identifiable_experiments=("coastdown",),
        )
    )
    result = bounded_grid_optimize(lambda value: (value - 1.2) ** 2, .5, 1.5, points=101)
    registry.fit("drag_gain", result.value, .05)
    report = calibration_report_template(metadata, registry)
    assert result.value == pytest.approx(1.2)
    assert report["digital_twin_claim"] == "NOT_JUSTIFIED"
