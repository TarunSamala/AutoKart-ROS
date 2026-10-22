from pathlib import Path

from kart_sim.config import load_kart_config
from kart_sim.dynamic_kart import DynamicFourWheelKart
from kart_sim.types import PlanarKartState, VehicleCommand

ROOT = Path(__file__).parents[2]


def test_dynamic_kart_does_not_instantly_apply_steering_or_brake() -> None:
    kart = DynamicFourWheelKart(load_kart_config(ROOT / "configs/karts/prototype_dynamic.yaml"), PlanarKartState(u_mps=5.0))
    kart.step(VehicleCommand(steering_rad=0.2, brake=1.0))
    assert kart.steering.feedback.linkage_angle_rad == 0.0
    assert kart.brakes.feedback.brake_torque_nm == 0.0
    for _ in range(300): kart.step(VehicleCommand(steering_rad=0.2, brake=1.0))
    assert kart.steering.feedback.linkage_angle_rad > 0
    assert kart.brakes.feedback.brake_torque_nm > 0


def test_linkage_break_is_observable_and_motor_fault_removes_chain_torque() -> None:
    kart = DynamicFourWheelKart(load_kart_config(ROOT / "configs/karts/prototype_dynamic.yaml"))
    kart.inject_fault("steering:linkage_break")
    for _ in range(300): kart.step(VehicleCommand(steering_rad=0.3))
    assert kart.steering.feedback.shaft_angle_rad > 0
    assert kart.steering.feedback.linkage_angle_rad == 0.0
    kart.inject_fault("motor:torque_loss")
    kart.step(VehicleCommand(drive=1.0))
    assert kart.motor.feedback.chain_torque_nm == 0.0
