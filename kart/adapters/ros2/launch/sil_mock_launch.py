"""Dependency-free launch contract used by deterministic CI/SIL tests.

Native ROS 2 launch files should instantiate the same bridge configuration only
when rclpy and required message packages are installed.
"""

from kart_sim.ros2_sil import DeterministicRosBridge, SimulationClock


def create_bridge(physics_dt_s: float = 0.001) -> DeterministicRosBridge:
    return DeterministicRosBridge(SimulationClock(physics_dt_s))
