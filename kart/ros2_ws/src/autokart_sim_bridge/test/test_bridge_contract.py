import unittest
from pathlib import Path


PACKAGE = Path(__file__).resolve().parents[1]
VCU_BRIDGE = (PACKAGE / "autokart_sim_bridge/vcu_bridge.py").read_text(
    encoding="utf-8"
)
FOLLOWER_RUNNER = (PACKAGE / "autokart_sim_bridge/gazebo_full_lap.py").read_text(
    encoding="utf-8"
)


class BridgeContractTest(unittest.TestCase):
    def test_motion_commands_remain_vcu_gated(self) -> None:
        self.assertIn('"control/motion_command"', VCU_BRIDGE)
        self.assertIn('"integrated_mode"', VCU_BRIDGE)
        self.assertIn("direct ROS actuator commands are prohibited", VCU_BRIDGE)
        for prohibited in ("/motor/torque", "/brake/effort", "/steering/effort"):
            self.assertNotIn(prohibited, VCU_BRIDGE)

    def test_native_runner_publishes_immutable_snapshots(self) -> None:
        self.assertIn("AuthoritativeVehicleSnapshot", FOLLOWER_RUNNER)
        self.assertIn("snapshot_sink=recorder.publish_snapshot", FOLLOWER_RUNNER)
        self.assertIn("timestamp_violations", FOLLOWER_RUNNER)
        self.assertIn("cumulative_captured", FOLLOWER_RUNNER)


if __name__ == "__main__":
    unittest.main()
