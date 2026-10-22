import unittest
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from one_cone_logic import OneConePlanner, Sample, Decision

def sample(**kwargs):
    values=dict(cone_gap_m=0.6, wheel_speed_mps=0.1, observation_age_s=0.01,
                source='LIVE', calibration_status='MOUNT_VERIFIED', depth_valid=True,
                wheel_speed_valid=True, emergency_stop_clear=True)
    values.update(kwargs)
    return Sample(**values)

class OneConeTests(unittest.TestCase):
    def test_beyond_envelope(self):
        result=OneConePlanner().update(sample())
        self.assertEqual(result.decision,Decision.APPROACH_CANDIDATE)
        self.assertGreater(result.trigger_gap_m,0.15)
    def test_at_envelope(self):
        result=OneConePlanner().update(sample(cone_gap_m=0.15))
        self.assertEqual(result.decision,Decision.STOP)
    def test_sim_never_authorizes(self):
        self.assertEqual(OneConePlanner().update(sample(source='SIMULATED')).decision,Decision.STOP)
    def test_missing_calibration(self):
        self.assertEqual(OneConePlanner().update(sample(calibration_status='NOT_VERIFIED')).decision,Decision.STOP)
    def test_estop(self):
        self.assertEqual(OneConePlanner().update(sample(emergency_stop_clear=False)).decision,Decision.STOP)
    def test_no_encoder(self):
        self.assertEqual(OneConePlanner().update(sample(wheel_speed_valid=False)).decision,Decision.STOP)
    def test_outlier(self):
        self.assertEqual(OneConePlanner().update(sample(cone_gap_m=float('nan'))).decision,Decision.STOP)
    def test_stale(self):
        self.assertEqual(OneConePlanner().update(sample(observation_age_s=0.3)).decision,Decision.STOP)
    def test_overspeed(self):
        self.assertEqual(OneConePlanner().update(sample(wheel_speed_mps=0.3)).decision,Decision.STOP)
    def test_latch(self):
        planner=OneConePlanner()
        planner.update(sample(cone_gap_m=0.1))
        self.assertEqual(planner.update(sample(cone_gap_m=1.0)).decision,Decision.STOP)
    def test_envelope_number(self):
        result=OneConePlanner().update(sample(cone_gap_m=1.0,wheel_speed_mps=0.2))
        self.assertAlmostEqual(result.trigger_gap_m,0.33,places=6)

if __name__=='__main__': unittest.main()
