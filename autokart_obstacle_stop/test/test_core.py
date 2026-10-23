import unittest

from autokart_obstacle_stop.core import ConeSample, GuardConfig, ObstacleStopGuard


def cone(x, y, *, confidence=0.95, sigma=0.05, cone_id=1):
    return ConeSample(cone_id, x, y, confidence, sigma)


def guard(**overrides):
    values = dict(
        minimum_x_m=0.10,
        stop_distance_m=3.0,
        corridor_half_width_m=0.75,
        minimum_confidence=0.50,
        perception_timeout_s=0.50,
        stop_hold_s=1.0,
        clear_frames_required=3,
        allow_ground_truth=False,
    )
    values.update(overrides)
    return ObstacleStopGuard(GuardConfig(**values))


def ingest(subject, now, cones, *, frame="base_link", ground_truth=False):
    return subject.ingest_frame(
        now, frame_id=frame, ground_truth=ground_truth, cones=cones
    )


class ObstacleStopGuardTests(unittest.TestCase):
    def test_starts_fail_closed_until_perception_arrives(self):
        decision = guard().decision(0.0)
        self.assertTrue(decision.stop_required)
        self.assertEqual(decision.reason, "PERCEPTION_MISSING_OR_STALE")

    def test_forward_cone_inside_corridor_requests_stop(self):
        decision = ingest(guard(), 1.0, [cone(2.0, 0.1, cone_id=7)])
        self.assertTrue(decision.stop_required)
        self.assertEqual(decision.reason, "OBSTACLE_IN_FORWARD_CORRIDOR")
        self.assertEqual(decision.cone_id, 7)
        self.assertAlmostEqual(decision.nearest_clearance_m, 1.95)

    def test_track_boundary_cones_do_not_stop_center_corridor(self):
        subject = guard(stop_hold_s=0.0, clear_frames_required=1)
        decision = ingest(subject, 1.0, [cone(2.0, 1.5), cone(2.0, -1.5)])
        self.assertFalse(decision.stop_required)
        self.assertEqual(decision.reason, "FORWARD_CORRIDOR_CLEAR")

    def test_cones_behind_or_beyond_range_do_not_stop(self):
        subject = guard(stop_hold_s=0.0, clear_frames_required=1)
        decision = ingest(subject, 1.0, [cone(-0.2, 0.0), cone(4.0, 0.0)])
        self.assertFalse(decision.stop_required)

    def test_low_confidence_cone_does_not_stop(self):
        subject = guard(stop_hold_s=0.0, clear_frames_required=1)
        decision = ingest(subject, 1.0, [cone(1.0, 0.0, confidence=0.49)])
        self.assertFalse(decision.stop_required)

    def test_uncertainty_expands_cone_into_corridor(self):
        decision = ingest(guard(), 1.0, [cone(2.0, 0.9, sigma=0.2)])
        self.assertTrue(decision.stop_required)

    def test_stop_latch_requires_hold_time_and_clear_frames(self):
        subject = guard(stop_hold_s=1.0, clear_frames_required=3)
        ingest(subject, 1.0, [cone(2.0, 0.0)])
        self.assertTrue(ingest(subject, 1.2, []).stop_required)
        self.assertTrue(ingest(subject, 1.6, []).stop_required)
        self.assertTrue(ingest(subject, 1.9, []).stop_required)
        self.assertFalse(ingest(subject, 2.1, []).stop_required)

    def test_stale_perception_fails_closed_after_clear_frame(self):
        subject = guard(stop_hold_s=0.0, clear_frames_required=1)
        self.assertFalse(ingest(subject, 1.0, []).stop_required)
        decision = subject.decision(1.51)
        self.assertTrue(decision.stop_required)
        self.assertEqual(decision.reason, "PERCEPTION_MISSING_OR_STALE")

    def test_wrong_frame_and_ground_truth_fail_closed(self):
        wrong_frame = ingest(guard(), 1.0, [], frame="camera_optical")
        self.assertTrue(wrong_frame.stop_required)
        self.assertEqual(wrong_frame.reason, "PERCEPTION_FRAME_NOT_BASE_LINK")
        synthetic = ingest(guard(), 1.0, [], ground_truth=True)
        self.assertTrue(synthetic.stop_required)
        self.assertEqual(synthetic.reason, "GROUND_TRUTH_NOT_ALLOWED_FOR_ACTUATION")

    def test_invalid_configuration_is_rejected(self):
        with self.assertRaises(ValueError):
            GuardConfig(stop_distance_m=0.05).validate()


if __name__ == "__main__":
    unittest.main()
