from math import atan

import pytest

from kart_sim.autonomy import MessageHeader, PurePursuitController, Trajectory, TrajectoryPoint
from kart_sim.types import PlanarKartState


@pytest.mark.parametrize("wheelbase_m,lateral_m", [(.8, 1.0), (1.2, 1.0), (1.6, -1.0), (1.2, 0.0)])
def test_pure_pursuit_uses_configured_wheelbase(wheelbase_m, lateral_m):
    path = Trajectory(
        MessageHeader(0.0, "map", 1, "test"),
        (TrajectoryPoint(0.0, 0.0, 0.0), TrajectoryPoint(1.0, 2.0, lateral_m)),
        1.0,
    )
    controller = PurePursuitController(
        lookahead_m=1.0,
        speed_lookahead_gain_s=0.0,
        wheelbase_m=wheelbase_m,
        maximum_steering_rad=1.0,
        maximum_steering_rate_radps=100.0,
    )
    output = controller.compute(PlanarKartState(), path, 0.1)
    assert output.steering_rad == pytest.approx(atan(2.0 * wheelbase_m * lateral_m / (4.0 + lateral_m ** 2)))
    assert output.target_xy_m == (2.0, lateral_m)
