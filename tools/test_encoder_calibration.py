import pytest

from encoder_calibration import counts_per_metre


def test_counts_per_metre_uses_absolute_count_change_and_measured_distance():
    assert counts_per_metre(100, 500, 2) == 200
    assert counts_per_metre(500, 100, 2) == 200
    with pytest.raises(ValueError):
        counts_per_metre(100, 100, 2)
