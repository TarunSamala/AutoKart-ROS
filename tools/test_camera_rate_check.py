from types import SimpleNamespace

from camera_rate_check import Stats


def hdr(sec, nanosec):
    return SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=sec, nanosec=nanosec)))


def test_stats_computes_rate_and_monotonicity(monkeypatch):
    clock = [0.0]
    monkeypatch.setattr("camera_rate_check.time.monotonic", lambda: clock[0])
    stats = Stats("/topic")
    for i in range(5):
        clock[0] = i * 0.1
        stats.on_msg(hdr(i, 0))
    summary = stats.summary()
    assert summary["frames_received"] == 5
    assert summary["nonmonotonic_stamp_count"] == 0
    assert abs(summary["observed_rate_hz"] - 10.0) < 1e-6


def test_stats_flags_nonmonotonic_stamps(monkeypatch):
    clock = [0.0]
    monkeypatch.setattr("camera_rate_check.time.monotonic", lambda: clock[0])
    stats = Stats("/topic")
    stats.on_msg(hdr(2, 0))
    clock[0] = 0.1
    stats.on_msg(hdr(1, 0))  # stamp went backwards
    assert stats.nonmonotonic == 1


def test_stats_empty_summary_has_no_rate():
    stats = Stats("/topic")
    summary = stats.summary()
    assert summary["frames_received"] == 0
    assert summary["observed_rate_hz"] == 0.0
    assert summary["mean_interval_s"] is None
