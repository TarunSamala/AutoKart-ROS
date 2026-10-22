from kart_sim.watchdog import WindowedTaskWatchdog


def test_nominal_too_fast_stuck_deadline_and_expiry_are_distinct():
    nominal = WindowedTaskWatchdog(.003, .005)
    assert nominal.observe(0, 1).healthy
    assert nominal.observe(.004, 2).healthy
    fast = WindowedTaskWatchdog(.003, .005)
    fast.observe(0, 1)
    assert fast.observe(.001, 2).fault == "WATCHDOG_TOO_FAST"
    stuck = WindowedTaskWatchdog(.003, .005)
    stuck.observe(0, 1)
    assert stuck.observe(.004, 1).fault == "WATCHDOG_SEQUENCE_STUCK"
    deadline = WindowedTaskWatchdog(.003, .005)
    deadline.observe(0, 1)
    assert deadline.observe(.006, 2).fault == "WATCHDOG_DEADLINE"
    expired = WindowedTaskWatchdog(.003, .005)
    expired.observe(0, 1)
    assert expired.check_expired(.006).fault == "WATCHDOG_EXPIRED"
