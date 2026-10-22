from stop_trigger_test import StopTriggerObserver


def _planner(stamp=10, at=1_100_000_000, gap=0.8):
    return {"source": "LIVE", "valid": True, "decision": "STOP_CANDIDATE",
            "processed_at_monotonic_ns": at, "sample_time_ns": stamp,
            "bumper_to_cone_m": gap, "stop_gap_from_bumper_m": 0.4,
            "stopping_distance_m": 0.5, "target_speed_mps": 0.0,
            "dead_reckoned": False}


def _tel(at, seq, dac, en, brk, state="ARMED"):
    return {"event": "vcu_tel", "time_ns": at, "ack_seq": seq, "dac_mv": dac,
            "en": en, "brk": brk, "state": state, "fault": 0}


def test_complete_wheels_off_telemetry_chain():
    observer = StopTriggerObserver(timeout_s=1.0)
    observer.add_trace(_tel(1_000_000_000, 1, 1500, True, False))
    observer.add_planner(_planner())
    observer.add_trace({"event": "bridge_tx", "time_ns": 1_120_000_000,
                        "frame_stamp_ns": 10, "seq": 9, "requested_dac_mv": 0})
    observer.add_trace(_tel(1_150_000_000, 9, 0, False, True))
    assert observer.result["result"] == "PASS_TELEMETRY_ONLY"
    assert observer.result["trigger_to_tel_ms"] == 50


def test_missing_active_or_mismatched_ack_or_brake_fails_closed():
    observer = StopTriggerObserver(timeout_s=0.1)
    observer.add_planner(_planner())
    assert observer.trigger is None
    observer.add_trace(_tel(1_000_000_000, 1, 1500, True, False))
    observer.add_planner(_planner(gap=1.0))
    assert observer.trigger is None  # cone not inside stopping envelope
    observer.add_planner(_planner())
    observer.add_trace({"event": "bridge_tx", "time_ns": 1_120_000_000,
                        "frame_stamp_ns": 10, "seq": 9, "requested_dac_mv": 0})
    observer.add_trace(_tel(1_150_000_000, 8, 0, False, True))
    observer.add_trace(_tel(1_160_000_000, 9, 0, False, False))
    assert observer.result is None
    assert observer.timed_out(1_201_000_000)


def test_old_armed_sample_and_late_zero_telemetry_do_not_pass():
    observer = StopTriggerObserver(timeout_s=0.1)
    observer.add_trace(_tel(1_000_000_000, 1, 1500, True, False))
    observer.add_planner(_planner(at=1_600_000_000))
    assert observer.trigger is None
    observer.add_trace(_tel(1_550_000_000, 2, 1500, True, False))
    observer.add_planner(_planner(at=1_600_000_000))
    observer.add_trace({"event": "bridge_tx", "time_ns": 1_620_000_000,
                        "frame_stamp_ns": 10, "seq": 9, "requested_dac_mv": 0})
    observer.add_trace(_tel(1_701_000_000, 9, 0, False, True))
    assert observer.result is None
