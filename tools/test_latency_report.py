import json
from types import SimpleNamespace

import pytest

from latency_report import LatencyCollector, TRACE_TOPIC, TOPICS, percentile


def test_only_full_confirmed_dac_zero_chain_is_measured():
    collector = LatencyCollector()
    stamp = 1_000_000_000
    for index, topic in enumerate(TOPICS):
        collector.add(topic, SimpleNamespace(payload_json=json.dumps({
            "source": "LIVE", "sample_time_ns": stamp,
            "processed_at_monotonic_ns": stamp + (index + 1) * 10_000_000,
            "decision": "STOP_CANDIDATE", "target_speed_mps": 0.0})))
    collector.add(TRACE_TOPIC, SimpleNamespace(data=json.dumps({"event": "bridge_tx",
        "frame_stamp_ns": stamp, "seq": 7, "time_ns": stamp + 50_000_000,
        "requested_dac_mv": 0})))
    collector.add(TRACE_TOPIC, SimpleNamespace(data=json.dumps({"event": "vcu_tel",
        "ack_seq": 7, "time_ns": stamp + 80_000_000, "dac_mv": 0})))
    rows = collector.complete()
    assert len(rows) == 1 and rows[0]["camera_to_tel_ms"] == pytest.approx(80)
    assert rows[0]["stage_ms"]["vcu_tel"] == pytest.approx(30)
    assert percentile([10, 20, 30], 0.95) == pytest.approx(29)
