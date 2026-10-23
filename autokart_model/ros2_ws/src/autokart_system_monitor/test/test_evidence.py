from autokart_system_monitor.evidence import TopicEvidence


def test_rate_is_arrival_rate_and_stale_source_stamp_is_distinct():
    evidence = TopicEvidence()
    evidence.observe(1.0, 1_000_000_000, {'source':'SYNTHETIC', 'valid': False})
    evidence.observe(1.1, 1_000_000_000, {'source':'SYNTHETIC', 'valid': False})
    snapshot = evidence.snapshot(5_000_000_000)
    assert abs(snapshot['observed_arrival_rate_hz'] - 10) < 1e-8
    assert snapshot['source_age_s'] == 4
    assert snapshot['nonmonotonic_stamp_count'] == 1
    assert snapshot['clock_domain_verified'] is False
    assert snapshot['reported']['source'] == 'SYNTHETIC'
    assert snapshot['queue_backlog'] is None


def test_bounded_arrival_history_and_malformed_evidence():
    evidence = TopicEvidence()
    for index in range(1000):
        evidence.observe(index, payload=[], malformed=True)
    assert len(evidence.arrivals) == 128
    assert evidence.snapshot(0)['malformed'] is True
    assert evidence.snapshot(0)['source_age_s'] is None
