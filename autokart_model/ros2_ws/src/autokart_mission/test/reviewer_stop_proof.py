"""Independent senior review regressions: targets and wheel-only cannot prove stop."""
import pytest
from autokart_mission.stop_verification import StopVerifier
from autokart_mission.braking_fsm import BrakingFSM


def evidence(seq, source='LIVE_PHYSICAL', **changes):
    result = dict(schema_version=1, source=source, sample_time_ns=1_000_000_000+seq*100_000_000,
                  sequence=seq, boot_id=17, clock_valid=True, wheel_valid=True,
                  ground_valid=True, ground_independent=True, ground_speed_source='external_optical',
                  vcu_valid=True, stop_active=True, brake_instrumented=True, brake_valid=True,
                  brake_applied=True, estop_active=False, faults=[], wheel_speed_mps=0.,ground_speed_mps=0.)
    result.update(changes)
    return result

@pytest.mark.parametrize('changes', [dict(ground_independent=False), dict(ground_valid=False),
    dict(ground_speed_source='wheel_odom'), dict(brake_applied=False),dict(vcu_valid=False),
    dict(clock_valid=False),dict(ground_speed_mps=0.3),dict(faults=['SENSOR_FAULT'])])
def test_disqualifying_feedback_never_completes_hold(changes):
    verifier = StopVerifier('LIVE_PHYSICAL')
    for seq in range(15):
        sample = evidence(seq, **changes)
        result = verifier.update(sample, sample['sample_time_ns'])
        assert not result['verified'] and not result['physical_stop_verified']


def test_duplicate_ack_like_packet_never_builds_hold():
    verifier = StopVerifier('LIVE')
    sample = evidence(1)
    for _ in range(100):
        result = verifier.update(sample, sample['sample_time_ns'])
    assert result['distinct_samples'] == 1 and not result['verified']


def test_synthetic_stationary_hold_is_never_physical_stop_proof():
    verifier = StopVerifier('SIMULATED')
    for seq in range(15):
        sample = evidence(seq,source='SYNTHETIC')
        result = verifier.update(sample,sample['sample_time_ns'])
    assert result['verified'] and not result['physical_stop_verified']


def test_stop_latch_survives_moving_feedback_and_requires_explicit_disarmed_reset():
    fsm = BrakingFSM()
    fsm.update(corridor_valid=True,ready=True,stop_trigger=False,verification={})
    fsm.update(corridor_valid=True,ready=True,stop_trigger=False,verification={})
    assert fsm.request_start()
    fsm.update(corridor_valid=True,ready=True,stop_trigger=False,verification={})
    fsm.update(corridor_valid=True,ready=True,stop_trigger=True,verification={})
    for _ in range(3):
        snapshot = fsm.update(corridor_valid=True,ready=True,stop_trigger=False,verification={'verified':False})
    assert snapshot['stop_latched'] and snapshot['state']=='STOP_UNVERIFIED'
    assert not fsm.request_start()
    assert not fsm.reset(disarmed=False,verification={'verified':True})
    assert not fsm.reset(disarmed=True,verification={'verified':False})
    assert fsm.reset(disarmed=True,verification={'verified':True})
