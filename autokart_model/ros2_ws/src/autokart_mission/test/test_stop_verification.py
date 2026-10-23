"""Independent movement evidence must survive faults and sustained hold."""
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import pytest
from autokart_mission.stop_verification import StopVerifier
from autokart_mission.braking_fsm import BrakingFSM


def evidence(stamp, seq, **overrides):
    value = dict(schema_version=1, source='SIMULATED', sample_time_ns=stamp, sequence=seq,
                 boot_id=1, wheel_speed_mps=0., wheel_valid=True, ground_speed_mps=0.,
                 ground_valid=True, ground_independent=True, ground_speed_source='synthetic_optical_fixture',
                 brake_instrumented=True, brake_valid=True, brake_applied=True,
                 vcu_valid=True, stop_active=True, estop_active=False, faults=[], clock_valid=True,
                 disarmed=True)
    value.update(overrides)
    return value


def held(verifier, start=1_000_000_000, **overrides):
    for i in range(12):
        t = start + i*100_000_000
        result = verifier.update(evidence(t, i, **overrides), t)
    return result


def running_fsm():
    f = BrakingFSM()
    args = dict(corridor_valid=True, ready=True, stop_trigger=False, verification={})
    assert f.update(**args)['state'] == 'CORRIDOR_VALID'
    assert f.update(**args)['state'] == 'READY'
    assert f.update(**args)['state'] == 'READY'  # no auto start
    assert f.request_start()
    assert f.update(**args)['state'] == 'RUNNING'
    return f


def test_normal_simulated_completion_and_no_physical_claim():
    verifier = StopVerifier('SIMULATED')
    f = running_fsm()
    args = dict(corridor_valid=True, ready=True, stop_trigger=True, verification=verifier.result)
    assert f.update(**args)['state'] == 'STOP_TRIGGERED'
    assert f.update(**args)['state'] == 'STOPPING'
    args['verification'] = held(verifier)
    assert args['verification']['verified']
    assert not args['verification']['physical_stop_verified']
    assert f.update(**args)['state'] == 'STOP_VERIFIED'
    assert not f.reset(disarmed=False, verification=args['verification'])
    assert f.reset(disarmed=True, verification=args['verification'])
    assert f.state == 'WAITING'


@pytest.mark.parametrize('overrides,reason', [
    ({'wheel_valid':False}, 'ENCODER_INVALID'),
    ({'brake_valid':False}, 'BRAKE_FEEDBACK_MISSING_OR_UNAPPLIED'),
    ({'brake_applied':False}, 'BRAKE_FEEDBACK_MISSING_OR_UNAPPLIED'),
    ({'vcu_valid':False}, 'VCU_DISCONNECTED_OR_INVALID'),
    ({'ground_valid':False}, 'GROUND_MOTION_UNAVAILABLE'),
    ({'ground_independent':False}, 'GROUND_MOTION_NOT_INDEPENDENT'),
    ({'ground_speed_source':'wheel_odometry'}, 'GROUND_MOTION_SOURCE_INVALID'),
    ({'faults':['ENCODER_FAILURE']}, 'BLOCKING_VCU_FAULT_OR_UNKNOWN'),
    ({'wheel_speed_mps':float('nan')}, 'WHEELS_NOT_STATIONARY'),
    ({'source':'LIVE'}, 'STOP_EVIDENCE_SOURCE_MISMATCH'),
])
def test_faults_never_verify(overrides, reason):
    result = held(StopVerifier('SIMULATED'), **overrides)
    assert not result['verified'] and reason in result['reason_codes']


def test_stop_ack_or_target_alone_cannot_verify():
    v = StopVerifier('SIMULATED')
    result = v.update({'schema_version':1, 'source':'SIMULATED', 'stop_ack':True,
                       'target_speed_mps':0.0}, 1_000_000_000)
    assert not result['verified']


def test_stale_encoder_and_telemetry_revoke_completed_hold():
    v = StopVerifier('SIMULATED')
    assert held(v)['verified']
    stale = v.update(evidence(2_100_000_000,11), 2_400_000_000)
    assert not stale['verified']
    assert 'STOP_TELEMETRY_STALE_OR_UNTIMED' in stale['reason_codes']


def test_duplicate_sample_never_accumulates_hold():
    v = StopVerifier('SIMULATED')
    for i in range(15):
        result = v.update(evidence(1_000_000_000,0),1_000_000_000+i*10_000_000)
        assert not result['verified']
    assert result['distinct_samples'] == 1


def test_wheel_lock_chassis_slide_fails():
    assert not held(StopVerifier('SIMULATED'),ground_speed_mps=0.4)['verified']


def test_wheel_oscillation_restarts_hold():
    v=StopVerifier('SIMULATED')
    for i in range(30):
        t=1_000_000_000+i*100_000_000
        result=v.update(evidence(t,i,wheel_speed_mps=0.06 if i%8==0 else 0.),t)
        assert not result['verified']


def test_boot_change_and_nonmonotone_sequence_fail():
    v=StopVerifier('SIMULATED'); assert held(v)['verified']
    result=v.update(evidence(2_200_000_000,12,boot_id=2),2_200_000_000)
    assert 'STOP_VCU_SESSION_CHANGED' in result['reason_codes']
    result=v.update(evidence(2_300_000_000,11,boot_id=2),2_300_000_000)
    assert 'STOP_EVIDENCE_NONMONOTONE' in result['reason_codes']


def test_interruption_estop_manual_reset_and_hold_latch():
    f=running_fsm()
    args=dict(corridor_valid=True,ready=True,stop_trigger=False,verification={})
    assert f.update(**args,interrupted=True)['state']=='FAULT'
    assert not f.request_start() and not f.reset(disarmed=True,verification={})
    verification=held(StopVerifier('SIMULATED'))
    assert f.reset(disarmed=True,verification=verification)
    assert f.update(**args,emergency=True)['state']=='FAULT'
    assert f.update(**args)['state']=='FAULT'


def test_stop_completion_is_revoked_by_missing_feedback():
    f=running_fsm(); args=dict(corridor_valid=True,ready=True,stop_trigger=True,verification={})
    f.update(**args); f.update(**args)
    args['verification']=held(StopVerifier('SIMULATED'))
    assert f.update(**args)['state']=='STOP_VERIFIED'
    args['verification']={}
    assert f.update(**args)['state']=='STOP_UNVERIFIED'
    assert f.stop_latched
