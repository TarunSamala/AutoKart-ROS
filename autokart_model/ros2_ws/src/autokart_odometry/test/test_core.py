import math
import pytest
from autokart_odometry.core import Config,Estimator

def fixture(rate=0.0):
    e=Estimator(Config(counts_per_rev=100,circumference_m=1.0,wheelbase_m=1.0,calibration_verified=True,calibration_hash="fixture"))
    for t in range(0,21): e.add_imu(1_000_000_000+t*10_000_000,rate,"SYNTHETIC")
    return e

def wheel(count=0,stamp=1_000_000_000,tick=0,seq=0,**kw):
    return dict(schema_version=1,source="SYNTHETIC",valid=True,clock_valid=True,calibration_verified=True,calibration_hash="fixture",steering_valid=True,steering_actual_rad=0.0,sample_time_ns=stamp,sample_time_us=tick,axle_count_total=count,sequence=seq,boot_id=1,**kw)

def step(e,count=10,**changes):
    assert e.update(wheel(),1_000_000_000) is None
    w=wheel(count,1_100_000_000,100000,1); w.update(changes)
    return e.update(w,1_100_000_000)

def test_straight_forward_reverse_distances():
    for n in (10,-10):
        e=fixture(); v=step(e,n)
        assert v["x"]==pytest.approx(n/100)
        assert v["distance_travelled_total"]==pytest.approx(abs(n)/100)
        assert v["signed_distance"]==pytest.approx(n/100)
        assert v["position_variance"]>0

def test_gyro_turn_uses_midpoint_and_alignment():
    e=fixture(0.2)
    e.update(wheel(),1_000_000_000)
    w=wheel(10,1_100_000_000,100000,1); w["steering_actual_rad"]=math.atan(.2)
    v=e.update(w,1_100_000_000)
    assert v["yaw"]==pytest.approx(.02)
    assert v["y"]==pytest.approx(.1*math.sin(.01))

@pytest.mark.parametrize("changes,reason",[({"boot_id":2},"BOOT_OR_SOURCE_CHANGED"),({"sequence":0},"NONMONOTONIC_WHEEL"),({"sample_time_us":0},"NONMONOTONIC_WHEEL"),({"clock_valid":False},"WHEEL_UNQUALIFIED"),({"calibration_hash":"wrong"},"WHEEL_UNQUALIFIED"),({"axle_count_total":2**63},"WHEEL_MALFORMED"),({"sample_time_ns":1_120_000_000},"WHEEL_STALE_OR_SOURCE_INVALID"),({"sample_time_us":90000},"CLOCK_INTERVAL_INVALID")])
def test_faults_freeze_pose(changes,reason):
    e=fixture(); assert step(e,**changes) is None
    assert e.reason==reason and e.x==0

def test_dropped_packet_recovers_cumulative_counts():
    e=fixture(); e.update(wheel(),1_000_000_000)
    v=e.update(wheel(20,1_200_000_000,200000,4),1_200_000_000)
    assert v["x"]==pytest.approx(.2)

def test_missing_imu_gap_slip_and_speed_fail_closed():
    e=fixture(); e.imu.clear(); assert step(e) is None; assert e.reason=="IMU_NOT_BRACKETED"
    e=fixture(1); assert step(e) is None; assert e.reason=="SLIP_OR_SENSOR_DISAGREEMENT"
    e=fixture(); assert step(e,1000) is None; assert e.reason=="KINEMATIC_LIMIT"


def test_startup_missing_imu_bracket_rebaselines_before_first_pose_only():
    e=Estimator(Config(counts_per_rev=100,circumference_m=1.0,wheelbase_m=1.0,
        calibration_verified=True,calibration_hash="fixture"))
    assert e.update(wheel(),1_000_000_000) is None
    for stamp in range(1_100_000_000,1_210_000_000,10_000_000):
        e.add_imu(stamp,0.0,"SYNTHETIC")
    assert e.update(wheel(10,1_100_000_000,100000,1),1_100_000_000) is None
    assert e.reason=="IMU_NOT_BRACKETED" and e.previous["sequence"]==1
    result=e.update(wheel(20,1_200_000_000,200000,2),1_200_000_000)
    assert result is not None and result["x"]==pytest.approx(.1)


def test_startup_stale_wheel_rebaselines_but_post_pose_stale_latches_continuity():
    e=fixture()
    assert e.update(wheel(),1_300_000_000) is None
    assert e.reason=="WHEEL_STALE_OR_SOURCE_INVALID" and not e.continuity_lost
    assert e.previous["sequence"]==0
    # Establish an actual output with a fresh interval from that baseline.
    for stamp in range(1_000_000_000,1_410_000_000,10_000_000):
        if not e.imu or stamp>e.imu[-1][0]: e.add_imu(stamp,0.0,"SYNTHETIC")
    result=e.update(wheel(10,1_100_000_000,100000,1),1_100_000_000)
    assert result is not None
    assert e.update(wheel(20,1_200_000_000,200000,2),1_500_000_000) is None
    assert e.reason=="WHEEL_STALE_OR_SOURCE_INVALID" and e.continuity_lost

def test_defaults_no_guessed_calibration():
    e=Estimator(); assert e.update(wheel(),1_000_000_000) is None
    assert e.reason=="CALIBRATION_UNVERIFIED"

def test_source_mismatch_and_imu_timestamp_rejected():
    e=fixture(); assert not e.add_imu(1,0,"SYNTHETIC")
    assert step(e,source="REPLAY") is None
    assert e.reason=="BOOT_OR_SOURCE_CHANGED"

@pytest.mark.parametrize('key',['valid','clock_valid','steering_valid','calibration_verified'])
def test_truthy_strings_cannot_replace_qualification_booleans(key):
    e=fixture(); assert step(e,**{key:'true'}) is None
    assert e.reason=='WHEEL_UNQUALIFIED'


def test_nan_kinematic_bound_rejected_at_configuration():
    with pytest.raises(ValueError): Estimator(Config(max_speed_mps=float('nan')))


def test_imu_gap_inside_interval_and_time_regression_fail():
    e=fixture(); e.imu.clear()
    e.add_imu(1_000_000_000,0,'SYNTHETIC')
    e.add_imu(1_100_000_000,0,'SYNTHETIC')
    assert step(e) is None
    assert e.reason=='IMU_GAP_OR_SOURCE_MISMATCH'


@pytest.mark.parametrize('bad',["invalid","nonmonotonic"])
def test_rejected_imu_interval_never_bridges_to_future_good_data(bad):
    e=fixture(); e.update(wheel(),1_000_000_000)
    assert e.update(wheel(10,1_100_000_000,100000,1),1_100_000_000) is not None
    if bad=='invalid': assert not e.add_imu(1_210_000_000,float('nan'),'SYNTHETIC')
    else: assert not e.add_imu(1_100_000_000,0,'SYNTHETIC')
    assert e.continuity_lost and not e.imu
    for t in range(1_000_000_000,1_210_000_000,10_000_000): e.add_imu(t,0,'SYNTHETIC')
    assert e.update(wheel(20,1_200_000_000,200000,2),1_200_000_000) is None
    assert e.reason=='POSE_CONTINUITY_LOST_REINITIALIZE'
