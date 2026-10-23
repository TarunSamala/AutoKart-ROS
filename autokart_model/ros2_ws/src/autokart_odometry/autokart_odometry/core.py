"""Wheel/IMU integration in SI units. No ROS and no hardware access.

Existing autokart_localization.WheelOdometry uses bicycle steering as yaw;
this implementation adds timestamp aligned gyro intervals required by spec.
It must replace that publisher in a launch graph, never run alongside it.
"""
from dataclasses import dataclass
from collections import deque
import math

@dataclass(frozen=True)
class Config:
    counts_per_rev: float = 0.0
    circumference_m: float = 0.0
    wheelbase_m: float = 0.0
    encoder_sign: int = 1
    calibration_verified: bool = False
    calibration_hash: str = ""
    max_interval_s: float = 0.2
    max_imu_gap_s: float = 0.03
    max_speed_mps: float = 5.0
    max_yaw_rate_radps: float = 3.0
    slip_threshold_radps: float = 0.5
    distance_variance_per_m: float = 0.01
    yaw_variance_per_s: float = 0.001

class Estimator:
    def __init__(self, config=Config()):
        self.cfg=config
        for key in ("max_interval_s","max_imu_gap_s","max_speed_mps","max_yaw_rate_radps","slip_threshold_radps"):
            value=getattr(config,key)
            if not math.isfinite(value) or value<=0: raise ValueError(f"invalid {key}")
        self.imu=deque(maxlen=2000)
        self.previous=None
        self.x=self.y=self.yaw=self.speed=self.yaw_rate=0.0
        self.signed_distance=self.distance_total=0.0
        self.rpm=0.0
        self.position_variance=self.yaw_variance=0.0
        self.reason="INITIALIZING"
        self.continuity_lost=False
        self.source=None
        self.has_valid_output=False

    def add_imu(self, stamp_ns, gyro_z, source, valid=True):
        if not valid or source not in ("SYNTHETIC","SIMULATED","REPLAY","LIVE") or not math.isfinite(gyro_z):
            self._fail("IMU_INVALID",self.has_valid_output)
            self.imu.clear()
            return False
        if type(stamp_ns) is not int or stamp_ns<=0 or (self.imu and stamp_ns<=self.imu[-1][0]):
            self._fail("IMU_TIMESTAMP_NONMONOTONIC",self.has_valid_output)
            self.imu.clear()
            return False
        self.imu.append((stamp_ns,gyro_z,source))
        return True

    def _fail(self, reason, discontinuity=False):
        self.reason=reason
        self.continuity_lost |= discontinuity
        return None

    def update(self,wheel,now_ns):
        c=self.cfg
        if not c.calibration_verified or not c.calibration_hash or not all(math.isfinite(v) and v>0 for v in (c.counts_per_rev,c.circumference_m,c.wheelbase_m,c.distance_variance_per_m,c.yaw_variance_per_s)) or c.encoder_sign not in (-1,1):
            return self._fail("CALIBRATION_UNVERIFIED")
        if type(wheel.get("schema_version")) is not int or wheel.get("schema_version")!=1 or wheel.get("valid") is not True or wheel.get("clock_valid") is not True or wheel.get("calibration_verified") is not True or wheel.get("calibration_hash")!=c.calibration_hash or wheel.get("steering_valid") is not True:
            return self._fail("WHEEL_UNQUALIFIED",self.has_valid_output)
        try:
            stamp=wheel["sample_time_ns"]; seq=wheel["sequence"]; count=wheel["axle_count_total"]; tick=wheel["sample_time_us"]; boot=wheel["boot_id"]; steering=wheel["steering_actual_rad"]; source=wheel["source"]
            if any(type(v) is not int for v in (stamp,seq,count,tick,boot)) or not (0<=seq<2**32 and 0<=boot<2**32 and 0<=tick<2**64 and -(2**63)<=count<2**63) or isinstance(steering,bool) or not math.isfinite(steering):
                raise ValueError()
        except (KeyError,TypeError,ValueError): return self._fail("WHEEL_MALFORMED",self.has_valid_output)
        if source not in ("SYNTHETIC","SIMULATED","REPLAY","LIVE") or stamp>now_ns or now_ns-stamp>int(c.max_interval_s*1e9):
            if not self.has_valid_output and source in ("SYNTHETIC","SIMULATED","REPLAY","LIVE") and stamp<=now_ns:
                self.previous=dict(wheel); self.source=source
            return self._fail("WHEEL_STALE_OR_SOURCE_INVALID",self.has_valid_output)
        if self.previous is None:
            self.previous=dict(wheel); self.source=source
            return self._fail("ENCODER_BASELINE")
        prev=self.previous
        if boot!=prev["boot_id"] or source!=self.source:
            self.previous=dict(wheel)
            return self._fail("BOOT_OR_SOURCE_CHANGED",self.has_valid_output)
        delta_seq=(seq-prev["sequence"])&0xffffffff
        dt=(tick-prev["sample_time_us"])/1e6
        ros_dt=(stamp-prev["sample_time_ns"])/1e9
        if delta_seq==0 or delta_seq>=2**31 or dt<=0 or ros_dt<=0:
            return self._fail("NONMONOTONIC_WHEEL",self.has_valid_output)
        if dt>c.max_interval_s or abs(ros_dt-dt)>0.002:
            if not self.has_valid_output:
                self.previous=dict(wheel)
            return self._fail("CLOCK_INTERVAL_INVALID",self.has_valid_output)
        samples=[v for v in self.imu if v[0]>=prev["sample_time_ns"]-int(c.max_imu_gap_s*1e9) and v[0]<=stamp+int(c.max_imu_gap_s*1e9)]
        if not samples or samples[0][0]>prev["sample_time_ns"] or samples[-1][0]<stamp:
            # Before the first pose has ever been published, a startup wheel
            # baseline can legitimately predate the first paired IMU sample.
            # Advance that unpublished baseline so the next fully bracketed
            # interval can initialize.  Once any pose is published, the same
            # gap is a continuity loss and is never healed silently.
            if not self.has_valid_output:
                self.previous=dict(wheel)
            return self._fail("IMU_NOT_BRACKETED")
        area=0.0
        for left,right in zip(samples,samples[1:]):
            a=max(left[0],prev["sample_time_ns"]); z=min(right[0],stamp)
            if z<=a: continue
            gap=(right[0]-left[0])/1e9
            if gap>c.max_imu_gap_s or left[2]!=source or right[2]!=source:
                return self._fail("IMU_GAP_OR_SOURCE_MISMATCH",True)
            def at(t): return left[1]+(right[1]-left[1])*(t-left[0])/(right[0]-left[0])
            area+=(at(a)+at(z))*0.5*(z-a)/1e9
        ds=(count-prev["axle_count_total"])/c.counts_per_rev*c.circumference_m*c.encoder_sign
        speed=ds/dt; rate=area/dt
        self.previous=dict(wheel)
        if abs(speed)>c.max_speed_mps or abs(rate)>c.max_yaw_rate_radps or abs(steering)>=math.pi/2:
            return self._fail("KINEMATIC_LIMIT",True)
        if abs(rate-speed*math.tan(steering)/c.wheelbase_m)>c.slip_threshold_radps:
            return self._fail("SLIP_OR_SENSOR_DISAGREEMENT",True)
        if self.continuity_lost: return self._fail("POSE_CONTINUITY_LOST_REINITIALIZE")
        mid=self.yaw+area/2
        self.x+=ds*math.cos(mid); self.y+=ds*math.sin(mid)
        self.yaw=math.atan2(math.sin(self.yaw+area),math.cos(self.yaw+area))
        self.speed=speed; self.yaw_rate=rate
        self.rpm=(count-prev["axle_count_total"])/c.counts_per_rev*c.encoder_sign/dt*60
        self.signed_distance+=ds; self.distance_total+=abs(ds)
        self.position_variance+=abs(ds)*c.distance_variance_per_m
        self.yaw_variance+=dt*c.yaw_variance_per_s
        self.reason="OK"
        self.has_valid_output=True
        return dict(x=self.x,y=self.y,yaw=self.yaw,speed=speed,yaw_rate=rate,
                    stamp_ns=stamp,source=source,valid=True,distance_travelled_total=self.distance_total,
                    signed_distance=self.signed_distance,rpm=self.rpm,position_variance=self.position_variance,
                    yaw_variance=self.yaw_variance)
