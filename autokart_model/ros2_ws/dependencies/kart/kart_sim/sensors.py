"""Deterministic asynchronous headless sensor foundation; ground truth is never a sensor message."""
from __future__ import annotations
from dataclasses import dataclass, asdict
from random import Random
from math import floor, pi, sqrt
from typing import Any

@dataclass(frozen=True)
class SensorHeader:
    sensor_id:str; sensor_type:str; sequence:int; frame_id:str; capture_start_s:float; capture_end_s:float; measurement_timestamp_s:float; delivery_timestamp_s:float; sensor_clock_timestamp_s:float; valid:bool; quality:str; fault_flags:tuple[str,...]=()

@dataclass(frozen=True)
class SensorMessage:
    header:SensorHeader; payload:dict[str,Any]

class AsyncSensor:
    def __init__(self,sensor_id:str,sensor_type:str,frame_id:str,rate_hz:float,*,phase_s:float=0,acquisition_s:float=0,processing_s:float=0,transport_s:float=0,jitter_s:float=0,clock_offset_s:float=0,clock_drift_ppm:float=0,seed:int=1) -> None:
        if rate_hz<=0 or min(phase_s,acquisition_s,processing_s,transport_s,jitter_s)<0: raise ValueError("invalid sensor timing")
        self.sensor_id,self.sensor_type,self.frame_id=sensor_id,sensor_type,frame_id; self.period=1/rate_hz; self.next_capture=phase_s; self.acquisition_s=acquisition_s; self.processing_s=processing_s; self.transport_s=transport_s; self.jitter_s=jitter_s; self.offset=clock_offset_s; self.drift=clock_drift_ppm*1e-6; self.random=Random(seed); self.sequence=0; self.queue:list[SensorMessage]=[]; self.fault: str|None=None
    def clock(self,t:float)->float:return (1+self.drift)*t+self.offset
    def update(self,t:float,truth:dict[str,Any])->list[SensorMessage]:
        while t+1e-12>=self.next_capture:
            start=self.next_capture; end=start+self.acquisition_s; measurement=end; delay=max(0,self.processing_s+self.transport_s+self.random.gauss(0,self.jitter_s)); delivery=end+delay
            self.sequence+=1; valid=self.fault not in ("dropout","power_loss"); payload=self.measure(truth) if valid else {}
            if self.fault=="frozen" and self.queue: payload=self.queue[-1].payload
            flags=() if self.fault is None else (self.fault,); h=SensorHeader(self.sensor_id,self.sensor_type,self.sequence,self.frame_id,start,end,measurement,delivery,self.clock(measurement),valid,"VALID" if valid else "INVALID",flags); self.queue.append(SensorMessage(h,payload)); self.next_capture+=self.period
        ready=[m for m in self.queue if m.header.delivery_timestamp_s<=t+1e-12]; self.queue=[m for m in self.queue if m not in ready]; return ready
    def measure(self,truth:dict[str,Any])->dict[str,Any]:return {}

class ImuSensor(AsyncSensor):
    def __init__(self,sensor_id:str,frame_id:str,rate_hz:float,*,noise_std:float=.0,bias_mps2:float=0,bias_radps:float=0,**kwargs): super().__init__(sensor_id,"imu",frame_id,rate_hz,**kwargs); self.noise_std=noise_std; self.ba=bias_mps2;self.bg=bias_radps
    def measure(self,t:dict[str,Any])->dict[str,Any]:
        # Body planar IMU: specific force includes gravity upward at rest.
        return {"accel_mps2":[t.get("ax_mps2",0)+self.ba+self.random.gauss(0,self.noise_std),t.get("ay_mps2",0)+self.random.gauss(0,self.noise_std),9.80665+self.random.gauss(0,self.noise_std)],"gyro_radps":[0,0,t.get("yaw_rate_radps",0)+self.bg+self.random.gauss(0,self.noise_std)]}

class WheelEncoder(AsyncSensor):
    def __init__(self,sensor_id:str,frame_id:str,rate_hz:float,*,cpr:int=360,**kwargs):
        if cpr<=0:raise ValueError("invalid encoder CPR")
        super().__init__(sensor_id,"wheel_encoder",frame_id,rate_hz,**kwargs);self.cpr=cpr;self.count=0;self._last_angle=0.
    def measure(self,t:dict[str,Any])->dict[str,Any]:
        angle=t.get("wheel_angle_rad",0.); new=floor(angle/(2*pi/self.cpr)); delta=new-self.count;self.count=new;return {"count":self.count,"delta_count":delta,"wheel_speed_radps":t.get("wheel_speed_radps",0)}
