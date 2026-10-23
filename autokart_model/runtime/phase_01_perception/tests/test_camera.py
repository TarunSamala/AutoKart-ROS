import pytest
from autokart_perception.camera import HP60C,CameraError

def test_unbuilt_adapter_message(tmp_path):
    with pytest.raises(OSError):HP60C(library=tmp_path/'missing.so')

def test_mock_disconnect(monkeypatch):
    import autokart_perception.camera as camera
    class Fn:
        def __init__(self,value):self.value=value
        def __call__(self,*args):return self.value(*args) if callable(self.value) else self.value
    class Lib:
        hp_start=Fn(0);hp_connect=Fn(0);hp_read=Fn(0);hp_stop=Fn(None)
    monkeypatch.setattr(camera.ct,'CDLL',lambda _:Lib())
    monkeypatch.setattr(camera,'ROOT',camera.ROOT)
    cam=HP60C(library='mock')
    cam.started=True
    with pytest.raises(CameraError,match='CAMERA_DISCONNECTED'):cam.read()
    cam.close()

def test_packed_rows_preserved(monkeypatch):
    import ctypes as ct
    import time
    import numpy as np
    import autokart_perception.camera as camera
    class Fn:
        def __init__(self,value):self.value=value
        def __call__(self,*args):return self.value(*args) if callable(self.value) else self.value
    class Lib:
        hp_start=Fn(0);hp_connect=Fn(1);hp_stop=Fn(None)
        def __init__(self):self.hp_read=Fn(self.read_impl)
        def read_impl(self,rgb,depth,capacity,meta,timeout):
            rows=np.arange(480,dtype=np.uint8)[:,None,None]
            image=np.broadcast_to(rows,(480,640,3)).copy()
            raw=np.broadcast_to(np.arange(480,dtype=np.uint16)[:,None],(480,640)).copy()
            ct.memmove(rgb,image.ctypes.data,image.nbytes)
            ct.memmove(depth,raw.ctypes.data,raw.nbytes)
            m=ct.cast(meta,ct.POINTER(camera._Meta)).contents
            m.width=640;m.height=480;m.alignment=1;m.connected=1
            m.arrival_ns=time.monotonic_ns()
            m.rgb_ts=1000;m.depth_ts=1000
            for i,v in enumerate((500,500,320,240)):m.calibration[4+i]=v
            return 1
    monkeypatch.setattr(camera.ct,'CDLL',lambda _:Lib())
    cam=HP60C(library='mock');cam.started=True
    f=cam.read()
    assert f.rgb[:5,0,0].tolist()==[0,1,2,3,4]
    assert f.depth[:5,0].tolist()==[0,1,2,3,4]
    assert f.calibration['depth_scale_qualified'] is False
    cam.close()


@pytest.mark.parametrize(('changes','reason'),[
    ({'rgb_id':8,'depth_id':7},'RGB_DEPTH_FRAME_MISMATCH'),
    ({'rgb_ts':0},'CAMERA_TIMESTAMP_UNAVAILABLE'),
    ({'rgb_ts':1060},'TIMESTAMP_MISMATCH'),
    ({'connected':0},'CAMERA_DISCONNECTED'),
])
def test_unpaired_or_unverifiable_sdk_frame_rejected(monkeypatch,changes,reason):
    import ctypes as ct
    import time
    import autokart_perception.camera as camera

    class Fn:
        def __init__(self,value):self.value=value
        def __call__(self,*args):return self.value(*args) if callable(self.value) else self.value
    class Lib:
        hp_start=Fn(0);hp_connect=Fn(1);hp_stop=Fn(None)
        def __init__(self):self.hp_read=Fn(self.read_impl)
        def read_impl(self,rgb,depth,capacity,meta,timeout):
            m=ct.cast(meta,ct.POINTER(camera._Meta)).contents
            m.width=640;m.height=480;m.alignment=1;m.connected=1
            m.arrival_ns=time.monotonic_ns()
            m.rgb_id=7;m.depth_id=7;m.rgb_ts=1000;m.depth_ts=1000
            for i,v in enumerate((500,500,320,240)):m.calibration[4+i]=v
            for name,value in changes.items():setattr(m,name,value)
            return 1

    monkeypatch.setattr(camera.ct,'CDLL',lambda _:Lib())
    cam=HP60C(library='mock');cam.started=True
    with pytest.raises(CameraError,match=reason):cam.read()
    assert cam.last_arrival_ns==0
    cam.close()
