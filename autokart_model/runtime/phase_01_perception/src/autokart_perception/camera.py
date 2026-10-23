import ctypes as ct
import time
from pathlib import Path
import numpy as np
from .types import Frame, Intrinsics

ROOT = Path(__file__).resolve().parents[3]

class CameraError(RuntimeError):
    pass

class _Meta(ct.Structure):
    _fields_ = [('width',ct.c_int),('height',ct.c_int),('alignment',ct.c_int),('connected',ct.c_int),
        ('rgb_id',ct.c_uint32),('depth_id',ct.c_uint32),('rgb_ts',ct.c_uint64),('depth_ts',ct.c_uint64),
        ('arrival_ns',ct.c_uint64),('drops',ct.c_uint64),('received',ct.c_uint64),
        ('capture_elapsed_ns',ct.c_uint64),('calibration',ct.c_float*32)]

class HP60C:
    def __init__(self, library=None, max_age_ms=250, configuration=None):
        if library is None:
            matches = list((ROOT/'phase_01_perception/native').glob('build-*/libhp60c_bridge.so'))
            if not matches:
                raise CameraError('SDK_ADAPTER_NOT_BUILT: run ./phase_01_perception/build_native.sh')
            library = matches[0]
        self.lib = ct.CDLL(str(library))
        self.configuration = Path(configuration) if configuration is not None else (
            ROOT/'vendor/linux_ros/linux/configurationfiles/hp60c_v2_00_20230704_configEncrypt.json')
        self.lib.hp_start.argtypes = [ct.c_char_p]
        self.lib.hp_start.restype = ct.c_int
        self.lib.hp_connect.restype = ct.c_int
        self.lib.hp_read.argtypes = [ct.POINTER(ct.c_uint8),ct.POINTER(ct.c_uint16),ct.c_uint,ct.POINTER(_Meta),ct.c_int]
        self.lib.hp_read.restype = ct.c_int
        self.lib.hp_stop.restype = None
        self.max_age_ns = int(max_age_ms*1e6)
        self.started = False
        self.rgb_buffer = np.empty((480,640,3),dtype=np.uint8)
        self.depth_buffer = np.empty((480,640),dtype=np.uint16)
        self.last_arrival_ns = 0

    def start(self):
        config = self.configuration
        if not config.exists():
            raise CameraError('SDK_NOT_PREPARED: run ./setup.sh')
        rc = self.lib.hp_start(str(config).encode())
        if rc < 0:
            raise CameraError(f'SDK_START_FAILED:{rc}')
        self.started = True

    def read(self, timeout_ms=500):
        if not self.started:
            self.start()
        state = self.lib.hp_connect()
        if state < 0:
            raise CameraError(f'CAMERA_OPEN_FAILED:{state}')
        if state == 0:
            raise CameraError('CAMERA_DISCONNECTED')
        meta = _Meta()
        rc = self.lib.hp_read(self.rgb_buffer.ctypes.data_as(ct.POINTER(ct.c_uint8)),
            self.depth_buffer.ctypes.data_as(ct.POINTER(ct.c_uint16)), 640*480, ct.byref(meta), timeout_ms)
        if rc < 0:
            raise CameraError('CAMERA_DISCONNECTED' if rc == -2 else f'FRAME_ERROR:{rc}')
        if rc == 0:
            raise CameraError('FRAME_TIMEOUT')
        now_ns = time.monotonic_ns()
        if (meta.arrival_ns <= self.last_arrival_ns or meta.arrival_ns > now_ns + 10_000_000
                or now_ns-meta.arrival_ns > self.max_age_ns):
            raise CameraError('STALE_FRAME')
        h,w=meta.height,meta.width
        if not (h == 480 and w == 640):
            raise CameraError(f'UNSUPPORTED_FRAME_FORMAT:{w}x{h}')
        if meta.connected != 1:
            raise CameraError('CAMERA_DISCONNECTED')
        if meta.rgb_id != meta.depth_id:
            raise CameraError('RGB_DEPTH_FRAME_MISMATCH')
        if not meta.rgb_ts or not meta.depth_ts:
            raise CameraError('CAMERA_TIMESTAMP_UNAVAILABLE')
        # The SDK exports host timestamps in milliseconds. A callback pair with
        # more than 50 ms separation cannot support a common 3D observation.
        if abs(meta.rgb_ts-meta.depth_ts) > 50:
            raise CameraError('TIMESTAMP_MISMATCH')
        p=list(meta.calibration)
        intr=Intrinsics(p[4],p[5],p[6],p[7])
        if not intr.valid() or not (0 <= intr.cx < w and 0 <= intr.cy < h):
            raise CameraError('CALIBRATION_UNAVAILABLE')
        self.last_arrival_ns = meta.arrival_ns
        return Frame(self.rgb_buffer[:h,:w].copy(), self.depth_buffer[:h,:w].copy(), meta.arrival_ns,
            meta.rgb_ts,meta.depth_ts,meta.rgb_id,meta.depth_id,intr,0.001,meta.alignment==1,'LIVE',
            meta.drops, {'sdk_parameters':p,'alignment_status':meta.alignment,
                         'depth_scale_source':'VENDOR_RAW16_MM_CONVENTION',
                         'intrinsics_qualified':False,'depth_scale_qualified':False,
                         'alignment_qualified':False,'extrinsics_qualified':False},
            meta.received,meta.capture_elapsed_ns)

    def close(self):
        if self.started:
            self.lib.hp_stop()
            self.started=False

    def __enter__(self):
        self.start(); return self
    def __exit__(self,*args):
        self.close()
