import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from builtin_interfaces.msg import Time
from autokart_hp60c.backends import SyntheticBackend
from autokart_hp60c.node import intrinsics_to_camera_info


def test_unaligned_pixels_never_get_fabricated_registration_intrinsics():
    backend=SyntheticBackend(period_s=0);backend.start();frame=backend.read();frame.aligned=False
    info=intrinsics_to_camera_info(frame,'depth',Time())
    assert info.k[0]==0.0 and info.p[0]==0.0
    backend.close()
