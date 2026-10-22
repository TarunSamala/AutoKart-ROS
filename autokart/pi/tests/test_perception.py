import math
import numpy as np
from autokart_perception.core import (Intrinsics, Mount, bbox_depth_m, camera_to_vehicle, cone_position, decode_output,
                                      letterbox, nms, pixel_to_camera)


def test_camera_to_vehicle_axes():
    # straight ahead 2 m, no pitch, mounted 0.5 m up
    x, y, z = camera_to_vehicle((0.0, 0.0, 2.0), Mount(0.1, 0.0, 0.5, 0.0))
    assert abs(x - 2.1) < 1e-9 and abs(y) < 1e-9 and abs(z - 0.5) < 1e-9
    # a point to the camera's RIGHT (+X) must be vehicle RIGHT (-y)
    assert camera_to_vehicle((1.0, 0.0, 2.0), Mount())[1] < 0
    # a point BELOW the optical axis (+Y) must be lower in the vehicle frame
    assert camera_to_vehicle((0.0, 1.0, 2.0), Mount())[2] < 0.5 - 0.9


def test_pitch_down_ground_point():
    # camera 0.5 m high pitched 20 deg down; a pixel on the optical axis hits the ground at 0.5/tan(20)
    m = Mount(0, 0, 0.5, 20.0)
    d = 0.5 / math.sin(math.radians(20))
    x, y, z = camera_to_vehicle((0.0, 0.0, d), m)
    assert abs(z) < 1e-9 and abs(x - 0.5 / math.tan(math.radians(20))) < 1e-9


def test_depth_and_position():
    depth = np.full((480, 640), 3.0, np.float32)
    k = Intrinsics(600, 600, 320, 240)
    box = (300, 200, 340, 280)
    assert abs(bbox_depth_m(depth, box, 0.2, 10) - 3.0) < 1e-6
    x, y, z, r = cone_position(box, depth, k, Mount(0, 0, 0.0, 0.0))
    assert abs(x - 3.0) < 1e-6 and abs(y) < 1e-6 and r >= 3.0
    depth[:] = 0.0  # no valid depth
    assert cone_position(box, depth, k, Mount()) is None
    # object to the right of centre -> negative y
    r = cone_position((420, 200, 460, 280), np.full((480, 640), 3.0, np.float32), k, Mount())
    assert r[1] < 0


def test_decode_end_to_end_head():
    out = np.zeros((1, 300, 6), np.float32)
    out[0, 0] = [100, 100, 200, 300, 0.9, 1]
    out[0, 1] = [10, 10, 20, 20, 0.1, 0]       # below threshold
    img = np.zeros((480, 640, 3), np.uint8)
    x, scale, px, py = letterbox(img, 640)
    assert x.shape == (1, 3, 640, 640) and scale == 1.0 and py == 80
    res = decode_output(out, 0.4, 0.5, scale, px, py, 640, 480)
    assert len(res) == 1 and res[0][5] == 1 and res[0][1] == 20 and abs(res[0][4] - 0.9) < 1e-6


def test_decode_raw_head_with_nms():
    n = 50
    raw = np.zeros((1, 7, n), np.float32)       # 4 + 3 classes
    raw[0, :4, 0] = [320, 320, 100, 100]; raw[0, 4, 0] = 0.9
    raw[0, :4, 1] = [322, 322, 100, 100]; raw[0, 4, 1] = 0.8   # duplicate, must be suppressed
    raw[0, :4, 2] = [500, 500, 50, 50];   raw[0, 5, 2] = 0.7
    res = decode_output(raw, 0.4, 0.5, 1.0, 0, 0, 640, 640)
    assert len(res) == 2 and sorted(r[5] for r in res) == [0, 1]
    keep = nms(np.array([[0, 0, 10, 10], [1, 1, 11, 11], [50, 50, 60, 60]], float), np.array([0.9, 0.8, 0.7]), 0.5)
    assert keep == [0, 2]
