"""Perception maths: YOLO pre/post-processing, depth sampling, back-projection, camera->vehicle transform.

No ROS dependency. Camera optical frame: X right, Y down, Z forward.
Vehicle frame: x forward, y left, z up (origin at the kart reference point).
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Optional, Tuple

import numpy as np


@dataclass
class Intrinsics:
    fx: float
    fy: float
    cx: float
    cy: float


@dataclass
class Mount:
    x: float = 0.0          # camera position in the vehicle frame (m)
    y: float = 0.0
    z: float = 0.5
    pitch_deg: float = 0.0  # positive = camera tilted DOWN
    yaw_deg: float = 0.0    # positive = camera turned LEFT


# ------------------------------------------------------------------ YOLO I/O
def letterbox(img_rgb: np.ndarray, size: int = 640) -> Tuple[np.ndarray, float, int, int]:
    """Resize with preserved aspect ratio and pad to size x size. Returns NCHW float32 in [0,1]."""
    h, w = img_rgb.shape[:2]
    scale = min(size / h, size / w)
    nh, nw = int(round(h * scale)), int(round(w * scale))
    import cv2  # local import keeps the module importable without OpenCV for pure-math tests
    resized = cv2.resize(img_rgb, (nw, nh), interpolation=cv2.INTER_LINEAR)
    canvas = np.full((size, size, 3), 114, dtype=np.uint8)
    px, py = (size - nw) // 2, (size - nh) // 2
    canvas[py:py + nh, px:px + nw] = resized
    x = canvas.astype(np.float32) / 255.0
    return np.ascontiguousarray(x.transpose(2, 0, 1)[None]), scale, px, py


def _iou(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    x1 = np.maximum(a[0], b[:, 0]); y1 = np.maximum(a[1], b[:, 1])
    x2 = np.minimum(a[2], b[:, 2]); y2 = np.minimum(a[3], b[:, 3])
    inter = np.clip(x2 - x1, 0, None) * np.clip(y2 - y1, 0, None)
    ua = (a[2] - a[0]) * (a[3] - a[1]) + (b[:, 2] - b[:, 0]) * (b[:, 3] - b[:, 1]) - inter
    return inter / np.maximum(ua, 1e-9)


def nms(boxes: np.ndarray, scores: np.ndarray, iou_thres: float) -> List[int]:
    order = scores.argsort()[::-1]
    keep: List[int] = []
    while order.size:
        i = int(order[0])
        keep.append(i)
        if order.size == 1:
            break
        rest = order[1:]
        order = rest[_iou(boxes[i], boxes[rest]) < iou_thres]
    return keep


def decode_output(out: np.ndarray, conf_thres: float, iou_thres: float, scale: float, pad_x: int, pad_y: int,
                  img_w: int, img_h: int) -> List[Tuple[int, int, int, int, float, int]]:
    """Accepts either an end-to-end (NMS-free) head [1,N,6] = x1,y1,x2,y2,conf,cls
    or a raw head [1,4+nc,N] = cx,cy,w,h,class scores. Returns boxes in ORIGINAL image pixels."""
    a = np.asarray(out)
    if a.ndim == 3:
        a = a[0]
    if a.shape[-1] == 6:
        det = a  # end-to-end head [N,6]
        keep = det[:, 4] >= conf_thres
        det = det[keep]
        boxes, scores, cls = det[:, :4], det[:, 4], det[:, 5].astype(int)
    else:
        raw = a if a.shape[0] < a.shape[1] else a.T  # -> [4+nc, N]
        cxcywh, cls_scores = raw[:4].T, raw[4:].T
        scores = cls_scores.max(axis=1)
        cls = cls_scores.argmax(axis=1)
        keep = scores >= conf_thres
        cxcywh, scores, cls = cxcywh[keep], scores[keep], cls[keep]
        boxes = np.stack([cxcywh[:, 0] - cxcywh[:, 2] / 2, cxcywh[:, 1] - cxcywh[:, 3] / 2,
                          cxcywh[:, 0] + cxcywh[:, 2] / 2, cxcywh[:, 1] + cxcywh[:, 3] / 2], axis=1)
        if len(boxes):
            idx = nms(boxes, scores, iou_thres)
            boxes, scores, cls = boxes[idx], scores[idx], cls[idx]
    res = []
    for (x1, y1, x2, y2), s, c in zip(boxes, scores, cls):
        x1 = (x1 - pad_x) / scale; x2 = (x2 - pad_x) / scale
        y1 = (y1 - pad_y) / scale; y2 = (y2 - pad_y) / scale
        x1, x2 = max(0, min(img_w - 1, x1)), max(0, min(img_w - 1, x2))
        y1, y2 = max(0, min(img_h - 1, y1)), max(0, min(img_h - 1, y2))
        if x2 > x1 and y2 > y1:
            res.append((int(x1), int(y1), int(x2), int(y2), float(s), int(c)))
    return res


# ------------------------------------------------------------------ depth + geometry
def bbox_depth_m(depth_m: np.ndarray, box: Tuple[int, int, int, int], min_m: float, max_m: float,
                 min_valid_frac: float = 0.2) -> Optional[float]:
    """Median depth of the lower-middle part of the box (the solid part of a cone). None if too few valid pixels."""
    x1, y1, x2, y2 = box
    w, h = x2 - x1, y2 - y1
    rx1, rx2 = int(x1 + 0.35 * w), max(int(x1 + 0.65 * w), int(x1 + 0.35 * w) + 1)
    ry1, ry2 = int(y1 + 0.40 * h), max(int(y1 + 0.90 * h), int(y1 + 0.40 * h) + 1)
    roi = depth_m[ry1:ry2, rx1:rx2].astype(np.float32)
    if roi.size == 0:
        return None
    valid = roi[(roi >= min_m) & (roi <= max_m)]
    if valid.size < max(3, min_valid_frac * roi.size):
        return None
    return float(np.median(valid))


def pixel_to_camera(u: float, v: float, z: float, k: Intrinsics) -> Tuple[float, float, float]:
    return (u - k.cx) * z / k.fx, (v - k.cy) * z / k.fy, z


def camera_to_vehicle(p_cam: Tuple[float, float, float], m: Mount) -> Tuple[float, float, float]:
    X, Y, Z = p_cam
    xb, yb, zb = Z, -X, -Y                      # body: forward, left, up
    th = math.radians(m.pitch_deg)
    xv = xb * math.cos(th) + zb * math.sin(th)
    zv = -xb * math.sin(th) + zb * math.cos(th)
    yv = yb
    ps = math.radians(m.yaw_deg)
    xr = xv * math.cos(ps) - yv * math.sin(ps)
    yr = xv * math.sin(ps) + yv * math.cos(ps)
    return xr + m.x, yr + m.y, zv + m.z


def cone_position(box: Tuple[int, int, int, int], depth_m: np.ndarray, k: Intrinsics, m: Mount,
                  min_m: float = 0.2, max_m: float = 12.0, radial: bool = False):
    """Returns (x,y,z,range_m) in the vehicle frame, or None when depth is unusable."""
    d = bbox_depth_m(depth_m, box, min_m, max_m)
    if d is None:
        return None
    u = 0.5 * (box[0] + box[2])
    v = 0.5 * (box[1] + box[3])
    z = d
    if radial:  # convert radial range to Z depth
        ray = math.sqrt(((u - k.cx) / k.fx) ** 2 + ((v - k.cy) / k.fy) ** 2 + 1.0)
        z = d / ray
    p = pixel_to_camera(u, v, z, k)
    x, y, zz = camera_to_vehicle(p, m)
    return x, y, zz, math.sqrt(p[0] ** 2 + p[1] ** 2 + p[2] ** 2)
