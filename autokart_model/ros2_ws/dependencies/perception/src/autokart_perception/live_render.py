"""Camera and RAW16-depth previews for a single diagnostic frame snapshot."""

from __future__ import annotations

from math import isfinite

import cv2
import numpy as np

from .types import Frame
from .sdk import SDKResult


def depth_preview(depth: np.ndarray) -> tuple[np.ndarray, dict]:
    """Colorize nonzero RAW16 values; never assign an unverified metric unit."""
    if depth.shape != (480, 640) or depth.dtype != np.uint16:
        raise ValueError("INVALID_RAW16_DEPTH")
    valid = depth > 0
    count = int(np.count_nonzero(valid))
    stats = {"valid_pixels": count, "invalid_pixels": int(depth.size - count),
             "valid_percent": float(100 * count / depth.size),
             "raw_unit": "UNVERIFIED", "median_raw": None,
             "p5_raw": None, "p95_raw": None}
    output = np.zeros((480, 640, 3), dtype=np.uint8)
    if not count:
        return output, stats
    values = depth[valid]
    p5, p95 = (float(v) for v in np.percentile(values, (5, 95)))
    stats.update(median_raw=float(np.median(values)), p5_raw=p5, p95_raw=p95)
    span = max(1.0, p95 - p5)
    normalized = np.clip((depth.astype(np.float32) - p5) * (255.0 / span), 0, 255).astype(np.uint8)
    output = cv2.applyColorMap(normalized, cv2.COLORMAP_TURBO)
    output[~valid] = 0
    return output, stats


def project_base_to_pixel(xyz_m: tuple[float, float, float], camera_to_base: np.ndarray,
                          intrinsics) -> tuple[int, int] | None:
    """Project a surveyed base_link point through a valid camera extrinsic."""
    matrix = np.asarray(camera_to_base, dtype=float)
    if matrix.shape != (4, 4) or not np.isfinite(matrix).all() or not intrinsics.valid():
        return None
    if not all(isfinite(value) for value in xyz_m):
        return None
    try:
        optical = np.linalg.inv(matrix) @ np.array([*xyz_m, 1.0], dtype=float)
    except np.linalg.LinAlgError:
        return None
    if optical[2] <= 0:
        return None
    u = intrinsics.fx * optical[0] / optical[2] + intrinsics.cx
    v = intrinsics.fy * optical[1] / optical[2] + intrinsics.cy
    if not (isfinite(u) and isfinite(v)):
        return None
    return int(round(u)), int(round(v))


def rgb_preview(frame: Frame, result: SDKResult, *, camera_to_base: np.ndarray | None = None,
                extrinsic_verified: bool = False) -> np.ndarray:
    if frame.rgb.shape != (480, 640, 3) or frame.rgb.dtype != np.uint8:
        raise ValueError("INVALID_BGR8_RGB")
    image = frame.rgb.copy()
    detections = result.perception_result.detections if result.perception_result else ()
    colors = {"blue": (255, 140, 60), "yellow": (0, 220, 255),
              "orange": (0, 150, 255), "large_orange": (0, 95, 255),
              "unknown": (170, 170, 170)}
    label_regions = []
    for detection in detections:
        color = colors.get(detection.label, colors["unknown"])
        cv2.rectangle(image, detection.box[:2], detection.box[2:], color, 2)
        label = f"#{detection.id} {detection.label} {detection.confidence:.0%}"
        (text_w, text_h), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, .43, 1)
        width, height = text_w + 8, text_h + 8
        x1, y1, x2, _ = detection.box
        choices = [(max(0, min(x1, 640 - width)), max(0, y1 - height)),
                   (max(0, min(x1, 640 - width)), min(480 - height, y1 + 3)),
                   (max(0, min(x2 - width, 640 - width)), max(0, y1 - height))]
        def overlap(candidate):
            x, y = candidate
            return sum(max(0, min(x + width, rx + rw) - max(x, rx)) *
                       max(0, min(y + height, ry + rh) - max(y, ry))
                       for rx, ry, rw, rh in label_regions)
        x, y = min(choices, key=overlap)
        label_regions.append((x, y, width, height))
        cv2.rectangle(image, (x, y), (x + width, y + height), (6, 16, 26), -1)
        cv2.rectangle(image, (x, y), (x + width, y + height), color, 1)
        cv2.putText(image, label, (x + 4, y + text_h + 3),
                    cv2.FONT_HERSHEY_SIMPLEX, .43, (255, 255, 255), 1, cv2.LINE_AA)
    if extrinsic_verified and camera_to_base is not None and result.planning.valid:
        points = [project_base_to_pixel((x, y, 0.0), camera_to_base, frame.intrinsics)
                  for x, y in result.geometry.centerline_m]
        points = [point for point in points if point is not None]
        if len(points) > 1:
            cv2.polylines(image, [np.asarray(points, dtype=np.int32)], False,
                          (255, 255, 255), 2, cv2.LINE_AA)
    else:
        cv2.putText(image, "PATH PROJECTION UNAVAILABLE: calibration/pose unverified",
                    (8, 470), cv2.FONT_HERSHEY_SIMPLEX, .45, (40, 40, 255), 1, cv2.LINE_AA)
    return image


def encode_jpeg(image: np.ndarray) -> bytes:
    ok, data = cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, 82])
    if not ok:
        raise RuntimeError("JPEG_ENCODE_FAILED")
    return data.tobytes()
