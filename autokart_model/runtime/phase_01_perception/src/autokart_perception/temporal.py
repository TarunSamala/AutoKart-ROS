"""Short-lived display smoothing for a verified stationary bench.

The live pipeline does not provide a camera-motion estimate to this filter.
Its default therefore preserves current-frame detections and cone positions.
Predictions are labeled for display only and never enter the geometry graph.
"""
import math
from dataclasses import dataclass

import numpy as np

from .types import Cone, Detection, Frame, Intrinsics


@dataclass(frozen=True)
class RelativeCameraMotion:
    """Previous optical XYZ to current optical XYZ; never an odom pose."""

    status: str
    previous_timestamp_ns: int
    current_timestamp_ns: int
    matched_points: int = 0
    inlier_count: int = 0
    reprojection_rms_px: float | None = None
    rotation_matrix: tuple[tuple[float, float, float], ...] | None = None
    translation_m: tuple[float, float, float] | None = None
    image_only_translation_std_m: float | None = None
    image_only_rotation_std_rad: float | None = None
    qualified: bool = False


def estimate_motion_from_correspondences(
    previous_xyz, current_uv, intrinsics: Intrinsics,
    previous_timestamp_ns: int, current_timestamp_ns: int,
    *, maximum_gap_ns: int = 200_000_000, minimum_inliers: int = 12,
) -> RelativeCameraMotion:
    """Fit an RGB-D relative motion candidate from optical 3D/2D matches.

    RANSAC and the image Jacobian bound image-fit uncertainty only. Depth,
    intrinsics, alignment, and optical-to-base errors remain unmeasured here,
    so even a successful fit is provisional and cannot authorize fusion.
    """
    def reject(reason, matches=0, inliers=0, rms=None):
        return RelativeCameraMotion(reason, previous_timestamp_ns, current_timestamp_ns,
                                    matches, inliers, rms)

    if (type(previous_timestamp_ns) is not int or type(current_timestamp_ns) is not int or
            previous_timestamp_ns <= 0 or
            not 0 < current_timestamp_ns - previous_timestamp_ns <= maximum_gap_ns):
        return reject('MOTION_TIMING_INVALID')
    if not isinstance(intrinsics, Intrinsics) or not intrinsics.valid():
        return reject('MOTION_INTRINSICS_INVALID')
    try:
        xyz = np.asarray(previous_xyz, dtype=np.float64)
        uv = np.asarray(current_uv, dtype=np.float64)
    except (TypeError, ValueError):
        return reject('MOTION_MATCHES_INVALID')
    if (xyz.ndim != 2 or xyz.shape[1] != 3 or uv.shape != (len(xyz), 2) or
            len(xyz) < minimum_inliers or not np.isfinite(xyz).all() or
            not np.isfinite(uv).all() or np.any(xyz[:, 2] <= 0)):
        return reject('MOTION_MATCHES_INVALID', len(xyz) if xyz.ndim == 2 else 0)
    try:
        import cv2
    except ImportError:
        return reject('MOTION_DEPENDENCY_UNAVAILABLE', len(xyz))
    camera = np.array(((intrinsics.fx, 0, intrinsics.cx),
                       (0, intrinsics.fy, intrinsics.cy), (0, 0, 1)), dtype=np.float64)
    try:
        solved, rvec, tvec, inliers = cv2.solvePnPRansac(
            xyz, uv, camera, None, iterationsCount=200, reprojectionError=2.5,
            confidence=0.99, flags=cv2.SOLVEPNP_EPNP)
    except cv2.error:
        return reject('MOTION_FIT_FAILED', len(xyz))
    if not solved or inliers is None or len(inliers) < minimum_inliers or len(inliers) < .5 * len(xyz):
        return reject('MOTION_INLIERS_INSUFFICIENT', len(xyz),
                      0 if inliers is None else len(inliers))
    indices = inliers.ravel()
    try:
        if not hasattr(cv2, 'solvePnPRefineLM'):
            return reject('MOTION_DEPENDENCY_UNAVAILABLE', len(xyz), len(indices))
        rvec, tvec = cv2.solvePnPRefineLM(xyz[indices], uv[indices], camera, None, rvec, tvec)
        projected, jacobian = cv2.projectPoints(xyz[indices], rvec, tvec, camera, None)
        residual = projected.reshape(-1, 2) - uv[indices]
        rms = float(np.sqrt(np.mean(np.sum(residual ** 2, axis=1))))
        information = jacobian[:, :6].T @ jacobian[:, :6]
        singular = np.linalg.svd(information, compute_uv=False)
        if (not np.isfinite(rms) or rms > 2.5 or not np.isfinite(singular).all() or
                singular[-1] <= 0 or singular[0] / singular[-1] > 1e10):
            return reject('MOTION_FIT_UNSTABLE', len(xyz), len(indices), rms)
        # This covariance excludes depth and calibration error by design.
        covariance = np.linalg.inv(information) * max(rms, .5) ** 2
        rotation_std = float(np.sqrt(np.max(np.diag(covariance)[:3])))
        translation_std = float(np.sqrt(np.max(np.diag(covariance)[3:6])))
        rotation, _ = cv2.Rodrigues(rvec)
        if (not math.isfinite(rotation_std) or not math.isfinite(translation_std) or
                rotation_std > math.radians(5) or translation_std > .25 or
                not np.isfinite(tvec).all() or np.linalg.norm(tvec) > 2.0):
            return reject('MOTION_UNCERTAINTY_HIGH', len(xyz), len(indices), rms)
    except (AttributeError, cv2.error, np.linalg.LinAlgError):
        return reject('MOTION_FIT_FAILED', len(xyz), len(indices))
    return RelativeCameraMotion(
        'PROVISIONAL_CAMERA_MOTION', previous_timestamp_ns, current_timestamp_ns,
        len(xyz), len(indices), rms,
        tuple(tuple(float(value) for value in row) for row in rotation),
        tuple(float(value) for value in tvec.ravel()),
        translation_std, rotation_std)


def estimate_rgbd_motion(previous: Frame, current: Frame) -> RelativeCameraMotion:
    """ORB/depth frontend for a provisional previous-to-current optical fit."""
    t0, t1 = previous.timestamp_ns, current.timestamp_ns
    def reject(reason):
        return RelativeCameraMotion(reason, t0, t1)
    if (type(t0) is not int or type(t1) is not int or t0 <= 0 or
            not 0 < t1 - t0 <= 200_000_000):
        return reject('MOTION_TIMING_INVALID')
    if (not previous.aligned or not current.aligned or
            previous.rgb_sequence != previous.depth_sequence or
            current.rgb_sequence != current.depth_sequence or
            current.rgb_sequence <= previous.rgb_sequence or
            (previous.rgb_timestamp and previous.depth_timestamp and
             abs(previous.rgb_timestamp - previous.depth_timestamp) > 50) or
            (current.rgb_timestamp and current.depth_timestamp and
             abs(current.rgb_timestamp - current.depth_timestamp) > 50)):
        return reject('MOTION_ALIGNMENT_INVALID')
    if (not previous.intrinsics.valid() or not current.intrinsics.valid() or
            previous.intrinsics != current.intrinsics or
            not np.isfinite(previous.depth_scale) or previous.depth_scale <= 0 or
            current.depth_scale != previous.depth_scale):
        return reject('MOTION_CALIBRATION_INVALID')
    if (previous.rgb.ndim != 3 or current.rgb.shape != previous.rgb.shape or
            previous.rgb.shape[2] != 3 or previous.rgb.dtype != np.uint8 or
            current.rgb.dtype != np.uint8 or previous.depth.shape != previous.rgb.shape[:2] or
            current.depth.shape != current.rgb.shape[:2] or
            previous.depth.dtype != np.uint16 or current.depth.dtype != np.uint16):
        return reject('MOTION_FRAME_INVALID')
    try:
        import cv2
    except ImportError:
        return reject('MOTION_DEPENDENCY_UNAVAILABLE')
    orb = cv2.ORB_create(nfeatures=1200)
    old_keys, old_descriptors = orb.detectAndCompute(previous.rgb, None)
    new_keys, new_descriptors = orb.detectAndCompute(current.rgb, None)
    if old_descriptors is None or new_descriptors is None:
        return reject('MOTION_FEATURES_INSUFFICIENT')
    matches = cv2.BFMatcher(cv2.NORM_HAMMING).knnMatch(old_descriptors, new_descriptors, k=2)
    xyz, uv, used_current = [], [], set()
    for pair in matches:
        if len(pair) != 2 or pair[0].distance >= .75 * pair[1].distance:
            continue
        if pair[0].trainIdx in used_current:
            continue
        old_point = old_keys[pair[0].queryIdx].pt
        new_point = new_keys[pair[0].trainIdx].pt
        u, v = (int(round(value)) for value in old_point)
        if not (0 <= v < previous.depth.shape[0] and 0 <= u < previous.depth.shape[1]):
            continue
        depth = float(previous.depth[v, u]) * previous.depth_scale
        if not np.isfinite(depth) or not .2 <= depth <= 12.0:
            continue
        intr = previous.intrinsics
        xyz.append(((old_point[0] - intr.cx) * depth / intr.fx,
                    (old_point[1] - intr.cy) * depth / intr.fy, depth))
        uv.append(new_point)
        used_current.add(pair[0].trainIdx)
    return estimate_motion_from_correspondences(xyz, uv, previous.intrinsics, t0, t1)


def _iou(a,b):
    x1=max(a[0],b[0]);y1=max(a[1],b[1]);x2=min(a[2],b[2]);y2=min(a[3],b[3])
    area=max(0,x2-x1)*max(0,y2-y1)
    aa=max(0,a[2]-a[0])*max(0,a[3]-a[1]);bb=max(0,b[2]-b[0])*max(0,b[3]-b[1])
    return area/(aa+bb-area) if aa+bb>area else 0.0


class StationaryDisplayFilter:
    def __init__(self,hold_ms=150,tau_ms=120):
        self.hold_ns=int(hold_ms*1e6);self.tau_ns=int(tau_ms*1e6)
        self.previous=[];self.last_ns=None;self.next_id=0

    def reset(self):
        self.previous=[];self.last_ns=None

    def update(self,detections,cones,timestamp_ns,*,stationary_verified=False):
        """Return current observations and display predictions.

        Smoothing is opt-in after an external stationarity check. Pixel IoU
        alone cannot establish that camera-relative XY belongs to one frame.
        """
        if (self.last_ns is not None and
                (timestamp_ns<=self.last_ns or timestamp_ns-self.last_ns>self.hold_ns)):
            self.reset()
        self.last_ns=timestamp_ns
        metric={c.id:c for c in cones};available=set(range(len(self.previous)))
        current=[];display_boxes=[];display_cones=[]
        for detection in detections:
            candidates=[(_iou(detection.box,self.previous[j]['box']),j) for j in available
                        if self.previous[j]['label']==detection.label]
            score,index=max(candidates,default=(0.,None))
            old=self.previous[index] if index is not None and score>=.2 else None
            if old is not None:available.remove(index)
            smooth_old=old if old is not None and stationary_verified and old['stationary_verified'] else None
            alpha=(1-math.exp(-(timestamp_ns-smooth_old['timestamp_ns'])/self.tau_ns)) if smooth_old else 1.
            box=np.asarray(detection.box,dtype=float)
            smooth_box=box if smooth_old is None else alpha*box+(1-alpha)*smooth_old['box']
            current_cone=metric.get(detection.id)
            xy=np.asarray(current_cone.xy,dtype=float) if current_cone else None
            if xy is not None and smooth_old is not None and smooth_old['xy'] is not None and \
                    np.linalg.norm(xy-smooth_old['xy'])<.5:
                xy=alpha*xy+(1-alpha)*smooth_old['xy']
            track_id=old['track_id'] if old else self.next_id
            if old is None:self.next_id+=1
            current.append({'track_id':track_id,'label':detection.label,
                            'box':smooth_box,'xy':xy,'timestamp_ns':timestamp_ns,
                            'stationary_verified':stationary_verified})
            display_boxes.append(Detection(tuple(int(round(v)) for v in smooth_box),
                                           detection.label,detection.confidence,detection.id))
            if current_cone is not None:
                display_cones.append(Cone(current_cone.id,current_cone.label,current_cone.confidence,
                                          tuple(float(v) for v in xy),current_cone.sigma_m,
                                          current_cone.distance_m,current_cone.box))
        predicted=[]
        for index in available:
            old=self.previous[index];age=timestamp_ns-old['timestamp_ns']
            if stationary_verified and old['stationary_verified'] and 0<age<=self.hold_ns:
                current.append(old)
                if old['xy'] is not None:
                    predicted.append({'track_id':old['track_id'],'xy':old['xy'].tolist(),
                                      'age_ms':age/1e6,'status':'PREDICTED_DISPLAY_ONLY'})
        self.previous=current
        return display_boxes,display_cones,predicted
