"""Perception validity contract. This package has no vehicle command interface."""
from dataclasses import dataclass
import math

import numpy as np


STATES = frozenset({'VALID', 'DEGRADED', 'AMBIGUOUS', 'STALE', 'SENSOR_FAULT'})


@dataclass(frozen=True)
class SafetyAssessment:
    state: str
    reasons: tuple[str, ...]
    source: str
    frame_timestamp_ns: int
    produced_timestamp_ns: int
    age_ms: float | None
    confidence: float
    speed_limit_advisory_mps: float
    vehicle_authorized: bool = False


def _intersects(a, b, c, d):
    def orientation(p, q, r):
        return float(np.linalg.det(np.stack((q-p, r-p))))
    o1, o2 = orientation(a, b, c), orientation(a, b, d)
    o3, o4 = orientation(c, d, a), orientation(c, d, b)
    return o1*o2 < -1e-12 and o3*o4 < -1e-12


def topology_reasons(gates, centerline, cones, min_width_m, max_width_m):
    """Independent final check on graph-derived output; no inferred missing cones."""
    reasons=[]
    by_id={c.id:np.asarray(c.xy,dtype=float) for c in cones}
    seen=set(); spans=[]
    for gate in gates:
        pair=(gate.left_id,gate.right_id)
        if pair[0]==pair[1] or any(i in seen for i in pair):reasons.append('GATE_CONE_REUSE')
        seen.update(pair)
        if any(i not in by_id for i in pair):
            reasons.append('GATE_ENDPOINT_MISSING');continue
        left,right=(by_id[i] for i in pair)
        if not np.isfinite(left).all() or not np.isfinite(right).all() or left[1]<=right[1]:
            reasons.append('GATE_SIDE_INVALID');continue
        width=float(np.linalg.norm(left-right))
        if not min_width_m<=width<=max_width_m or not math.isfinite(gate.quality):
            reasons.append('GATE_WIDTH_INVALID')
        spans.append((left,right))
    for i,(a,b) in enumerate(spans):
        for c,d in spans[i+1:]:
            if _intersects(a,b,c,d):reasons.append('GATE_CROSSING')
    if centerline:
        points=np.asarray(centerline,dtype=float)
        if points.ndim!=2 or points.shape[1]!=2 or not np.isfinite(points).all():
            reasons.append('PATH_COORDINATES_INVALID')
        else:
            segments=np.diff(points,axis=0)
            if len(segments) and np.any(segments[:,0]<=0):reasons.append('PATH_NOT_FORWARD')
            for i in range(len(segments)):
                for j in range(i+2,len(segments)):
                    if _intersects(points[i],points[i+1],points[j],points[j+1]):
                        reasons.append('PATH_SELF_INTERSECTION')
    return tuple(dict.fromkeys(reasons))


def assess(source, frame_timestamp_ns, produced_ns, age_ms, status, diagnostics,
           boundaries, gates, centerline, path_profile, topology_errors, config):
    """Classify current observations. Even VALID is perception-only, never motion approval."""
    reasons=list(topology_errors)
    if source not in ('LIVE','REPLAY','SYNTHETIC'):reasons.append('INVALID_SOURCE')
    if source!='LIVE':reasons.append('NON_LIVE_SOURCE')
    if status.startswith('INVALID_REPLAY'):reasons.append('REPLAY_INTEGRITY_FAILURE')
    if status.startswith('INFERENCE_FAILURE'):reasons.append('INFERENCE_FAILURE')
    if status in ('INVALID_FRAME_FORMAT','CALIBRATION_UNAVAILABLE','DEPTH_ALIGNMENT_UNAVAILABLE',
                  'MOUNT_CALIBRATION_UNAVAILABLE','MOUNT_CALIBRATION_INVALID','MODEL_NOT_AVAILABLE',
                  'METRIC_LOCALIZATION_FAILED','TIMESTAMP_MISMATCH'):
        reasons.append(status)
    if diagnostics.get('calibration_mismatch'):reasons.append('CALIBRATION_MISMATCH')
    if diagnostics.get('model')!='READY':reasons.append('MODEL_UNVERIFIED')
    if diagnostics.get('valid_depth_pct',100)<config.get('min_valid_depth_pct',1.):
        reasons.append('DEPTH_UNAVAILABLE')
    if diagnostics.get('replay_provenance_missing'):reasons.append('CALIBRATION_PROVENANCE_MISSING')
    if diagnostics.get('frame_frozen'):reasons.append('FRAME_FROZEN')
    if diagnostics.get('frame_out_of_order'):reasons.append('FRAME_OUT_OF_ORDER')
    if diagnostics.get('inference_ms',0)>config.get('max_inference_ms',150):
        reasons.append('INFERENCE_OVERLOAD')
    if diagnostics.get('processing_ms',0)>config.get('max_processing_ms',150):
        reasons.append('PROCESSING_OVERLOAD')
    if source=='LIVE' and (age_ms is None or age_ms<0 or age_ms>config.get('max_frame_age_ms',150)):
        reasons.append('FRAME_TOO_OLD')
    if any(g.status!='CANDIDATE' for g in gates):reasons.append('GATE_AMBIGUOUS')
    if boundaries is None or boundaries.status!='CANDIDATE':reasons.append('BOUNDARIES_AMBIGUOUS')
    elif boundaries.mode=='GEOMETRY':reasons.append('GEOMETRY_ONLY_BOUNDARIES')
    if not centerline:reasons.append('CENTERLINE_AMBIGUOUS')
    elif len(centerline)<3:reasons.append('INSUFFICIENT_PATH_SUPPORT')
    if centerline and len(centerline)>=3 and not path_profile:reasons.append('CURVATURE_INVALID')
    if diagnostics.get('calibration_status')!='MOUNT_VERIFIED':reasons.append('CALIBRATION_UNVERIFIED')
    reasons=tuple(dict.fromkeys(reasons))
    fault={'INVALID_SOURCE','REPLAY_INTEGRITY_FAILURE','INFERENCE_FAILURE','INVALID_FRAME_FORMAT',
           'CALIBRATION_UNAVAILABLE','DEPTH_ALIGNMENT_UNAVAILABLE','MOUNT_CALIBRATION_UNAVAILABLE',
           'MOUNT_CALIBRATION_INVALID','MODEL_NOT_AVAILABLE','METRIC_LOCALIZATION_FAILED',
           'CALIBRATION_MISMATCH','FRAME_FROZEN','FRAME_OUT_OF_ORDER','TIMESTAMP_MISMATCH'}
    fault.add('DEPTH_UNAVAILABLE')
    stale={'INFERENCE_OVERLOAD','PROCESSING_OVERLOAD','FRAME_TOO_OLD'}
    ambiguous={'GATE_CONE_REUSE','GATE_ENDPOINT_MISSING','GATE_SIDE_INVALID','GATE_WIDTH_INVALID',
               'GATE_CROSSING','PATH_COORDINATES_INVALID','PATH_NOT_FORWARD',
               'PATH_SELF_INTERSECTION','GATE_AMBIGUOUS','BOUNDARIES_AMBIGUOUS',
               'CENTERLINE_AMBIGUOUS','INSUFFICIENT_PATH_SUPPORT','CURVATURE_INVALID'}
    if any(r in fault for r in reasons):state='SENSOR_FAULT'
    elif any(r in stale for r in reasons):state='STALE'
    elif any(r in ambiguous for r in reasons):state='AMBIGUOUS'
    elif reasons:state='DEGRADED'
    else:state='VALID'
    confidence=min([boundaries.confidence]+[g.quality for g in gates],default=0.) if centerline else 0.
    # Braking and stopping distance are not qualified in Phase 1. Zero is a hold
    # advisory, not an inferred safe operating speed or an actuator command.
    return SafetyAssessment(state,reasons,source,int(frame_timestamp_ns),int(produced_ns),
                            float(age_ms) if age_ms is not None else None,float(confidence),0.0,False)
