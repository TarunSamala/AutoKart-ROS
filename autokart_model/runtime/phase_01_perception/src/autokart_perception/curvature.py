"""Observed local centerline geometry; no extrapolation or control output."""
import math

import numpy as np

from .types import PathSample


def profile_centerline(points, min_segment_m=.15, max_abs_curvature_m_inv=2.0):
    """Three-point signed circle curvature; undefined at observed endpoints."""
    values=np.asarray(points,dtype=float)
    if values.ndim!=2 or values.shape[1]!=2 or len(values)<3 or not np.isfinite(values).all():
        return [],'CURVATURE_AMBIGUOUS'
    segments=np.diff(values,axis=0)
    lengths=np.linalg.norm(segments,axis=1)
    if np.any(lengths<min_segment_m) or np.any(segments[:,0]<=0):
        return [],'CURVATURE_AMBIGUOUS'
    samples=[]
    for index,point in enumerate(values):
        tangent=(segments[0] if index==0 else segments[-1] if index==len(values)-1
                 else values[index+1]-values[index-1])
        heading=math.atan2(float(tangent[1]),float(tangent[0]))
        curvature=None
        if 0<index<len(values)-1:
            a=segments[index-1];b=segments[index]
            chord=np.linalg.norm(a+b)
            if chord<min_segment_m:return [],'CURVATURE_AMBIGUOUS'
            curvature=float(2*np.linalg.det(np.stack((a,b)))/(lengths[index-1]*lengths[index]*chord))
            if not math.isfinite(curvature) or abs(curvature)>max_abs_curvature_m_inv:
                return [],'CURVATURE_AMBIGUOUS'
        samples.append(PathSample((float(point[0]),float(point[1])),heading,
                                  0. if index==0 else float(lengths[index-1]),curvature))
    return samples,'PROVISIONAL_CURVATURE'
