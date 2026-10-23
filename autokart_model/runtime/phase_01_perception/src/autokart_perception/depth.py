import cv2
import numpy as np
from .types import Cone

class DepthError(ValueError):
    pass

def validate_frame(frame, max_age_ms=250, now_ns=None):
    import time
    now_ns = time.monotonic_ns() if now_ns is None else now_ns
    if frame.rgb.shape != (480,640,3) or frame.rgb.dtype != np.uint8 or frame.depth.shape != (480,640) or frame.depth.dtype != np.uint16:
        raise DepthError('INVALID_FRAME_FORMAT')
    if not frame.intrinsics.valid() or not np.isfinite(frame.depth_scale) or frame.depth_scale <= 0:
        raise DepthError('CALIBRATION_UNAVAILABLE')
    if frame.source == 'LIVE' and (now_ns-frame.timestamp_ns > max_age_ms*1e6 or frame.timestamp_ns > now_ns+10_000_000):
        raise DepthError('STALE_FRAME')
    if frame.source in ('LIVE','REPLAY') and frame.rgb_sequence != frame.depth_sequence:
        raise DepthError('RGB_DEPTH_FRAME_MISMATCH')
    # HP60C SDK host timestamps are milliseconds (vendor SDK manual).
    if frame.rgb_timestamp and frame.depth_timestamp and abs(frame.rgb_timestamp-frame.depth_timestamp) > 50:
        raise DepthError('TIMESTAMP_MISMATCH')
    if not frame.aligned:
        raise DepthError('DEPTH_ALIGNMENT_UNAVAILABLE')

def sample_depth_diagnostic(depth, box, scale, min_m=0.3, max_m=20.0, min_pixels=12, max_mad_m=0.25, stats=None, clipped=False):
    if stats is None:stats={}
    stats.update({'valid_depth_pixels':0,'coherent_depth_pixels':0,'required_depth_pixels':min_pixels})
    x1,y1,x2,y2=box
    h,w=depth.shape
    xa=max(0,int(x1+0.25*(x2-x1))); xb=min(w,int(x2-0.25*(x2-x1)))
    # A clipped box has no visible foot. Sample its central visible body,
    # away from the bottom image edge where the floor/background can intrude.
    lower,upper=(0.35,0.70) if clipped else (0.65,0.90)
    ya=max(0,int(y1+lower*(y2-y1))); yb=min(h,int(y1+upper*(y2-y1)))
    stats['sample_roi']=[xa,ya,xb,yb]
    if xa>=xb or ya>=yb:return None,'DEPTH_REGION_EMPTY'
    values=depth[ya:yb,xa:xb].astype(np.float32)*scale
    valid=(values>=min_m)&(values<=max_m)&np.isfinite(values)
    stats['valid_depth_pixels']=int(np.count_nonzero(valid))
    stats['region_pixels']=int(values.size)
    stats['valid_depth_fraction']=float(stats['valid_depth_pixels']/values.size)
    if stats['valid_depth_pixels']<min_pixels:return None,'INSUFFICIENT_VALID_DEPTH'
    # Use the near coherent mode in the cone's lower central region, not the
    # background median. Reject weak or discontinuous support.
    seed=float(np.quantile(values[valid],0.2))
    stats['near_depth_seed_m']=seed
    stats['valid_depth_p10_m']=float(np.quantile(values[valid],.1))
    stats['valid_depth_p90_m']=float(np.quantile(values[valid],.9))
    near=valid & (np.abs(values-seed)<=max(0.08,min(max_mad_m,0.15)))
    count,labels,components,_=cv2.connectedComponentsWithStats(near.astype(np.uint8),8)
    if count<2:return None,'DEPTH_DISCONTINUITY'
    component=1+int(np.argmax(components[1:,cv2.CC_STAT_AREA]))
    mask=labels==component
    coverage=np.count_nonzero(mask)/values.size
    stats['coherent_depth_pixels']=int(np.count_nonzero(mask))
    if stats['coherent_depth_pixels']<min_pixels or coverage<0.2:return None,'INSUFFICIENT_COHERENT_DEPTH'
    selected=values[mask]
    med=float(np.median(selected)); mad=float(np.median(np.abs(selected-med)))
    stats['depth_mad_m']=mad
    if mad>max_mad_m:return None,'DEPTH_DISCONTINUITY'
    yy,xx=np.nonzero(mask)
    u=float(xa+np.median(xx)); v=float(ya+np.median(yy))
    return (med,max(0.03,1.4826*mad),coverage,u,v),'DEPTH_OK'

def sample_depth(depth, box, scale, min_m=0.3, max_m=20.0, min_pixels=12, max_mad_m=0.25):
    return sample_depth_diagnostic(depth,box,scale,min_m,max_m,min_pixels,max_mad_m)[0]

def valid_mount(matrix):
    m=np.asarray(matrix,dtype=float)
    if m.shape!=(4,4) or not np.isfinite(m).all():return False
    r=m[:3,:3]
    return (m[2,3]>0 and np.allclose(m[3],[0,0,0,1],atol=1e-5) and
        np.allclose(r.T@r,np.eye(3),atol=1e-3) and np.isclose(np.linalg.det(r),1,atol=1e-3))

def localize(frame, detection, camera_to_kart, config):
    return localize_diagnostic(frame,detection,camera_to_kart,config)[0]

def localize_diagnostic(frame, detection, camera_to_kart, config):
    depth_stats={}
    clipped=detection.box[3] >= frame.rgb.shape[0] - 1
    sample,depth_status=sample_depth_diagnostic(frame.depth,detection.box,frame.depth_scale,
        config['depth_min_m'],config['depth_max_m'],config['min_depth_pixels'],config['max_depth_mad_m'],depth_stats,
        clipped=clipped)
    detail={'id':detection.id,'label':detection.label,'confidence':detection.confidence,
            'box':detection.box,'depth_status':depth_status,'localization_status':depth_status,
            'position_kind':'INVALID_DEPTH' if sample is None else 'UNCLASSIFIED',**depth_stats}
    if sample is None:return None,detail
    z,sigma,coverage,u,v=sample
    detail.update({'sampled_depth_m':z,'depth_support_fraction':coverage,
                   'sample_uv':[u,v],'depth_sigma_m':sigma})
    intr=frame.intrinsics
    optical=np.array([(u-intr.cx)*z/intr.fx,(v-intr.cy)*z/intr.fy,z,1.0])
    detail['optical_xyz_m']=[float(value) for value in optical[:3]]
    # This is a visible surface sample, never a measured cone centre or foot.
    # Include depth scatter and pixel/box support ambiguity in the uncertainty.
    pixel_sigma=max(1.0,0.25*(detection.box[2]-detection.box[0]))
    lateral_sigma=z*pixel_sigma/min(intr.fx,intr.fy)
    detail['surface_uncertainty_m']=float(np.hypot(np.hypot(sigma,lateral_sigma),
                                                   config['cone_radius_uncertainty_m']))
    if clipped:
        detail['localization_status']='PROVISIONAL_3D_POSITION'
        detail['position_kind']='PROVISIONAL_3D_POSITION'
        detail['ground_contact_status']='UNAVAILABLE_CONE_FOOT_CLIPPED'
        if valid_mount(camera_to_kart):
            xyz=np.asarray(camera_to_kart,dtype=float)@optical
            detail['nominal_body_xyz_m']=[float(value) for value in xyz[:3]]
        return None,detail
    if not valid_mount(camera_to_kart):
        detail['localization_status']='MOUNT_CALIBRATION_UNAVAILABLE'
        detail['position_kind']='PROVISIONAL_3D_POSITION'
        return None,detail
    xyz=np.asarray(camera_to_kart,dtype=float)@optical
    detail['surface_xyz_m']=[float(v) for v in xyz[:3]]
    if xyz[0]<=0 or not np.isfinite(xyz).all() or not -0.1<=xyz[2]<=0.45:
        detail['localization_status']='DEPTH_POINT_OUTSIDE_CONE_GEOMETRY'
        detail['position_kind']='INVALID_GEOMETRY'
        return None,detail
    # Intersect the cone-bottom image ray with the measured ground plane. The
    # lower-region depth verifies that the ray is associated with nearby cone
    # surface, and limits parallax from a poor box or background association.
    x1,y1,x2,y2=detection.box
    foot_ray=np.array([((x1+x2)/2-intr.cx)/intr.fx,(y2-1-intr.cy)/intr.fy,1.0])
    transform=np.asarray(camera_to_kart,dtype=float)
    direction=transform[:3,:3]@foot_ray
    detail['ground_ray_vertical_component']=float(direction[2])
    if direction[2]>=-1e-6:
        detail['localization_status']='GROUND_RAY_ABOVE_HORIZON'
        detail['position_kind']='INVALID_GEOMETRY'
        return None,detail
    ground=transform[:3,3]+(-transform[2,3]/direction[2])*direction
    offset=float(np.linalg.norm(ground[:2]-xyz[:2]))
    detail['ground_candidate_xy_m']=[float(v) for v in ground[:2]]
    detail['ground_contact_offset_m']=offset
    detail['max_ground_contact_offset_m']=config['max_ground_contact_offset_m']
    if ground[0]<=0 or offset>config['max_ground_contact_offset_m']:
        detail['localization_status']='GROUND_CONTACT_INCONSISTENT'
        detail['position_kind']='INVALID_GEOMETRY'
        return None,detail
    uncertainty=float(np.hypot(np.hypot(sigma,config['cone_radius_uncertainty_m']),
                                  np.linalg.norm(ground[:2]-xyz[:2])))
    cone=Cone(detection.id,detection.label,detection.confidence,(float(ground[0]),float(ground[1])),
        uncertainty,float(np.hypot(ground[0],ground[1])),detection.box)
    detail.update({'localization_status':'METRIC_OK','ground_xy_m':cone.xy,'distance_m':cone.distance_m,
                   'uncertainty_m':cone.sigma_m,'position_kind':'GROUND_CONTACT_CANDIDATE'})
    return cone,detail
