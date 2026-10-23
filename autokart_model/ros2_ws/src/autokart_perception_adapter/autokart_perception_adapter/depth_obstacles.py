"""Conservative metric-depth obstacle extraction. No ROS, device I/O, or guessed calibration.

A return describes visible measured surfaces, not the unseen volume behind them. Coverage
requires returns in every configured corridor cell and remains incomplete under occlusion.
"""
from dataclasses import dataclass
from math import ceil, floor, hypot, isfinite
from numbers import Real
from typing import Sequence


@dataclass(frozen=True)
class DepthObstacleConfig:
    image_width: int
    image_height: int
    fx: float
    fy: float
    cx: float
    cy: float
    depth_scale_m: float
    min_depth_m: float
    max_depth_m: float
    T_base_camera: Sequence[Sequence[float]]
    calibration_hash: str
    roi: tuple[int, int, int, int]  # half-open u0,v0,u1,v1
    ground_z_m: float  # surveyed base_link z-coordinate of ground
    ground_tolerance_m: float
    obstacle_min_height_m: float
    obstacle_max_height_m: float
    corridor_near_m: float
    corridor_far_m: float
    corridor_half_width_m: float
    coverage_cell_m: float
    minimum_samples_per_cell: int
    maximum_invalid_fraction: float


def _finite_number(value):
    return isinstance(value, Real) and not isinstance(value, bool) and isfinite(value)


def _configuration_error(c):
    if not isinstance(c, DepthObstacleConfig):
        return 'CONFIGURATION_INVALID'
    numeric = (c.fx,c.fy,c.cx,c.cy,c.depth_scale_m,c.min_depth_m,c.max_depth_m,
               c.ground_z_m,c.ground_tolerance_m,c.obstacle_min_height_m,
               c.obstacle_max_height_m,c.corridor_near_m,c.corridor_far_m,
               c.corridor_half_width_m,c.coverage_cell_m,c.maximum_invalid_fraction)
    if not all(_finite_number(v) for v in numeric):
        return 'CONFIGURATION_NONFINITE_OR_NONNUMERIC'
    if any(type(v) is not int or v <= 0 for v in (c.image_width,c.image_height,c.minimum_samples_per_cell)):
        return 'IMAGE_OR_SAMPLE_LIMIT_INVALID'
    if min(c.fx,c.fy,c.depth_scale_m,c.min_depth_m,c.corridor_half_width_m,c.coverage_cell_m) <= 0 or \
       c.max_depth_m <= c.min_depth_m or c.corridor_near_m < 0 or c.corridor_far_m <= c.corridor_near_m or \
       c.ground_tolerance_m < 0 or c.obstacle_min_height_m <= c.ground_tolerance_m or \
       c.obstacle_max_height_m < c.obstacle_min_height_m or not 0 <= c.maximum_invalid_fraction <= 1:
        return 'CONFIGURATION_BOUNDS_INVALID'
    if not isinstance(c.calibration_hash,str) or not c.calibration_hash.strip():
        return 'CALIBRATION_IDENTITY_MISSING'
    if not isinstance(c.roi,(tuple,list)) or len(c.roi)!=4 or any(type(v) is not int for v in c.roi):
        return 'ROI_INVALID'
    u0,v0,u1,v1=c.roi
    if not (0 <= u0 < u1 <= c.image_width and 0 <= v0 < v1 <= c.image_height):
        return 'ROI_INVALID'
    if (u1-u0)*(v1-v0)>1_000_000:
        return 'ROI_COMPUTE_BUDGET_EXCEEDED'
    try:
        t=c.T_base_camera
        if len(t)!=4 or any(len(row)!=4 for row in t) or not all(_finite_number(v) for row in t for v in row):
            return 'TRANSFORM_INVALID'
        if any(abs(t[3][j]-(1. if j==3 else 0.))>1e-6 for j in range(4)):
            return 'TRANSFORM_NOT_HOMOGENEOUS'
        if any(abs(sum(t[k][i]*t[k][j] for k in range(3))-(1. if i==j else 0.))>1e-6 for i in range(3) for j in range(3)):
            return 'TRANSFORM_NOT_RIGID'
        determinant=(t[0][0]*(t[1][1]*t[2][2]-t[1][2]*t[2][1])-
                     t[0][1]*(t[1][0]*t[2][2]-t[1][2]*t[2][0])+
                     t[0][2]*(t[1][0]*t[2][1]-t[1][1]*t[2][0]))
        if abs(determinant-1.)>1e-6:
            return 'TRANSFORM_REFLECTION'
    except (TypeError,IndexError,ValueError):
        return 'TRANSFORM_INVALID'
    nx=ceil((c.corridor_far_m-c.corridor_near_m)/c.coverage_cell_m)
    ny=ceil(2*c.corridor_half_width_m/c.coverage_cell_m)
    if nx*ny>100_000:
        return 'COVERAGE_COMPUTE_BUDGET_EXCEEDED'
    return None


def extract_depth_obstacles(depth, config, *, calibration_verified: bool,
                            depth_alignment_verified: bool) -> dict:
    """Project optical-Z depth using explicit calibration and cluster raised surfaces.

    ``depth`` is a rectangular indexable 2D array (lists or NumPy). Scale converts each
    raw value to optical-axis metres. ``T_base_camera`` maps optical XYZ to base_link.
    Calibration and depth/camera alignment must be actual booleans, true. Source timestamp
    pairing/provenance and downstream freshness are the caller's responsibility.
    """
    result=dict(valid=False,frame_id='base_link',calibration_hash=None,obstacles=[],
                coverage_complete=False,coverage_region=None,reason_codes=[],
                invalid_depth_fraction=None,observed_cells=0,total_cells=0,
                measured_point_count=0,unknown_cells=[])
    error=_configuration_error(config)
    if error:
        result['reason_codes']=[error]
        return result
    c=config
    result['calibration_hash']=c.calibration_hash
    if calibration_verified is not True or depth_alignment_verified is not True:
        result['reason_codes']=['CALIBRATION_UNVERIFIED' if calibration_verified is not True else 'DEPTH_ALIGNMENT_UNVERIFIED']
        return result
    try:
        if len(depth)!=c.image_height or any(len(row)!=c.image_width for row in depth):
            raise ValueError('dimensions')
    except (TypeError,ValueError):
        result['reason_codes']=['DEPTH_DIMENSIONS_INVALID']
        return result
    nx=ceil((c.corridor_far_m-c.corridor_near_m)/c.coverage_cell_m)
    ny=ceil(2*c.corridor_half_width_m/c.coverage_cell_m)
    result['coverage_region']=dict(frame_id='base_link',near_m=c.corridor_near_m,
        far_m=c.corridor_far_m,half_width_m=c.corridor_half_width_m,cell_m=c.coverage_cell_m,
        ground_z_m=c.ground_z_m,min_depth_m=c.min_depth_m,max_depth_m=c.max_depth_m,roi=list(c.roi))
    counts={}
    raised={}
    invalid=0
    points=0
    unclassified=0
    u0,v0,u1,v1=c.roi
    t=c.T_base_camera
    for v in range(v0,v1):
        for u in range(u0,u1):
            raw=depth[v][u]
            if not _finite_number(raw) or raw<=0:
                invalid+=1
                continue
            z=float(raw)*c.depth_scale_m
            if not c.min_depth_m<=z<=c.max_depth_m:
                invalid+=1
                continue
            optical=((u-c.cx)*z/c.fx,(v-c.cy)*z/c.fy,z)
            x,y,height=tuple(sum(t[i][j]*optical[j] for j in range(3))+t[i][3] for i in range(3))
            if not all(isfinite(value) for value in (x,y,height)):
                invalid+=1
                continue
            if not (c.corridor_near_m<=x<c.corridor_far_m and -c.corridor_half_width_m<=y<c.corridor_half_width_m):
                continue
            key=(floor((x-c.corridor_near_m)/c.coverage_cell_m),floor((y+c.corridor_half_width_m)/c.coverage_cell_m))
            above_ground=height-c.ground_z_m
            floor_hit=abs(above_ground)<=c.ground_tolerance_m
            obstacle_hit=c.obstacle_min_height_m<=above_ground<=c.obstacle_max_height_m
            if not floor_hit and not obstacle_hit:
                unclassified+=1
                continue
            counts[key]=counts.get(key,0)+1
            points+=1
            if obstacle_hit:
                raised.setdefault(key,[]).append((x,y,height))
    # Adjacent XY occupancy cells cluster measured surfaces; no smoothing fills unseen cells.
    remaining=set(raised)
    while remaining:
        initial=min(remaining)
        remaining.remove(initial)
        queue=[initial]
        cluster=[]
        while queue:
            key=queue.pop()
            cluster.extend(raised[key])
            for dx in (-1,0,1):
                for dy in (-1,0,1):
                    neighbor=(key[0]+dx,key[1]+dy)
                    if neighbor in remaining:
                        remaining.remove(neighbor)
                        queue.append(neighbor)
        xs,ys,zs=zip(*cluster)
        # Cell half-width includes spatial discretization uncertainty; unseen object extent
        # is still unknown, so the caller must retain unknown-corridor vetoes.
        pad=c.coverage_cell_m/2
        xmin,xmax=min(xs)-pad,max(xs)+pad
        ymin,ymax=min(ys)-pad,max(ys)+pad
        result['obstacles'].append(dict(x_m=(xmin+xmax)/2,y_m=(ymin+ymax)/2,
            radius_m=hypot(xmax-xmin,ymax-ymin)/2,
            polygon_base_link=[[xmin,ymin],[xmax,ymin],[xmax,ymax],[xmin,ymax]],
            min_z_m=min(zs),max_z_m=max(zs),point_count=len(cluster),
            observed_surface_only=True))
    unknown=[(i,j) for i in range(nx) for j in range(ny) if counts.get((i,j),0)<c.minimum_samples_per_cell]
    fraction=invalid/((u1-u0)*(v1-v0))
    reasons=[]
    if unknown: reasons.append('UNKNOWN_CORRIDOR_CELLS')
    if fraction>c.maximum_invalid_fraction: reasons.append('INVALID_DEPTH_COVERAGE')
    if unclassified: reasons.append('UNCLASSIFIED_HEIGHT_RETURNS')
    result.update(valid=True,coverage_complete=not unknown and fraction<=c.maximum_invalid_fraction and not unclassified,
                  reason_codes=reasons,invalid_depth_fraction=fraction,observed_cells=nx*ny-len(unknown),
                  total_cells=nx*ny,measured_point_count=points,unknown_cells=unknown)
    return result
