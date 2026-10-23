"""Local track-side hypotheses for a stationary camera, never a persistent map."""
import math

import numpy as np

from .types import Boundaries


def estimate_boundaries(cones, config):
    """Use color when observed, otherwise cautiously split around kart-forward X.

    The geometry mode requires two observations on each side, plausible width,
    and smooth forward progression. Orange carries no left/right semantics.
    """
    empty=lambda mode,reason: Boundaries((),(),mode,'AMBIGUOUS',0.0,reason)
    usable=[]
    for cone in cones:
        try:point=np.asarray(cone.xy,dtype=float)
        except (ValueError,TypeError):continue
        if (point.shape==(2,) and np.isfinite(point).all() and point[0]>0 and
                np.isfinite(cone.sigma_m) and 0<=cone.sigma_m<=config.get('max_cone_sigma_m',.6) and
                np.isfinite(cone.confidence) and cone.confidence>0):
            usable.append(cone)
    if len(usable)<4:return empty('NONE','INSUFFICIENT_METRIC_CONES')
    left_color=config.get('left_boundary_color','blue')
    right_color=config.get('right_boundary_color','yellow')
    if left_color==right_color or {left_color,right_color}!={'blue','yellow'}:
        return empty('COLOR','INVALID_BOUNDARY_COLOR_CONFIG')
    color_left=[c for c in usable if c.label==left_color]
    color_right=[c for c in usable if c.label==right_color]
    if color_left and color_right:
        mode='COLOR'
        left,right=color_left,color_right
        if len(left)<2 or len(right)<2:return empty(mode,'INSUFFICIENT_COLOR_BOUNDARIES')
    else:
        mode='GEOMETRY'
        # A side cannot be inferred from a point on the kart-forward axis.
        clearance=config.get('min_boundary_lateral_m',.25)
        if any(abs(c.xy[1])<clearance for c in usable):
            return empty(mode,'CONE_NEAR_CENTER_AXIS')
        left=[c for c in usable if c.xy[1]>0]
        right=[c for c in usable if c.xy[1]<0]
        if len(left)<2 or len(right)<2:return empty(mode,'ONE_SIDED_OBSERVATIONS')
    left.sort(key=lambda c:c.xy[0]);right.sort(key=lambda c:c.xy[0])
    if np.median([c.xy[1] for c in left])<=np.median([c.xy[1] for c in right]):
        return empty(mode,'BOUNDARY_SIDE_ORDER_INCONSISTENT')
    max_slope=math.tan(math.radians(config.get('max_boundary_heading_deg',45)))
    min_spacing=config.get('min_boundary_spacing_m',.15)
    for side in (left,right):
        for first,second in zip(side,side[1:]):
            dx=second.xy[0]-first.xy[0]
            if dx<min_spacing or abs(second.xy[1]-first.xy[1])>max_slope*dx:
                return empty(mode,'BOUNDARY_HEADING_DISCONTINUITY')
    width=float(np.median([c.xy[1] for c in left])-np.median([c.xy[1] for c in right]))
    if not config['min_track_width_m']<=width<=config['max_track_width_m']:
        return empty(mode,'BOUNDARY_WIDTH_OUT_OF_RANGE')
    # The median can hide a locally pinched or crossed corridor.  Both sides
    # are already monotone in forward X, so their piecewise-linear lateral
    # separation is extremal at a cone's X coordinate in the common interval.
    # Check every such knot; no extrapolation is allowed beyond observations.
    left_xy=np.asarray([c.xy for c in left],dtype=float)
    right_xy=np.asarray([c.xy for c in right],dtype=float)
    start=max(left_xy[0,0],right_xy[0,0])
    end=min(left_xy[-1,0],right_xy[-1,0])
    if end<=start:
        return empty(mode,'BOUNDARY_NO_FORWARD_OVERLAP')
    knots=np.unique(np.concatenate((left_xy[:,0],right_xy[:,0],[start,end])))
    knots=knots[(knots>=start)&(knots<=end)]
    local_width=np.interp(knots,left_xy[:,0],left_xy[:,1])-np.interp(
        knots,right_xy[:,0],right_xy[:,1])
    if np.any((local_width<config['min_track_width_m']) |
              (local_width>config['max_track_width_m'])):
        return empty(mode,'LOCAL_BOUNDARY_WIDTH_OUT_OF_RANGE')
    confidence=float(min(c.confidence for c in left+right)*
                     math.exp(-max(c.sigma_m for c in left+right))*(1.0 if mode=='COLOR' else .8))
    return Boundaries(tuple(c.id for c in left),tuple(c.id for c in right),mode,
                      'CANDIDATE',confidence,'COLOR_SIDES' if mode=='COLOR' else 'FORWARD_AXIS_SIDE_HYPOTHESIS')
