import math
import numpy as np

def _crosses(g1,g2,by_id):
    a=np.asarray(by_id[g1.left_id].xy); b=np.asarray(by_id[g1.right_id].xy)
    c=np.asarray(by_id[g2.left_id].xy); d=np.asarray(by_id[g2.right_id].xy)
    def orient(p,q,r):return np.linalg.det(np.stack((q-p,r-p)))
    return orient(a,b,c)*orient(a,b,d)<0 and orient(c,d,a)*orient(c,d,b)<0

def generate_centerline(gates, min_gates=2, max_lateral_jump_m=2.0,
                        max_heading_change_deg=45, max_width_change_m=1.0, cones=None):
    if any(g.status=='AMBIGUOUS' for g in gates):return [],'CENTERLINE_AMBIGUOUS'
    valid=sorted((g for g in gates if g.status=='CANDIDATE'),key=lambda g:g.midpoint[0])
    if len(valid)<min_gates:return [],'CENTERLINE_AMBIGUOUS'
    by_id={c.id:c for c in cones} if cones else {}
    chosen=[];used=set();prev_heading=None
    for gate in valid:
        if gate.left_id in used or gate.right_id in used:return [],'CENTERLINE_AMBIGUOUS'
        if chosen:
            prev=chosen[-1]
            dx=gate.midpoint[0]-prev.midpoint[0]
            dy=gate.midpoint[1]-prev.midpoint[1]
            if dx<0.2 or abs(dy)>max_lateral_jump_m or abs(gate.width-prev.width)>max_width_change_m:
                return [],'CENTERLINE_AMBIGUOUS'
            heading=math.atan2(dy,dx)
            if prev_heading is not None and abs(math.atan2(math.sin(heading-prev_heading),math.cos(heading-prev_heading)))>math.radians(max_heading_change_deg):
                return [],'CENTERLINE_AMBIGUOUS'
            if by_id and _crosses(prev,gate,by_id):return [],'CENTERLINE_AMBIGUOUS'
            prev_heading=heading
        chosen.append(gate);used.update((gate.left_id,gate.right_id))
    return ([g.midpoint for g in chosen],'PROVISIONAL') if len(chosen)>=min_gates else ([], 'CENTERLINE_AMBIGUOUS')
