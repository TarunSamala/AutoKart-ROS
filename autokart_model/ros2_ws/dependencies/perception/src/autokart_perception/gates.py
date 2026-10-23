import numpy as np
from functools import lru_cache
from .types import Gate

def candidate_gates(cones, edges, config):
    by_id={c.id:c for c in cones}; candidates=[]
    for a,b in edges:
        if a not in by_id or b not in by_id: continue
        ca,cb=by_id[a],by_id[b]
        pa,pb=np.asarray(ca.xy),np.asarray(cb.xy)
        width=float(np.linalg.norm(pa-pb))
        if not config['min_track_width_m']<=width<=config['max_track_width_m']: continue
        left,right=(ca,cb) if pa[1]>pb[1] else (cb,ca)
        colors=(left.label,right.label)
        if colors==('blue','yellow'): color_score=1.0; color_confirmed=True
        elif colors in (('orange','orange'),('unknown','unknown')): color_score=0.35; color_confirmed=False
        else: continue
        mid=tuple(float(v) for v in (pa+pb)/2)
        if mid[0]<=0: continue
        # A transverse gate has similar forward distance on both boundaries.
        heading_score=max(0.,1.-abs(pa[0]-pb[0])/max(width,1e-6))
        quality=min(ca.confidence,cb.confidence)*color_score*heading_score
        status='CANDIDATE' if color_confirmed and quality>=config['min_gate_quality'] else 'AMBIGUOUS'
        candidates.append(Gate(left.id,right.id,mid,width,float(quality),status))
    return sorted(candidates,key=lambda g:(g.midpoint[0],-g.quality))


def select_track_gates(cones, graph_edges, boundaries, config):
    """Maximum-score order-preserving match across two boundary hypotheses.

    Graph connectivity improves a score, but a crossing need not be Gabriel.
    Monotone matching prevents crossed gates and repeated cone assignments.
    """
    if boundaries.status!='CANDIDATE':return []
    by_id={c.id:c for c in cones}
    left=[by_id[i] for i in boundaries.left_ids if i in by_id]
    right=[by_id[i] for i in boundaries.right_ids if i in by_id]
    if len(left)<2 or len(right)<2:return []
    graph={tuple(sorted(edge)) for edge in graph_edges}
    max_dx=config.get('max_gate_forward_delta_m',.8)
    min_width=config['min_track_width_m'];max_width=config['max_track_width_m']
    target=(min_width+max_width)/2
    scores={}
    for i,a in enumerate(left):
        for j,b in enumerate(right):
            pa=np.asarray(a.xy,dtype=float);pb=np.asarray(b.xy,dtype=float)
            if not np.isfinite(pa).all() or not np.isfinite(pb).all():continue
            width=float(np.linalg.norm(pa-pb));dx=abs(pa[0]-pb[0])
            midpoint=(pa+pb)/2
            if not (min_width<=width<=max_width and midpoint[0]>0 and dx<=max_dx):continue
            width_factor=1-.2*abs(width-target)/max((max_width-min_width)/2,1e-6)
            quality=(min(a.confidence,b.confidence)*np.exp(-max(a.sigma_m,b.sigma_m))*
                     (1. if boundaries.mode=='COLOR' else .75)*
                     (1-.5*dx/max_dx)*width_factor)
            if tuple(sorted((a.id,b.id))) in graph:quality=min(1.,quality+.05)
            if quality>=config['min_gate_quality']:
                scores[i,j]=(float(quality),width,(float(midpoint[0]),float(midpoint[1])))
    @lru_cache(None)
    def best(i,j):
        if i==len(left) or j==len(right):return 0.,()
        options=[best(i+1,j),best(i,j+1)]
        if (i,j) in scores:
            score,path=best(i+1,j+1)
            options.append((score+scores[i,j][0],((i,j),)+path))
        return max(options,key=lambda item:(item[0],len(item[1])))
    chosen=best(0,0)[1]
    gates=[]
    for number,(i,j) in enumerate(chosen):
        quality,width,midpoint=scores[i,j]
        alternate=max((value[0] for (ii,jj),value in scores.items()
                       if (ii==i or jj==j) and (ii,jj)!=(i,j)),default=-1.)
        status='AMBIGUOUS' if alternate>=quality-config.get('gate_ambiguity_margin',.06) else 'CANDIDATE'
        gates.append(Gate(left[i].id,right[j].id,midpoint,width,quality,status,number))
    return gates
