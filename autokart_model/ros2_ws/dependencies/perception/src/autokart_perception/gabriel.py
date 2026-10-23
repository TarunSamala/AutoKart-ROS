import numpy as np

def filter_gabriel(graph, tolerance_m2=1e-9):
    by_id=dict(zip(graph.ids,graph.vertices))
    kept=[]
    for a,b in graph.edges:
        pa,pb=by_id[a],by_id[b]
        if np.linalg.norm(pa-pb)<=1e-6: continue
        if all(np.dot(p-pa,p-pb)>=-tolerance_m2 for i,p in by_id.items() if i not in (a,b)):
            kept.append((a,b))
    return tuple(kept)
