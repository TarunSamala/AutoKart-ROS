import numpy as np
from scipy.spatial import Delaunay, QhullError
from .types import Graph

def triangulate(cones, tol=1e-6):
    valid=[]
    for c in cones:
        try:
            p=np.asarray(c.xy,dtype=float)
            if p.shape==(2,) and np.isfinite(p).all():valid.append(c)
        except (TypeError,ValueError):
            continue
    valid=sorted(valid,key=lambda c:c.id)
    ids=[]; xy=[]
    for cone in valid:
        if cone.id in ids or any(np.linalg.norm(np.asarray(cone.xy)-p)<tol for p in xy): continue
        ids.append(cone.id); xy.append(np.asarray(cone.xy,dtype=float))
    points=np.asarray(xy,dtype=float).reshape(-1,2)
    empty=lambda status: Graph(tuple(ids),points,(),(),status)
    if len(points)<3: return empty('INSUFFICIENT_POINTS')
    if np.linalg.matrix_rank(points-points.mean(axis=0),tol=tol)<2: return empty('COLLINEAR')
    try:
        tri=Delaunay(points)
    except (QhullError,ValueError):
        return empty('DEGENERATE_GEOMETRY')
    triangles=[]; edges=set()
    for simplex in tri.simplices:
        a,b,c=[int(i) for i in simplex]
        ab=points[b]-points[a]; ac=points[c]-points[a]
        area=abs(ab[0]*ac[1]-ab[1]*ac[0])/2
        if area<=tol: continue
        triangles.append(tuple(sorted((ids[a],ids[b],ids[c]))))
        for i,j in ((a,b),(b,c),(a,c)): edges.add(tuple(sorted((ids[i],ids[j]))))
    return Graph(tuple(ids),points,tuple(sorted(edges)),tuple(sorted(set(triangles))), 'OK' if triangles else 'DEGENERATE_GEOMETRY')
