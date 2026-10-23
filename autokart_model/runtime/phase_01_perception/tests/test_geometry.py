from autokart_perception.geometry import triangulate
from autokart_perception.types import Cone

def cone(i,x,y):return Cone(i,'unknown',1,(x,y),.1,1,(0,0,1,1))

def test_triangle_and_unique_edges():
    g=triangulate([cone(2,0,1),cone(0,0,0),cone(1,1,0)])
    assert g.status=='OK' and g.edges==((0,1),(0,2),(1,2)) and g.triangles==((0,1,2),)

def test_degenerate_cases():
    assert triangulate([]).status=='INSUFFICIENT_POINTS'
    assert triangulate([cone(0,0,0),cone(1,1,1),cone(2,2,2)]).status=='COLLINEAR'
    assert triangulate([cone(0,0,0),cone(1,0,0),cone(2,1,0)]).status=='INSUFFICIENT_POINTS'
    assert triangulate([cone(0,0,0),cone(1,float('nan'),1),cone(2,1,0)]).status=='INSUFFICIENT_POINTS'
