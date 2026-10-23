import numpy as np
from autokart_perception.gabriel import filter_gabriel
from autokart_perception.types import Graph

def test_strict_interior_rejected_and_boundary_kept():
    g=Graph((0,1,2),np.array([[0.,0.],[2.,0.],[1.,0.]]),((0,1),),(),'OK')
    assert filter_gabriel(g)==()
    boundary=Graph((0,1,2),np.array([[0.,0.],[2.,0.],[1.,1.]]),((0,1),),(),'OK')
    assert filter_gabriel(boundary)==((0,1),)
