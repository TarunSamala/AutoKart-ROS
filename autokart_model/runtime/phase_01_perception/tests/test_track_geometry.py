import numpy as np
import yaml
from pathlib import Path

from autokart_perception.boundaries import estimate_boundaries
from autokart_perception.centerline import generate_centerline
from autokart_perception.curvature import profile_centerline
from autokart_perception.gates import select_track_gates
from autokart_perception.geometry import triangulate
from autokart_perception.gabriel import filter_gabriel
from autokart_perception.types import Cone
from autokart_perception import PoseEstimate, process_cones


CFG=yaml.safe_load((Path(__file__).parents[1]/'config/perception.yaml').read_text())


def cones(points,label='orange'):
    return [Cone(i,label,.95,point,.05,float(np.linalg.norm(point)),(0,0,0,0))
            for i,point in enumerate(points)]


def test_geometry_only_boundaries_gates_centerline_and_curvature():
    observed=cones([(1,1.5),(1,-1.5),(2,1.7),(2,-1.3),(3,2.1),(3,-.9)])
    boundaries=estimate_boundaries(observed,CFG)
    assert (boundaries.mode,boundaries.status)==('GEOMETRY','CANDIDATE')
    # No Gabriel crossing edges are supplied: valid ordered gates are still found.
    gates=select_track_gates(observed,(),boundaries,CFG)
    assert [(g.id,g.left_id,g.right_id,g.status) for g in gates]==[
        (0,0,1,'CANDIDATE'),(1,2,3,'CANDIDATE'),(2,4,5,'CANDIDATE')]
    assert all(abs(g.width-3)<1e-6 for g in gates)
    center,status=generate_centerline(gates,cones=observed)
    assert status=='PROVISIONAL' and np.allclose(center,[(1,0),(2,.2),(3,.6)])
    profile,curvature_status=profile_centerline(center)
    assert curvature_status=='PROVISIONAL_CURVATURE'
    assert profile[0].curvature_m_inv is None and profile[-1].curvature_m_inv is None
    assert 0.17<profile[1].curvature_m_inv<0.18


def test_operator_reported_three_pair_bench_layout_is_diagnostic_only():
    # Operator layout: camera centered, first pair 0.84 m forward, 0.30 m
    # between pairs, 0.58 m pair width, one orange cone at the far center.
    # These are nominal coordinates, not a surveyed camera calibration.
    observed=[]
    for x in (.84,1.14,1.44):
        for y,label in ((.29,'blue'),(-.29,'yellow')):
            observed.append(Cone(len(observed),label,.95,(x,y),.03,
                                 float(np.hypot(x,y)),(0,0,0,0)))
    observed.append(Cone(6,'orange',.95,(1.44,0.),.03,1.44,(0,0,0,0)))
    graph=triangulate(observed)
    gabriel=filter_gabriel(graph)
    assert graph.status=='OK' and graph.edges and gabriel
    assert estimate_boundaries(observed,CFG).reason=='BOUNDARY_WIDTH_OUT_OF_RANGE'
    bench={**CFG,'min_track_width_m':.45,'max_track_width_m':.75}
    boundaries=estimate_boundaries(observed,bench)
    assert (boundaries.mode,boundaries.status)==('COLOR','CANDIDATE')
    gates=select_track_gates(observed,graph.edges,boundaries,bench)
    assert len(gates)==3 and all(g.status=='CANDIDATE' for g in gates)
    center,status=generate_centerline(gates,cones=observed)
    assert status=='PROVISIONAL'
    assert np.allclose(center,[(.84,0),(1.14,0),(1.44,0)])


def test_boundary_ambiguity_and_rejected_gates():
    one_sided=cones([(1,1.5),(2,1.5),(3,1.5),(4,1.5)])
    assert estimate_boundaries(one_sided,CFG).reason=='ONE_SIDED_OBSERVATIONS'
    near_center=cones([(1,1.5),(1,-1.5),(2,0.01),(2,-1.5)])
    assert estimate_boundaries(near_center,CFG).reason=='CONE_NEAR_CENTER_AXIS'
    too_narrow=cones([(1,.4),(1,-.4),(2,.4),(2,-.4)])
    assert estimate_boundaries(too_narrow,CFG).status=='AMBIGUOUS'
    observed=cones([(1,1.5),(1,-1.5),(2,1.5),(2,-1.5)])
    boundaries=estimate_boundaries(observed,CFG)
    gates=select_track_gates(observed,((0,2),(1,3)),boundaries,CFG)
    assert {(g.left_id,g.right_id) for g in gates}=={(0,1),(2,3)}
    assert all(g.status=='CANDIDATE' for g in gates)
    assert profile_centerline([(1,0),(1.01,0),(2,0)])[1]=='CURVATURE_AMBIGUOUS'


def test_color_side_order_and_graph_quality_filter():
    observed=cones([(1,1.5),(1,-1.5),(2,1.5),(2,-1.5)])
    colored=[Cone(c.id,'blue' if c.xy[1]>0 else 'yellow',c.confidence,c.xy,c.sigma_m,
                  c.distance_m,c.box) for c in observed]
    assert estimate_boundaries(colored,CFG).mode=='COLOR'
    swapped=[Cone(c.id,'yellow' if c.xy[1]>0 else 'blue',c.confidence,c.xy,c.sigma_m,
                  c.distance_m,c.box) for c in observed]
    assert estimate_boundaries(swapped,CFG).reason=='BOUNDARY_SIDE_ORDER_INCONSISTENT'
    noisy=colored.copy()
    c=noisy[0];noisy[0]=Cone(c.id,c.label,c.confidence,c.xy,.9,c.distance_m,c.box)
    assert estimate_boundaries(noisy,CFG).status=='AMBIGUOUS'


def test_specified_gentle_bend_through_existing_sdk():
    # Exact synthetic local coordinates in metres; these are geometric ground
    # truth for the test, not measurements of the HP60C or its mount.
    left=[(1,1.5),(2,1.7),(3,2.0),(4,2.3)]
    right=[(1,-1.5),(2,-1.3),(3,-1.0),(4,-.7)]
    observed=[]
    for pair in zip(left,right):
        for point,label in zip(pair,('blue','yellow')):
            observed.append(Cone(len(observed),label,.95,point,.05,
                                 float(np.linalg.norm(point)),(0,0,0,0),1_000_000_000))
    result=process_cones(observed,PoseEstimate(0,0,0,0,1_000_000_000,.05),
                         timestamp_ns=1_000_000_000,source='SIMULATED',
                         mission='STRAIGHT_TRACK')
    assert result.geometry.graph_status=='OK'
    assert len(result.geometry.delaunay_edges)>0
    assert set(result.geometry.gabriel_edges)<=set(result.geometry.delaunay_edges)
    assert result.geometry.boundaries.status=='CANDIDATE'
    assert [(g.left_id,g.right_id) for g in result.geometry.gates]==[
        (0,1),(2,3),(4,5),(6,7)]
    assert np.allclose(result.geometry.centerline_m,[(1,0),(2,.2),(3,.5),(4,.8)])
    assert result.geometry.topology_reasons==()
    assert result.planning.valid
    assert not result.vehicle_authorized
    assert not result.control_candidate.published_to_vcu


def test_local_boundary_crossing_cannot_hide_behind_valid_median_width():
    # The first three pairs form a 3 m corridor. The fourth blue side crosses
    # below yellow; the median remains 3 m, so the old check accepted it.
    observed=[]
    for x,blue_y in ((1,1.5),(3,1.5),(5,1.5),(9,-2.0)):
        for y,label in ((blue_y,'blue'),(-1.5,'yellow')):
            observed.append(Cone(len(observed),label,.95,(x,y),.05,
                                 float(np.hypot(x,y)),(0,0,0,0),1_000_000_000))
    result=process_cones(observed,PoseEstimate(0,0,0,0,1_000_000_000,.05),
                         timestamp_ns=1_000_000_000,source='SIMULATED',
                         mission='STRAIGHT_TRACK')
    assert result.geometry.boundaries.status=='AMBIGUOUS'
    assert result.geometry.boundaries.reason=='LOCAL_BOUNDARY_WIDTH_OUT_OF_RANGE'
    assert result.geometry.gates==()
    assert result.geometry.centerline_m==()
    assert not result.planning.valid


def test_local_pinched_width_rejected_even_with_valid_median():
    observed=[]
    for x,blue_y in ((1,1.5),(3,1.5),(5,1.5),(9,.1)):
        for y,label in ((blue_y,'blue'),(-1.5,'yellow')):
            observed.append(Cone(len(observed),label,.95,(x,y),.05,
                                 float(np.hypot(x,y)),(0,0,0,0)))
    boundary=estimate_boundaries(observed,CFG)
    assert boundary.status=='AMBIGUOUS'
    assert boundary.reason=='LOCAL_BOUNDARY_WIDTH_OUT_OF_RANGE'
