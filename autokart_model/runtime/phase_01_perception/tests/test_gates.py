from autokart_perception.gates import candidate_gates
from autokart_perception.synthetic import synthetic_cones
from autokart_perception.centerline import generate_centerline

def test_gate_midpoint_and_ambiguity():
    cones=synthetic_cones()
    config={'min_track_width_m':2,'max_track_width_m':3.8,'min_gate_quality':.3}
    gates=candidate_gates(cones,((0,1),(2,3)),config)
    assert [(g.midpoint,g.width,g.status) for g in gates]==[((2.,0.),3.,'CANDIDATE'),((5.,0.),3.,'CANDIDATE')]
    assert generate_centerline(gates)[1]=='PROVISIONAL'
    assert generate_centerline(gates[:1])[1]=='CENTERLINE_AMBIGUOUS'
    assert candidate_gates(cones,((0,2),),config)==[]

def test_ambiguous_color_and_zigzag():
    from autokart_perception.types import Gate
    cones=synthetic_cones()
    orange=[type(c)(c.id,'orange',c.confidence,c.xy,c.sigma_m,c.distance_m,c.box) for c in cones]
    config={'min_track_width_m':2,'max_track_width_m':3.8,'min_gate_quality':.3}
    assert candidate_gates(orange,((0,1),),config)[0].status=='AMBIGUOUS'
    gates=[Gate(2*i,2*i+1,(2+i,1.9*(i%2)),3,.9,'CANDIDATE') for i in range(4)]
    assert generate_centerline(gates)[1]=='CENTERLINE_AMBIGUOUS'
