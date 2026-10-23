from pathlib import Path
import pytest
import yaml
from autokart_perception.pipeline import Pipeline
from autokart_perception.synthetic import Synthetic
from autokart_perception.detector import ConeDetector,DetectorError
from autokart_perception.visualization import Viewer

CFG=yaml.safe_load((Path(__file__).parents[1]/'config/perception.yaml').read_text())

def test_synthetic_integration_and_visualization():
    r=Pipeline(None,CFG,True).process(Synthetic().read())
    assert len(r.cones)==6 and len(r.gates)==3 and len(r.centerline)==3
    assert Viewer().draw(r).shape==(590,1280,3)

def test_missing_model():
    with pytest.raises(DetectorError,match='MODEL_NOT_AVAILABLE'):ConeDetector('/nonexistent/cones.onnx')

def test_no_stale_geometry_on_failure():
    p=Pipeline(None,CFG,True); f=Synthetic().read()
    assert p.process(f).centerline
    p.synthetic=False
    assert p.process(f).centerline==[]

def test_mismatched_replay_pair_cannot_reuse_recorded_observations():
    f=Synthetic().read()
    f.source='REPLAY';f.rgb_sequence=41;f.depth_sequence=42
    f.calibration['recorded_cones']=[dict(id=0,label='blue',confidence=.9,
        xy=(1.0,1.0),sigma_m=.1,distance_m=1.4,box=(1,1,20,20))]
    result=Pipeline(None,CFG).process(f)
    assert result.status=='INVALID_REPLAY:RGB_DEPTH_FRAME_MISMATCH'
    assert result.cones==[]
    assert result.diagnostics['graph_eligible_cones']==0

def test_downloaded_checkpoint_runs_if_present():
    import numpy as np
    model=Path(__file__).parents[1]/'models/coneslayer-simplified.onnx'
    if not model.exists():pytest.skip('checkpoint not downloaded')
    detector=ConeDetector(model)
    detections=detector.detect(np.zeros((480,640,3),np.uint8))
    assert detections==[] and detector.latency_ms>0

def test_configured_bench_pose_reaches_metric_graph_with_measured_geometry():
    import numpy as np
    from autokart_perception.types import Detection
    config=dict(CFG)
    config['camera_mount']={'height_reference':'optical_center','height_m':.18,
        'x_forward_m':0,'y_left_m':0,'pitch_down_deg':0,'roll_deg':0,'yaw_left_deg':0}
    frame=Synthetic().read();frame.depth[:]=0
    observations=[(220,301,1500),(420,301,1500),(245,286,2000),(395,286,2000)]
    detections=[]
    for i,(u,bottom,depth) in enumerate(observations):
        box=(u-20,bottom-81,u+20,bottom)
        frame.depth[bottom-81:bottom,u-20:u+20]=depth
        detections.append(Detection(box,'orange',.9,i))
    class Detector:
        verified_checkpoint=True;version='test';backend='test';latency_ms=0
        def detect(self,image):return detections
    result=Pipeline(Detector(),config).process(frame)
    assert result.diagnostics['calibration_status']=='MOUNT_CONFIGURED_UNVERIFIED'
    assert len(result.cones)==4 and len(result.delaunay.edges)>=4
    assert len(result.gabriel_edges)>=3
    assert all(item['localization_status']=='METRIC_OK' for item in result.diagnostics['cone_details'])
    assert np.allclose([cone.xy for cone in result.cones],
                       [(1.5,.3),(1.5,-.3),(2,.3),(2,-.3)],atol=.05)
    assert not result.centerline  # Orange alone does not establish left/right gates.
