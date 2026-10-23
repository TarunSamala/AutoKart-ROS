import json
from pathlib import Path
import yaml
from autokart_perception.calibration import validate_survey
from autokart_perception.pipeline import Pipeline
from autokart_perception.recording import Recorder
from autokart_perception.synthetic import Synthetic


def test_survey_validation_against_analytic_level_camera(tmp_path):
    config=yaml.safe_load((Path(__file__).parents[1]/'config/perception.yaml').read_text())
    config['camera_mount']={'height_reference':'optical_center','height_m':.2,
        'x_forward_m':0,'y_left_m':0,'pitch_down_deg':0,'roll_deg':0,'yaw_left_deg':0}
    frame=Synthetic().read();frame.depth[:]=2000
    recording=tmp_path/'session';Recorder(recording).write(Pipeline(None,config,True).process(frame))
    labels={'schema_version':1,'sessions':{'session':{'000001':[
        {'box':[300,200,340,300],'label':'orange','distance_m':1.695,'xy_m':[.2*500/59,0]}]}}}
    annotations=tmp_path/'labels.json';annotations.write_text(json.dumps(labels))
    passed=validate_survey([recording],annotations,config,.02)
    assert passed['status']=='NON_LIVE_PASS' and passed['localized_cones']==1
    assert passed['recorded_sources']=={'SYNTHETIC':1}
    assert passed['blocker']=='SURVEY_REQUIRES_LIVE_HP60C_RECORDINGS'
    assert passed['invalid_depth_rate']==0
    assert passed['synchronization_mismatches']==0
    assert validate_survey([recording],annotations,config,.02,require_grid=True)['status']=='FAIL'
    labels['sessions']['session']['000001'][0]['xy_m']=[2,0]
    annotations.write_text(json.dumps(labels))
    failed=validate_survey([recording],annotations,config,.02)
    assert failed['status']=='FAIL' and failed['error_m']['max']>.02
