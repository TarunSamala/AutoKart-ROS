import json
from pathlib import Path
import numpy as np
import pytest
from autokart_perception.evaluation import (box_iou,evaluate_predictions,load_annotations,
    make_template,recording_files,verify_heldout_sessions)
from autokart_perception import evaluation
from autokart_perception.types import Detection


def test_matching_and_average_precision():
    key=('session','000001')
    truth={key:[{'box':(10,10,30,30),'distance_m':.8},
                {'box':(100,100,140,140),'distance_m':2.4}]}
    pred={key:[Detection((10,10,30,30),'orange',.9,0),
               Detection((200,200,220,220),'orange',.7,1)]}
    result=evaluate_predictions(pred,truth)
    assert box_iou((10,10,30,30),(10,10,30,30))==1
    assert (result['true_positives'],result['false_positives'],result['false_negatives'])==(1,1,1)
    assert result['precision']==.5 and result['recall']==.5
    assert 0<result['map50']<1 and result['map50_95']<=result['map50']
    assert result['per_distance_recall']['0.6-1m']['recall']==1
    assert result['per_distance_recall']['2-3m']['recall']==0


def test_template_unreviewed_frames_excluded(tmp_path):
    recording=tmp_path/'session';recording.mkdir()
    np.savez_compressed(recording/'000001.npz',rgb=np.zeros((480,640,3),np.uint8),depth=np.zeros((480,640),np.uint16))
    sessions=recording_files([recording])
    path=tmp_path/'labels.json'
    assert make_template(sessions,path)==1
    assert (tmp_path/'labels'/'index.html').exists()
    with pytest.raises(ValueError,match='NO_ANNOTATED_FRAMES'):load_annotations(path,sessions)
    doc=json.loads(path.read_text());doc['sessions']['session']['000001']=[]
    path.write_text(json.dumps(doc))
    assert len(load_annotations(path,sessions))==1
    with pytest.raises(FileExistsError):make_template(sessions,path)


def test_class_mismatch_counts_false_positive_and_missed_cone():
    key=('session','000001')
    truth={key:[{'box':(10,10,30,30),'label':'yellow','distance_m':1.0}]}
    wrong={key:[Detection((10,10,30,30),'blue',.9,0)]}
    result=evaluate_predictions(wrong,truth)
    assert (result['true_positives'],result['false_positives'],result['false_negatives'])==(0,1,1)
    correct={key:[Detection((10,10,30,30),'yellow',.9,0)]}
    assert evaluate_predictions(correct,truth)['recall']==1


def test_heldout_session_gate_and_difficulty_coverage():
    audit={'status':'EXPERIMENTAL_READY','sessions_by_split':{
        'train':['recording-a'],'val':['recording-b'],'test':['recording-c']}}
    assert verify_heldout_sessions(audit,['recording-c'])
    with pytest.raises(ValueError,match='HP60C_SESSION_LEAKAGE'):
        verify_heldout_sessions(audit,['recording-a'])
    with pytest.raises(ValueError,match='TRAINING_SESSION_PROVENANCE_MISSING'):
        verify_heldout_sessions({'status':'EXPERIMENTAL_READY'},['recording-c'])
    truth={('recording-c','1'):[{'box':(1,1,10,10),'label':'orange',
        'distance_m':3.2,'difficulty':['small','distant','occluded']}],
        ('recording-c','2'):[]}
    result=evaluate_predictions({key:[] for key in truth},truth)
    assert result['coverage']['negative_frames']==1
    assert result['coverage']['difficulty_cones']['occluded']==1


def test_ab_cli_uses_same_frames_and_thresholds(tmp_path,monkeypatch):
    session=tmp_path/'heldout';session.mkdir()
    np.savez_compressed(session/'000001.npz',rgb=np.zeros((480,640,3),np.uint8))
    labels=tmp_path/'labels.json'
    labels.write_text(json.dumps({'schema_version':1,'sessions':{'heldout':{
        '000001':[{'box':[10,10,30,30],'label':'orange','distance_m':1.5,
                   'difficulty':['small']}]
    }}}))
    audit=tmp_path/'audit.json';audit.write_text(json.dumps({'status':'EXPERIMENTAL_READY',
        'sessions_by_split':{'train':['train-session'],'val':['val-session'],'test':['heldout']}}))
    candidate=tmp_path/'candidate.onnx';candidate.write_bytes(b'fake')
    candidate.with_suffix('.json').write_text('{}')
    class FakeDetector:
        version='fake';backend='unit';input_size=640;latency_ms=2
        def __init__(self,*args,**kwargs):pass
        def infer(self,image):return (),None
        def postprocess(self,rows,geometry,confidence,iou):
            assert (confidence,iou)==(.54,.5)
            return [Detection((10,10,30,30),'orange',.9,0)]
    monkeypatch.setattr(evaluation,'ConeDetector',FakeDetector)
    monkeypatch.setattr(evaluation,'Yolo11OnnxDetector',FakeDetector)
    output=tmp_path/'comparison.json'
    assert evaluation.main(['--recording',str(session),'--annotations',str(labels),
        '--training-audit',str(audit),'--compare-model',str(candidate),
        '--confidence','.54','--nms-iou','.5','--output',str(output)])==0
    report=json.loads(output.read_text())
    assert report['evaluation_set']=='SAME_RECORDED_FRAMES_AND_THRESHOLDS'
    assert report['comparison_status']=='MEASURED'
    assert report['baseline']['runs']==report['candidate']['runs']
