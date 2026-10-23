import numpy as np
import pytest
from autokart_perception.pipeline import Pipeline
from autokart_perception.synthetic import Synthetic
from autokart_perception.recording import Recorder,Replay
from test_pipeline import CFG

def test_roundtrip_and_no_overwrite(tmp_path):
    r=Pipeline(None,CFG,True).process(Synthetic().read())
    path=tmp_path/'capture';Recorder(path).write(r)
    with pytest.raises(FileExistsError):Recorder(path)
    replay=Replay(path);f=replay.read()
    assert f.source=='REPLAY' and np.array_equal(f.rgb,r.frame.rgb) and np.array_equal(f.depth,r.frame.depth)
    assert replay.read() is None

def test_replay_rejects_invalid_alignment(tmp_path):
    r=Pipeline(None,CFG,True).process(Synthetic().read())
    path=tmp_path/'capture';Recorder(path).write(r)
    import json
    meta_path=path/'000001.json';meta=json.loads(meta_path.read_text())
    meta['aligned']=False;meta_path.write_text(json.dumps(meta))
    result=Pipeline(None,CFG).process(Replay(path).read())
    assert result.status.startswith('INVALID_REPLAY') and result.cones==[]

def test_replay_manual_labels_expose_visible_recall(tmp_path):
    import json
    r=Pipeline(None,CFG,True).process(Synthetic().read())
    path=tmp_path/'capture';Recorder(path).write(r)
    labels=tmp_path/'labels.json'
    labels.write_text(json.dumps({'schema_version':1,'sessions':{'capture':{'000001':[
        {'box':[190,260,230,380],'label':'orange','distance_m':2.0}]}}}))
    frame=Replay(path,labels).read()
    result=Pipeline(None,CFG).process(frame)
    assert result.diagnostics['annotated_visible_cones']==1
    assert result.diagnostics['visible_cone_recall']==0
