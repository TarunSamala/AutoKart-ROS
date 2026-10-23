import json
import time

import numpy as np
import pytest

from autokart_perception.pipeline import Pipeline
from autokart_perception.recording import Recorder, Replay
from autokart_perception.safety import assess, topology_reasons, STATES
from autokart_perception.synthetic import Synthetic
from autokart_perception.types import Boundaries, Cone, Gate, PathSample
from test_pipeline import CFG


def test_state_contract_never_authorizes_motion():
    assert STATES=={'VALID','DEGRADED','AMBIGUOUS','STALE','SENSOR_FAULT'}
    boundaries=Boundaries((0,2,4),(1,3,5),'COLOR','CANDIDATE',.9,'COLOR_SIDES')
    gates=[Gate(0,1,(1.,0.),3.,.9,'CANDIDATE'),Gate(2,3,(2.,0.),3.,.9,'CANDIDATE'),
           Gate(4,5,(3.,0.),3.,.9,'CANDIDATE')]
    kwargs=dict(frame_timestamp_ns=1_000_000_000,produced_ns=1_020_000_000,age_ms=20.,
        status='OK',diagnostics={'calibration_status':'MOUNT_VERIFIED','processing_ms':10.,
                                 'model':'READY','valid_depth_pct':80.},
        boundaries=boundaries,gates=gates,centerline=[(1.,0.),(2.,0.),(3.,0.)],
        path_profile=[PathSample((2.,0.),0.,1.,0.)],
        topology_errors=(),config=CFG)
    valid=assess('LIVE',**kwargs)
    assert valid.state=='VALID' and not valid.vehicle_authorized
    assert valid.speed_limit_advisory_mps==0.
    replay=assess('REPLAY',**kwargs)
    assert replay.state=='DEGRADED' and 'NON_LIVE_SOURCE' in replay.reasons
    assert not replay.vehicle_authorized


def test_frozen_out_of_order_and_stale_live_frames():
    pipeline=Pipeline(None,CFG,True)
    frame=Synthetic().read();frame.source='LIVE'
    pipeline.process(frame)
    frozen=pipeline.process(frame)
    assert frozen.safety.state=='SENSOR_FAULT'
    assert 'FRAME_FROZEN' in frozen.safety.reasons and not frozen.centerline
    older=Synthetic().read();older.source='LIVE'
    older.timestamp_ns=frame.timestamp_ns-1
    assert 'FRAME_OUT_OF_ORDER' in pipeline.process(older).safety.reasons
    fresh_pipeline=Pipeline(None,CFG,True)
    stale=Synthetic().read();stale.source='LIVE';stale.timestamp_ns-=300_000_000
    result=fresh_pipeline.process(stale)
    assert result.safety.state=='STALE' and 'FRAME_TOO_OLD' in result.safety.reasons
    assert not result.centerline


def test_inference_failure_and_overload():
    class Broken:
        verified_checkpoint=True;version='mock';backend='test';latency_ms=0.
        def detect(self,image):raise RuntimeError('injected failure')
    frame=Synthetic().read();frame.source='LIVE'
    result=Pipeline(Broken(),CFG).process(frame)
    assert result.safety.state=='SENSOR_FAULT' and 'INFERENCE_FAILURE' in result.safety.reasons
    class Slow(Broken):
        def detect(self,image):
            time.sleep(.16)
            self.latency_ms=160.
            return []
    frame=Synthetic().read();frame.source='LIVE'
    result=Pipeline(Slow(),CFG).process(frame)
    assert result.safety.state=='STALE'
    assert 'INFERENCE_OVERLOAD' in result.safety.reasons


def test_invalid_depth_and_wrong_side_assignment():
    from autokart_perception.types import Detection
    class Detector:
        verified_checkpoint=True;version='mock';backend='test';latency_ms=1.
        def detect(self,image):return [Detection((200,240,240,320),'orange',.9,0)]
    frame=Synthetic().read();frame.source='LIVE';frame.depth[:]=0
    result=Pipeline(Detector(),CFG).process(frame)
    assert result.safety.state=='SENSOR_FAULT'
    assert 'METRIC_LOCALIZATION_FAILED' in result.safety.reasons
    left=Cone(0,'blue',.9,(1.,-1.),.1,1.,(0,0,0,0))
    right=Cone(1,'yellow',.9,(1.,1.),.1,1.,(0,0,0,0))
    gate=Gate(0,1,(1.,0.),2.,.9,'CANDIDATE')
    assert 'GATE_SIDE_INVALID' in topology_reasons([gate],[],[left,right],2.,3.8)
    malformed=Synthetic().read();malformed.source='LIVE'
    malformed.depth=np.array([['bad']],dtype=object)
    invalid=Pipeline(Detector(),CFG).process(malformed)
    assert invalid.safety.state=='SENSOR_FAULT'
    assert 'INVALID_FRAME_FORMAT' in invalid.safety.reasons


def test_crossed_gate_rejected_even_when_widths_plausible():
    cones=[Cone(0,'blue',.9,(1.,1.5),.1,1.,(0,0,0,0)),
           Cone(1,'yellow',.9,(1.,-1.5),.1,1.,(0,0,0,0)),
           Cone(2,'blue',.9,(.5,1.5),.1,1.,(0,0,0,0)),
           Cone(3,'yellow',.9,(1.5,-1.5),.1,1.,(0,0,0,0))]
    gates=[Gate(0,1,(1.,0.),3.,.8,'CANDIDATE'),
           Gate(2,3,(1.,0.),float(np.sqrt(10)),.8,'CANDIDATE')]
    assert 'GATE_CROSSING' in topology_reasons(gates,[],cones,2.,3.8)
    assert 'GATE_CONE_REUSE' in topology_reasons([gates[0],gates[0]],[],cones,2.,3.8)


def test_recording_integrity_calibration_mismatch_and_fault_log(tmp_path):
    result=Pipeline(None,CFG,True).process(Synthetic().read())
    path=tmp_path/'session';recorder=Recorder(path,'mock','abc123');recorder.write(result)
    metadata=json.loads((path/'000001.json').read_text())
    assert metadata['schema_version']==2 and metadata['model_sha256']=='abc123'
    assert metadata['calibration_sha256'] and metadata['array_sha256']['depth']
    assert metadata['safety']['vehicle_authorized'] is False
    assert metadata['diagnostics']['stage_timestamps_ns']['gates']>0
    recorder.record_fault('CAMERA_DISCONNECTED')
    events=[json.loads(line) for line in (path/'events.jsonl').read_text().splitlines()]
    assert [event['to'] for event in events]==['DEGRADED','SENSOR_FAULT']
    cfg={**CFG,'camera_mount':{**CFG['camera_mount'],'height_m':.25}}
    mismatch=Pipeline(None,cfg).process(Replay(path).read())
    assert mismatch.safety.state=='SENSOR_FAULT'
    assert 'CALIBRATION_MISMATCH' in mismatch.safety.reasons and not mismatch.centerline
    with np.load(path/'000001.npz',allow_pickle=False) as data:
        rgb=data['rgb'].copy();depth=data['depth'].copy()
    depth[0,0]+=1
    np.savez_compressed(path/'000001.npz',rgb=rgb,depth=depth)
    with pytest.raises(ValueError,match='REPLAY_ARRAY_INTEGRITY_FAILURE'):
        Replay(path).read()


def test_new_pipeline_instance_does_not_reuse_previous_geometry():
    first=Pipeline(None,CFG,True).process(Synthetic().read())
    assert first.centerline
    restarted=Pipeline(None,CFG)
    frame=Synthetic().read()
    result=restarted.process(frame)
    assert result.centerline==[] and result.safety.state=='SENSOR_FAULT'


def test_camera_loss_latches_until_process_restart():
    pipeline=Pipeline(None,CFG,True)
    assert pipeline.process(Synthetic().read()).centerline
    pipeline.note_sensor_fault('CAMERA_DISCONNECTED')
    recovered=pipeline.process(Synthetic().read())
    assert recovered.safety.state=='SENSOR_FAULT'
    assert 'MANUAL_RESTART_REQUIRED' in recovered.safety.reasons
    assert not recovered.centerline
    assert Pipeline(None,CFG,True).process(Synthetic().read()).centerline


def test_recorded_session_replays_deterministic_geometry(tmp_path):
    source=Synthetic();pipeline=Pipeline(None,CFG,True)
    path=tmp_path/'two-frames';recorder=Recorder(path)
    originals=[]
    for _ in range(2):
        result=pipeline.process(source.read());recorder.write(result)
        originals.append((result.centerline,[(g.left_id,g.right_id) for g in result.gates]))
    replay=Replay(path);reader=Pipeline(None,CFG)
    observed=[]
    for _ in range(2):
        result=reader.process(replay.read())
        observed.append((result.centerline,[(g.left_id,g.right_id) for g in result.gates]))
        assert result.safety.state=='DEGRADED'
        assert 'NON_LIVE_SOURCE' in result.safety.reasons
        assert not result.safety.vehicle_authorized
    assert originals==observed
    assert replay.read() is None
    (path/'notes.json').write_text('{}')
    assert len(Replay(path).files)==2
    (path/'000001.json').rename(path/'000003.json')
    with pytest.raises(ValueError,match='REPLAY_SEQUENCE_GAP'):
        Replay(path)
