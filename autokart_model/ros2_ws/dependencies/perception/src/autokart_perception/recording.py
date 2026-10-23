import json
import hashlib
import time
from dataclasses import asdict
from pathlib import Path
import numpy as np
from .types import Frame, Intrinsics

class Recorder:
    def __init__(self,path,model_version='none',model_sha256=None):
        self.path=Path(path)
        self.path.mkdir(parents=True,exist_ok=False)
        self.index=0; self.model_version=model_version;self.model_sha256=model_sha256
        self.last_state=None

    def _transition(self,state,reasons,source,timestamp_ns):
        if state==self.last_state:return
        event={'event':'SAFETY_TRANSITION','timestamp_ns':int(timestamp_ns),
               'source':source,'from':self.last_state,'to':state,'reasons':list(reasons),
               'vehicle_authorized':False}
        with (self.path/'events.jsonl').open('a') as stream:
            stream.write(json.dumps(event,sort_keys=True)+'\n')
        self.last_state=state

    def record_fault(self,reason,source='LIVE'):
        self._transition('SENSOR_FAULT',(str(reason),),source,time.monotonic_ns())

    def write(self,result):
        self.index+=1
        name=f'{self.index:06d}'
        f=result.frame
        np.savez_compressed(self.path/f'{name}.npz',rgb=f.rgb,depth=f.depth)
        metadata={'schema_version':2,'timestamp_ns':f.timestamp_ns,'rgb_timestamp':f.rgb_timestamp,
          'depth_timestamp':f.depth_timestamp,'rgb_sequence':f.rgb_sequence,
          'depth_sequence':f.depth_sequence,'intrinsics':vars(f.intrinsics),
          'depth_scale':f.depth_scale,'aligned':f.aligned,'source':f.source,
          'drops':f.drops,'received_frames':f.received_frames,
          'capture_elapsed_ns':f.capture_elapsed_ns,
          'calibration':f.calibration,'model_version':self.model_version,
          'model_sha256':self.model_sha256,
          'calibration_sha256':result.diagnostics.get('calibration_sha256'),
          'array_sha256':{'rgb':hashlib.sha256(f.rgb.tobytes()).hexdigest(),
                          'depth':hashlib.sha256(f.depth.tobytes()).hexdigest()},
          'diagnostics':result.diagnostics,
          'safety':asdict(result.safety) if result.safety is not None else None,
          'rejected_gates':result.diagnostics.get('rejected_gates',[]),
          'detections':[vars(d) for d in result.detections],
          'cones':[vars(c) for c in result.cones]}
        (self.path/f'{name}.json').write_text(json.dumps(metadata,indent=2))
        if result.safety is not None:
            self._transition(result.safety.state,result.safety.reasons,f.source,
                             result.safety.produced_timestamp_ns)

class Replay:
    def __init__(self,path,annotations=None):
        self.files=sorted(p for p in Path(path).glob('*.json')
                          if len(p.stem)==6 and p.stem.isdecimal())
        if not self.files: raise FileNotFoundError('REPLAY_EMPTY')
        if [int(p.stem) for p in self.files]!=list(range(1,len(self.files)+1)):
            raise ValueError('REPLAY_SEQUENCE_GAP')
        self.index=0
        self.manual_labels={}
        if annotations:
            from .evaluation import recording_files, load_annotations
            sessions=recording_files([path])
            self.manual_labels={stem:items for (session,stem),(_,items) in
                                load_annotations(annotations,sessions).items()}

    def read(self):
        if self.index>=len(self.files): return None
        meta_path=self.files[self.index]; self.index+=1
        meta=json.loads(meta_path.read_text())
        if meta.get('schema_version',1) not in (1,2):raise ValueError('UNSUPPORTED_RECORDING_SCHEMA')
        if not meta_path.with_suffix('.npz').is_file():raise ValueError('REPLAY_ARRAY_MISSING')
        with np.load(meta_path.with_suffix('.npz'),allow_pickle=False) as array:
            rgb=array['rgb'].copy(); depth=array['depth'].copy()
        digests=meta.get('array_sha256')
        if digests is not None and (hashlib.sha256(rgb.tobytes()).hexdigest()!=digests.get('rgb') or
                                    hashlib.sha256(depth.tobytes()).hexdigest()!=digests.get('depth')):
            raise ValueError('REPLAY_ARRAY_INTEGRITY_FAILURE')
        calibration=dict(meta['calibration'])
        calibration['recorded_mount_sha256']=meta.get('calibration_sha256')
        calibration['recorded_model_sha256']=meta.get('model_sha256')
        calibration['recorded_stage_timestamps_ns']=meta.get('diagnostics',{}).get('stage_timestamps_ns')
        calibration['recorded_detections']=meta.get('detections',[])
        calibration['recorded_cones']=meta.get('cones',[])
        calibration['recorded_model_version']=meta.get('model_version','unknown')
        calibration['recorded_source']=meta.get('source')
        calibration['recorded_diagnostics']=meta.get('diagnostics',{})
        if meta_path.stem in self.manual_labels:
            calibration['manual_annotations']=self.manual_labels[meta_path.stem]
        return Frame(rgb,depth,meta['timestamp_ns'],meta['rgb_timestamp'],meta['depth_timestamp'],
            meta['rgb_sequence'],meta['depth_sequence'],Intrinsics(**meta['intrinsics']),
            meta['depth_scale'],meta['aligned'],'REPLAY',meta['drops'],calibration,
            meta.get('received_frames',0),meta.get('capture_elapsed_ns',0))
