"""Raw SDK baseline: read as fast as the SDK delivers, measure rate, age, faults, inference."""
import sys, time, collections, json
import numpy as np
sys.path[:0]=['/home/mark1/Desktop/autonomous_kart/autonomy_stack_phase1/phase_01_perception/src']
from pathlib import Path
from autokart_perception.camera import HP60C, CameraError
from autokart_perception.detector import Yolo26OnnxDetector
A='/home/mark1/Desktop/autonomous_kart/autonomy_stack_phase1'
cam=HP60C(library=Path(A+'/phase_01_perception/native/build-x86_64-linux-gnu/libhp60c_bridge.so'),
          configuration=Path(A+'/vendor/linux_ros/linux/configurationfiles/hp60c_v2_00_20230704_configEncrypt.json'),max_age_ms=150)
det=Yolo26OnnxDetector(Path('/home/mark1/Desktop/autonomous_kart/models/yolo26s/best.onnx'))
mode=sys.argv[1] if len(sys.argv)>1 else 'capture'
dur=float(sys.argv[2]) if len(sys.argv)>2 else 15
faults=collections.Counter(); ages=[]; stamps=[]; infer=[]; labels=collections.Counter(); swapped=collections.Counter()
end=time.time()+dur; first=None
while time.time()<end:
    try: f=cam.read(timeout_ms=500)
    except CameraError as e: faults[str(e)]+=1; continue
    now=time.monotonic_ns(); ages.append((now-f.timestamp_ns)/1e6); stamps.append(f.timestamp_ns)
    if first is None: first=f.calibration.get('sdk_parameters')
    if mode=='detect':
        d=det.detect(f.rgb); infer.append(det.latency_ms)
        for x in d: labels[x.label]+=1
        if len(infer)%5==0:
            for x in det.detect(f.rgb[:,:,::-1].copy()): swapped[x.label]+=1
cam.close()
dt=np.diff(stamps)/1e9 if len(stamps)>1 else np.array([0])
q=lambda a,p: round(float(np.percentile(a,p)),1) if len(a) else None
print(json.dumps({'mode':mode,'frames':len(stamps),'fps':round(len(stamps)/dur,2),'sdk_interval_ms_p50':q(dt*1000,50),
 'read_age_ms_p50':q(ages,50),'read_age_ms_p95':q(ages,95),'faults':dict(faults),'backend':det.backend,
 'inference_ms_p50':q(infer,50),'inference_ms_p95':q(infer,95),'labels':dict(labels),'labels_if_channels_swapped':dict(swapped),
 'intrinsics_params':first[4:8] if first else None},indent=1))
