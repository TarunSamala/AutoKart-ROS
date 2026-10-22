import sys, time, json, cProfile, pstats, io, numpy as np, cv2
sys.path[:0]=['/home/mark1/Desktop/autonomous_kart/autonomy_stack_phase1/phase_01_perception/src','/home/mark1/Desktop/autonomous_kart/kart']
from pathlib import Path
from autokart_perception.camera import HP60C, CameraError
from autokart_perception.detector import Yolo26OnnxDetector
from autokart_perception.application import CoreRuntime
from autokart_perception.pipeline import Pipeline
A='/home/mark1/Desktop/autonomous_kart/autonomy_stack_phase1'
cv2.setNumThreads(6)
core=CoreRuntime()
cam=HP60C(library=Path(A+'/phase_01_perception/native/build-x86_64-linux-gnu/libhp60c_bridge.so'),
          configuration=Path(A+'/vendor/linux_ros/linux/configurationfiles/hp60c_v2_00_20230704_configEncrypt.json'),max_age_ms=150)
det=Yolo26OnnxDetector(Path('/home/mark1/Desktop/autonomous_kart/models/yolo26s/best.onnx'))
pipe=Pipeline(det,core.config)
prof=cProfile.Profile(); totals=[]; ages=[]; saved=False; stages=[]
end=time.time()+float(sys.argv[1])
while time.time()<end:
    try: f=cam.read()
    except CameraError: continue
    t=time.perf_counter(); prof.enable()
    r=core.sdk.process_frame(f,pipeline=pipe)
    prof.disable(); totals.append((time.perf_counter()-t)*1000)
    p=r.perception_result
    ages.append(p.diagnostics.get('frame_age_ms')); stages.append(p.safety.reasons)
    if not saved and p.detections:
        img=f.rgb.copy()
        for d in p.detections: cv2.rectangle(img,d.box[:2],d.box[2:],(0,255,0),2); cv2.putText(img,f'{d.label} {d.confidence:.2f}',(d.box[0],d.box[1]-4),0,.5,(0,255,0),1)
        cv2.imwrite(sys.argv[2],img); saved=True
cam.close()
q=lambda a,p: round(float(np.percentile(a,p)),1)
print(json.dumps({'frames':len(totals),'process_ms_p50':q(totals,50),'process_ms_p95':q(totals,95),'age_ms_p50':q(ages,50),'age_ms_p95':q(ages,95),'backend':det.backend,'last_reasons':stages[-1]}))
s=io.StringIO(); pstats.Stats(prof,stream=s).sort_stats('cumulative').print_stats(18); print(s.getvalue()[:3500])
