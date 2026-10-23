"""Small LAN-capable browser viewer. No actuator routes or frame queue."""
import json
import sys
import threading
import time
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse, parse_qs

import cv2
import numpy as np

from .camera import CameraError
from .visualization import Viewer

FLAGS=('boxes','labels','delaunay','gabriel','boundaries','gates','centerline','curvature')
HTML='''<!doctype html>
<html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>AutoKart · Phase 1</title><style>
:root{color-scheme:dark;font-family:system-ui,-apple-system,sans-serif;background:#0b1018;color:#e6edf7}
*{box-sizing:border-box}body{margin:0}.shell{max-width:1520px;margin:auto;padding:24px}
header{display:flex;justify-content:space-between;align-items:center;gap:16px;flex-wrap:wrap;margin-bottom:18px}
h1{font-size:1.55rem;margin:0}h1 small{font-size:.8rem;font-weight:500;color:#94a3b8;margin-left:10px}
.badge{padding:6px 11px;border-radius:999px;background:#173a2c;color:#69e4a6;font-size:.75rem;font-weight:700;letter-spacing:.08em}
.badge.error{background:#462b2d;color:#ffaaa5}.card{border:1px solid #263345;background:#111a27;border-radius:16px;overflow:hidden}
.stream{display:block;width:100%;height:auto;background:#080d13}.bar{display:flex;gap:12px;flex-wrap:wrap;align-items:center;padding:12px 16px;border-top:1px solid #263345}
.metric{min-width:120px}.metric strong{display:block;font-size:1.08rem}.metric span{font-size:.74rem;color:#94a3b8;text-transform:uppercase;letter-spacing:.07em}
.row{display:grid;grid-template-columns:minmax(0,1fr) minmax(0,1fr);gap:16px;margin-top:16px}@media(max-width:760px){.row{grid-template-columns:minmax(0,1fr)}}
.panel{padding:16px}.panel h2{margin:0 0 12px;font-size:1rem}.switches{display:flex;gap:9px;flex-wrap:wrap}
.detail{font-family:ui-monospace,monospace;font-size:.81rem;line-height:1.5;color:#c1cde0}.detail div{border-top:1px solid #263345;padding:7px 0;white-space:pre-line;overflow-wrap:anywhere}
.history{width:100%;height:110px;background:#0c1420;border-radius:8px}
.cal-grid{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:10px}.cal-grid label{display:block;color:#aabbd0;font-size:.82rem}
.cal-grid input{display:block;width:100%;margin-top:4px;padding:8px;border:1px solid #39495d;border-radius:7px;background:#0b1420;color:#e6edf7;font:inherit}
.cal-actions{display:flex;align-items:center;gap:12px;flex-wrap:wrap;margin-top:12px}.cal-note{font-size:.78rem;color:#aabbd0;margin-top:10px}
.cal-message{font-size:.82rem;color:#9dddb8}.cal-message.error{color:#ffaaa5}
.switches form{display:inline}.switch{border:1px solid #39495d;border-radius:999px;padding:8px 12px;background:#1b2737;color:#dce6f5;cursor:pointer;font:inherit;font-size:.82rem}
.switch.off{background:#141b26;color:#77889f;border-color:#263345}.status{font-family:ui-monospace,monospace;font-size:.84rem;line-height:1.6;color:#c1cde0;min-height:3em;white-space:pre-wrap;overflow-wrap:anywhere}
footer{margin-top:15px;color:#8b9bb0;font-size:.8rem}.live-dot{display:inline-block;width:7px;height:7px;background:#69e4a6;border-radius:50%;margin-right:6px}
</style></head><body><div class="shell"><header><h1>AutoKart <small>Phase 1 · stationary perception</small></h1><span id="source" class="badge">CONNECTING</span></header>
<div class="card"><img class="stream" src="/stream.mjpg" alt="Camera and track visualization"><div class="bar">
<div class="metric"><strong id="rgbfps">—</strong><span>RGB FPS</span></div><div class="metric"><strong id="outfps">—</strong><span>Output FPS</span></div>
<div class="metric"><strong id="infer">—</strong><span>Inference</span></div><div class="metric"><strong id="valid">—</strong><span>Valid depth</span></div>
<div class="metric"><strong id="cones">—</strong><span id="cone-label">Detected / metric</span></div><div class="metric"><strong id="drops">—</strong><span>Mailbox overwrites</span></div>
<div class="metric"><strong id="edges">—</strong><span>Delaunay / Gabriel</span></div><div class="metric"><strong id="recall">—</strong><span>Labeled recall</span></div>
<div class="metric"><strong id="gates">—</strong><span>Gate candidates</span></div><div class="metric"><strong id="rejected">—</strong><span>Rejected metric</span></div>
</div></div><div class="row"><section class="card panel"><h2>Overlays</h2><div id="switches" class="switches"></div></section>
<section class="card panel"><h2>Diagnostics</h2><div id="status" class="status">Waiting for frames…</div></section></div>
<div class="row"><section class="card panel"><h2>Calibration &amp; Cone Diagnostics</h2>
<div id="cal-status" class="status">Waiting for calibration…</div>
<form id="cal-form"><div class="cal-grid">
<label>Lens height above floor (m)<input name="height_m" type="number" min="0.05" max="2" step="0.001" required></label>
<label>Downward pitch (°)<input name="pitch_down_deg" type="number" min="-60" max="75" step="0.1" required></label>
<label>Roll (°)<input name="roll_deg" type="number" min="-45" max="45" step="0.1" required></label>
<label>Yaw left (°)<input name="yaw_left_deg" type="number" min="-180" max="180" step="0.1" required></label>
</div><div class="cal-actions"><button class="switch" type="submit">Apply for this session</button><span id="cal-message" class="cal-message"></span></div></form>
<div class="cal-note">Angles use the bench frame: pitch down, yaw toward left. Changes apply to new frames only and are not saved to disk or marked surveyed. Recheck measured cone positions after moving the camera.</div>
<h2 style="margin-top:18px">Individual cones</h2><div id="cone-detail" class="detail">No detections in current frame.</div></section>
<section class="card panel"><h2>FPS history · RGB / processed</h2><canvas id="fps-history" class="history" width="560" height="110"></canvas></section></div>
<footer><span class="live-dot"></span>RGB and top-down view update from the newest frame. Mailbox overwrites count completed frame pairs replaced before processing; they do not measure USB loss. Survey validation is required before trusting metric cone positions. No vehicle controls.</footer>
</div><script>
const names={boxes:'Boxes',labels:'Labels',delaunay:'Delaunay',gabriel:'Gabriel',boundaries:'Boundaries',gates:'Gates',centerline:'Centerline',curvature:'Curvature'};
const $=id=>document.getElementById(id);let flags={};let seenMountRevision=null;
function mountFields(m,revision){if(!m||revision===seenMountRevision)return;
 for(const key of ['height_m','pitch_down_deg','roll_deg','yaw_left_deg'])$('cal-form').elements[key].value=m[key];
 seenMountRevision=revision}
$('cal-form').onsubmit=async e=>{e.preventDefault();const values={};
 for(const key of ['height_m','pitch_down_deg','roll_deg','yaw_left_deg'])values[key]=Number($('cal-form').elements[key].value);
 const message=$('cal-message');message.classList.remove('error');message.textContent='Applying…';
 try{const r=await fetch('/api/calibration',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(values)});
 const body=await r.json();if(!r.ok)throw new Error(body.error||'HTTP '+r.status);
 seenMountRevision=null;mountFields(body.mount,body.mount_revision);message.textContent='Applied to new frames · validation still required';
 }catch(err){message.classList.add('error');message.textContent=err.message}};
function drawHistory(rows){const c=$('fps-history'),ctx=c.getContext('2d');ctx.clearRect(0,0,c.width,c.height);
 const max=Math.max(20,...rows.map(r=>Math.max(r.rgb||0,r.processed||0)));ctx.strokeStyle='#294057';ctx.lineWidth=1;
 for(const v of [0,5,10,15,20]){const y=c.height-8-v/max*(c.height-16);ctx.beginPath();ctx.moveTo(0,y);ctx.lineTo(c.width,y);ctx.stroke()}
 for(const [field,color] of [['rgb','#67d5ff'],['processed','#69e4a6']]){ctx.strokeStyle=color;ctx.lineWidth=2;ctx.beginPath();rows.forEach((r,i)=>{const x=rows.length>1?i*(c.width-1)/(rows.length-1):0;
 const y=c.height-8-(r[field]||0)/max*(c.height-16);i?ctx.lineTo(x,y):ctx.moveTo(x,y)});ctx.stroke()}}
function switches(){const box=$('switches');for(const [key,label] of Object.entries(names)){
 let form=box.querySelector('form[data-flag="'+key+'"]');
 if(!form){form=document.createElement('form');form.method='post';form.action='/toggle';form.dataset.flag=key;
  const hidden=document.createElement('input');hidden.type='hidden';hidden.name='flag';hidden.value=key;form.appendChild(hidden);
  const b=document.createElement('button');b.type='submit';b.textContent=label;form.appendChild(b);box.appendChild(form)}
 form.querySelector('button').className='switch'+(flags[key]?'':' off')}}
async function refresh(){try{const r=await fetch('/api/status',{cache:'no-store'});const s=await r.json();
 if(JSON.stringify(flags)!==JSON.stringify(s.toggles)){flags=s.toggles;switches()}
 $('source').textContent=s.source+' · '+(s.error?'ERROR':'ACTIVE');$('source').classList.toggle('error',!!s.error);
 const d=s.diagnostics||{};const f=(v,n=1)=>Number.isFinite(v)?v.toFixed(n):'—';
 $('rgbfps').textContent=f(d.rgb_fps);$('outfps').textContent=f(d.end_to_end_fps);$('infer').textContent=f(d.inference_ms)+' ms';
 $('valid').textContent=f(d.valid_depth_pct)+'%';$('cones').textContent=(d.detected_cones??0)+' / '+(d.localized_cones??0);$('cone-label').textContent=s.source==='SYNTHETIC'?'Detected / synthetic points':'Detected / metric';$('drops').textContent=d.mailbox_overwrites??0;
 $('edges').textContent=(d.delaunay_edges??0)+' / '+(d.gabriel_edges??0);
 $('gates').textContent=d.accepted_gate_candidates??0;$('rejected').textContent=d.rejected_metric_cones??0;
 $('recall').textContent=Number.isFinite(d.visible_cone_recall)?f(100*d.visible_cone_recall)+'%':'—';
 mountFields(s.calibration?.mount,s.calibration?.revision);
 $('cal-status').textContent='Pose: '+(s.calibration?.status||'—')+' ('+(s.calibration?.source||'—')+')'+'\\nRGB/depth registration: '+(d.rgb_depth_registration||'NO_FRAME')+' · Graph-eligible cones: '+(d.localized_cones??0)+'\\nLens origin X '+f(s.calibration?.mount?.x_forward_m,3)+' m, Y '+f(s.calibration?.mount?.y_left_m,3)+' m; height '+f(s.calibration?.mount?.height_m,3)+' m';
 $('status').textContent='Camera: '+(d.camera||'—')+'   Model: '+(d.model||'—')+' '+(d.model_version||'')+'   Device: '+(d.inference_device||'—')+'\\nThresholds: confidence '+f(d.confidence_threshold,2)+' · NMS IoU '+f(d.nms_iou_threshold,2)+'\\nCalibration: '+(d.calibration_status||'—')+'   Graph: '+(d.graph||'—')+'\\nState: '+(s.error||s.status||'waiting')+'   Frame: '+s.sequence+'\\nAge: '+f(d.frame_age_ms)+' ms   Inference: '+f(d.inference_ms)+' ms   Depth: '+f(d.depth_localization_ms)+' ms   Geometry: '+f(d.geometry_ms)+' ms   Render: '+f(d.render_ms)+' ms'+'\\nFPS: RGB '+f(d.rgb_fps)+' · depth '+f(d.depth_fps)+' · inference '+f(d.inference_fps)+' · output '+f(d.end_to_end_fps)+'\\nSides: '+(d.boundary_mode||'—')+' '+(d.boundary_status||'—')+' ('+(d.left_boundary_cones??0)+' left / '+(d.right_boundary_cones??0)+' right); '+(d.boundary_reason||'—')+'\\nCenterline: '+(d.centerline_status||'—')+' · Curvature: '+(d.curvature_status||'—')+' · max |k| '+f(d.max_abs_curvature_m_inv,2)+'/m'+'\\nReceived: '+(d.received_frames??'—')+'   Processed: '+(d.processed_frames??'—')+'   Elapsed: '+f(d.capture_elapsed_s)+' s'+'\\nMailbox overwrite rate: '+f(d.mailbox_overwrite_pct)+'%';
 const detail=$('cone-detail');detail.replaceChildren();for(const item of d.cone_details||[]){const row=document.createElement('div');
 row.textContent='#'+item.id+' '+item.label+' · confidence '+f(item.confidence,2)+' · '+item.localization_status+'\\n'+
 'valid depth '+(item.valid_depth_pixels??'—')+'/'+(item.required_depth_pixels??'—')+' px; coherent '+(item.coherent_depth_pixels??'—')+' px; median '+f(item.sampled_depth_m,3)+' m; MAD '+f(item.depth_mad_m,3)+' m; spread σ '+f(item.depth_sigma_m,3)+' m'+'\\n'+
 'estimated ground '+(item.ground_xy_m?'X '+f(item.ground_xy_m[0],3)+' Y '+f(item.ground_xy_m[1],3)+' m':'—')+'; candidate '+(item.ground_candidate_xy_m?'X '+f(item.ground_candidate_xy_m[0],3)+' Y '+f(item.ground_candidate_xy_m[1],3)+' m':'—')+'; ground offset '+f(item.ground_contact_offset_m,3)+'/'+f(item.max_ground_contact_offset_m,3)+' m; estimated distance '+f(item.distance_m,3)+' m; measured '+f(item.measured_distance_m,3)+' m'+
 (Number.isFinite(item.distance_error_m)?'; distance error '+f(item.distance_error_m,3)+' m':'')+
 (Number.isFinite(item.position_error_m)?'; XY error '+f(item.position_error_m,3)+' m':'')+'; depth '+item.depth_status;
 detail.appendChild(row)}
 if(!detail.childElementCount)detail.textContent='No detections in current frame.';drawHistory(s.fps_history||[]);
 }catch(e){$('source').textContent='OFFLINE';$('source').classList.add('error');$('status').textContent='Dashboard server unavailable'}}
refresh();setInterval(refresh,1000);
</script></body></html>'''

class Dashboard:
    def __init__(self,source,pipeline,recorder=None,frame_limit=0):
        self.source=source;self.pipeline=pipeline;self.recorder=recorder;self.frame_limit=frame_limit
        self.viewer=Viewer(web=True)
        self.cv=threading.Condition();self.stop_event=threading.Event()
        self.process_lock=threading.Lock()
        self.jpeg=None;self.sequence=0;self.status='WAITING_FOR_CAMERA';self.error=None;self.diagnostics={}
        self.fps_history=deque(maxlen=120);self.last_history_second=-1
        self.mount_revision=0;self.mount_source='config'
        self.worker=None

    def _placeholder(self,message):
        canvas=np.full((590,1280,3),20,dtype=np.uint8)
        cv2.putText(canvas,'AutoKart Phase 1',(40,70),cv2.FONT_HERSHEY_SIMPLEX,1,(235,235,235),2)
        cv2.putText(canvas,message[:90],(40,130),cv2.FONT_HERSHEY_SIMPLEX,.7,(120,180,255),2)
        return canvas

    def _publish(self,canvas,status,error,diagnostics=None):
        ok,encoded=cv2.imencode('.jpg',canvas,[cv2.IMWRITE_JPEG_QUALITY,82])
        if not ok:raise RuntimeError('JPEG_ENCODE_FAILED')
        with self.cv:
            self.jpeg=encoded.tobytes();self.sequence+=1
            self.status=status;self.error=error
            detector=self.pipeline.detector
            base={'camera':'ERROR' if error else 'WAITING',
                  'model':('READY' if detector.verified_checkpoint else 'USER_MODEL_UNVERIFIED') if detector else 'MODEL_NOT_AVAILABLE',
                  'model_version':detector.version if detector else 'none',
                  'inference_device':detector.backend if detector else 'none',
                  'confidence_threshold':self.pipeline.config['confidence'],
                  'nms_iou_threshold':self.pipeline.config['nms_iou'],
                  'calibration_status':self.pipeline.mount_status,
                  'rgb_depth_registration':'NO_FRAME',
                  'safety_state':'SENSOR_FAULT' if error else 'AMBIGUOUS',
                  'safety_reasons':[str(error)] if error else ['NO_CURRENT_FRAME'],
                  'speed_limit_advisory_mps':0.0,'vehicle_authorized':False}
            self.diagnostics={**base,**(diagnostics or {})}
            if error:self.fps_history.clear();self.last_history_second=-1
            second=int(time.monotonic())
            if diagnostics and second!=self.last_history_second:
                self.fps_history.append({'rgb':diagnostics.get('rgb_fps',0),
                                         'processed':diagnostics.get('end_to_end_fps',0)})
                self.last_history_second=second
            self.cv.notify_all()

    def _run(self):
        count=0
        self._publish(self._placeholder('Waiting for camera frames...'),'WAITING_FOR_CAMERA',None)
        while not self.stop_event.is_set() and (not self.frame_limit or count<self.frame_limit):
            try:
                frame=self.source.read()
                if frame is None:
                    self._publish(self._placeholder('Replay complete'),'REPLAY_COMPLETE',None)
                    break
                with self.process_lock:
                    result=self.pipeline.process(frame)
                    if self.recorder:self.recorder.write(result)
                    render_start=time.perf_counter()
                    with self.cv:canvas=self.viewer.draw(result)
                    result.diagnostics['render_ms']=(time.perf_counter()-render_start)*1000
                    result.diagnostics['capture_to_publish_ms']=(time.monotonic_ns()-frame.timestamp_ns)/1e6 if frame.source=='LIVE' else None
                    self._publish(canvas,result.status,None,result.diagnostics)
                count+=1
                if frame.source in ('SYNTHETIC','REPLAY'):self.stop_event.wait(.1)
            except CameraError as exc:
                message=str(exc)
                self.pipeline.note_sensor_fault(message)
                if self.recorder:self.recorder.record_fault(message,self.pipeline_source)
                if message!=self.error:print(f'Camera status: {message}',file=sys.stderr)
                self._publish(self._placeholder(message),message,message)
                self.stop_event.wait(1)
            except Exception as exc:
                message=f'PIPELINE_ERROR:{type(exc).__name__}:{exc}'
                if self.recorder:self.recorder.record_fault(message,self.pipeline_source)
                if message!=self.error:print(message,file=sys.stderr)
                self._publish(self._placeholder(message),message,message)
                self.stop_event.wait(.5)
        if self.frame_limit and count>=self.frame_limit:
            self._publish(self._placeholder('Frame limit reached'),'COMPLETE',None)

    def start(self):
        self.worker=threading.Thread(target=self._run,name='autokart-producer',daemon=True)
        self.worker.start()

    def close(self):
        self.stop_event.set()
        with self.cv:self.cv.notify_all()
        if self.worker:self.worker.join(timeout=3)
        if hasattr(self.source,'close'):self.source.close()

    def status_json(self):
        with self.process_lock:
            calibration={'status':self.pipeline.mount_status,'mount':self.pipeline.mount_parameters(),
                         'revision':self.mount_revision,'source':self.mount_source}
            with self.cv:
                return {'source':self.pipeline_source,'sequence':self.sequence,'status':self.status,
                        'error':self.error,'diagnostics':self.diagnostics,'toggles':dict(self.viewer.flags),
                        'calibration':calibration,'fps_history':list(self.fps_history)}

    def set_calibration(self,changes):
        with self.process_lock:
            mount=self.pipeline.set_mount(changes)
            self.mount_revision+=1;self.mount_source='session'
            self._publish(self._placeholder('Calibration changed; waiting for new frame'),
                          'CALIBRATION_UPDATED_WAITING_FOR_FRAME',None)
            return {'status':self.pipeline.mount_status,'mount':mount,
                    'revision':self.mount_revision,'source':self.mount_source}

    @property
    def pipeline_source(self):
        name=type(self.source).__name__.upper()
        return {'HP60C':'LIVE','REPLAY':'REPLAY','SYNTHETIC':'SYNTHETIC'}.get(name,name)

    def set_toggles(self,changes):
        if not isinstance(changes,dict) or any(k not in FLAGS or type(v) is not bool for k,v in changes.items()):
            raise ValueError('INVALID_TOGGLES')
        with self.cv:
            self.viewer.flags.update(changes)
            return dict(self.viewer.flags)


def make_server(dashboard,host,port):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self,format,*args):
            pass
        def _json(self,obj,status=200):
            data=json.dumps(obj).encode()
            self.send_response(status);self.send_header('Content-Type','application/json')
            self.send_header('Cache-Control','no-store');self.send_header('Content-Length',str(len(data)))
            self.end_headers();self.wfile.write(data)
        def do_GET(self):
            path=urlparse(self.path).path
            if path=='/':
                data=HTML.encode();self.send_response(200)
                self.send_header('Content-Type','text/html; charset=utf-8');self.send_header('Content-Length',str(len(data)))
                self.end_headers();self.wfile.write(data)
            elif path=='/api/status':self._json(dashboard.status_json())
            elif path=='/stream.mjpg':
                self.send_response(200);self.send_header('Content-Type','multipart/x-mixed-replace; boundary=frame')
                self.send_header('Cache-Control','no-store');self.end_headers()
                last=0
                try:
                    while not dashboard.stop_event.is_set():
                        with dashboard.cv:
                            dashboard.cv.wait_for(lambda:dashboard.sequence>last or dashboard.stop_event.is_set(),timeout=2)
                            if dashboard.stop_event.is_set():break
                            if dashboard.sequence<=last:continue
                            last=dashboard.sequence;image=dashboard.jpeg
                        if image is None:continue
                        self.wfile.write(b'--frame\r\nContent-Type: image/jpeg\r\nContent-Length: '+str(len(image)).encode()+b'\r\n\r\n')
                        self.wfile.write(image+b'\r\n');self.wfile.flush()
                except (BrokenPipeError,ConnectionResetError,ConnectionAbortedError):pass
            else:self._json({'error':'NOT_FOUND'},404)
        def do_POST(self):
            path=urlparse(self.path).path
            if path=='/api/calibration':
                origin=self.headers.get('Origin')
                if origin and origin not in ('http://'+self.headers.get('Host',''),
                                          'https://'+self.headers.get('Host','')):
                    return self._json({'error':'CROSS_ORIGIN_REJECTED'},403)
                try:
                    size=int(self.headers.get('Content-Length','0'))
                    if not 0<size<=1024:raise ValueError('INVALID_BODY_SIZE')
                    if self.headers.get('Content-Type','').split(';')[0].strip()!='application/json':
                        raise ValueError('JSON_REQUIRED')
                    changes=json.loads(self.rfile.read(size))
                    return self._json(dashboard.set_calibration(changes))
                except (ValueError,TypeError,json.JSONDecodeError) as exc:
                    return self._json({'error':str(exc)},400)
            if path=='/toggle':
                try:
                    size=int(self.headers.get('Content-Length','0'))
                    if not 0<size<=256:raise ValueError('INVALID_BODY_SIZE')
                    flag=parse_qs(self.rfile.read(size).decode()).get('flag',[None])[0]
                    if flag not in FLAGS:raise ValueError('INVALID_FLAG')
                    current=dashboard.status_json()['toggles'][flag]
                    dashboard.set_toggles({flag:not current})
                except (ValueError,UnicodeDecodeError):return self._json({'error':'INVALID_TOGGLE'},400)
                self.send_response(303);self.send_header('Location','/');self.send_header('Content-Length','0')
                self.end_headers();return
            if path!='/api/toggles':return self._json({'error':'NOT_FOUND'},404)
            try:
                size=int(self.headers.get('Content-Length','0'))
                if not 0<size<=2048:raise ValueError('INVALID_BODY_SIZE')
                changes=json.loads(self.rfile.read(size))
                self._json(dashboard.set_toggles(changes))
            except (ValueError,json.JSONDecodeError):self._json({'error':'INVALID_TOGGLES'},400)
    class Server(ThreadingHTTPServer):
        daemon_threads=True
        allow_reuse_address=True
    return Server((host,port),Handler)


def serve_dashboard(source,pipeline,recorder,host,port,frame_limit=0):
    dashboard=Dashboard(source,pipeline,recorder,frame_limit)
    server=make_server(dashboard,host,port)
    dashboard.start()
    print(f'AutoKart dashboard: http://{host}:{server.server_address[1]}',flush=True)
    try:server.serve_forever(poll_interval=.2)
    except KeyboardInterrupt:pass
    finally:
        dashboard.close();server.server_close()
    return 0
