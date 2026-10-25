#!/usr/bin/env python3
"""Integrated HP60C blue-cone guard and BLD750 encoder bench dashboard.

This host-side test UI talks to the proven MicroPython controller over HTTP.
Only a metric-valid blue cone inside the bumper corridor can trigger the
automatic lock. Other cone colours are visualized but never stop the motor.
Camera, browser, and telemetry faults are reported without issuing /lock.
"""
from __future__ import annotations

import argparse
import json
import statistics
import sys
import threading
import time
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlsplit
from urllib.request import urlopen


ROOT = Path(__file__).resolve().parents[1]
PERCEPTION = ROOT / "autonomy_stack_phase1/phase_01_perception/src"
GEOMETRY_PACKAGE = ROOT / "kart/ros2_ws/src/autokart_vehicle_geometry"
sys.path[:0] = [str(PERCEPTION), str(GEOMETRY_PACKAGE), str(ROOT / "tools")]

from autokart_vehicle_geometry.geometry import GEOMETRY  # noqa: E402


HTML = r'''<!doctype html><html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>AutoKart Cone Brake Demonstrator</title><style>
:root{--paper:#f4f6f2;--panel:#fffefa;--ink:#162525;--muted:#657471;--line:#cdd5cf;--green:#087865;--green2:#15a187;--red:#cf3c32;--blue:#2868c7;--orange:#e35d0b;--yellow:#cba400;--graph:#7653a6}*{box-sizing:border-box}body{margin:0;background:var(--paper);color:var(--ink);font-family:Inter,ui-sans-serif,system-ui,sans-serif}.app{max-width:1560px;margin:auto;padding:12px}.header{display:flex;justify-content:space-between;align-items:center;gap:16px;margin-bottom:10px}.brand small,.title,.kicker{font:700 11px ui-monospace,monospace;letter-spacing:.11em;color:#526663}.brand h1{margin:4px 0;font-size:25px}.actions{display:flex;gap:9px}.button{border:1px solid transparent;border-radius:6px;padding:12px 20px;font-weight:850;font-size:14px;cursor:pointer;transition:.12s}.button:active{transform:translateY(1px)}.button:disabled{opacity:.35;cursor:not-allowed}.start{background:var(--green);color:white;box-shadow:0 3px 12px #08786530}.stop{background:#fff;border-color:#e0a39e;color:var(--red)}.status{padding:9px 12px;border:1px solid var(--line);border-radius:6px;font:750 11px ui-monospace,monospace;background:#fff}.status.run{border-color:#74b8a9;color:var(--green)}.status.stopState{border-color:#e0a39e;color:var(--red)}.gridTop{display:grid;grid-template-columns:1.35fr .9fr;gap:12px}.stack{display:grid;gap:12px}.panel{background:var(--panel);border:1px solid var(--line);border-radius:7px;padding:12px;box-shadow:0 1px 2px #18342a0a}.title{margin-bottom:9px}.map,.camera,.chart{position:relative;background:#e1e6e2;overflow:hidden}.map{height:520px}.camera{aspect-ratio:4/3}.camera img,canvas{display:block;width:100%;height:100%;object-fit:contain}.cameraOverlay{position:absolute;left:10px;bottom:10px;background:#fffefae8;border:1px solid #b9c4bd;border-radius:4px;padding:6px 8px;font:700 11px ui-monospace,monospace}.legend{display:flex;gap:17px;flex-wrap:wrap;margin-top:8px;color:var(--muted);font:11px ui-monospace,monospace}.swatch{display:inline-block;width:9px;height:9px;margin-right:5px;border-radius:2px}.controller{display:grid;grid-template-columns:1fr 1fr;gap:15px}.metric label{display:flex;justify-content:space-between;color:var(--muted);font:11px ui-monospace,monospace}.metric strong{font:700 27px ui-monospace,monospace;display:block;margin:2px 0}.bar{height:8px;background:#dfe5df}.bar i{display:block;height:100%;background:var(--green)}.facts{display:grid;grid-template-columns:auto 1fr;gap:5px 18px;margin-top:14px;font:12px ui-monospace,monospace}.facts dt{color:var(--muted)}.facts dd{margin:0}.charts{display:grid;grid-template-columns:1fr 1fr;gap:12px;margin-top:12px}.chart{height:220px}.counts{display:grid;grid-template-columns:repeat(4,1fr);gap:8px;margin-top:10px}.count{border-left:3px solid var(--green);padding:4px 8px}.count b{display:block;font:700 22px ui-monospace,monospace}.count span{font:10px ui-monospace,monospace;color:var(--muted)}.message{min-height:18px;color:var(--muted);font:11px ui-monospace,monospace;margin-top:8px}.log{margin-top:12px}.tableWrap{overflow:auto;max-height:300px}table{width:100%;border-collapse:collapse;font:11px ui-monospace,monospace}th,td{border-bottom:1px solid #dce2dd;padding:7px 10px;text-align:right;white-space:nowrap}th{position:sticky;top:0;background:var(--panel);color:#526663;letter-spacing:.05em}th:first-child,td:first-child{text-align:left}.notice{margin-top:10px;padding:8px 10px;border:1px solid #e4c68a;background:#fff8e8;color:#795d1b;font:11px ui-monospace,monospace}.good{color:var(--green)}.bad{color:var(--red)}@media(max-width:950px){.gridTop,.charts{grid-template-columns:1fr}.map{height:430px}.header{align-items:flex-start;flex-direction:column}.counts{grid-template-columns:1fr 1fr}}@media(max-width:520px){.controller{grid-template-columns:1fr}.actions{width:100%}.button{flex:1;padding:11px}.map{height:360px}}
</style></head><body><main class="app"><header class="header"><div class="brand"><small>AUTOKART / LIVE PERCEPTION DEMONSTRATOR</small><h1>Blue-Cone Brake Mission</h1><small>HP60C · YOLO26S · AS5047P · BLD750</small></div><div class="actions"><span id="mission" class="status stopState">INITIALISING</span><button id="start" class="button start">START · 0.50 V</button><button id="stop" class="button stop">STOP</button></div></header>
<section class="gridTop"><div class="stack"><article class="panel"><div class="title">LOCAL METRIC MAP · HEAT MAP · DELAUNAY · GABRIEL</div><div class="map"><canvas id="map"></canvas></div><div class="legend"><span><i class="swatch" style="background:var(--orange)"></i>orange detection</span><span><i class="swatch" style="background:var(--blue)"></i>blue stop cone</span><span><i class="swatch" style="background:#8b9893"></i>Delaunay</span><span><i class="swatch" style="background:var(--graph)"></i>Gabriel</span><span>heat = current metric cone density</span></div><div class="counts"><div class="count"><b id="orangeCount">0</b><span>ORANGE CONES</span></div><div class="count"><b id="blueCount">0</b><span>BLUE CONES</span></div><div class="count"><b id="edgeCount">0 / 0</b><span>DELAUNAY / GABRIEL</span></div><div class="count"><b id="blueGap">—</b><span>BLUE BUMPER GAP</span></div></div></article></div>
<aside class="stack"><article class="panel"><div class="title">HP60C RGB-D FRAME · LIVE DETECTIONS</div><div class="camera"><img id="camera" alt="Live HP60C detections"><div id="cameraOverlay" class="cameraOverlay">WAITING FOR FRAME</div></div><div id="cameraMeta" class="message">Camera initialising</div></article><article class="panel"><div class="title">VEHICLE AND CONTROLLER</div><div class="controller"><div class="metric"><label><span>MOTOR RPM</span><span id="rpmTarget">0.50 V target</span></label><strong id="rpm">0</strong><div class="bar"><i id="rpmBar" style="width:0%"></i></div></div><div class="metric"><label><span>AXLE RPM</span><span>AS5047P</span></label><strong id="axle">0</strong><div class="bar"><i id="axleBar" style="width:0%;background:var(--blue)"></i></div></div><div class="metric"><label><span>THROTTLE</span><span id="targetV">0.00 V</span></label><strong id="actualV">0.00 V</strong><div class="bar"><i id="throttleBar" style="width:0%"></i></div></div><div class="metric"><label><span>NOMINAL SPEED</span><span>encoder-derived</span></label><strong id="speed">0.00 km/h</strong><div class="bar"><i id="speedBar" style="width:0%;background:var(--orange)"></i></div></div></div><dl class="facts"><dt>Mission state</dt><dd id="state">INITIALISING</dd><dt>Drive state</dt><dd id="driveState">—</dd><dt>Encoder</dt><dd id="encoder">—</dd><dt>Distance</dt><dd id="distance">0.000 m</dd><dt>Motor turns</dt><dd id="turns">0.000</dd><dt>Camera processing</dt><dd id="processing">—</dd></dl><div id="message" class="message">Preparing one-click mission</div></article></aside></section>
<section class="charts"><article class="panel"><div class="title">ENCODER RPM OVER TIME</div><div class="chart"><canvas id="rpmChart"></canvas></div><div class="legend"><span><i class="swatch" style="background:var(--blue)"></i>motor RPM</span><span><i class="swatch" style="background:var(--green2)"></i>axle RPM</span></div></article><article class="panel"><div class="title">THROTTLE OUTPUT OVER TIME</div><div class="chart"><canvas id="throttleChart"></canvas></div><div class="legend"><span><i class="swatch" style="background:var(--orange)"></i>actual SV</span><span><i class="swatch" style="background:#26322f"></i>target SV</span></div></article></section>
<article class="panel log"><div class="title">PER-SECOND RUN LOG</div><div class="tableWrap"><table><thead><tr><th>STATE</th><th>T S</th><th>DIST M</th><th>SPEED KM/H</th><th>MOTOR RPM</th><th>AXLE RPM</th><th>DAC V</th><th>ORANGE</th><th>BLUE</th><th>BLUE GAP M</th></tr></thead><tbody id="logBody"><tr><td colspan="10">Waiting for START</td></tr></tbody></table></div></article>
<div class="notice">WHEELS-OFF PRESENTATION TEST. START performs one atomic action: zero encoder → arm → Forward → 0.50 V. A single qualified blue-cone frame at ≤0.40 m commands full brake. The guard retries until the ESP32 confirms WHEEL LOCKED. STOP remains available as a manual operator command.</div></main>
<script>
const $=id=>document.getElementById(id),hist={rpm:[],axle:[],actual:[],target:[]},log=[];let latest=null,lastFrame=-1,runStart=null,lastLog=0;
const num=(v,n=2)=>Number.isFinite(Number(v))?Number(v).toFixed(n):'—';function push(a,v){a.push(Number(v)||0);if(a.length>240)a.shift()}function fit(c){const r=devicePixelRatio||1,w=c.clientWidth,h=c.clientHeight;if(c.width!==Math.round(w*r)||c.height!==Math.round(h*r)){c.width=Math.round(w*r);c.height=Math.round(h*r)}const x=c.getContext('2d');x.setTransform(r,0,0,r,0,0);return[x,w,h]}
async function action(name){try{const r=await fetch('/api/control',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({action:name})}),d=await r.json();$('message').textContent=d.message||d.error;if(!r.ok)throw Error(d.error||'Rejected');if(name==='start'){runStart=performance.now();log.length=0;$('logBody').innerHTML=''}}catch(e){$('message').textContent=String(e)}}$('start').onclick=()=>action('start');$('stop').onclick=()=>action('lock');
function lineChart(id,series,max){const[x,w,h]=fit($(id));x.clearRect(0,0,w,h);x.fillStyle='#e1e6e2';x.fillRect(0,0,w,h);x.strokeStyle='#c5cfca';x.lineWidth=1;for(let i=1;i<5;i++){const y=i*h/5;x.beginPath();x.moveTo(0,y);x.lineTo(w,y);x.stroke()}series.forEach(([a,color])=>{if(a.length<2)return;x.strokeStyle=color;x.lineWidth=2;x.beginPath();a.forEach((v,i)=>{const px=i*w/239,py=h-12-Math.min(1,Math.abs(v)/Math.max(1,max))*(h-24);i?x.lineTo(px,py):x.moveTo(px,py)});x.stroke()})}
function drawMap(d){const[x,w,h]=fit($('map')),ox=w/2,oy=h-34,scale=Math.min((h-70)/5,(w-90)/5.5),pt=c=>[ox-c.y*scale,oy-c.x*scale],cones=d.metric_cones||[],by=new Map(cones.map(c=>[c.id,c]));x.fillStyle='#e1e6e2';x.fillRect(0,0,w,h);x.font='11px ui-monospace';x.strokeStyle='#c5cfca';for(let m=0;m<=5;m++){const y=oy-m*scale;x.beginPath();x.moveTo(36,y);x.lineTo(w-18,y);x.stroke();x.fillStyle='#6a7773';x.fillText(m+' m',5,y-3)}x.fillStyle='#0d786516';x.beginPath();x.moveTo(ox,oy);x.lineTo(ox-2.2*scale,oy-4*scale);x.arc(ox,oy,4*scale,-2.07,-1.07);x.closePath();x.fill();for(const c of cones){const p=pt(c),g=x.createRadialGradient(p[0],p[1],2,p[0],p[1],45);g.addColorStop(0,c.label==='blue'?'#2868c790':'#e35d0b86');g.addColorStop(1,'#ffffff00');x.fillStyle=g;x.fillRect(p[0]-48,p[1]-48,96,96)}function edges(a,color,width,dash=[]){x.strokeStyle=color;x.lineWidth=width;x.setLineDash(dash);for(const e of a||[]){const A=by.get(e[0]),B=by.get(e[1]);if(!A||!B)continue;const p=pt(A),q=pt(B);x.beginPath();x.moveTo(...p);x.lineTo(...q);x.stroke()}x.setLineDash([])}edges(d.delaunay_edges,'#87948f',1);edges(d.gabriel_edges,'#7653a6',2,[6,4]);for(const c of cones){const p=pt(c);x.fillStyle=c.label==='blue'?'#2868c7':c.label.includes('orange')?'#e35d0b':'#cba400';x.beginPath();x.arc(...p,6,0,Math.PI*2);x.fill();x.fillStyle='#26322f';x.fillText('#'+c.id,p[0]+8,p[1]-7)}x.save();x.translate(ox,oy);x.fillStyle='#3f4b48';x.fillRect(-22,-32,44,30);x.fillStyle='#087865';x.fillRect(-3,-38,6,33);x.restore()}
function addLog(d,c,state){if(!runStart||performance.now()-lastLog<1000)return;lastLog=performance.now();log.unshift({state,t:(performance.now()-runStart)/1000,dist:c.distance_m,speed:c.speed_kmh,rpm:c.rpm,axle:c.axle_rpm,v:c.voltage,o:d.orange_count,b:d.blue_count,g:d.blue_gap_m});if(log.length>120)log.pop();$('logBody').innerHTML=log.map(r=>`<tr><td>${r.state}</td><td>${num(r.t,0)}</td><td>${num(r.dist,3)}</td><td>${num(r.speed,2)}</td><td>${num(r.rpm,0)}</td><td>${num(r.axle,0)}</td><td>${num(r.v,2)}</td><td>${r.o||0}</td><td>${r.b||0}</td><td>${r.g==null?'—':num(r.g,3)}</td></tr>`).join('')}
async function refresh(){try{const r=await fetch('/api/status',{cache:'no-store'}),d=await r.json(),c=d.controller||{};latest=d;const running=!!(c.armed&&!c.wheel_locked&&Number(c.target_voltage)>0),blueStop=!!d.guard_latched,state=blueStop?'STOPPED · BLUE CONE':running?'RUNNING · 0.50 V':d.camera_ok?'READY':'INITIALISING';$('mission').textContent=state;$('mission').className='status '+(running?'run':'stopState');$('start').disabled=!d.controls_enabled||!d.camera_ok||running;$('stop').disabled=!!c.wheel_locked;$('state').textContent=state;$('driveState').textContent=c.drive_state||'—';$('encoder').textContent=c.encoder_ready?'MAGNET OK':'CHECK '+(c.magnet||'OFFLINE');$('encoder').className=c.encoder_ready?'good':'bad';$('rpm').textContent=num(Math.abs(c.rpm),0);$('axle').textContent=num(Math.abs(c.axle_rpm),0);$('actualV').textContent=num(c.voltage)+' V';$('targetV').textContent=num(c.target_voltage)+' V';$('speed').textContent=num(c.speed_kmh)+' km/h';$('distance').textContent=num(c.distance_m,3)+' m';$('turns').textContent=num(c.motor_revs,3);$('processing').textContent=num(d.camera_fps,1)+' fps · '+num(d.processing_ms,0)+' ms';$('orangeCount').textContent=d.orange_count||0;$('blueCount').textContent=d.blue_count||0;$('edgeCount').textContent=(d.delaunay_edges||[]).length+' / '+(d.gabriel_edges||[]).length;$('blueGap').textContent=d.blue_gap_m==null?'—':num(d.blue_gap_m,3)+' m';$('rpmBar').style.width=Math.min(100,Math.abs(c.rpm||0)/20)+'%';$('axleBar').style.width=Math.min(100,Math.abs(c.axle_rpm||0)/8)+'%';$('throttleBar').style.width=Math.min(100,(c.voltage||0)/.5*100)+'%';$('speedBar').style.width=Math.min(100,(c.speed_kmh||0)/10*100)+'%';$('cameraOverlay').textContent='ORANGE '+(d.orange_count||0)+' · BLUE '+(d.blue_count||0)+(d.blue_gap_m==null?'':' · GAP '+num(d.blue_gap_m,2)+' m');$('cameraMeta').textContent=(d.summary||d.camera_error||'No detections')+' · '+num(d.camera_fps,1)+' Hz';if(d.frame_id!==lastFrame){lastFrame=d.frame_id;$('camera').src='/snapshot.jpg?f='+lastFrame}push(hist.rpm,c.rpm);push(hist.axle,c.axle_rpm);push(hist.actual,c.voltage);push(hist.target,c.target_voltage);drawMap(d);lineChart('rpmChart',[[hist.rpm,'#2868c7'],[hist.axle,'#15a187']],Math.max(100,...hist.rpm.map(Math.abs)));lineChart('throttleChart',[[hist.actual,'#e35d0b'],[hist.target,'#26322f']],.6);addLog(d,c,state)}catch(e){$('mission').textContent='DASHBOARD OFFLINE';$('message').textContent=String(e)}}setInterval(refresh,250);addEventListener('resize',()=>latest&&drawMap(latest));refresh();
</script></body></html>'''


class SharedState:
    def __init__(self, controls_enabled: bool):
        self.lock = threading.RLock()
        self.controller: dict = {}
        self.camera_ok = False
        self.camera_error: str | None = None
        self.guard_ready = False
        self.guard_latched = False
        self.blue_gap_m: float | None = None
        self.orange_count = 0
        self.blue_count = 0
        self.metric_cones: list[dict] = []
        self.delaunay_edges: list[list[int]] = []
        self.gabriel_edges: list[list[int]] = []
        self.graph_status = "WAITING"
        self.camera_fps = 0.0
        self.processing_ms = 0.0
        self.summary = "Camera starting"
        self.frame_id = 0
        self.jpeg: bytes | None = None
        self.controls_enabled = controls_enabled
        self.stop = threading.Event()

    def status(self) -> dict:
        with self.lock:
            return {"controller": dict(self.controller), "camera_ok": self.camera_ok,
                    "camera_error": self.camera_error, "guard_ready": self.guard_ready,
                    "guard_latched": self.guard_latched, "blue_gap_m": self.blue_gap_m,
                    "orange_count": self.orange_count, "blue_count": self.blue_count,
                    "metric_cones": list(self.metric_cones),
                    "delaunay_edges": list(self.delaunay_edges),
                    "gabriel_edges": list(self.gabriel_edges),
                    "graph_status": self.graph_status,
                    "camera_fps": self.camera_fps,
                    "processing_ms": self.processing_ms,
                    "summary": self.summary, "frame_id": self.frame_id,
                    "controls_enabled": self.controls_enabled}


class Controller:
    def __init__(self, base: str):
        self.base = base.rstrip("/")
        self.io_lock = threading.Lock()

    def get(self, path: str, timeout: float = 1.25) -> str:
        with self.io_lock, urlopen(self.base + path, timeout=timeout) as response:
            return response.read().decode("utf-8")

    def status(self) -> dict:
        return json.loads(self.get("/status"))

    def lock_and_verify(self) -> dict:
        reply = self.get("/lock")
        status = self.status()
        valid = (status.get("wheel_locked") and not status.get("armed") and
                 not status.get("running") and not status.get("brake_released") and
                 float(status.get("voltage", 1)) <= 0.01 and
                 float(status.get("target_voltage", 1)) <= 0.01)
        if not valid:
            raise RuntimeError("LOCK_VERIFICATION_FAILED: " + json.dumps(status, sort_keys=True))
        return {"reply": reply, "status": status}


def blue_cone_lock(state: SharedState, controller: Controller, reason: str) -> None:
    with state.lock:
        if state.guard_latched:
            return
        state.guard_latched = True
        state.guard_ready = False
        state.summary = reason + " — applying full brake"
    try:
        result = controller.lock_and_verify()
        with state.lock:
            state.controller = result["status"]
            state.summary = reason + " — FULL BRAKE VERIFIED"
        print(state.summary)
    except Exception as exc:
        with state.lock:
            # A network hiccup must never permanently suppress later brake
            # attempts.  Release the software latch so the next qualifying
            # frame retries /lock until the controller confirms safe state.
            state.guard_latched = False
            state.guard_ready = state.controls_enabled
            state.summary = reason + " — LOCK RETRY: " + str(exc)
        print(state.summary, file=sys.stderr)


def controller_worker(state: SharedState, controller: Controller) -> None:
    while not state.stop.is_set():
        try:
            status = controller.status()
            with state.lock:
                state.controller = status
        except Exception as exc:
            with state.lock:
                state.summary = "CONTROLLER LINK ERROR: " + str(exc)
        state.stop.wait(0.18)


def draw_frame(frame, detections, details, trigger_gap: float, effective_labels):
    import cv2
    image = frame.rgb.copy()
    colors = {"blue": (255, 150, 40), "yellow": (0, 220, 255),
              "orange": (0, 145, 255), "large_orange": (0, 90, 255)}
    for detection in detections:
        x1, y1, x2, y2 = detection.box
        effective_label = effective_labels.get(detection.id, detection.label)
        color = colors.get(effective_label, (180, 180, 180))
        cv2.rectangle(image, (x1, y1), (x2, y2), color, 2)
        detail = details.get(detection.id, {})
        gap = detail.get("gap")
        label = f"{effective_label} {detection.confidence:.0%}"
        if gap is not None:
            label += f" | gap {gap:.2f}m"
        cv2.rectangle(image, (x1, max(0, y1 - 23)),
                      (min(image.shape[1] - 1, x1 + 245), y1), (5, 14, 23), -1)
        cv2.putText(image, label, (x1 + 4, max(15, y1 - 6)),
                    cv2.FONT_HERSHEY_SIMPLEX, .48, (255, 255, 255), 1, cv2.LINE_AA)
    cv2.putText(image, f"BLUE LOCK <= {trigger_gap:.2f}m bumper gap", (12, 464),
                cv2.FONT_HERSHEY_SIMPLEX, .58, (70, 80, 255), 2, cv2.LINE_AA)
    ok, encoded = cv2.imencode(".jpg", image, [int(cv2.IMWRITE_JPEG_QUALITY), 82])
    if not ok:
        raise RuntimeError("JPEG_ENCODE_FAILED")
    return encoded.tobytes()


def camera_worker(state: SharedState, controller: Controller, args) -> None:
    import cv2
    import numpy as np
    import yaml
    from autokart_perception.camera import CameraError, HP60C
    from autokart_perception.depth import localize_diagnostic
    from autokart_perception.gabriel import filter_gabriel
    from autokart_perception.geometry import triangulate
    from autokart_perception.live_mission import live_detector
    from autokart_perception.mount import mount_from_config

    config_path = PERCEPTION / "autokart_perception/data/perception.yaml"
    config = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    config = {**config, "camera_to_kart": None,
              "camera_mount": {**config["camera_mount"],
                               "pitch_down_deg": args.pitch_deg}}
    mount, mount_status = mount_from_config(config)
    if mount_status != "MOUNT_CONFIGURED_UNVERIFIED":
        with state.lock:
            state.camera_error = "CAMERA MOUNT UNAVAILABLE: " + mount_status
        return
    detector = live_detector("yolo26s")
    # Keep the camera feed useful for track observation: orange, yellow and
    # blue cones may all be displayed at this lower threshold.  The automatic
    # brake gate below remains independently restricted to high-confidence
    # BLUE detections only.
    detector.confidence = args.display_confidence
    bumper_offset = float(GEOMETRY.camera_to_front_bumper_horizontal_m)
    corridor = float(GEOMETRY.bumper_width_m) / 2 + float(GEOMETRY.lateral_margin_m)
    confirmations = deque(maxlen=args.confirmations)
    warmup_deadline = time.monotonic() + 10.0
    previous_frame_at = None
    try:
        with HP60C() as camera:
            while not state.stop.is_set():
                cycle_started = time.monotonic()
                try:
                    frame = camera.read()
                except CameraError as exc:
                    if time.monotonic() < warmup_deadline:
                        with state.lock:
                            state.camera_error = "CAMERA WARM-UP: " + str(exc)
                        continue
                    raise
                detections = detector.detect(frame.rgb)
                details = {}
                blocking = []
                metric_cones = []
                effective_labels = {}
                for detection in detections:
                    # YOLO can occasionally call a dark-blue cone orange.  Use
                    # only strongly saturated blue pixels inside the detected
                    # cone box as a conservative second opinion.  This never
                    # turns an arbitrary image region into a cone: a YOLO cone
                    # detection is still mandatory.
                    x1, y1, x2, y2 = detection.box
                    roi = frame.rgb[max(0, y1):max(0, y2), max(0, x1):max(0, x2)]
                    effective_label = detection.label
                    if roi.size and detection.label != "blue":
                        # HP60C Frame.rgb is BGR8 despite the historical field
                        # name (the shared preview code documents INVALID_BGR8_RGB).
                        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
                        blue_pixels = cv2.inRange(
                            hsv, np.array((90, 70, 25), dtype=np.uint8),
                            np.array((140, 255, 255), dtype=np.uint8)
                        )
                        coloured = cv2.inRange(
                            hsv, np.array((0, 55, 25), dtype=np.uint8),
                            np.array((179, 255, 255), dtype=np.uint8)
                        )
                        coloured_count = int(cv2.countNonZero(coloured))
                        blue_fraction = (int(cv2.countNonZero(blue_pixels)) /
                                         max(1, coloured_count))
                        if coloured_count >= 40 and blue_fraction >= 0.55:
                            effective_label = "blue"
                    effective_labels[detection.id] = effective_label
                    cone, detail = localize_diagnostic(frame, detection, mount, config)
                    if cone is None or detail.get("localization_status") != "METRIC_OK":
                        continue
                    x, y = map(float, cone.xy)
                    metric_cones.append(cone)
                    gap = x - bumper_offset
                    details[detection.id] = {"x": x, "y": y, "gap": gap,
                                             "metric": True}
                    if (effective_label == "blue" and
                            detection.confidence >= args.confidence and
                            x > bumper_offset and abs(y) <= corridor):
                        blocking.append((gap, y, float(detection.confidence)))
                graph = triangulate(metric_cones)
                gabriel = filter_gabriel(graph)
                candidate = min(blocking) if blocking else None
                if candidate:
                    confirmations.append(candidate[0])
                else:
                    confirmations.clear()
                jpeg = draw_frame(frame, detections, details, args.trigger_gap,
                                  effective_labels)
                now = time.monotonic()
                instantaneous_fps = 0.0 if previous_frame_at is None else 1.0 / max(
                    0.001, now - previous_frame_at
                )
                previous_frame_at = now
                with state.lock:
                    state.camera_ok = True
                    state.camera_error = None
                    state.guard_ready = (state.controls_enabled and not state.guard_latched)
                    state.blue_gap_m = candidate[0] if candidate else None
                    state.orange_count = sum(
                        effective_labels.get(detection.id, detection.label)
                        in ("orange", "large_orange")
                        for detection in detections
                    )
                    state.blue_count = sum(
                        effective_labels.get(detection.id, detection.label) == "blue"
                        for detection in detections
                    )
                    state.metric_cones = [
                        {"id": cone.id,
                         "label": effective_labels.get(cone.id, cone.label),
                         "x": float(cone.xy[0]), "y": float(cone.xy[1]),
                         "confidence": float(cone.confidence)}
                        for cone in metric_cones
                    ]
                    state.delaunay_edges = [list(edge) for edge in graph.edges]
                    state.gabriel_edges = [list(edge) for edge in gabriel]
                    state.graph_status = graph.status
                    state.camera_fps = instantaneous_fps
                    state.processing_ms = (time.monotonic() - cycle_started) * 1000.0
                    state.frame_id += 1
                    state.jpeg = jpeg
                    state.summary = (f"{len(detections)} cone detections"
                                     if candidate is None else
                                     f"BLUE gap={candidate[0]:.3f} m y={candidate[1]:+.3f} m "
                                     f"confidence={candidate[2]:.2f}")
                if (state.controls_enabled and len(confirmations) == args.confirmations and
                        statistics.median(confirmations) <= args.trigger_gap):
                    blue_cone_lock(state, controller,
                                   f"BLUE CONE {statistics.median(confirmations):.3f} m")
                    confirmations.clear()
    except Exception as exc:
        with state.lock:
            state.camera_ok = False
            state.guard_ready = False
            state.camera_error = f"{type(exc).__name__}: {exc}"


def make_handler(state: SharedState, controller: Controller):
    allowed = {"arm": "/arm", "disarm": "/disarm", "lock": "/lock",
               "forward": "/drive?d=FORWARD", "reverse": "/drive?d=REVERSE",
               "zero": "/zero"}

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *_):
            pass

        def send_body(self, code: int, body: bytes, content_type: str):
            self.send_response(code)
            self.send_header("Content-Type", content_type)
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def send_json(self, code: int, body: dict):
            self.send_body(code, json.dumps(body).encode(),
                           "application/json; charset=utf-8")

        def do_GET(self):
            path = urlsplit(self.path).path
            if path == "/":
                self.send_body(200, HTML.encode(), "text/html; charset=utf-8")
            elif path == "/api/status":
                self.send_json(200, state.status())
            elif path == "/snapshot.jpg":
                with state.lock:
                    jpeg = state.jpeg
                if jpeg is None:
                    self.send_json(503, {"error": "NO CAMERA FRAME"})
                else:
                    self.send_body(200, jpeg, "image/jpeg")
            else:
                self.send_json(404, {"error": "NOT FOUND"})

        def do_POST(self):
            if urlsplit(self.path).path != "/api/control":
                self.send_json(404, {"error": "NOT FOUND"})
                return
            try:
                length = int(self.headers.get("Content-Length", "0"))
                payload = json.loads(self.rfile.read(length) or b"{}")
                action, arg = payload.get("action"), payload.get("arg")
                with state.lock:
                    ready = (state.controls_enabled and state.camera_ok and
                             not state.guard_latched)
                    status = dict(state.controller)
                if action == "reset_guard":
                    if not status.get("wheel_locked"):
                        raise RuntimeError("LOCK WHEEL BEFORE RESETTING GUARD")
                    with state.lock:
                        state.guard_latched = False
                        state.guard_ready = state.controls_enabled and state.camera_ok
                        state.summary = "BLUE GUARD RESET"
                    self.send_json(200, {"message": "BLUE GUARD RESET"})
                    return
                if action == "start":
                    if not state.controls_enabled or not state.camera_ok:
                        raise RuntimeError("CAMERA/BLUE GUARD NOT READY")
                    if not status.get("wheel_locked") and not status.get("armed"):
                        raise RuntimeError("CONTROLLER IS NOT IN A KNOWN SAFE START STATE")
                    with state.lock:
                        state.guard_latched = False
                        state.guard_ready = True
                        state.summary = "START REQUESTED — FORWARD 0.50 V"
                    reply = controller.get("/test_start")
                    self.send_json(200, {"message": reply})
                    return
                if action == "lock":
                    result = controller.lock_and_verify()
                    with state.lock:
                        state.controller = result["status"]
                    self.send_json(200, {"message": "FULL BRAKE VERIFIED"})
                    return
                if action not in allowed and action != "gear":
                    raise RuntimeError("UNKNOWN ACTION")
                if action in ("arm", "forward", "reverse", "gear") and not ready:
                    raise RuntimeError("BLUE GUARD/CAMERA NOT READY")
                if action == "gear":
                    gear = int(arg)
                    if gear < 0 or gear > 5:
                        raise RuntimeError("GEAR MUST BE 0..5")
                    path = "/gear?n=" + str(gear)
                else:
                    path = allowed[action]
                reply = controller.get(path)
                self.send_json(200, {"message": reply})
            except Exception as exc:
                self.send_json(409, {"error": str(exc)})

    return Handler


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--controller", default="http://192.168.4.1")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8781)
    parser.add_argument("--trigger-gap", type=float, default=0.40)
    parser.add_argument("--pitch-deg", type=float, default=16.37)
    parser.add_argument("--confidence", type=float, default=0.80)
    parser.add_argument("--display-confidence", type=float, default=0.35)
    parser.add_argument("--confirmations", type=int, default=1)
    parser.add_argument("--enable-lock", action="store_true")
    parser.add_argument("--presentation-mode", action="store_true",
                        help="enable the one-click UI without a terminal prompt; wheels-off only")
    args = parser.parse_args()
    if not (0.35 <= args.trigger_gap <= 2.0):
        parser.error("trigger gap must be 0.35..2.0 m")
    if (not (0.5 <= args.confidence <= 1.0) or
            not (0.1 <= args.display_confidence <= args.confidence) or
            not (1 <= args.confirmations <= 5)):
        parser.error("invalid confidence or confirmations")

    controller = Controller(args.controller)
    initial = controller.status()
    if not initial.get("gp_online") or not initial.get("encoder_ready"):
        raise SystemExit("NO-GO: GP8630N or AS5047P unhealthy: " + json.dumps(initial))
    controls_enabled = args.enable_lock or args.presentation_mode
    if controls_enabled:
        if not initial.get("wheel_locked"):
            raise SystemExit("NO-GO: controller must start wheel-locked")
        if not args.presentation_mode:
            phrase = input("Kart jacked, wheels clear, restrained, power cut ready. Type WHEELS-OFF: ").strip()
            if phrase != "WHEELS-OFF":
                raise SystemExit("Motor controls not enabled")

    state = SharedState(controls_enabled)
    with state.lock:
        state.controller = initial
    controller_thread = threading.Thread(target=controller_worker,
                                         args=(state, controller), daemon=True)
    camera_thread = threading.Thread(target=camera_worker,
                                     args=(state, controller, args), daemon=True)
    controller_thread.start()
    camera_thread.start()
    server = ThreadingHTTPServer((args.host, args.port), make_handler(state, controller))
    print(f"Dashboard: http://127.0.0.1:{args.port}/")
    print(f"Blue-only full-brake threshold: {args.trigger_gap:.2f} m bumper gap")
    print(f"Display confidence >= {args.display_confidence:.2f}; "
          f"blue brake confidence >= {args.confidence:.2f}")
    print("Motor controls:", "ENABLED (WHEELS-OFF)" if controls_enabled else "DISABLED")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nStopping dashboard")
    finally:
        state.stop.set()
        server.server_close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
