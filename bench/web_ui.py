"""Offline browser dashboard served by main.py (no CDN dependencies)."""
PAGE = r'''<!DOCTYPE html>
<html lang="en"><head><meta charset="UTF-8"/>
<meta name="viewport" content="width=device-width,initial-scale=1,maximum-scale=1,user-scalable=no">
<meta name="theme-color" content="#0a1019"><title>AutoKart RC · R86mini</title>
<style>
:root{--bg:#080c14;--panel:#141c29;--panel2:#182338;--line:#2c3a50;--text:#f3f7ff;--muted:#9cacc2;--cyan:#62caff;--green:#27cc8e;--red:#ec4156;--amber:#f6bd56}
*{box-sizing:border-box;-webkit-tap-highlight-color:transparent}html,body{margin:0;min-height:100%;background:radial-gradient(ellipse at top,#13233a,#080c14 60%);color:var(--text);font-family:system-ui,Segoe UI,Arial,sans-serif;overscroll-behavior:none}body{padding:12px}button,input{font:inherit}button{cursor:pointer;touch-action:none;user-select:none}button:disabled{opacity:.4;cursor:not-allowed}
.wrap{max-width:1120px;margin:auto}.header{display:flex;justify-content:space-between;gap:10px;flex-wrap:wrap;align-items:center;padding:10px 0 17px}.brand{font-weight:900;font-size:27px;letter-spacing:.3px}.tagline{font-size:11px;letter-spacing:1.4px;color:var(--muted);margin-top:3px}.badge{display:inline-flex;align-items:center;border:1px solid var(--line);background:#162033;border-radius:50px;padding:9px 13px;font-size:11px;font-weight:800;margin:2px}.ok{color:#6df4bf}.warn{color:#ffcf7b}.danger{color:#ff7185}
.banner{padding:11px 15px;border:1px solid #765530;border-radius:12px;background:#36291b;color:#ffdfa7;font-size:12px;margin-bottom:12px}.grid{display:grid;grid-template-columns:1.2fr 1.2fr .8fr;gap:11px}.box{border:1px solid var(--line);border-radius:17px;background:linear-gradient(145deg,var(--panel2),var(--panel));padding:16px;box-shadow:0 7px 20px #0004;min-width:0}.title{font-size:12px;font-weight:800;letter-spacing:1.5px;color:var(--muted);text-transform:uppercase;margin-bottom:13px}.split{display:grid;grid-template-columns:1fr 1fr;gap:9px}.steer{height:152px;background:#293851;border:1px solid #405674;color:white;border-radius:14px;font-size:58px}.steer:active,.steer.active{background:#49648c}.center{height:51px;width:100%;margin-top:11px;background:#344660;border:0;color:white;border-radius:12px;font-weight:900}.green{height:152px;border:0;border-radius:14px;background:linear-gradient(180deg,#29ca89,#116d4a);color:white;font-size:22px;font-weight:900}.red{height:152px;border:0;border-radius:14px;background:linear-gradient(180deg,#e54f5e,#971a2a);color:white;font-size:22px;font-weight:900}.green.active{filter:brightness(1.2)}.green,.red{white-space:normal} .slider{width:100%;accent-color:#33d3a4}.sliderbox{background:#0e1623;border:1px solid var(--line);padding:12px;border-radius:13px;margin-top:12px}.row{display:flex;justify-content:space-between;align-items:center;gap:12px}.small{font-size:11px;color:var(--muted)}.value{font-weight:900;font-size:19px}.mini{border:0;background:#304059;border-radius:10px;color:white;min-height:44px;padding:7px 8px;font-weight:800}.mini.green-mini{background:#0d835a}.mini.rose{background:#713343}.controls{display:grid;grid-template-columns:1fr 1fr;gap:8px}.controls .wide{grid-column:1/-1}.info{display:grid;grid-template-columns:repeat(4,1fr);gap:8px;margin-top:12px}.metric{background:#0c1321;border:1px solid var(--line);padding:12px;border-radius:12px}.k{color:var(--muted);font-size:10px;font-weight:800;letter-spacing:.9px}.v{font-size:18px;font-weight:900;margin-top:4px}.stop{width:100%;height:78px;border-radius:17px;background:linear-gradient(180deg,#fd3b55,#c10c24);border:3px solid #ff9ea8;color:white;font-size:28px;font-weight:1000;margin-top:13px;letter-spacing:1px}.stop:active{background:red}.foot{font-size:11px;color:var(--muted);text-align:center;margin:12px auto}.overlay{position:fixed;inset:0;z-index:99;background:#05050aee;display:none;justify-content:center;align-items:center;padding:18px}.overlay.show{display:flex}.dialog{border:2px solid var(--red);background:#24141d;padding:26px;border-radius:22px;max-width:460px;width:100%;text-align:center}.dialog h2{color:#ff7387;font-size:27px;margin:0}.dialog p{font-size:13px;line-height:1.5;margin:19px auto}.dialog button{width:100%;height:55px;border:0;border-radius:12px;background:#44536c;color:white;font-weight:900}.linkfail{display:none;color:white;background:#a8293c;position:fixed;top:8px;left:50%;transform:translateX(-50%);padding:10px 15px;border-radius:50px;z-index:80;font-size:12px}.linkfail.show{display:block}
@media(max-width:950px){.grid{grid-template-columns:1fr 1fr}.box.system{grid-column:1/-1}}@media(max-width:570px){body{padding:8px}.brand{font-size:21px}.tagline{font-size:9px}.grid{gap:7px}.box{padding:10px;border-radius:12px}.steer,.green,.red{height:115px;font-size:36px}.green,.red{font-size:17px}.info{grid-template-columns:repeat(2,1fr)}.stop{height:68px;font-size:22px}.small{font-size:9px}.value{font-size:16px}}
</style></head><body>
<div id="lost" class="linkfail">RC LINK LOST — CONTROL MAY BE STOPPED</div>
<div class="wrap">
 <div class="header"><div><div class="brand">AutoKart <span style="color:var(--cyan)">RC</span></div><div class="tagline">ESP32-S3 · R86mini 400 PPR · SW5–SW8 ON · TB6600 BRAKE</div></div>
 <div><span id="stateBadge" class="badge warn">BOOT</span><span id="ctrlBadge" class="badge warn">VIEW ONLY</span></div></div>
 <div class="banner">BENCH PROTOTYPE · A separate, tested hardwired stop remains mandatory. This WEB E-STOP cannot stop a crashed MCU. Before first ARM: physically center steering, fully release mechanical brake, then confirm reference below.</div>
 <div class="grid">
  <section class="box"><div class="title">Steering / R86mini</div>
   <div class="split"><button id="left" class="steer">◀</button><button id="right" class="steer">▶</button></div>
   <button id="center" class="center">CENTER STEERING</button>
  </section>
  <section class="box"><div class="title">Traction / Mechanical brake</div>
   <div class="split"><button id="forward" class="green">HOLD<br>FORWARD</button><button id="brake" class="red">BRAKE<br>NOW</button></div>
   <div class="sliderbox"><div class="row"><span class="small">DRIVE DAC COMMAND</span><span class="value" id="speedValue">0.40 V</span></div>
    <input id="speed" class="slider" type="range" min="0" max="70" step="1" value="40"></div>
  </section>
  <section class="box system"><div class="title">System</div>
   <div class="controls"><button class="mini green-mini" id="arm">ARM</button><button class="mini rose" id="disarm">DISARM</button>
   <button class="mini wide" id="claim">TAKE / RECLAIM CONTROL</button>
   <button class="mini wide" id="reference">CONFIRM STEER CENTER + BRAKE RELEASED</button></div>
   <div class="small" style="margin-top:12px;line-height:1.4">Reference confirmation commands the mechanical brake to APPLY. Keep hands clear. Disable traction power / lift driven wheels for first test.</div>
  </section>
 </div>
 <div class="info">
  <div class="metric"><div class="k">STATE</div><div class="v" id="state">BOOT</div></div>
  <div class="metric"><div class="k">STEER EST</div><div class="v" id="steerValue">0.0°</div></div>
  <div class="metric"><div class="k">BRAKE EST</div><div class="v" id="brakeValue">0.0°</div></div>
  <div class="metric"><div class="k">DAC CMD</div><div class="v" id="dacValue">0.00 V</div></div>
 </div>
 <button id="estop" class="stop">WEB E-STOP</button>
 <div class="foot">FORWARD release → DAC 0 V (coast). BRAKE NOW → BLD STOP + BLD BRAKE + stepper brake. RC heartbeat or FORWARD hold loss → latched stop.</div>
</div>
<div id="overlay" class="overlay"><div class="dialog"><h2>WEB STOP LATCHED</h2><p id="reason">Traction command disabled; software brake commanded.</p><button id="reset">RESET STOP (DOES NOT ARM)</button></div></div>
<script>
'use strict';
const $=s=>document.getElementById(s);
const BASE='/';
let armed=false,stopped=false,referenced=false,control=false;
let controlToken=localStorage.getItem('autokart_rc_token')||'';
let steerSeq=0,heldSteer=0,steerLong=false,steerTimer=null,forwardHeld=false,keepTimer=null;
let polling=false,errors=0;
function call(path){return fetch(path,{cache:'no-store'}).then(async r=>({ok:r.ok,status:r.status,text:await r.text()}));}
function withToken(path){return path+(path.includes('?')?'&':'?')+'token='+encodeURIComponent(controlToken);}
async function cmd(path){if(!control||!controlToken)throw Error('NO_CONTROL');
 let r=await call(withToken(path));if(r.status===403){setControl(false);throw Error('LEASE_EXPIRED');}if(!r.ok)throw Error(r.text);return r.text;}
function setControl(b){control=!!b;$('ctrlBadge').textContent=b?'CONTROL OWNER':'VIEW ONLY';$('ctrlBadge').className='badge '+(b?'ok':'warn');
 for(let id of ['left','right','center','forward','brake','speed','arm','disarm','reference','reset']){$(id).disabled=!b;}}
async function claim(){try{let p='/claim'+(controlToken?'?token='+encodeURIComponent(controlToken):'');let r=await call(p);
 if(r.ok&&r.text.startsWith('TOKEN:')){controlToken=r.text.slice(6);localStorage.setItem('autokart_rc_token',controlToken);setControl(true);return;}}
 catch(e){}setControl(false);}
function guarded(path){cmd(path).catch(e=>{if(!String(e).includes('LEASE_EXPIRED'))console.warn(path,e);});}
function steer(path){steerSeq++;guarded(path+(path.includes('?')?'&':'?')+'seq='+steerSeq);}
function beginSteer(e,dir){if(!control||!armed||stopped||heldSteer)return;e.preventDefault();heldSteer=dir;steerLong=false;
 let b=dir<0?$('left'):$('right');b.classList.add('active');try{b.setPointerCapture(e.pointerId);}catch(_e){}
 steer('/steer_nudge?v='+(dir*3));clearTimeout(steerTimer);
 steerTimer=setTimeout(()=>{if(heldSteer===dir&&armed&&!stopped){steerLong=true;steer(dir<0?'/steer_left':'/steer_right');}},140);}
function endSteer(dir){if(heldSteer!==dir)return;clearTimeout(steerTimer);$('left').classList.remove('active');$('right').classList.remove('active');
 if(steerLong&&control)steer('/steer_hold');heldSteer=0;steerLong=false;}
$('left').addEventListener('pointerdown',e=>beginSteer(e,-1));$('right').addEventListener('pointerdown',e=>beginSteer(e,1));
for(let [b,d] of [['left',-1],['right',1]])for(let name of ['pointerup','pointercancel','lostpointercapture'])$(b).addEventListener(name,()=>endSteer(d));
$('center').addEventListener('click',()=>{if(armed&&!stopped&&control){heldSteer=0;steer('/steer_center');}});
function endForward(){if(!forwardHeld)return;forwardHeld=false;$('forward').classList.remove('active');clearInterval(keepTimer);keepTimer=null;
 if(control)guarded('/drive_off');}
$('forward').addEventListener('pointerdown',e=>{if(!control||!armed||stopped||forwardHeld)return;e.preventDefault();forwardHeld=true;
 $('forward').classList.add('active');try{$('forward').setPointerCapture(e.pointerId);}catch(_e){}
 guarded('/drive_on');keepTimer=setInterval(()=>{if(forwardHeld&&control)guarded('/drive_keep');},140);});
for(let name of ['pointerup','pointercancel','lostpointercapture'])$('forward').addEventListener(name,endForward);
$('brake').addEventListener('pointerdown',e=>{if(!control||stopped)return;e.preventDefault();endForward();guarded('/brake_now');});
let speedWait=null;$('speed').addEventListener('input',()=>{let v=Number($('speed').value)/100;$('speedValue').textContent=v.toFixed(2)+' V';
 clearTimeout(speedWait);speedWait=setTimeout(()=>{if(control&&armed)guarded('/speed_set?v='+v.toFixed(2));},110);});
$('arm').onclick=()=>guarded('/arm');$('disarm').onclick=()=>{endForward();guarded('/disarm');};
$('claim').onclick=claim;
$('reference').onclick=()=>{if(confirm('Have you PHYSICALLY CENTERED steering and FULLY RELEASED the mechanical brake? Wheels lifted, hands clear? Brake motor will APPLY after confirmation.'))guarded('/reference');};
$('estop').onclick=()=>{endForward();stopped=true;$('overlay').classList.add('show');call('/estop').catch(()=>{});};
$('reset').onclick=()=>guarded('/estop_reset');
async function poll(){if(polling)return;polling=true;
 try{let r=await(control?cmd('/heartbeat').then(text=>({ok:true,text})):call('/status'));
 if(!r.ok)throw Error('STATUS '+r.status);let a=r.text.split('|');if(a.length!==10)throw Error('BAD STATUS');errors=0;$('lost').classList.remove('show');
 armed=a[1]==='1';stopped=a[2]==='1';referenced=a[3]==='1';let state=a[0];
 $('state').textContent=state;$('steerValue').textContent=Number(a[4]).toFixed(1)+'°';$('brakeValue').textContent=Number(a[5]).toFixed(1)+'°';
 $('dacValue').textContent=Number(a[6]).toFixed(2)+' V';
 if(document.activeElement!==$('speed')){$('speed').value=Math.round(Number(a[7])*100);$('speedValue').textContent=Number(a[7]).toFixed(2)+' V';}
 $('stateBadge').textContent=state;$('stateBadge').className='badge '+(stopped?'danger':armed?'ok':'warn');
 if(stopped){$('overlay').classList.add('show');$('reason').textContent=a[9]+' · Stop remains latched until manually reset. Reset will NOT re-arm.';}
 else $('overlay').classList.remove('show');
 if((!armed||stopped)&&forwardHeld){forwardHeld=false;clearInterval(keepTimer);keepTimer=null;$('forward').classList.remove('active');}
 }catch(e){errors++;if(errors>=3)$('lost').classList.add('show');if(String(e).includes('LEASE_EXPIRED')){setControl(false);}}
 finally{polling=false;setTimeout(poll,250);}}
document.addEventListener('visibilitychange',()=>{if(document.hidden){endForward();if(heldSteer)endSteer(heldSteer);if(control)guarded('/disarm');}});
window.addEventListener('pagehide',()=>{if(control&&controlToken)try{fetch(withToken('/disarm'),{keepalive:true});}catch(_e){}});
setControl(false);claim();poll();
</script></body></html>'''
