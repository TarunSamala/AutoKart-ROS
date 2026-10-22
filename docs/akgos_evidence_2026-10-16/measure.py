import json,time,urllib.request,sys
vals={k:[] for k in ('capture_to_gateway_ms','processing_ms','inference_ms','frame_age_ms','receive_fps')};stale=0;n=0;latched=set();d={}
for _ in range(int(sys.argv[1])):
    try: d=json.load(urllib.request.urlopen('http://localhost:18790/mission/api/camera'))
    except Exception: time.sleep(.5); continue
    n+=1
    if d.get('available'):
        for k in vals:
            if isinstance(d.get(k),(int,float)): vals[k].append(d[k])
        stale+='FRAME_TOO_OLD' in d.get('reason_codes',[]); latched.add(d.get('latched_fault'))
    time.sleep(0.5)
q=lambda a,p: round(sorted(a)[min(len(a)-1,int(p*len(a)))],1) if a else None
print(json.dumps({'samples':n,'backend':d.get('model_backend'),'latched_values':sorted(map(str,latched)),'frame_too_old_share':round(stale/max(n,1),2),**{k:{'p50':q(v,.5),'p95':q(v,.95)} for k,v in vals.items()}}))
