"""Offline, session-aware evaluation of recorded HP60C cone images.

Annotations are manual ground truth. A null frame is unreviewed and excluded;
an empty list is a reviewed frame with no cones. Predictions are always rerun
from the original recorded RGB array, never from saved detector outputs.
"""
import argparse
import json
import statistics
from pathlib import Path

import cv2
import numpy as np

from .detector import ConeDetector, Yolo11OnnxDetector, MODEL_SHA256

ANNOTATOR_HTML='''<!doctype html><html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1"><title>HP60C cone annotations</title>
<style>body{font:16px system-ui;background:#101820;color:#eaf2fa;margin:20px;max-width:1000px}
button,input,select{font:inherit;margin:4px;padding:7px}canvas{max-width:100%;border:1px solid #677;touch-action:none}
small{color:#abc}</style></head><body><h1>HP60C cone annotations</h1>
<p>Drag around every visible cone and choose its observed class. Enter measured distance and surveyed X/Y before drawing when available.
Mark a frame empty if you checked it and found no cones. Unreviewed frames stay excluded from evaluation.</p>
<div><button id="prev">Previous</button><button id="next">Next</button><button id="undo">Undo last box</button>
<button id="empty">Mark empty</button><button id="save">Download annotations</button></div>
<div>Class <select id="label"><option value="orange">Orange</option><option value="blue">Blue</option>
<option value="yellow">Yellow</option><option value="large_orange">Large orange</option><option value="unknown">Unknown</option></select>
Distance m <input id="distance" type="number" step="0.01" min="0" placeholder="optional">
Survey X m <input id="x" type="number" step="0.01" placeholder="optional">
Survey Y m <input id="y" type="number" step="0.01" placeholder="optional"></div>
<div>Difficulty tags (comma separated: small, distant, occluded, low_light)
<input id="tags" placeholder="optional"></div>
<p id="state"></p><canvas id="view" width="640" height="480"></canvas>
<p><small>Original 640×480 frames. Boxes use pixel coordinates. Download a new JSON after every labeling session; this page does not overwrite recordings.</small></p>
<script>const doc=__LABELS__;const keys=Object.entries(doc.sessions).flatMap(([s,frames])=>Object.keys(frames).map(f=>[s,f]));
let index=0,start=null;const canvas=document.querySelector('#view'),ctx=canvas.getContext('2d'),image=new Image();
const get=()=>{const [s,f]=keys[index];return doc.sessions[s][f]};
function draw(){ctx.clearRect(0,0,640,480);if(image.complete&&image.naturalWidth)ctx.drawImage(image,0,0,640,480);
 const boxes=get();if(Array.isArray(boxes))for(const item of boxes){const [x1,y1,x2,y2]=item.box;
 ctx.strokeStyle='#ff8e3b';ctx.lineWidth=2;ctx.strokeRect(x1,y1,x2-x1,y2-y1)}
 const [s,f]=keys[index];document.querySelector('#state').textContent=`${index+1}/${keys.length} · ${s}/${f} · ${boxes===null?'UNREVIEWED':boxes.length+' cones'}`}
function open(){const [s,f]=keys[index];image.onload=draw;image.src=`${encodeURIComponent(s)}/${encodeURIComponent(f)}.jpg`;draw()}
function pos(e){const r=canvas.getBoundingClientRect();return [Math.max(0,Math.min(640,Math.round((e.clientX-r.left)*640/r.width))),
 Math.max(0,Math.min(480,Math.round((e.clientY-r.top)*480/r.height)))]}
canvas.onpointerdown=e=>{start=pos(e);canvas.setPointerCapture(e.pointerId)};
canvas.onpointerup=e=>{if(!start)return;const end=pos(e),box=[Math.min(start[0],end[0]),Math.min(start[1],end[1]),Math.max(start[0],end[0]),Math.max(start[1],end[1])];start=null;
 if(box[2]-box[0]<4||box[3]-box[1]<4)return;const [s,f]=keys[index];if(doc.sessions[s][f]===null)doc.sessions[s][f]=[];
 const item={box,label:document.querySelector('#label').value},distance=document.querySelector('#distance').value,x=document.querySelector('#x').value,y=document.querySelector('#y').value;
 if(distance)item.distance_m=Number(distance);if(x&&y)item.xy_m=[Number(x),Number(y)];
 const tags=document.querySelector('#tags').value.split(',').map(v=>v.trim()).filter(Boolean);
 if(tags.length)item.difficulty=tags;doc.sessions[s][f].push(item);draw()};
// Difficulty tags describe observed image conditions; measured distance remains separate.
document.querySelector('#prev').onclick=()=>{index=Math.max(0,index-1);open()};
document.querySelector('#next').onclick=()=>{index=Math.min(keys.length-1,index+1);open()};
document.querySelector('#undo').onclick=()=>{const boxes=get();if(Array.isArray(boxes)&&boxes.length)boxes.pop();draw()};
document.querySelector('#empty').onclick=()=>{const [s,f]=keys[index];doc.sessions[s][f]=[];draw()};
document.querySelector('#save').onclick=()=>{const blob=new Blob([JSON.stringify(doc,null,2)],{type:'application/json'});
 const a=document.createElement('a');a.href=URL.createObjectURL(blob);a.download='hp60c-annotations-reviewed.json';a.click();setTimeout(()=>URL.revokeObjectURL(a.href),1000)};open();
</script></body></html>'''


def recording_files(paths):
    sessions={}
    for path in map(Path,paths):
        files=sorted(path.glob('*.npz'))
        if not files:raise ValueError(f'RECORDING_EMPTY:{path}')
        if path.name in sessions:raise ValueError(f'DUPLICATE_SESSION_NAME:{path.name}')
        sessions[path.name]={p.stem:p for p in files}
    return sessions


def verify_heldout_sessions(audit, session_names):
    if audit.get('status') not in ('READY_FOR_TRAINING','EXPERIMENTAL_READY'):
        raise ValueError('TRAINING_AUDIT_NOT_READY')
    sessions=audit.get('sessions_by_split',{})
    if not isinstance(sessions,dict) or not all(k in sessions for k in ('train','val','test')):
        raise ValueError('TRAINING_SESSION_PROVENANCE_MISSING')
    if set(session_names) & (set(sessions['train'])|set(sessions['val'])):
        raise ValueError('HP60C_SESSION_LEAKAGE')
    return True


def make_template(sessions, output):
    output=Path(output)
    if output.exists():raise FileExistsError(output)
    images=output.with_suffix('')
    if images.exists():raise FileExistsError(images)
    images.mkdir(parents=True)
    labels={'schema_version':1,'sessions':{}}
    for session,files in sessions.items():
        folder=images/session;folder.mkdir()
        labels['sessions'][session]={}
        for stem,path in files.items():
            with np.load(path,allow_pickle=False) as array:rgb=array['rgb']
            if rgb.shape!=(480,640,3) or rgb.dtype!=np.uint8:raise ValueError(f'INVALID_RGB:{path}')
            cv2.imwrite(str(folder/f'{stem}.jpg'),rgb)
            labels['sessions'][session][stem]=None
    output.write_text(json.dumps(labels,indent=2)+'\n')
    (images/'index.html').write_text(ANNOTATOR_HTML.replace('__LABELS__',json.dumps(labels).replace('</','<\\/')))
    return len(list(images.glob('*/*.jpg')))


def load_annotations(path,sessions):
    doc=json.loads(Path(path).read_text())
    if doc.get('schema_version')!=1 or not isinstance(doc.get('sessions'),dict):
        raise ValueError('INVALID_ANNOTATION_SCHEMA')
    labeled={}
    for session,files in sessions.items():
        entries=doc['sessions'].get(session,{})
        for stem,recording in files.items():
            items=entries.get(stem)
            if items is None:continue
            if not isinstance(items,list):raise ValueError(f'INVALID_ANNOTATIONS:{session}/{stem}')
            normalized=[]
            for item in items:
                if not isinstance(item,dict) or item.get('label','orange') not in ('blue','yellow','orange','large_orange','unknown'):
                    raise ValueError(f'UNSUPPORTED_ANNOTATION_CLASS:{session}/{stem}')
                box=item.get('box')
                if not isinstance(box,list) or len(box)!=4 or not all(type(v) in (int,float) for v in box):
                    raise ValueError(f'INVALID_BOX:{session}/{stem}')
                x1,y1,x2,y2=box
                if not all(np.isfinite(box)) or not (0<=x1<x2<=640 and 0<=y1<y2<=480):
                    raise ValueError(f'INVALID_BOX:{session}/{stem}')
                distance=item.get('distance_m')
                if distance is not None and (not isinstance(distance,(int,float)) or not np.isfinite(distance) or distance<=0):
                    raise ValueError(f'INVALID_DISTANCE:{session}/{stem}')
                xy=item.get('xy_m')
                if xy is not None and (not isinstance(xy,list) or len(xy)!=2 or
                    not all(isinstance(v,(int,float)) and np.isfinite(v) for v in xy)):
                    raise ValueError(f'INVALID_SURVEY_XY:{session}/{stem}')
                difficulty=item.get('difficulty',[])
                if not isinstance(difficulty,list) or len(set(difficulty))!=len(difficulty) or any(
                    tag not in ('small','distant','occluded','low_light') for tag in difficulty):
                    raise ValueError(f'INVALID_DIFFICULTY:{session}/{stem}')
                normalized.append({'box':tuple(box),'label':item.get('label','orange'),
                                   'distance_m':distance,'xy_m':tuple(xy) if xy is not None else None,
                                   'difficulty':difficulty})
            labeled[(session,stem)]=(recording,normalized)
    if not labeled:raise ValueError('NO_ANNOTATED_FRAMES')
    return labeled


def box_iou(a,b):
    x1=max(a[0],b[0]);y1=max(a[1],b[1]);x2=min(a[2],b[2]);y2=min(a[3],b[3])
    overlap=max(0,x2-x1)*max(0,y2-y1)
    aa=(a[2]-a[0])*(a[3]-a[1]);bb=(b[2]-b[0])*(b[3]-b[1])
    return overlap/(aa+bb-overlap) if aa+bb>overlap else 0.0


def match(predictions, truths, iou_threshold):
    """Return ranked (score, TP) and matched truth IDs at one IoU threshold."""
    ranked=[];matched=set()
    for key,preds in predictions.items():
        gt=truths[key]
        used=set()
        for detection in sorted(preds,key=lambda d:d.confidence,reverse=True):
            best=-1;best_iou=iou_threshold
            for j,item in enumerate(gt):
                if j in used:continue
                score=box_iou(detection.box,item['box']) if detection.label==item.get('label','orange') else 0
                if score>=best_iou:best=j;best_iou=score
            if best>=0:used.add(best);matched.add((key,best))
            ranked.append((detection.confidence,best>=0))
    ranked.sort(key=lambda item:item[0],reverse=True)
    return ranked,matched


def average_precision(ranked, ground_truth_count):
    if ground_truth_count==0:return None
    if not ranked:return 0.0
    tp=np.cumsum([hit for _,hit in ranked]);fp=np.arange(1,len(ranked)+1)-tp
    recall=tp/ground_truth_count;precision=tp/(tp+fp)
    return float(np.mean([max(precision[recall>=r],default=0.0) for r in np.linspace(0,1,101)]))


def distance_recall(truths,matched):
    edges=[(0,.6,'<0.6m'),(.6,1,'0.6-1m'),(1,2,'1-2m'),(2,3,'2-3m'),(3,float('inf'),'>3m')]
    counts={name:[0,0] for _,_,name in edges}
    unknown=0
    for key,items in truths.items():
        for j,item in enumerate(items):
            distance=item['distance_m']
            if distance is None:unknown+=1;continue
            name=next(name for low,high,name in edges if low<=distance<high)
            counts[name][1]+=1
            counts[name][0]+=int((key,j) in matched)
    return {name:{'matched':a,'total':b,'recall':a/b if b else None} for name,(a,b) in counts.items()},unknown


def evaluate_predictions(predictions, truths):
    total=sum(map(len,truths.values()))
    ranked,matched=match(predictions,truths,.5)
    tp=len(matched);fp=len(ranked)-tp;fn=total-tp
    classes=sorted({item.get('label','orange') for items in truths.values() for item in items})
    per_class={}
    for klass in classes:
        class_truth={key:[item for item in items if item.get('label','orange')==klass] for key,items in truths.items()}
        class_pred={key:[d for d in predictions[key] if d.label==klass] for key in truths}
        count=sum(map(len,class_truth.values()))
        ap=[average_precision(match(class_pred,class_truth,round(float(t),2))[0],count)
            for t in np.arange(.5,1,.05)]
        per_class[klass]={'ground_truth_cones':count,'ap50':ap[0],
                          'map50_95':float(np.mean(ap))}
    bins,unknown=distance_recall(truths,matched)
    coverage={'negative_frames':sum(not items for items in truths.values()),
              'positive_frames':sum(bool(items) for items in truths.values()),
              'difficulty_cones':{tag:sum(tag in item.get('difficulty',[]) for items in truths.values()
                                          for item in items) for tag in ('small','distant','occluded','low_light')}}
    return {'annotated_frames':len(truths),'ground_truth_cones':total,'coverage':coverage,'true_positives':tp,
            'false_positives':fp,'false_negatives':fn,
            'precision':tp/(tp+fp) if tp+fp else None,
            'recall':tp/total if total else None,
            'map50':float(np.mean([v['ap50'] for v in per_class.values()])) if total else None,
            'map50_95':float(np.mean([v['map50_95'] for v in per_class.values()])) if total else None,
            'per_class':per_class,
            'per_distance_recall':bins,'distance_unknown_cones':unknown}


def evaluate_recordings(detector,labeled,confidences=(.54,),nms_ious=(.5,)):
    raw={};truths={};latencies=[]
    for key,(path,items) in labeled.items():
        with np.load(path,allow_pickle=False) as array:image=array['rgb']
        if image.shape!=(480,640,3) or image.dtype!=np.uint8:raise ValueError(f'INVALID_RGB:{path}')
        raw[key]=detector.infer(image)
        truths[key]=items;latencies.append(detector.latency_ms)
    runs=[]
    for confidence in confidences:
        for nms_iou in nms_ious:
            predictions={key:detector.postprocess(rows,geometry,confidence,nms_iou)
                         for key,(rows,geometry) in raw.items()}
            runs.append({'confidence':confidence,'nms_iou':nms_iou,**evaluate_predictions(predictions,truths)})
    return {'model':detector.version,'backend':detector.backend,'input_size':detector.input_size,
            'sessions':sorted({key[0] for key in labeled}),
            'inference_latency_ms':{'median':statistics.median(latencies),
                'p95':float(np.percentile(latencies,95))},'runs':runs}


def main(argv=None):
    parser=argparse.ArgumentParser(description='Evaluate cone detection on manually labeled HP60C recordings')
    parser.add_argument('--recording',action='append',required=True,help='Repeat for distinct recording sessions')
    mode=parser.add_mutually_exclusive_group(required=True)
    mode.add_argument('--make-template',help='New annotation JSON and adjacent JPEG previews')
    mode.add_argument('--annotations',help='Completed manual annotation JSON')
    parser.add_argument('--model',default=str(Path(__file__).resolve().parents[2]/'models/coneslayer-simplified.onnx'))
    parser.add_argument('--compare-model',help='Evaluate a second manifest-backed ONNX on the same held-out frames and thresholds')
    parser.add_argument('--training-audit',help='Audit JSON whose train/val sessions must not overlap evaluation')
    parser.add_argument('--custom-one-class-label',choices=['orange'],
                        help='Explicitly declare orange semantics for a compatible custom ONNX model')
    parser.add_argument('--confidence',type=float,nargs='+',default=[.25,.4,.54,.7])
    parser.add_argument('--nms-iou',type=float,nargs='+',default=[.45,.5,.6])
    parser.add_argument('--output',help='New JSON evaluation report; existing file refused')
    args=parser.parse_args(argv)
    if args.compare_model and not args.training_audit:
        parser.error('--training-audit is required for A/B comparison')
    sessions=recording_files(args.recording)
    if args.make_template:
        print(json.dumps({'annotation_template':args.make_template,
                          'exported_frames':make_template(sessions,args.make_template)}))
        return 0
    labeled=load_annotations(args.annotations,sessions)
    if args.training_audit:
        audit=json.loads(Path(args.training_audit).read_text())
        verify_heldout_sessions(audit,sessions)
    built_in=Path(args.model).resolve()==(Path(__file__).resolve().parents[2]/'models/coneslayer-simplified.onnx').resolve()
    trained_manifest=Path(args.model).with_suffix('.json').is_file() and not built_in
    if not built_in and not trained_manifest and args.custom_one_class_label!='orange':
        raise ValueError('CUSTOM_MODEL_CLASS_MAPPING_UNVERIFIED')
    detector=(Yolo11OnnxDetector(args.model) if trained_manifest else
              ConeDetector(args.model,expected_sha256=MODEL_SHA256 if built_in else None,
                           label='orange'))
    report=evaluate_recordings(detector,labeled,args.confidence,args.nms_iou)
    report['model_verification']=('PINNED_CHECKPOINT' if built_in else
        'TRAINED_MANIFEST_HP60C_UNVERIFIED' if trained_manifest else 'USER_DECLARED_CLASS_UNVERIFIED')
    if args.compare_model:
        if not Path(args.compare_model).with_suffix('.json').is_file():
            raise ValueError('COMPARE_MODEL_MANIFEST_REQUIRED')
        candidate=Yolo11OnnxDetector(args.compare_model)
        compared=evaluate_recordings(candidate,labeled,args.confidence,args.nms_iou)
        report={'evaluation_set':'SAME_RECORDED_FRAMES_AND_THRESHOLDS',
                'baseline':report,'candidate':compared,
                'comparison_status':'NO_GROUND_TRUTH_CONES' if compared['runs'][0]['ground_truth_cones']==0 else 'MEASURED'}
    text=json.dumps(report,indent=2)
    if args.output:
        output=Path(args.output)
        if output.exists():raise FileExistsError(output)
        output.write_text(text+'\n')
        primary=report['baseline'] if args.compare_model else report
        print(json.dumps({'report':str(output),'annotated_frames':primary['runs'][0]['annotated_frames'],
                          'ground_truth_cones':primary['runs'][0]['ground_truth_cones'],
                          'threshold_settings':len(primary['runs'])}))
    else:print(text)
    return 0


if __name__=='__main__':raise SystemExit(main())
