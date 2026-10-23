import argparse
import hashlib
import json
import sys
import time
from pathlib import Path
import yaml
from .camera import HP60C,CameraError
from .detector import ConeDetector,Yolo11OnnxDetector,MODEL_SHA256
from .pipeline import Pipeline
from .recording import Recorder,Replay
from .synthetic import Synthetic
from .visualization import Viewer

ROOT=Path(__file__).resolve().parents[2]

def main(argv=None):
    parser=argparse.ArgumentParser(description='Stationary HP60C cone perception (no actuator interface)')
    parser.add_argument('--source',choices=('hp60c','replay','synthetic'),required=True)
    parser.add_argument('--path',help='Replay directory')
    parser.add_argument('--annotations',help='Manual ground truth JSON for replay recall diagnostics')
    parser.add_argument('--record',help='New recording directory; existing directories are refused')
    parser.add_argument('--model',default=str(ROOT/'models/coneslayer-simplified.onnx'))
    parser.add_argument('--headless',action='store_true')
    parser.add_argument('--web',action='store_true',help='Serve a browser dashboard until Ctrl+C')
    parser.add_argument('--host',default='127.0.0.1',help='Dashboard bind address; use 0.0.0.0 for LAN access')
    parser.add_argument('--port',type=int,default=8765,help='Dashboard HTTP port')
    parser.add_argument('--frames',type=int,default=0,help='Stop after N frames; 0 runs until quit')
    parser.add_argument('--connect-timeout',type=float,default=10.0,
                        help='Seconds to wait for an HP60C after startup or disconnect')
    parser.add_argument('--config',default=str(ROOT/'config/perception.yaml'))
    args=parser.parse_args(argv)
    config=yaml.safe_load(Path(args.config).read_text())
    detector=None
    if args.source=='hp60c':
        default_model=ROOT/'models/coneslayer-simplified.onnx'
        builtin=Path(args.model).resolve()==default_model.resolve()
        try:
            if not builtin and Path(args.model).with_suffix('.json').is_file():
                detector=Yolo11OnnxDetector(args.model,config['confidence'],config['nms_iou'])
            else:
                detector=ConeDetector(args.model,config['confidence'],config['nms_iou'],
                                      label='orange' if builtin else 'unknown',
                                      expected_sha256=MODEL_SHA256 if builtin else None)
        except Exception as e:print(f'Model status: {e}',file=sys.stderr)
    try:
        if args.source=='hp60c':source=HP60C(max_age_ms=config['max_frame_age_ms'])
        elif args.source=='replay':source=Replay(args.path,args.annotations) if args.path else parser.error('--path is required for replay')
        else:source=Synthetic()
        model_hash=hashlib.sha256(Path(args.model).read_bytes()).hexdigest() if detector else None
        recorder=Recorder(args.record,detector.version if detector else 'none',model_hash) if args.record else None
    except (CameraError,FileNotFoundError,FileExistsError) as e:
        print(f'Startup status: {e}',file=sys.stderr)
        return 2
    pipeline=Pipeline(detector,config,synthetic=args.source=='synthetic')
    if args.web:
        from .webui import serve_dashboard
        return serve_dashboard(source,pipeline,recorder,args.host,args.port,args.frames)
    viewer=None if args.headless else Viewer()
    count=0; disconnected_since=None; last_error=None
    try:
        while args.frames==0 or count<args.frames:
            try:frame=source.read()
            except CameraError as e:
                pipeline.note_sensor_fault(str(e))
                if recorder:recorder.record_fault(str(e))
                if disconnected_since is None:disconnected_since=time.monotonic()
                if time.monotonic()-disconnected_since>=args.connect_timeout:
                    print(f'Camera status: {e}; connect timeout',file=sys.stderr)
                    return 2
                if str(e)!=last_error:
                    print(f'Camera status: {e}',file=sys.stderr)
                    last_error=str(e)
                time.sleep(0.5);continue
            except (ValueError,KeyError,OSError) as e:
                if recorder:recorder.record_fault(f'REPLAY_READ_FAILURE:{e}',args.source.upper())
                print(f'Replay status: REPLAY_READ_FAILURE:{e}',file=sys.stderr)
                return 2
            if frame is None:break
            disconnected_since=None;last_error=None;result=pipeline.process(frame);count+=1
            if recorder:recorder.write(result)
            if viewer:
                if not viewer.show(result):break
            else:print(json.dumps({'frame':count,'source':frame.source,'status':result.status,
                'safety_state':result.safety.state,'safety_reasons':result.safety.reasons,
                'vehicle_authorized':result.safety.vehicle_authorized,
                'speed_limit_advisory_mps':result.safety.speed_limit_advisory_mps,
                'cones':len(result.cones),'gates':len(result.gates),'centerline':len(result.centerline),
                'processing_ms':round(result.diagnostics['processing_ms'],2),
                'inference_ms':round(result.diagnostics.get('inference_ms',0),2),
                'rgb_fps':round(result.diagnostics['rgb_fps'],2),
                'output_fps':round(result.diagnostics['end_to_end_fps'],2),
                'frame_age_ms':round(result.diagnostics['frame_age_ms'],2),
                'drops':frame.drops}))
    except KeyboardInterrupt:pass
    finally:
        if isinstance(source,HP60C):source.close()
        if viewer:viewer.close()
    return 0

if __name__=='__main__':raise SystemExit(main())
