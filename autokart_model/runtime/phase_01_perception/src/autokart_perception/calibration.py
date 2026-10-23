"""Validate measured camera mounting against independently surveyed cones."""
import argparse
import json
from pathlib import Path

import numpy as np
import yaml

from .depth import DepthError, validate_frame, localize_diagnostic
from .evaluation import load_annotations, recording_files
from .mount import mount_from_config
from .recording import Replay
from .types import Detection


GRID_DISTANCES_M=(.5,1.,2.,3.)

def validate_survey(recordings,annotations,config,tolerance_m,require_grid=False,grid_distances=GRID_DISTANCES_M):
    if not np.isfinite(tolerance_m) or tolerance_m<=0:raise ValueError('INVALID_TOLERANCE')
    sessions=recording_files(recordings)
    try:
        labeled=load_annotations(annotations,sessions)
    except ValueError as exc:
        if str(exc)!='NO_ANNOTATED_FRAMES':raise
        labeled={}
    mount,mount_status=mount_from_config(config)
    report={'mount_status':mount_status,'tolerance_m':tolerance_m,'surveyed_cones':0,
            'localized_cones':0,'rejections':[],'errors_m':[],'status':'NOT_EXECUTED',
            'frames':0,'synchronization_mismatches':0,'invalid_depth_pixels':0,
            'depth_pixels':0,'required_grid':require_grid,'recorded_sources':{},
            'grid_distances_m':[float(d) for d in grid_distances]}
    grid=set()
    if mount_status!='MOUNT_CONFIGURED_UNVERIFIED':return report
    for path in map(Path,recordings):
        replay=Replay(path)
        for meta_path in replay.files:
            frame=replay.read();key=(path.name,meta_path.stem)
            report['frames']+=1
            recorded_source=frame.calibration.get('recorded_source','UNKNOWN')
            report['recorded_sources'][recorded_source]=report['recorded_sources'].get(recorded_source,0)+1
            report['depth_pixels']+=int(frame.depth.size)
            report['invalid_depth_pixels']+=int(np.count_nonzero(frame.depth==0))
            if (frame.rgb_sequence!=frame.depth_sequence or
                    (frame.rgb_timestamp and frame.depth_timestamp and
                     abs(frame.rgb_timestamp-frame.depth_timestamp)>50)):
                report['synchronization_mismatches']+=1
            if key not in labeled:continue
            refs=labeled[key][1]
            for index,item in enumerate(refs):
                if item['xy_m'] is None:continue
                report['surveyed_cones']+=1
                distance=float(np.linalg.norm(item['xy_m']))
                nearest=min(grid_distances,key=lambda target:abs(target-distance))
                if abs(nearest-distance)<=.25:
                    middle=(item['box'][0]+item['box'][2])/2
                    region='left' if middle<640/3 else 'right' if middle>=1280/3 else 'center'
                    grid.add((nearest,region))
                try:
                    validate_frame(frame,config['max_frame_age_ms'])
                    cone,detail=localize_diagnostic(frame,Detection(item['box'],item['label'],1.0,index),mount,config)
                    if cone is None:raise DepthError(detail['localization_status'])
                except DepthError as exc:
                    report['rejections'].append({'session':path.name,'frame':meta_path.stem,
                                                 'cone_index':index,'reason':str(exc)})
                    continue
                report['localized_cones']+=1
                report['errors_m'].append(float(np.linalg.norm(np.asarray(cone.xy)-item['xy_m'])))
    if report['surveyed_cones']:
        errors=report.pop('errors_m')
        report['error_m']={'median':float(np.median(errors)) if errors else None,
                           'p95':float(np.percentile(errors,95)) if errors else None,
                           'max':max(errors) if errors else None}
        calculation_passed=(len(errors)==report['surveyed_cones'] and
            max(errors)<=tolerance_m and report['synchronization_mismatches']==0 and
            (not require_grid or len(grid)==3*len(grid_distances)))
        report['status']='PASS' if calculation_passed else 'FAIL'
        if calculation_passed and set(report['recorded_sources'])!={'LIVE'}:
            report['status']='NON_LIVE_PASS'
            report['blocker']='SURVEY_REQUIRES_LIVE_HP60C_RECORDINGS'
    else:
        report.pop('errors_m')
        report['blocker']='NO_INDEPENDENTLY_SURVEYED_CONES'
    report['invalid_depth_rate']=report['invalid_depth_pixels']/report['depth_pixels'] if report['depth_pixels'] else None
    report['grid_coverage']=[{'distance_m':distance,'region':region}
                             for distance,region in sorted(grid)]
    return report


def main(argv=None):
    parser=argparse.ArgumentParser(description='Compare HP60C ground-contact localization with surveyed cone positions')
    parser.add_argument('--recording',action='append',required=True)
    parser.add_argument('--annotations',required=True)
    parser.add_argument('--config',default=str(Path(__file__).resolve().parents[2]/'config/perception.yaml'))
    parser.add_argument('--tolerance-m',type=float,required=True,help='Maximum allowed position error, chosen before testing')
    parser.add_argument('--require-grid',action='store_true',help='Require 0.5, 1, 2, 3 m in each image third')
    parser.add_argument('--output',help='New JSON report; existing file refused')
    args=parser.parse_args(argv)
    report=validate_survey(args.recording,args.annotations,yaml.safe_load(Path(args.config).read_text()),
                           args.tolerance_m,args.require_grid)
    body=json.dumps(report,indent=2)+'\n'
    if args.output:
        path=Path(args.output)
        if path.exists():raise FileExistsError(path)
        path.write_text(body)
    print(body,end='')
    return 0 if report['status']=='PASS' else 2


if __name__=='__main__':raise SystemExit(main())
