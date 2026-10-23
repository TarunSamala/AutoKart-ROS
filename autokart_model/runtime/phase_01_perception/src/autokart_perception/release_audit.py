"""Evidence inventory for Phase 1. Never grants vehicle operation approval."""
import argparse
import hashlib
import json
import os
import re
import subprocess
import sys
import time
from pathlib import Path

import numpy as np
import yaml

from .mount import mount_from_config

ROOT=Path(__file__).resolve().parents[2]


def _optional_json(path):
    return json.loads(Path(path).read_text()) if path else None


def build_report(config, model_path, tests=None, evaluation=None, survey=None):
    matrix,mount_status=mount_from_config(config)
    calibration_hash=hashlib.sha256(np.asarray(matrix,dtype='<f8').tobytes()).hexdigest()
    model=Path(model_path)
    model_hash=hashlib.sha256(model.read_bytes()).hexdigest() if model.is_file() else None
    run=(evaluation or {}).get('runs',[{}])[0]
    positive_count=run.get('ground_truth_cones',0)
    gates={
        'automated_tests':{'status':'PASS' if tests and tests['exit_code']==0 else 'NOT_EXECUTED' if tests is None else 'FAIL',
                           'evidence':tests},
        'positive_cone_detection':{'status':'NOT_EXECUTED' if not positive_count else 'REVIEW_REQUIRED',
            'precision':run.get('precision'),'recall':run.get('recall'),
            'map50':run.get('map50'),'map50_95':run.get('map50_95'),
            'ground_truth_cones':positive_count},
        'surveyed_localization':{'status':'NOT_EXECUTED' if survey is None or survey.get('status')=='NOT_EXECUTED'
            else 'REVIEW_REQUIRED' if survey.get('status')=='PASS' else 'FAIL',
            'surveyed_cones':survey.get('surveyed_cones',0) if survey else 0,
            'p95_error_m':(survey.get('error_m') or {}).get('p95') if survey else None},
        'real_gate_topology':{'status':'NOT_EXECUTED'},
        'target_pi_latency_p50_p95_p99':{'status':'NOT_EXECUTED'},
        'braking_and_emergency_stop':{'status':'NOT_EXECUTED'},
        'independent_vehicle_safety':{'status':'NOT_EXECUTED'},
    }
    return {'schema_version':1,'generated_monotonic_ns':time.monotonic_ns(),
        'phase':'01','phase_status':'TESTING','vehicle_operation_approval':'BLOCKED',
        'operating_envelope':'STATIONARY_BENCH_ONLY','actuator_interface_present':False,
        'speed_limit_advisory_mps':0.0,'calibration_status':mount_status,
        'calibration_sha256':calibration_hash,'default_model_sha256':model_hash,
        'proposed_acceptance_targets_not_verified':{
            'cone_recall_min':.98,'cone_precision_min':.99,
            'localization_p95_error_m_max':.30,'valid_output_age_ms_max':150},
        'sensor_specification':{'hp60c_depth_range_m':[.2,4.],
            'horizontal_fov_deg':73.8,'vertical_fov_deg':58.8,
            'source':'https://www.ydlidar.com/products/view/29.html'},
        'guidance':'ISO 21448:2022 intended-functionality risk and verification principles; no compliance claim',
        'gates':gates}


def run_tests():
    env=os.environ.copy();env['PYTEST_DISABLE_PLUGIN_AUTOLOAD']='1'
    env['PYTHONPATH']=str(ROOT/'src')
    cmd=[sys.executable,'-m','pytest','-q',str(ROOT/'tests')]
    completed=subprocess.run(cmd,cwd=ROOT,env=env,capture_output=True,text=True,check=False)
    passed=re.search(r'(\d+) passed',completed.stdout)
    failed=re.search(r'(\d+) failed',completed.stdout)
    elapsed=re.search(r'in ([\d.]+)s',completed.stdout)
    return {'exit_code':completed.returncode,'passed':int(passed.group(1)) if passed else None,
            'failed':int(failed.group(1)) if failed else 0,
            'elapsed_s':float(elapsed.group(1)) if elapsed else None,
            'command':' '.join(cmd),'summary':completed.stdout[-500:] if completed.returncode else None,
            'stderr_tail':completed.stderr[-500:] if completed.returncode else None}


def main(argv=None):
    parser=argparse.ArgumentParser(description='Generate a blocked-by-default Phase 1 release evidence report')
    parser.add_argument('--output',required=True,help='New JSON path; existing file refused')
    parser.add_argument('--run-tests',action='store_true')
    parser.add_argument('--evaluation-report',help='Independent HP60C detector evaluation JSON')
    parser.add_argument('--survey-report',help='Surveyed-cone localization JSON')
    parser.add_argument('--config',default=str(ROOT/'config/perception.yaml'))
    parser.add_argument('--model',default=str(ROOT/'models/coneslayer-simplified.onnx'))
    args=parser.parse_args(argv)
    output=Path(args.output)
    if output.exists():raise FileExistsError(output)
    report=build_report(yaml.safe_load(Path(args.config).read_text()),args.model,
        run_tests() if args.run_tests else None,_optional_json(args.evaluation_report),
        _optional_json(args.survey_report))
    with output.open('x') as stream:json.dump(report,stream,indent=2);stream.write('\n')
    print(json.dumps({'report':str(output),'tests':report['gates']['automated_tests']['status'],
                      'vehicle_operation_approval':'BLOCKED'}))
    return 0 if report['gates']['automated_tests']['status']=='PASS' else 2


if __name__=='__main__':raise SystemExit(main())
