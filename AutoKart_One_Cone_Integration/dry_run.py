#!/usr/bin/env python3
"""Analyze JSON-lines from existing camera+encoder owner. No hardware output.

Example input per line:
{"cone_gap_m": 0.6, "wheel_speed_mps": 0.0, "observation_age_s": 0.01,
 "source":"LIVE", "calibration_status":"MOUNT_VERIFIED", "depth_valid":true,
 "wheel_speed_valid":true, "emergency_stop_clear":true}

These fields are a proposed INTEGRATION CONTRACT, not an assertion that the
existing HP60C application already publishes them.
"""
import argparse
import dataclasses
import json
import sys
from one_cone_logic import OneConePlanner, Sample

def main():
    p=argparse.ArgumentParser()
    p.add_argument('--jsonl', help='Path to source data; stdin if omitted')
    args=p.parse_args()
    planner=OneConePlanner()
    f=open(args.jsonl,encoding='utf8') if args.jsonl else sys.stdin
    try:
        for n,line in enumerate(f,1):
            if not line.strip():continue
            try:
                item=json.loads(line)
                sample=Sample(**item)
                result=planner.update(sample)
                print(json.dumps({'line':n,'decision':result.decision.value,
                    'reason':result.reason,'trigger_gap_m':result.trigger_gap_m}))
            except (ValueError,TypeError,KeyError) as exc:
                planner.latched_stop=True
                print(json.dumps({'line':n,'decision':'STOP_REQUEST',
                    'reason':'INVALID_INPUT','detail':str(exc)}))
    finally:
        if args.jsonl:f.close()

if __name__=='__main__':
    main()
