#!/usr/bin/env python3
"""J6 range and smoothness test. Raw SDK units; no jaw-width/force calibration.
Validate staged targets up to the selected upper target, then upper→0→upper.
Default upper target 50 reflects this arm's observed saturation near 49.8.
Only J6 is commanded; all J0..J5 are guarded. No enable/zero/retry/release.
"""
import argparse
import json
import math
import time
from pathlib import Path
from j4_smooth_test import Session

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--nic',default='enp99s0')
    parser.add_argument('--rate-hz',type=int,choices=(5,10),default=10)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--upper-limit',type=float,default=50,help='Selected SDK upper target, default observed 50, <=65; not physical jaw width')
    parser.add_argument('--expected-start',type=float,help='Previously measured J6 start, checked within 1 SDK unit')
    parser.add_argument('--execute',action='store_true')
    args=parser.parse_args()
    if not math.isfinite(args.upper_limit) or not 0<args.upper_limit<=65:
        parser.error('--upper-limit must be within (0,65]')
    if args.expected_start is not None and (not math.isfinite(args.expected_start) or not 0<=args.expected_start<=65):
        parser.error('--expected-start must be within [0,65]')
    if not args.execute:
        print(f'Plan only: J6 staged approach to {args.upper_limit:g}; {args.upper_limit:g}→0→{args.upper_limit:g}, 60 s per leg, {args.rate_hz} Hz; SDK units, no compensation.')
        return
    args.output.mkdir(parents=True,exist_ok=False)
    session=Session(Path(__file__).resolve().parents[1]/'build/arm_control',args.nic,args.output,args.rate_hz,joint=6)
    summary=dict(completed=False,joint=6,rate_hz=args.rate_hz,requested_range_sdk_units=[0,args.upper_limit],
        leg_duration_s=60,reference_peak_speed_sdk_units_s=args.upper_limit*1.875/60,
        expected_start=args.expected_start,
        compensation=False,physical_opening_validated=False,force_validated=False,
        note='Raw SDK position, host receipt timestamps; command range is not a measured jaw opening.')
    try:
        session.pump(3)
        session.monitor.check(time.monotonic())
        start=session.monitor.angles[6]
        if not 0<=start<=65:
            raise RuntimeError('J6 initial feedback outside [0,65]')
        if args.expected_start is None and not 0<=start<=10:
            raise RuntimeError('Expected J6 start in [0,10]; no motion sent')
        if args.expected_start is not None and abs(start-args.expected_start)>1:
            raise RuntimeError('J6 start differs from expected by >1 SDK unit')
        if start>args.upper_limit:
            raise RuntimeError('Start above selected upper limit; no motion sent')
        session.monitor.baseline=list(session.monitor.angles)
        session.monitor.target=start
        session.active=True
        previous=start
        targets=[t for t in (10,25,40,55) if start<t<args.upper_limit]+[args.upper_limit]
        for target in targets:
            session.segment(f'range_{target}',previous,target,max(8,math.ceil((target-previous)/1.25)))
            previous=target
        session.segment('close_full',args.upper_limit,0,60)
        session.segment('open_full',0,args.upper_limit,60)
        summary['completed']=True
    except (Exception,KeyboardInterrupt) as exc:
        summary['error']=str(exc) or type(exc).__name__
        print('STOP:',summary['error'],flush=True)
    finally:
        summary.update(final_angles_sdk_units=session.monitor.angles,
            commands_sent=session.commands,cli_confirmations=session.acks,
            non_target_max_drift_deg=session.monitor.max_drift,
            completed_segments=session.completed_segments,final_phase=session.phase,
            last_commanded_target_sdk_units=session.monitor.target)
        session.close()
        (args.output/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    if not summary['completed']:raise SystemExit(1)

if __name__=='__main__':main()
