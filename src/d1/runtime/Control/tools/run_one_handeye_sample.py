#!/usr/bin/env python3
"""Run exactly one explicitly reviewed bounded step, then stationary sampling.

Never selects or executes another movement, enables, returns, or changes limits.
Each invocation needs reviewed seven-joint values and explicit --execute.
"""
import argparse
import json
from pathlib import Path
import subprocess
import sys

from handeye_joint_step import plan


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--joint',type=int,required=True)
    p.add_argument('--delta',type=float,required=True)
    p.add_argument('--expected-angles',type=float,nargs=7,required=True)
    p.add_argument('--motion-output',type=Path,required=True)
    p.add_argument('--capture-output',type=Path,required=True)
    p.add_argument('--execute',action='store_true')
    p.add_argument('--sampling-mode',choices=['strict','quantized'],default='strict')
    args=p.parse_args()
    proposed=plan(args.joint,args.delta,args.expected_angles)
    if not args.execute:
        print(json.dumps(proposed,indent=2));return 0
    if args.motion_output.exists() or any(Path(str(args.capture_output)+suffix).exists()
        for suffix in ('','_retry1','_retry2')):
        raise ValueError('Fresh output paths required')
    tools=Path(__file__).resolve().parent
    command=[sys.executable,str(tools/'handeye_joint_step.py'),'--joint',str(args.joint),
             '--delta',str(args.delta),'--expected-angles',*map(str,args.expected_angles),
             '--output',str(args.motion_output),'--execute']
    # The underlying controller enforces fresh feedback, all start limits,
    # camera health and non-target drift. Do not bypass or retry its rejection.
    completed=subprocess.run(command,capture_output=True,text=True)
    result_path=args.motion_output/'summary.json'
    if not result_path.exists():
        print(completed.stdout+completed.stderr);return 1
    motion=json.loads(result_path.read_text())
    print(json.dumps(dict(motion=str(args.motion_output),completed=motion['completed'],
        error=motion.get('error'),commands=motion.get('commands'),receipts=motion.get('receipts'),
        final_angles=motion.get('final_angles'),non_target_drift=motion.get('non_target_max_drift'),
        endpoints=motion.get('endpoints')),ensure_ascii=False),flush=True)
    if completed.returncode or not motion['completed']:
        return 1
    for suffix in ('','_retry1','_retry2'):
        folder=Path(str(args.capture_output)+suffix)
        subprocess.run([sys.executable,str(tools/'kr260_handeye_live.py'),'capture',
                        '--sampling-mode',args.sampling_mode,'--output',str(folder)],capture_output=True,text=True)
        capture=json.loads((folder/'capture.json').read_text())
        print(json.dumps(dict(capture=str(folder),complete=capture['complete'],
            error=capture.get('error'),q=capture.get('observation',{}).get('joint_median'),
            max_corner_shift=capture.get('visual_max_corner_shift_px')),ensure_ascii=False),flush=True)
        if capture['complete']:
            return 0
        # Only retry read-only sampling after a stationarity rejection.
        # The mechanical step is never retried.
        if not capture.get('error','').startswith('Robot not stationary:'):
            return 1
    return 1


if __name__=='__main__':
    raise SystemExit(main())
