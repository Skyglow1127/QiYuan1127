"""Explicit entry for the buffered board-clock READY path; importing is inert."""
from __future__ import annotations
import argparse
import json
from pathlib import Path
from .backend import Backend, DEFAULTS
from .execution_scope import FIXED_PREGRASP
from .reporting import load, write_json
from .closed_loop_runner import _cancellation, check_live_interface
POLICY='timed_group_v1'


def prepare(*, mode='live'):
    if mode=='live':check_live_interface()
    backend=Backend(mode=mode)
    settings=dict(DEFAULTS)
    review=backend.prepare(settings,fixed_pregrasp=True,scheduling_policy=POLICY,
        fixed_motion=dict(speed_deg_s=5.,acceleration_deg_s2=10.,rate_hz=10.,
                          gripper_speed_sdk_s=DEFAULTS['gripper_speed_sdk_s']))
    if review.get('scheduling_policy')!=POLICY or not review.get('ready'):
        raise RuntimeError('Timed trajectory preparation did not produce an executable review')
    return dict(review,review_path=str(Path(review['run_dir'])/'review.json'))


def execute(path,approval_text,*,mode='live'):
    if not isinstance(approval_text,str) or not approval_text.strip():
        raise ValueError('Actual approval of the saved path is required')
    backend=Backend(mode=mode);review=backend.resume_review(path)
    if review.get('scheduling_policy')!=POLICY or review.get('operation')!=FIXED_PREGRASP:
        raise ValueError('This entry only executes an exact saved timed READY review')
    if mode=='live':check_live_interface()
    claim=Path(str(path)+'.timed_consumed')
    with claim.open('x') as stream:
        json.dump(dict(approval_text=approval_text,manifest_sha256=review['approval_sha256'],
            scheduling_policy=POLICY),stream,ensure_ascii=False);stream.write('\n')
    with _cancellation(backend) as check_cancel:
        check_cancel();cameras=backend.start_cameras();check_cancel()
        if cameras.get('completed') is not True:
            raise RuntimeError('Approved cameras did not become ready')
        startup=backend.control_device('power_on');check_cancel()
        if startup.get('completed') is not True:
            raise RuntimeError('Approved startup did not complete')
        review=backend.resume_review(path);check_cancel()
        result=backend.execute(review,approval_text,execution_scope=FIXED_PREGRASP)
    write_json(Path(review['run_dir'])/'timed_result.json',result)
    return result


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('operation',choices=['prepare','execute'])
    parser.add_argument('--mode',choices=['live','demo'],default='live')
    parser.add_argument('--review',type=Path)
    parser.add_argument('--approval-text')
    args=parser.parse_args()
    if args.operation=='prepare':result=prepare(mode=args.mode)
    else:
        if args.review is None or not args.approval_text:parser.error('execute needs --review and --approval-text')
        result=execute(args.review,args.approval_text,mode=args.mode)
    print(json.dumps(result,ensure_ascii=False,indent=2))
if __name__=='__main__':main()
