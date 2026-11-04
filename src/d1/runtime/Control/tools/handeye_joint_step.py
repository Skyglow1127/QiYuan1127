#!/usr/bin/env python3
"""One bounded single-joint step, followed by stop/review; no automatic return.

Dual-camera freshness is a communications guard, NOT collision/person detection.
An operator must inspect the live scene and ensure emergency-stop readiness
before --execute. Never enables, homes, unloads or retries the arm.
"""
import argparse
import json
import math
from pathlib import Path
import threading
import time
import urllib.request

from j4_smooth_test import JOINT_LIMITS, Session
from joint_mapping_small_test import endpoints

ROOT = Path(__file__).resolve().parents[1]


def plan(joint, delta, expected):
    if type(joint) is not int or joint not in range(6):
        raise ValueError('Only arm joints 0..5; no gripper')
    if not math.isfinite(delta) or not .5 <= abs(delta) <= 2:
        raise ValueError('Single-step amplitude must be 0.5..2 degrees')
    if len(expected) != 7 or not all(math.isfinite(x) for x in expected):
        raise ValueError('Seven finite reviewed start values required')
    for q, (lo, hi) in zip(expected, JOINT_LIMITS):
        if not lo <= q <= hi:raise ValueError('Reviewed start outside existing limits')
    target=expected[joint]+delta
    lo,hi=JOINT_LIMITS[joint]
    if not lo <= target <= hi:raise ValueError('Target outside existing limits')
    return dict(joint=joint,delta_deg=delta,reviewed_start=expected,
                target_deg=target,duration_s=8,rate_hz=5,
                peak_reference_velocity_deg_s=abs(delta)*1.875/8,
                automatic_return=False,hardware_commands_sent=False)


class Cameras:
    def __init__(self, folder):
        self.folder=folder;folder.mkdir()
        self.stop=threading.Event();self.ready=False;self.error=None;self.stamp=0
        self.thread=threading.Thread(target=self.run,daemon=True)
        self.thread.start()

    def run(self):
        opener=urllib.request.build_opener(urllib.request.ProxyHandler({}))
        ids={};number=0
        try:
            with (self.folder/'frames.jsonl').open('x') as log:
                while not self.stop.is_set():
                    began=time.monotonic()
                    for role,port,serial in [('global',18761,'CC1365302K0'),('wrist',18760,'CC1365301EY')]:
                        with opener.open(f'http://127.0.0.1:{port}/status',timeout=.5) as r:s=json.load(r)
                        if s.get('error') or s.get('stopped') or s['device']['serial']!=serial:
                            raise RuntimeError('Wrong/unhealthy camera: '+role)
                        with opener.open(f'http://127.0.0.1:{port}/color.jpg',timeout=.5) as r:
                            raw=r.read();index=int(r.headers['X-Frame-Index']);age=float(r.headers['X-Board-Age-Ms'])
                        if not 0<=age<=250 or index<=ids.get(role,-1):
                            raise RuntimeError('Stale/reordered frame: '+role)
                        ids[role]=index;name=f'{role}_{number:05d}.jpg'
                        (self.folder/name).write_bytes(raw)
                        log.write(json.dumps(dict(role=role,serial=serial,file=name,index=index,age_ms=age,host_monotonic=time.monotonic()))+'\n');log.flush()
                    self.stamp=time.monotonic();self.ready=True;number+=1
                    self.stop.wait(max(0,.2-(time.monotonic()-began)))
        except Exception as e:self.error=str(e)

    def check(self):
        if self.error or not self.ready or time.monotonic()-self.stamp>.8:
            raise RuntimeError('Camera guard: '+str(self.error or 'not ready/stale'))

    def close(self):
        self.stop.set();self.thread.join(timeout=3)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--joint',type=int,required=True);p.add_argument('--delta',type=float,required=True)
    p.add_argument('--expected-angles',type=float,nargs=7,required=True)
    p.add_argument('--output',type=Path,required=True);p.add_argument('--execute',action='store_true')
    args=p.parse_args()
    try:report=plan(args.joint,args.delta,args.expected_angles)
    except ValueError as e:p.error(str(e))
    if not args.execute:print(json.dumps(report,indent=2));return 0
    args.output.mkdir(parents=True,exist_ok=False)
    report.update(completed=False,mapping_validated=False,handeye_calibrated=False)
    cameras=Cameras(args.output/'frames');session=None
    try:
        deadline=time.monotonic()+5
        while not cameras.ready:
            if cameras.error or time.monotonic()>deadline:cameras.check()
            time.sleep(.05)
        folder=args.output/'motion';folder.mkdir()
        session=Session(ROOT/'build/arm_control','enp99s0',folder,5,args.joint)
        session.pump(3);session.monitor.check(time.monotonic());cameras.check()
        measured=list(session.monitor.angles)
        if any(abs(a-b)>.3+1e-5 for a,b in zip(measured,args.expected_angles)):
            raise RuntimeError('Any joint differs from reviewed start by >0.3; no motion')
        for q,(lo,hi) in zip(measured,JOINT_LIMITS):
            if not lo<=q<=hi:raise RuntimeError('Measured joint outside existing limits')
        original=session.monitor.check
        def check(now):original(now);cameras.check()
        session.monitor.check=check
        session.monitor.baseline=measured;session.monitor.target=measured[args.joint];session.active=True
        report['measured_start']=measured
        # Endpoint is tied to the reviewed pose, never accumulated from drift.
        session.segment('step',measured[args.joint],report['target_deg'],8)
        endpoint=endpoints(folder,args.joint)[-1]
        if not endpoint['usable'] or endpoint['feedback_span_deg']>.3+1e-5:
            raise RuntimeError('Endpoint unstable; stop before any further motion')
        session.pump(1);cameras.check();report['completed']=True
    except (Exception,KeyboardInterrupt) as e:
        report['error']=str(e) or type(e).__name__
    finally:
        if session:
            report.update(final_angles=session.monitor.angles,final_status=session.monitor.status,
                          commands=session.commands,receipts=session.acks,
                          hardware_commands_sent=session.commands>0,
                          non_target_max_drift=session.monitor.max_drift)
            session.close();report['endpoints']=endpoints(args.output/'motion',args.joint)
        cameras.close();report['camera_error']=cameras.error
        (args.output/'summary.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2));return 0 if report['completed'] else 1


if __name__=='__main__':raise SystemExit(main())
