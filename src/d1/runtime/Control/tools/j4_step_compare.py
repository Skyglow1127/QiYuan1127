#!/usr/bin/env python3
"""Compare one-degree and 0.2-degree J4 steps, with bounded passive camera captures.

No calibration claim, gain change, SDK edit, homing, compensation, or retry.
Original shared feedback/receipt/non-target guards remain enabled.
"""
import argparse
import json
import math
from pathlib import Path
import shlex
import statistics
import subprocess
import time
import uuid

from j4_smooth_test import Session
from joint_mapping_small_test import check_home

STAGES = [('anchor', 1), ('coarse_down', 0), ('coarse_up', 1),
          ('fine_08', .8), ('fine_06', .6), ('fine_04', .4),
          ('fine_02', .2), ('fine_00', 0)]
SSH = ['ssh', '-o', 'ControlPath=/home/aup/codex/kr260-camera/ssh-camera',
       '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=5', 'ubuntu@192.168.28.2']


class ProbeSession(Session):
    def send(self, target):
        if not math.isfinite(target) or not 0 <= target <= 1:
            raise RuntimeError('J4 diagnostic targets must be in [0,1] degree')
        return super().send(target)


def samples(output, phase):
    out = []
    for raw in (output/'events.jsonl').read_text().splitlines():
        e = json.loads(raw)
        if e['phase'] != phase or not e.get('line', '').startswith('[rx] '):
            continue
        msg = json.loads(e['line'][5:])
        if msg.get('address') == 2 and msg.get('funcode') == 1:
            out.append([msg['data'][f'angle{i}'] for i in range(7)])
    return out


def endpoint(session, output, phase, target):
    tail = samples(output, phase)[-20:]
    if len(tail) < 10:
        raise RuntimeError('Insufficient endpoint feedback')
    values = [a[4] for a in tail]
    result = dict(median_deg=statistics.median(values), span_deg=max(values)-min(values),
                  error_deg=statistics.median(values)-target, frames=len(tail))
    if abs(session.monitor.angles[4]-target) > 1+1e-5 or abs(result['error_deg']) > 1+1e-5:
        raise RuntimeError('Endpoint error > 1 degree; no next command')
    if result['span_deg'] > .3+1e-5:
        raise RuntimeError('Endpoint still changing > 0.3 degree')
    return result


def capture(session, output, remote_root, name, target):
    path = remote_root+'/'+name
    inner = ('LD_LIBRARY_PATH=/home/ubuntu/kr260-camera/lib timeout 15s python3 -u '
             '/home/ubuntu/kr260-camera/capture_orbbec.py --mode both --align '
             '--seconds 2 --output '+shlex.quote(path))
    command = ('mkdir -p '+shlex.quote(remote_root)+' && cd '+shlex.quote(remote_root)
               +' && sg video -c '+shlex.quote(inner))
    start = time.monotonic()
    session.record('camera_request', remote=path)
    with (output/(name+'_camera.log')).open('x') as log:
        proc = subprocess.Popen(SSH+[command], stdout=log, stderr=subprocess.STDOUT)
        try:
            while proc.poll() is None:
                session.pump(.05)
                if abs(session.monitor.angles[4]-target)>1+1e-5:
                    raise RuntimeError('Endpoint drift > 1 degree during capture')
                if time.monotonic()-start > 25:
                    raise RuntimeError('Camera deadline exceeded; no next motion')
            if proc.returncode:
                raise RuntimeError('Camera capture failed; no next motion')
        finally:
            if proc.poll() is None:
                proc.terminate()
                try:
                    proc.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    proc.kill(); proc.wait()
    session.record('camera_complete', remote=path)
    return path


def run(args):
    output = args.output
    output.mkdir(parents=True, exist_ok=False)
    remote_root = '/home/ubuntu/kr260-camera/diagnostics/j4_steps_'+uuid.uuid4().hex[:12]
    result = dict(completed=False, stages=[], targets_deg=[0,1], max_step_deg=1,
                  endpoint_guard_deg=1, dwell_seconds=5, compensation=False,
                  cause_identified=False, remote_root=remote_root, camera_enabled=args.camera)
    s = ProbeSession(Path(__file__).resolve().parents[1]/'build/arm_control', args.nic, output, 5, 4)
    try:
        s.pump(3); s.monitor.check(time.monotonic()); check_home(s.monitor.angles)
        if not .5 <= s.monitor.angles[4] <= 1.5:
            raise RuntimeError('Expected current J4 near +1 degree; no automatic setup')
        result['initial_angles'] = list(s.monitor.angles)
        s.monitor.baseline = list(s.monitor.angles)
        s.monitor.target = s.monitor.angles[4]
        s.active = True
        for name, target in STAGES:
            item = dict(phase=name, target_deg=target)
            result['stages'].append(item)
            s.phase = name
            s.send(target)
            s.pump(3)
            item['at_3_seconds'] = endpoint(s, output, name, target)
            deadline = time.monotonic()+2
            while time.monotonic()<deadline:
                s.pump(min(.05, deadline-time.monotonic()))
                if abs(s.monitor.angles[4]-target)>1+1e-5:
                    raise RuntimeError('Endpoint error > 1 degree during dwell')
            item['at_5_seconds'] = endpoint(s, output, name, target)
            if args.camera:
                item['camera_path'] = capture(s, output, remote_root, name, target)
                item['after_camera'] = endpoint(s, output, name, target)
            item['completed'] = True
            print(name, json.dumps(item['at_5_seconds']), flush=True)
            (output/'summary.json').write_text(json.dumps(result, indent=2)+'\n')
        result['completed'] = True
    except (Exception, KeyboardInterrupt) as exc:
        result['error'] = str(exc) or type(exc).__name__
        print('STOP:',result['error'],flush=True)
    finally:
        result.update(final_angles=s.monitor.angles, commands=s.commands, receipts=s.acks,
                      max_non_target_drift=s.monitor.max_drift, last_target=s.monitor.target)
        s.close()
        (output/'summary.json').write_text(json.dumps(result, indent=2)+'\n')
    return 0 if result['completed'] else 1


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--execute', action='store_true')
    p.add_argument('--camera', action='store_true')
    p.add_argument('--nic', default='enp99s0')
    p.add_argument('--output', type=Path, required=True)
    args = p.parse_args()
    if args.execute:
        raise SystemExit(run(args))
    print('Plan only:', STAGES, '; 5 seconds per target, optional passive camera capture; no DDS.')
