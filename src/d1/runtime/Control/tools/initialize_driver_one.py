"""One authorized power/enable command; no position or automatic next action.

Includes SDK joint 6 (gripper). Does not normalize feedback or change position limits.
Adapted from the recorded 2026-10-22 one-command initialization helper.
"""
import argparse
import json
from pathlib import Path
import queue
import signal
import subprocess
import sys
import threading
import time

sys.path.insert(0, '/home/aup/D1/Control/tools')
from handeye_joint_step import Cameras
from j4_smooth_test import Monitor

p = argparse.ArgumentParser()
p.add_argument('--action', choices=['power-on', 'enable'], required=True)
p.add_argument('--joint', type=int, choices=range(7))
p.add_argument('--expected', type=float, nargs=7, required=True)
p.add_argument('--output', type=Path, required=True)
p.add_argument('--execute', action='store_true')
a = p.parse_args()
if (a.action == 'enable') != (a.joint is not None):
    p.error('Joint is required only for individual enable')
command = 'power on' if a.action == 'power-on' else f'enable {a.joint}'
if not a.execute:
    print(json.dumps({'command': command, 'post_observe_s': 5, 'motion_target_sent': False}))
    raise SystemExit(0)
a.output.mkdir(exist_ok=False)
report = dict(command=command, commands_sent=0, confirmed=False, passed=False,
              motion_target_sent=False, automatic_retry=False)
cameras = Cameras(a.output / 'frames')
proc = None
monitor = Monitor()
events = queue.Queue()
baseline = None
origin = time.monotonic()
with (a.output / 'events.jsonl').open('x') as log:
    def record(**kw):
        log.write(json.dumps(dict(t=time.monotonic()-origin, **kw))+'\n')
        log.flush()

    def pump(seconds, require_ready=True):
        end = time.monotonic()+seconds
        while time.monotonic() < end:
            try:
                stamp, line = events.get(timeout=.02)
                if line is None:
                    raise RuntimeError('Control process exited')
                record(line=line)
                if '[rx] ' in line:
                    monitor.observe(json.loads(line.split('[rx] ', 1)[1]), stamp)
                if line.startswith('[ok] '):
                    report['confirmed'] = True
                if line.startswith(('[设备拒绝]', '[执行/状态未确认]', '[接收超时]', '[未发送]', '[失败]')):
                    raise RuntimeError(line)
            except queue.Empty:
                pass
            cameras.check()
            now = time.monotonic()
            if require_ready:
                if monitor.angles is None or now-monitor.angle_time > .6:
                    raise RuntimeError('Stale angles')
                for key in ['power_status', 'enable_status', 'error_status']:
                    if key not in monitor.status or now-monitor.status_times[key] > .6:
                        raise RuntimeError('Stale status: '+key)
                if monitor.status['error_status'] != 0:
                    raise RuntimeError('Device error')
                if baseline is not None:
                    drift = max(abs(x-y) for x,y in zip(monitor.angles, baseline))
                    report['max_joint_drift'] = max(drift, report.get('max_joint_drift', 0))
                    if drift > .5+1e-5:
                        raise RuntimeError('Joint changed by more than 0.5')
                if (a.action == 'enable' or report['confirmed']) and monitor.status['power_status'] != 1:
                    raise RuntimeError('Motor power not ready')
    try:
        deadline = time.monotonic()+5
        while not cameras.ready:
            if cameras.error or time.monotonic() > deadline:
                cameras.check()
            time.sleep(.02)
        proc = subprocess.Popen(['/home/aup/D1/Control/build/arm_control', '--nic', 'enp99s0',
            '--joints', 'all', '--ack', 'exec', '--timeout-ms', '1000', '--verbose'],
            stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
            text=True, bufsize=1)
        def read():
            for line in proc.stdout:
                events.put((time.monotonic(), line.rstrip()))
            events.put((time.monotonic(), None))
        threading.Thread(target=read, daemon=True).start()
        pump(2, False)
        pump(.1)
        if any(abs(x-y) > .3+1e-5 for x,y in zip(monitor.angles, a.expected)):
            raise RuntimeError('Pose differs from reviewed start')
        if a.action == 'power-on' and monitor.status['power_status'] != 0:
            raise RuntimeError('Power already changed; review before sending')
        baseline = list(monitor.angles)
        report['baseline'] = baseline
        record(sent_command=command)
        proc.stdin.write(command+'\n')
        proc.stdin.flush()
        report['commands_sent'] = 1
        deadline = time.monotonic()+2
        while not report['confirmed'] and time.monotonic() < deadline:
            pump(.05)
        if not report['confirmed']:
            raise RuntimeError('No command confirmation')
        pump(5)
        report['passed'] = True
    except (Exception, KeyboardInterrupt) as e:
        report['error'] = str(e) or type(e).__name__
    finally:
        report.update(final_angles=monitor.angles, final_status=monitor.status)
        if proc is not None and proc.poll() is None:
            proc.send_signal(signal.SIGINT)
            try:
                proc.wait(timeout=2)
            except subprocess.TimeoutExpired:
                proc.terminate()
        cameras.close()
        (a.output/'summary.json').write_text(json.dumps(report, indent=2)+'\n')
print(json.dumps(report, indent=2))
raise SystemExit(0 if report['passed'] else 1)
