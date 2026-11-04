#!/usr/bin/env python3
"""Bounded J4 sweep using the existing CLI; SDK stays unchanged.
Configurable 5/10/20 Hz quintic references, receipt gated, independent feedback guards.
No automatic enable, zero, retry, compensation, or release.
"""
import argparse
import json
import math
from pathlib import Path
import queue
import signal
import subprocess
import threading
import time

JOINT_LIMITS = ((-135,135), (-90,90), (-90,90), (-135,135), (-90,90), (-135,135), (0,65))

def profile(start, end, seconds, elapsed):
    u = min(1., max(0., elapsed / seconds))
    return start + (end-start) * (10*u**3 - 15*u**4 + 6*u**5)


def check_start_pose(start, expected=None, speed_probe=False):
    if not math.isfinite(start) or abs(start)>90:
        raise RuntimeError('Measured start is outside command limits')
    if expected is None:
        if abs(start)>5:
            raise RuntimeError('Expected start within ±5 degrees; no motion sent')
    elif not math.isfinite(expected) or abs(expected)>90 or abs(start-expected)>1:
        raise RuntimeError('Measured start differs from explicitly expected pose by >1 degree')
    if speed_probe and start+10>90:
        raise RuntimeError('Speed probe would exceed the J4 limit')


class Monitor:
    def __init__(self, joint=4):
        if type(joint) is not int or not 0 <= joint <= 6:
            raise ValueError('Joint must be an integer in 0..6')
        self.joint = joint
        self.angles = None
        self.angle_time = None
        self.status = {}
        self.status_times = {}
        self.baseline = None
        self.target = None
        self.max_drift = 0.

    def observe(self, message, now):
        if message.get('address') != 2:
            return
        data = message.get('data', {})
        if message.get('funcode') == 1:
            a = [float(data[f'angle{i}']) for i in range(7)]
            if not all(math.isfinite(v) for v in a):
                raise RuntimeError('Non-finite feedback')
            self.angles, self.angle_time = a, now
        if message.get('funcode') == 3:
            for key in ('enable_status', 'power_status', 'error_status'):
                if key in data:
                    self.status[key] = data[key]
                    self.status_times[key] = now

    def check(self, now):
        if self.angle_time is None or now-self.angle_time > .6:
            raise RuntimeError('Angle feedback stale > 0.6 s')
        for key, required in [('enable_status',1),('power_status',1),('error_status',0)]:
            if self.status.get(key) != required or now-self.status_times.get(key, -1e9) > .6:
                raise RuntimeError('Invalid/stale status: '+key)
        low,high = JOINT_LIMITS[self.joint]
        margin = .5 if self.joint == 6 else 1
        lo,hi = low-margin,high+margin
        if not lo <= self.angles[self.joint] <= hi:
            raise RuntimeError(f'J{self.joint} feedback outside [{lo}, {hi}]')
        if self.baseline is not None:
            drift = max(abs(a-b) for i,(a,b) in enumerate(zip(self.angles,self.baseline)) if i!=self.joint)
            self.max_drift = max(self.max_drift, drift)
            if drift > .5+1e-6:
                raise RuntimeError('Non-target joint drift > 0.5')
        if self.target is not None and abs(self.angles[self.joint]-self.target)>3:
            unit='degrees' if self.joint<6 else 'SDK units'
            raise RuntimeError(f'J{self.joint} tracking error > 3 {unit}')


class Session:
    def __init__(self, binary, nic, output, rate_hz=5, joint=4):
        if rate_hz not in (5,10,20):
            raise ValueError('Supported rates: 5, 10, 20 Hz')
        self.rate_hz = rate_hz
        self.joint = joint
        self.value_suffix = 'deg' if joint<6 else 'sdk_units'
        self.output = output
        self.log = (output/'events.jsonl').open('x')
        self.origin = time.monotonic()
        self.monitor = Monitor(joint)
        self.events = queue.Queue()
        self.phase = 'preflight'
        self.acks = 0
        self.commands = 0
        self.completed_segments = []
        self.active = False
        self.proc = subprocess.Popen([str(binary),'--nic',nic,'--joints',str(joint),
            '--ack','recv','--timeout-ms','500','--verbose'],stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,bufsize=1)
        def read():
            for line in self.proc.stdout:
                self.events.put((time.monotonic(),line.rstrip()))
            self.events.put((time.monotonic(),None))
        self.reader = threading.Thread(target=read,daemon=True)
        self.reader.start()

    def record(self, kind, **data):
        recorded_at = time.monotonic()
        self.log.write(json.dumps(dict(t=recorded_at-self.origin,
            host_monotonic=recorded_at,session_origin_monotonic=self.origin,
            phase=self.phase,kind=kind,**data))+'\n')
        self.log.flush()

    def pump(self, duration):
        deadline = time.monotonic()+duration
        while time.monotonic()<deadline:
            try:
                stamp, line = self.events.get(timeout=min(.03,max(.001,deadline-time.monotonic())))
            except queue.Empty:
                if self.active:
                    self.monitor.check(time.monotonic())
                continue
            if line is None:
                raise RuntimeError('Controller exited unexpectedly')
            self.record('output',received_t=stamp-self.origin,line=line)
            if '[rx] ' in line:
                self.monitor.observe(json.loads(line.split('[rx] ',1)[1]),stamp)
            if line.startswith('[ok] '):
                self.acks += 1
            if line.startswith(('[设备拒绝]','[保护停止]','[反馈中断]','[接收超时]','[未发送]','[失败]')):
                raise RuntimeError(line)
            if self.active:
                self.monitor.check(time.monotonic())

    def send(self, target):
        self.monitor.check(time.monotonic())
        target = round(target,3)
        lo,hi = JOINT_LIMITS[self.joint]
        if not math.isfinite(target) or not lo<=target<=hi:
            raise RuntimeError(f'J{self.joint} target outside [{lo}, {hi}]')
        if self.monitor.target is not None and abs(target-self.monitor.target)>1:
            raise RuntimeError('Reference step > 1 position unit')
        before = self.acks
        self.record('command',joint=self.joint,**{'target_'+self.value_suffix:target})
        self.monitor.target = target
        self.proc.stdin.write(f'j {self.joint} {target}\n')
        self.proc.stdin.flush()
        self.commands += 1
        deadline = time.monotonic()+.6
        while self.acks==before:
            self.pump(.0025)
            if time.monotonic()>deadline:
                raise RuntimeError('Receipt deadline exceeded; no next command')

    def segment(self, name, start, end, seconds):
        self.phase = name
        self.record('segment',joint=self.joint,duration_s=seconds,rate_hz=self.rate_hz,
                    **{'start_'+self.value_suffix:start,'end_'+self.value_suffix:end})
        begin = time.monotonic()
        for step in range(int(seconds*self.rate_hz)+1):
            due = begin+step/self.rate_hz
            if due>time.monotonic():
                self.pump(due-time.monotonic())
            if time.monotonic()-due > .5/self.rate_hz:
                raise RuntimeError('Command schedule missed > half a period; no catch-up burst')
            self.send(profile(start,end,seconds,step/self.rate_hz))
            if step%(30*self.rate_hz)==0:
                print(f'{name}: t={step/self.rate_hz:.1f}s target={self.monitor.target:.3f} actual={self.monitor.angles[self.joint]:.3f}',flush=True)
        self.phase = name+'_settle'
        self.pump(3)
        tolerance=1 if self.joint<6 else .5
        if abs(self.monitor.angles[self.joint]-end)>tolerance+1e-5:
            raise RuntimeError(f'Endpoint error > {tolerance} position units; stop next phase')
        self.completed_segments.append(name)
        self.record('segment_complete',**{'endpoint_'+self.value_suffix:self.monitor.angles[self.joint]})
        print(f'{name} endpoint actual={self.monitor.angles[self.joint]:.3f}',flush=True)

    def close(self):
        # On failure only halt future commands; do not unload a gravity-loaded arm.
        self.active = False
        if self.proc.poll() is None:
            self.proc.send_signal(signal.SIGINT)
            try:
                self.proc.wait(timeout=3)
            except subprocess.TimeoutExpired:
                self.proc.kill()
                self.proc.wait()
        self.reader.join(timeout=1)
        self.proc.stdin.close()
        self.proc.stdout.close()
        self.log.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--nic',default='enp99s0')
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--rate-hz',type=int,choices=(5,10,20),default=5)
    parser.add_argument('--rate-probe',action='store_true',help='First hold the initial target for 5 s to verify command rate')
    parser.add_argument('--speed-scale',type=int,choices=(1,2),default=1,help='2 halves segment times; doubles reference speed, quadruples reference acceleration')
    parser.add_argument('--setup-speed-scale',type=int,choices=(1,2),help='Override speed scaling for preparation and return only')
    parser.add_argument('--expected-start-deg',type=float,help='Explicit measured start, verified within ±1 degree; omission requires start near zero')
    parser.add_argument('--speed-probe',action='store_true',help='First test a 10-degree out-and-back move, 16/speed-scale seconds per leg')
    parser.add_argument('--execute',action='store_true',help='Required to send real hardware commands')
    args = parser.parse_args()
    setup_scale = args.setup_speed_scale or args.speed_scale
    if args.expected_start_deg is not None and (not math.isfinite(args.expected_start_deg) or abs(args.expected_start_deg)>90):
        parser.error('--expected-start-deg must be finite and within ±90')
    if not args.execute:
        print(f'Plan only: measured start→−90° ({60/setup_scale:g} s), −90°→+90° ({120/args.speed_scale:g} s), +90°→0° ({60/setup_scale:g} s); {args.rate_hz} Hz quintic; no compensation; rate probe={args.rate_probe}; speed probe={args.speed_probe}.')
        return
    args.output.mkdir(parents=True,exist_ok=False)
    session = Session(Path(__file__).resolve().parents[1]/'build/arm_control',args.nic,args.output,args.rate_hz)
    summary = dict(completed=False,compensation=False,rate_hz=args.rate_hz,rate_probe=args.rate_probe,requested_range_deg=[-90,90],
                   speed_scale=args.speed_scale,speed_probe=args.speed_probe,
                   setup_speed_scale=setup_scale,expected_start_deg=args.expected_start_deg,
                   planned_sweep_peak_velocity_deg_s=2.8125*args.speed_scale,
                   note='Host arrival timestamps, SDK encoder measurements; receipt is not motion completion.')
    try:
        session.pump(3)
        session.monitor.check(time.monotonic())
        start = session.monitor.angles[4]
        check_start_pose(start,args.expected_start_deg,args.speed_probe)
        session.monitor.baseline = list(session.monitor.angles)
        session.monitor.target = start
        session.active = True
        if args.rate_probe:
            session.segment('rate_probe',start,start,5)
        if args.speed_probe:
            session.segment('speed_probe_out',start,start+10,16/args.speed_scale)
            session.segment('speed_probe_back',start+10,start,16/args.speed_scale)
        session.segment('prepare',start,-90,60/setup_scale)
        session.segment('sweep',-90,90,120/args.speed_scale)
        session.segment('return',90,0,60/setup_scale)
        summary['completed'] = True
    except (Exception,KeyboardInterrupt) as exc:
        summary['error'] = str(exc) or type(exc).__name__
        print('STOP:',summary['error'],flush=True)
    finally:
        summary.update(final_angles=session.monitor.angles,commands_sent=session.commands,
                       receipts=session.acks,non_target_max_drift=session.monitor.max_drift,
                       completed_segments=session.completed_segments,final_phase=session.phase,
                       last_commanded_target_deg=session.monitor.target)
        session.close()
        (args.output/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    if not summary['completed']:
        raise SystemExit(1)

if __name__=='__main__':
    main()
