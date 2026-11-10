#!/usr/bin/env python3
"""Local measured-state recorder. The only device program is a subscriber.

There is no motor command, enable, damping, power, or playback endpoint here.
"""
import argparse
from collections import deque
import csv
from datetime import datetime
import fcntl
import hashlib
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
from pathlib import Path
import secrets
import signal
import subprocess
import threading
import time
from urllib.parse import urlsplit
from zoneinfo import ZoneInfo

HERE = Path(__file__).resolve().parent
DEFAULT_MONITOR = Path('/media/aup/UbuntuData/ACT训练/02_程序与源码/D1/Control/build/d1_joint_monitor')
DEFAULT_OUTPUT = Path('/media/aup/UbuntuData/抓取实验/手动示教')
UNITS = ['deg'] * 6 + ['sdk_raw']

def finite(x):
    return isinstance(x, (float, int)) and not isinstance(x, bool) and math.isfinite(x)

def atomic_json(path, data):
    tmp = path.with_suffix(path.suffix + '.tmp')
    tmp.write_text(json.dumps(data, ensure_ascii=False, indent=2, allow_nan=False))
    tmp.replace(path)

class Journal:
    def __init__(self, path, clock=time.monotonic):
        self.path = Path(path)
        self.path.mkdir(parents=True, exist_ok=False)
        self.clock = clock
        self.lock = threading.RLock()
        self.started = clock()
        self.state = 'starting'
        self.source_kind = 'manual_demonstration_method_not_yet_confirmed'
        self.token = secrets.token_urlsafe(24)
        self.raw = (self.path / 'feedback_raw.jsonl').open('x', buffering=1)
        self.angles = (self.path / 'joint_samples.jsonl').open('x', buffering=1)
        self.events = (self.path / 'events.jsonl').open('x', buffering=1)
        self.csvfile = (self.path / 'joint_timeline.csv').open('x', newline='', buffering=1)
        self.csv = csv.writer(self.csvfile)
        self.csv.writerow(['sample', 'elapsed_s', 'dds_received_monotonic_s', 'host_unix_s', 'dt_s', 'gap_before',
                           *[f'J{i}_position_{u}' for i, u in enumerate(UNITS)],
                           *[f'J{i}_velocity_{u}_per_s' for i, u in enumerate(UNITS)],
                           *[f'J{i}_velocity_smoothed_{u}_per_s' for i, u in enumerate(UNITS)]])
        self.tail = deque(maxlen=6000)
        self.window = deque()
        self.previous = None
        self.sample_count = self.message_count = self.invalid_count = self.gap_count = 0
        self.nonmonotonic_count = 0
        self.max_gap = 0.0
        self.first_stamp = self.last_stamp = None
        self.qmin = self.qmax = None
        self.max_abs_speed = [0.] * 7
        self.protocol_status = {}
        self.protocol_status_times = {}
        self.stopped = None
        self.stop_reason = None
        self.error = None
        self.process = None
        self.collect_thread = None
        self.stop_cutoff = None
        self.excluded_after_cutoff = 0
        self.metadata = {
            'schema': 'd1-measured-manual-demonstration-v1',
            'trajectory_kind': 'measured_manual_demonstration',
            'created_at': datetime.now(ZoneInfo('Asia/Singapore')).isoformat(),
            'host_monotonic_start_s': self.started, 'joint_order': [f'J{i}' for i in range(7)],
            'joint_units': UNITS, 'velocity_source': 'Estimated from received joint positions and host DDS receipt times, not measured motor velocity',
            'velocity_smoothing': 'Endpoint slope over latest <=0.4 seconds; reset after >0.5 second gap',
            'time_basis': 'host DDS reception time; device encoder timestamps unavailable',
            'no_angle_unwrap_or_clipping': True, 'commands_recorded': False,
            'robot_commands_sent': 0, 'hardware_mode_changed': False,
            'camera_recorded': False, 'full_vla_training_dataset': False,
            'replay_executed': False,
            'replay_requires': 'Separate reviewed trajectory, current start-state/limits/path/velocity checks and explicit user approval',
            'status_flags_are': 'Communication software flags; not proof of torque, position holding, supply voltage, or hardware fault freedom',
        }
        atomic_json(self.path / 'metadata.json', self.metadata)
        self.event('recorder_started', note='Includes waiting/pre-roll until the user begins; no automatic trimming.')

    def event(self, label, **extra):
        with self.lock:
            if self.events.closed:
                return
            self.events.write(json.dumps({'elapsed_s': self.clock()-self.started,
                                         'host_monotonic_s': self.clock(), 'label': label,
                                         **extra}, ensure_ascii=False, allow_nan=False) + '\n')

    def feed(self, row):
        with self.lock:
            if self.stopped is not None:
                return
            now = self.clock()
            try:
                stamp = row['received_monotonic_s']
                msg = row['message']
                data = msg['data']
                if not finite(stamp) or stamp > now + .05 or stamp < 0 or not isinstance(data, dict):
                    raise ValueError('Invalid receipt timestamp or payload')
                json.dumps(row, allow_nan=False)
            except (KeyError, TypeError, ValueError):
                self.invalid_count += 1
                return
            if self.stop_cutoff is not None and stamp > self.stop_cutoff:
                self.excluded_after_cutoff += 1
                return
            self.raw.write(json.dumps(row, ensure_ascii=False, allow_nan=False) + '\n')
            self.message_count += 1
            for key in ('power_status', 'enable_status', 'error_status'):
                if finite(data.get(key)):
                    self.protocol_status[key] = data[key]
                    self.protocol_status_times[key] = stamp
            if not any(f'angle{i}' in data for i in range(7)):
                return
            if not all(finite(data.get(f'angle{i}')) for i in range(7)):
                self.invalid_count += 1
                return
            q = [float(data[f'angle{i}']) for i in range(7)]
            if self.last_stamp is not None and stamp <= self.last_stamp:
                self.nonmonotonic_count += 1
                return
            dt = None if self.last_stamp is None else stamp - self.last_stamp
            gap = dt is not None and dt > .5
            raw_v = [None] * 7
            if dt is not None:
                self.max_gap = max(self.max_gap, dt)
                if not gap:
                    raw_v = [(q[i] - self.previous['q'][i]) / dt for i in range(7)]
            if gap:
                self.gap_count += 1
                self.window.clear()
                self.event('feedback_gap', gap_s=dt)
            self.window.append((stamp, q))
            while len(self.window) > 1 and stamp-self.window[0][0] > .4:
                self.window.popleft()
            smooth_v = [None] * 7
            if len(self.window) > 1:
                first_t, first_q = self.window[0]
                smooth_v = [(q[i] - first_q[i])/(stamp-first_t) for i in range(7)]
            self.sample_count += 1
            if self.first_stamp is None:
                self.first_stamp = stamp
                self.qmin = q.copy(); self.qmax = q.copy()
            self.qmin = [min(a,b) for a,b in zip(self.qmin,q)]
            self.qmax = [max(a,b) for a,b in zip(self.qmax,q)]
            for i, v in enumerate(raw_v):
                if v is not None:
                    self.max_abs_speed[i] = max(self.max_abs_speed[i], abs(v))
            sample = {'sample': self.sample_count, 'elapsed_s': stamp-self.started,
                      'dds_received_monotonic_s': stamp, 'host_unix_s': time.time()-(now-stamp),
                      'q': q, 'dt_s': dt, 'gap_before': gap, 'velocity': raw_v,
                      'velocity_smoothed': smooth_v, 'protocol_status_flags': self.protocol_status.copy(),
                      'protocol_status_received_monotonic_s': self.protocol_status_times.copy(),
                      'protocol_status_age_s': {k:max(0., stamp-t) for k,t in self.protocol_status_times.items()}}
            self.angles.write(json.dumps(sample, allow_nan=False) + '\n')
            self.csv.writerow([self.sample_count, sample['elapsed_s'], stamp, sample['host_unix_s'], dt, gap,
                               *q, *raw_v, *smooth_v])
            self.previous = sample
            self.tail.append({'t': sample['elapsed_s'], 'q': q, 'v': smooth_v, 'gap': gap})
            self.last_stamp = stamp
            self.state = 'recording' if self.stop_cutoff is None else 'stopping'

    def status(self):
        with self.lock:
            now = self.clock()
            age = None if self.last_stamp is None else max(0., now-self.last_stamp)*1000
            alive = self.process is None or self.process.poll() is None
            fresh = self.stop_cutoff is None and self.stopped is None and alive and age is not None and age < 500
            span = 0 if self.first_stamp is None else self.last_stamp-self.first_stamp
            rate = (self.sample_count-1)/span if span > 0 else 0
            return {'state': self.state, 'ready': fresh and self.sample_count >= 20,
                    'fresh': fresh, 'age_ms': age, 'reader_alive': alive,
                    'elapsed_s': (self.stopped if self.stopped is not None else now)-self.started,
                    'sample_count': self.sample_count, 'rate_hz': rate,
                    'q': None if self.previous is None else self.previous['q'],
                    'velocity': None if self.previous is None else self.previous['velocity_smoothed'],
                    'path': str(self.path), 'source_kind': self.source_kind, 'csrf_token': self.token,
                    'quality': {'gap_count': self.gap_count, 'invalid_count': self.invalid_count,
                                'nonmonotonic_count': self.nonmonotonic_count, 'max_gap_s': self.max_gap},
                    'tail': list(self.tail)[-1200:], 'error': self.error,
                    'protocol_status_flags': self.protocol_status.copy(), 'robot_commands_sent': 0,
                    'protocol_status_age_s': {k:max(0., now-t) for k,t in self.protocol_status_times.items()},
                    'stop_reason': self.stop_reason}

    def sync(self):
        with self.lock:
            for f in (self.raw, self.angles, self.events, self.csvfile):
                if not f.closed:
                    f.flush()
                    import os
                    os.fsync(f.fileno())
            s = self.status(); s.pop('tail'); s.pop('csrf_token')
            atomic_json(self.path / 'status.json', s)

    def stop(self, reason):
        with self.lock:
            if self.stopped is not None or self.stop_cutoff is not None:
                return self.status()
            self.event('grasp_recording_ended', reason=reason,
                       note='End of recording is not an independently verified grasp success label.')
            self.stop_reason = reason
            self.stop_cutoff = self.clock()
            self.state = 'stopping'
            if self.process is not None and self.process.poll() is None:
                self.process.terminate()  # Verified subscriber: this sends no robot command.
        # Let the reader consume already-received frames up to the cutoff. Do
        # not hold the journal lock while it drains the subprocess pipe.
        drain_complete = True
        if self.collect_thread is not None and self.collect_thread is not threading.current_thread():
            self.collect_thread.join(timeout=3)
            drain_complete = not self.collect_thread.is_alive()
        with self.lock:
            self.stopped = self.stop_cutoff
            self.state = 'stopped'
            summary = self.status(); summary.pop('tail'); summary.pop('csrf_token')
            summary.update({'joint_min': self.qmin, 'joint_max': self.qmax,
                            'max_abs_velocity': self.max_abs_speed,
                            'trajectory_kind': 'measured_manual_demonstration',
                            'grasp_success_verified': False, 'replay_executed': False,
                            'commands_recorded': False, 'camera_recorded': False,
                            'stop_cutoff_monotonic_s': self.stop_cutoff,
                            'subscriber_pipe_drained': drain_complete,
                            'messages_excluded_after_cutoff': self.excluded_after_cutoff,
                            'sample_coverage_note': 'Valid frames consumed up to explicit stop cutoff; see pipe drain status. Transport-level packet loss cannot be proven absent.'})
            atomic_json(self.path / 'summary.json', summary)
            self.sync()
            for f in (self.raw, self.angles, self.events, self.csvfile):
                f.close()
            return self.status()

def main():
    p = argparse.ArgumentParser()
    p.add_argument('--monitor', type=Path, default=DEFAULT_MONITOR)
    p.add_argument('--nic', default='enp99s0')
    p.add_argument('--port', type=int, default=18770)
    p.add_argument('--output-root', type=Path, default=DEFAULT_OUTPUT)
    args = p.parse_args()
    HERE.mkdir(exist_ok=True)
    lock = (HERE / 'recorder.lock').open('a')
    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    journal = None
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *_): pass
        def reply(self, data, code=200, ctype='application/json; charset=utf-8'):
            b = data if isinstance(data, bytes) else json.dumps(data, ensure_ascii=False, allow_nan=False).encode()
            self.send_response(code); self.send_header('Content-Type', ctype)
            self.send_header('Content-Length', str(len(b))); self.send_header('Cache-Control', 'no-store')
            self.send_header('X-Content-Type-Options', 'nosniff'); self.end_headers()
            try: self.wfile.write(b)
            except (BrokenPipeError, ConnectionResetError): pass
        def do_GET(self):
            path = urlsplit(self.path).path
            if path == '/api/status': self.reply(journal.status())
            elif path == '/': self.reply((HERE/'panel.html').read_bytes(), ctype='text/html; charset=utf-8')
            else: self.reply({'error':'not found'},404)
        def do_POST(self):
            if not secrets.compare_digest(self.headers.get('X-Recorder-Token',''), journal.token):
                self.reply({'error':'invalid local recorder token'},403); return
            try:
                length = int(self.headers.get('Content-Length','0'))
                if not 0 <= length <= 4096: raise ValueError('body length')
                data = json.loads(self.rfile.read(length) or b'{}')
            except (ValueError, TypeError): self.reply({'error':'invalid body'},400); return
            path = urlsplit(self.path).path
            if path == '/api/stop': self.reply(journal.stop(str(data.get('reason','user_grasp_end'))[:250]))
            elif path == '/api/mark':
                label = str(data.get('label','manual_marker'))[:250]
                journal.event(label)
                if data.get('source_kind') in ('software_jog', 'verified_drag_teaching'):
                    journal.source_kind = data['source_kind']
                    journal.metadata['source_kind'] = journal.source_kind
                    atomic_json(journal.path/'metadata.json', journal.metadata)
                self.reply({'ok':True})
            else: self.reply({'error':'not found'},404)
    # Bind before opening the subscriber so a port conflict has no device side effects.
    server = ThreadingHTTPServer(('127.0.0.1', args.port), Handler)
    session = datetime.now(ZoneInfo('Asia/Singapore')).strftime('demo_%Y%m%d_%H%M%S')
    journal = Journal(args.output_root / session)
    monitor_hash = hashlib.sha256(args.monitor.read_bytes()).hexdigest()
    journal.metadata.update({'reader_binary': str(args.monitor.resolve()), 'reader_sha256': monitor_hash,
                             'nic': args.nic, 'source_topic': 'rt/arm_Feedback'})
    atomic_json(journal.path/'metadata.json', journal.metadata)
    diagnostics = (journal.path/'subscriber_diagnostics.log').open('a')
    journal.process = subprocess.Popen([str(args.monitor), args.nic], stdin=subprocess.DEVNULL,
                                       stdout=subprocess.PIPE, stderr=diagnostics, text=True, bufsize=1)
    import os
    atomic_json(HERE/'active_session.json', {'pid':os.getpid(), 'reader_pid':journal.process.pid,
                'url':f'http://127.0.0.1:{args.port}', 'path':str(journal.path), 'csrf_token':journal.token})
    def collect():
        try:
            for line in journal.process.stdout:
                try: row = json.loads(line)
                except ValueError:
                    diagnostics.write(line); diagnostics.flush(); continue
                journal.feed(row)
        finally:
            if journal.stopped is None and journal.stop_cutoff is None:
                journal.error = 'Feedback subscriber exited unexpectedly'
                journal.stop('reader_exited')
                journal.state = 'error'
                journal.sync()
    def flush_loop():
        while journal.stopped is None:
            time.sleep(1)
            journal.sync()
    journal.collect_thread = threading.Thread(target=collect, daemon=True)
    journal.collect_thread.start()
    threading.Thread(target=flush_loop, daemon=True).start()
    for sig in (signal.SIGTERM, signal.SIGINT):
        signal.signal(sig, lambda *_: threading.Thread(target=server.shutdown, daemon=True).start())
    print(f'RECORDER_INTERFACE http://127.0.0.1:{args.port} SESSION {journal.path}', flush=True)
    try:
        server.serve_forever(poll_interval=.2)
    finally:
        journal.stop('recorder_service_shutdown')
        server.server_close()
        if journal.process.poll() is None:
            journal.process.terminate()
        try: journal.process.wait(timeout=3)
        except subprocess.TimeoutExpired: journal.process.kill()
        diagnostics.close()

if __name__ == '__main__':
    main()
