#!/usr/bin/env python3
"""Local display-only HTTP endpoint, backed by one feedback-only DDS reader.

GET /status reports received SDK feedback; GET / hot-reads the display HTML.
No controller process, command topic, motor initialization or normalization.
"""
import argparse
import http.client
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
from pathlib import Path
import signal
import subprocess
import threading
import time
from urllib.parse import urlsplit


CONTROL = Path(__file__).resolve().parents[1]
DEFAULT_PANEL = Path('/home/aup/Camera/scripts/d1_joint_panel.html')
PREVIEW_PAGE = Path('/home/aup/Camera/scripts/kr260_live.html')
CAMERA_PORTS = {'global': 18761, 'wrist': 18760}
CAMERA_PATHS = {'status', 'depth.raw', 'color.mjpg', 'color.jpg'}
STATUS_SEMANTICS = {
    'kind': 'communication_node_software_flags',
    'legacy_fields': {
        'power': 'Fresh raw power_status software variable; not measured motor supply voltage.',
        'enabled': 'Fresh raw enable_status software variable; not verified position hold or torque.',
        'fault': 'Fresh raw error_status software variable; zero does not confirm fault-free hardware.',
    },
    'evidence': {
        'source': '/home/aup/D1/Control/logs/gripper_recovery_20261023_201607/controller_source/marm_communication_node.cpp',
        'sha256': '32f8843a7ecec70d6fc82f09335185f39c5451a71946c9ddb9306c909df963bd',
        'power_status': 'funcode 6 sets power_status=1 and acknowledges; no hardware power action in this branch.',
        'enable_status': 'funcode 4/5 set enable_status from mode<1000 and publish damping requests; position/zero branches also set it before execution.',
        'error_status': 'This communication-node source initializes error_status=0 and publishes it; no hardware fault update is present.',
        'scope': 'Source-code interpretation; this display does not attest deployed binary identity, supply voltage or holding torque.',
    },
    'hardware_confirmation': 'Not available from these feedback fields. Fresh angles or protocol flags do not prove position holding.',
}


def proxy_target(path):
    parts = path.split('/')
    if len(parts) == 4 and parts[1] == 'camera' and parts[2] in CAMERA_PORTS and parts[3] in CAMERA_PATHS:
        return CAMERA_PORTS[parts[2]], '/'+parts[3]
    return None


def relay_chunks(response, output):
    # read1 returns currently available bytes without waiting for a full chunk;
    # never buffer a whole endless MJPEG response.
    while True:
        chunk = response.read1(65536)
        if not chunk:
            break
        output.write(chunk)
        output.flush()


class FeedbackState:
    def __init__(self, clock=time.monotonic, wall=time.time, max_age_s=1.0):
        self.clock, self.wall, self.max_age_s = clock, wall, max_age_s
        self.lock = threading.Lock()
        self.angles = None
        self.angle_time = self.angle_wall = None
        self.status = {}
        self.status_times = {}
        self.angle_count = self.message_count = self.invalid_count = 0
        self.raw_angles = self.raw_status = None

    def feed(self, row):
        """Consume an observer record; never trust partial/nonfinite angles."""
        now = self.clock()
        try:
            stamp = row['received_monotonic_s']
            message = row['message']
            if (isinstance(stamp, bool) or not isinstance(stamp, (float, int))
                    or not math.isfinite(stamp) or stamp < 0 or stamp > now + .01):
                raise ValueError('Invalid source receipt time')
            data = message['data']
            if not isinstance(data, dict):
                raise ValueError('Missing feedback fields')
        except (KeyError, ValueError, TypeError):
            with self.lock:
                self.invalid_count += 1
            return
        with self.lock:
            self.message_count += 1
            keys = [f'angle{i}' for i in range(7)]
            if any(key in data for key in keys):
                valid = all(isinstance(data.get(key), (int, float)) and not isinstance(data.get(key), bool)
                            and math.isfinite(data[key]) for key in keys)
                if valid and (self.angle_time is None or stamp >= self.angle_time):
                    self.angles = [float(data[key]) for key in keys]
                    self.angle_time = stamp
                    self.angle_wall = self.wall() - max(0., now-stamp)
                    self.angle_count += 1
                    self.raw_angles = message
                elif not valid:
                    self.invalid_count += 1
            for key in ('power_status', 'enable_status', 'error_status'):
                value = data.get(key)
                if (isinstance(value, (int, float)) and not isinstance(value, bool)
                        and math.isfinite(value) and (key == 'error_status' or value in (0, 1))
                        and stamp >= self.status_times.get(key, -1)):
                    self.status[key] = value
                    self.status_times[key] = stamp
                    self.raw_status = message

    def snapshot(self, reader_alive=True):
        now = self.clock()
        with self.lock:
            age = None if self.angle_time is None else max(0., now-self.angle_time)*1000
            fresh = reader_alive and age is not None and age <= self.max_age_s*1000
            status_age = {key: max(0., now-stamp)*1000 for key, stamp in self.status_times.items()}
            valid_status = {key: (reader_alive and value <= self.max_age_s*1000)
                            for key, value in status_age.items()}
            values = {key: self.status.get(key) if valid_status.get(key) else None
                      for key in ('power_status', 'enable_status', 'error_status')}
            reason = None if fresh else ('reader_stopped' if not reader_alive else
                                         'waiting_for_feedback' if age is None else 'feedback_stale')
            return dict(schema='d1-joint-display-v1', fresh=fresh, available=fresh,
                age_ms=age, received_at_unix_ms=None if self.angle_wall is None else self.angle_wall*1000,
                sample_time_basis='host DDS receipt; not a device timestamp',
                max_age_ms=self.max_age_s*1000, reader_alive=reader_alive, unavailable_reason=reason,
                joints=[dict(id=f'J{i}', position=self.angles[i] if fresh else None,
                             last_position=None if self.angles is None else self.angles[i],
                             unit='deg' if i < 6 else 'raw') for i in range(7)],
                power=values['power_status'], enabled=values['enable_status'], fault=values['error_status'],
                protocol_status_flags=dict(values),
                hardware_power_confirmed=None, hardware_enable_confirmed=None,
                position_hold_confirmed=None, hardware_fault_state_confirmed=None,
                status_semantics=STATUS_SEMANTICS,
                status_fresh=all(valid_status.get(key, False) for key in values),
                status_age_ms=status_age, last_status=dict(self.status),
                raw_feedback=dict(angles=self.raw_angles, status=self.raw_status),
                counters=dict(angle_frames=self.angle_count, messages=self.message_count, invalid=self.invalid_count),
                source=dict(topic='rt/arm_Feedback', domain=0, transport='DDS subscription only'),
                j6_note='Uncalibrated SDK raw units; no mm/degree conversion or wrap correction.',
                display_only=True, motion_readiness_assessed=False, robot_commands_sent=0)


class Reader:
    def __init__(self, binary, nic, state):
        self.state = state
        self.process = subprocess.Popen([str(binary), nic], stdin=subprocess.DEVNULL,
            stdout=subprocess.PIPE, stderr=None, text=True, bufsize=1)
        self.thread = threading.Thread(target=self.collect, daemon=True)
        self.thread.start()

    def collect(self):
        for line in self.process.stdout:
            try:
                self.state.feed(json.loads(line))
            except (ValueError, TypeError):
                # SDK diagnostics are not feedback and never refresh timestamps.
                continue

    def alive(self):
        return self.process.poll() is None and self.thread.is_alive()

    def close(self):
        if self.process.poll() is None:
            self.process.terminate()
            try:
                self.process.wait(timeout=2)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait(timeout=2)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--nic', default='enp99s0')
    parser.add_argument('--port', type=int, default=18763)
    parser.add_argument('--reader', type=Path, default=CONTROL/'build/d1_joint_monitor')
    parser.add_argument('--page', type=Path, default=DEFAULT_PANEL)
    args = parser.parse_args()
    state = FeedbackState()
    reader = None

    class Handler(BaseHTTPRequestHandler):
        def setup(self):
            super().setup()
            self.connection.settimeout(2)

        def log_message(self, *_):
            pass

        def reply(self, body, content_type, status=200):
            self.send_response(status)
            self.send_header('Content-Type', content_type)
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Cache-Control', 'no-store')
            self.send_header('X-Content-Type-Options', 'nosniff')
            self.end_headers()
            try:
                self.wfile.write(body)
            except (BrokenPipeError, ConnectionResetError):
                pass

        def do_GET(self):
            path = urlsplit(self.path).path
            target = proxy_target(path)
            if target is not None:
                upstream = http.client.HTTPConnection('127.0.0.1', target[0], timeout=2)
                headers_sent = False
                try:
                    upstream.request('GET', target[1])
                    response = upstream.getresponse()
                    self.send_response(response.status)
                    for name in ('Content-Type', 'Content-Length', 'X-Width', 'X-Height',
                                 'X-Scale', 'X-Frame-Index', 'X-Board-Age-Ms'):
                        value = response.getheader(name)
                        if value is not None:
                            self.send_header(name, value)
                    self.send_header('Cache-Control', 'no-store')
                    self.send_header('X-Content-Type-Options', 'nosniff')
                    self.end_headers()
                    headers_sent = True
                    relay_chunks(response, self.wfile)
                except (OSError, http.client.HTTPException):
                    if not headers_sent:
                        self.reply(b'Camera preview upstream unavailable', 'text/plain; charset=utf-8', 502)
                finally:
                    upstream.close()
                return
            if path == '/status':
                data = state.snapshot(reader is not None and reader.alive())
                data['source']['nic'] = args.nic
                self.reply(json.dumps(data, allow_nan=False).encode(), 'application/json; charset=utf-8')
            elif path in ('/', '/preview'):
                try:
                    page = args.page if path == '/' else PREVIEW_PAGE
                    self.reply(page.read_bytes(), 'text/html; charset=utf-8')
                except OSError:
                    self.reply(b'Joint display page is not available; GET /status provides JSON.',
                               'text/plain; charset=utf-8', 503)
            else:
                self.reply(b'Not found', 'text/plain; charset=utf-8', 404)

    # Bind before spawning the reader: an occupied port creates no second DDS
    # subscription. Every shutdown below affects this service's child only.
    server = ThreadingHTTPServer(('127.0.0.1', args.port), Handler)
    try:
        reader = Reader(args.reader, args.nic, state)
        for sig in (signal.SIGINT, signal.SIGTERM):
            signal.signal(sig, lambda *_: threading.Thread(target=server.shutdown, daemon=True).start())
        print(f'READ_ONLY_READY http://127.0.0.1:{args.port} topic=rt/arm_Feedback', flush=True)
        server.serve_forever(poll_interval=.2)
    finally:
        server.server_close()
        if reader is not None:
            reader.close()


if __name__ == '__main__':
    main()
