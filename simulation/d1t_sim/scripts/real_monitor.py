#!/usr/bin/env python3
"""Display live SDK feedback with a subscriber-only child; HTTP cannot send commands."""
import argparse
import functools
import hashlib
import http.server
import json
import math
import pathlib
import signal
import subprocess
import threading
import time
from urllib.parse import urlsplit

ROOT = pathlib.Path(__file__).resolve().parents[1]


class Feedback:
    def __init__(self, config, candidate, model, clock=time.monotonic):
        self.config, self.clock = config, clock
        self.lock = threading.Lock()
        self.started = clock()
        self.last_angles = self.last_status = None
        self.angles = self.status = None
        self.frames = self.invalid = 0
        self.process = None
        self.signs = candidate['mapping']['signs']
        self.offsets = candidate['mapping']['zero_offsets_deg']
        if len(self.signs) != 6 or any(x not in (-1, 1) for x in self.signs):
            raise ValueError('Invalid preview sign mapping')
        if len(self.offsets) != 6 or not all(self.number(x) for x in self.offsets):
            raise ValueError('Invalid preview offsets')
        self.limits = {j['name']: j['limits'] for j in model['joints'] if j['type'] == 'revolute'}
        self.mapping_hash = hashlib.sha256(pathlib.Path(config['mapping_candidate']).read_bytes()).hexdigest()

    @staticmethod
    def number(value):
        return not isinstance(value, bool) and isinstance(value, (int, float)) and math.isfinite(value)

    def ingest(self, line):
        try:
            message = json.loads(line)
            if not isinstance(message, dict):
                raise ValueError('Expected object')
            if message.get('address') != 2:
                return  # Receipt acknowledgements cannot refresh measured state.
            code, data = message.get('funcode'), message.get('data')
            if code not in (1, 3):
                return
            if not isinstance(data, dict):
                raise ValueError('Expected data object')
            if code == 1:
                values = [data[f'angle{i}'] for i in range(7)]
                if not all(self.number(v) for v in values):
                    raise ValueError('Incomplete/nonfinite angles')
                with self.lock:
                    self.angles = [float(v) for v in values]
                    self.last_angles = self.clock()
                    self.frames += 1
            else:
                values = {k: data[k] for k in ('power_status', 'enable_status', 'error_status')}
                if any(not isinstance(v, int) or isinstance(v, bool) for v in values.values()):
                    raise ValueError('Invalid status')
                if values['power_status'] not in (0, 1) or values['enable_status'] not in (0, 1):
                    raise ValueError('Invalid power/enable status')
                with self.lock:
                    self.status = values
                    self.last_status = self.clock()
        except (ValueError, TypeError, KeyError):
            with self.lock:
                self.invalid += 1

    def state(self):
        with self.lock:
            now = self.clock()
            age = None if self.last_angles is None else max(0., now - self.last_angles)
            status_age = None if self.last_status is None else max(0., now - self.last_status)
            alive = self.process is not None and self.process.poll() is None
            connected = alive and all(a is not None and a <= self.config['max_age_s'] for a in (age, status_age))
            positions = {} if self.angles is None else {
                f'arm_Joint{i+1}': math.radians(self.signs[i] * self.angles[i] + self.offsets[i])
                for i in range(6)}
            warnings = []
            for name, value in positions.items():
                limit = self.limits[name]
                if not limit['lower'] <= value <= limit['upper']:
                    warnings.append(f'{name} 候选映射超出模型限位（未裁剪实际读数）')
            if self.status and self.status['error_status'] != 0:
                warnings.append('实机报告错误码：' + str(self.status['error_status']))
            return dict(source='hardware_readonly', connected=bool(connected), age=age,
                        status_age=status_age, reader_alive=alive,
                        angles_deg=self.angles, status=self.status, positions=positions,
                        mapping=dict(preview_only=True, description=self.config['description'],
                                     source=self.config['mapping_candidate'], sha256=self.mapping_hash),
                        limits_warning=warnings, frames=self.frames, invalid_frames=self.invalid,
                        uptime_s=now-self.started, command_capable=False,
                        max_age_s=self.config['max_age_s'], interface=self.config['interface'])

    def read(self):
        for line in self.process.stdout:
            self.ingest(line)


class Handler(http.server.SimpleHTTPRequestHandler):
    def __init__(self, *args, feedback, **kwargs):
        self.feedback = feedback
        super().__init__(*args, directory=str(ROOT), **kwargs)

    def log_message(self, *args):
        pass

    def reply(self, value, status=200):
        body = json.dumps(value, ensure_ascii=False, allow_nan=False).encode()
        self.send_response(status)
        self.send_header('Content-Type', 'application/json; charset=utf-8')
        self.send_header('Cache-Control', 'no-store')
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        path = urlsplit(self.path).path
        if path == '/api/state':
            return self.reply(self.feedback.state())
        if path == '/':
            self.path = '/web/real.html'
        elif not path.startswith(('/web/', '/generated/', '/meshes/')):
            return self.reply({'error': 'not found'}, 404)
        # Resolve paths before allowing static access; no home files or directory listings.
        resolved = pathlib.Path(self.translate_path(self.path)).resolve()
        if not resolved.is_relative_to(ROOT) or not resolved.is_file():
            return self.reply({'error': 'not found'}, 404)
        return super().do_GET()

    def do_POST(self):
        self.reply({'error': '当前为实机只读监视，未提供运动、使能或抓取接口。'}, 403)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=8766)
    parser.add_argument('--reader', type=pathlib.Path, default=ROOT/'hardware/build/d1t_feedback_reader')
    args = parser.parse_args()
    config = json.loads((ROOT/'config/hardware_monitor.json').read_text())
    candidate = json.loads(pathlib.Path(config['mapping_candidate']).read_text())
    model = json.loads((ROOT/'generated/model.json').read_text())
    feedback = Feedback(config, candidate, model)
    server = http.server.ThreadingHTTPServer(('127.0.0.1', args.port), functools.partial(Handler, feedback=feedback))
    # Child has no command reader on stdin and no DDS command publisher.
    feedback.process = subprocess.Popen([str(args.reader), config['interface']], stdin=subprocess.DEVNULL,
                                         stdout=subprocess.PIPE, text=True, bufsize=1)
    threading.Thread(target=feedback.read, daemon=True).start()
    def stop(*_):
        raise SystemExit(0)
    signal.signal(signal.SIGTERM, stop)
    print(f'D1-T 实机只读反馈：http://127.0.0.1:{args.port}', flush=True)
    try:
        server.serve_forever()
    finally:
        server.server_close()
        feedback.process.terminate()
        try:
            feedback.process.wait(timeout=3)
        except subprocess.TimeoutExpired:
            feedback.process.kill()
            feedback.process.wait()


if __name__ == '__main__':
    main()
