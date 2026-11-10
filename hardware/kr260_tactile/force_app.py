#!/usr/bin/env python3
"""Read-only IIO voltage/force recorder. No FPGA, GPIO or robot control writes."""
import argparse
import base64
from collections import deque
import csv
from datetime import datetime, timezone
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
import os
from pathlib import Path
import re
import secrets
import selectors
import shlex
import subprocess
import sys
import threading
import time
from urllib.parse import urlparse, parse_qs
import webbrowser

ROOT = Path(__file__).resolve().parent
FIELDS = ['seq', 't_s', 'wall_time', 'read_span_s', 'gap', 'source',
          'raw1', 'voltage1_v', 'force1_n', 'quality1',
          'raw2', 'voltage2_v', 'force2_n', 'quality2']


def number(x):
    if type(x) not in (int, float) or not math.isfinite(x):
        raise ValueError('需要有限数值')
    return x


def validate(c):
    if c.get('electrical_verified') is not True:
        raise ValueError('等待实物模拟测点、完整电压范围及 ADC 接线核实，尚未启用真实连续采集')
    if not str(c.get('verification_record', '')).strip():
        raise ValueError('缺少接线和电气验证记录')
    if not re.fullmatch(r'[A-Za-z0-9_][A-Za-z0-9_.@-]{0,200}', c.get('host', '')):
        raise ValueError('无效 SSH 别名')
    if not 1 <= number(c.get('rate_hz')) <= 50:
        raise ValueError('共享板卡采样率限制为 1–50 Hz')
    if not .2 <= number(c.get('stale_s')) <= 5 or c['stale_s'] < 2 / c['rate_hz']:
        raise ValueError('失效时间须为 0.2–5 s，且至少覆盖两个采样周期')
    channels = c.get('channels')
    if not isinstance(channels, list) or len(channels) != 2:
        raise ValueError('必须配置两个独立模拟通道')
    seen = set()
    for ch in channels:
        if ch.get('external_sensor_verified') is not True or not ch.get('sensor_id'):
            raise ValueError('必须核实外部传感器身份；板载电源监测通道不能作为压力输入')
        if not re.fullmatch(r'/sys/bus/iio/devices/iio:device\d+', ch.get('device', '')):
            raise ValueError('无效 IIO 路径')
        if not re.fullmatch(r'in_voltage\d+(?:-voltage\d+)?', ch.get('channel', '')):
            raise ValueError('需要经核实的电压通道')
        if not ch.get('device_name') or not ch.get('of_node', '').startswith('/sys/firmware/devicetree/base/'):
            raise ValueError('缺少 IIO 设备名称或设备树身份')
        key = (ch['device'], ch['channel'])
        if key in seen:
            raise ValueError('两个传感器不能使用相同 ADC 通道')
        seen.add(key)
        if not 0 <= number(ch.get('min_v')) < number(ch.get('max_v')) <= 1:
            raise ValueError('本入口仅支持已核实的 0–1 V 接地参考输入')
        if not (type(ch.get('raw_min')) is int and type(ch.get('raw_max')) is int
                and 0 <= ch['raw_min'] < ch['raw_max']):
            raise ValueError('原码范围须按真实驱动确认，不能假定 10 位读出格式')
        if number(ch.get('scale_mv')) <= 0:
            raise ValueError('缺少已核实的毫伏刻度')
        number(ch.get('offset_raw'))
        cal = ch.get('calibration', {})
        points = cal.get('points', [])
        if points:
            if len(points) < 3 or not cal.get('reference') or not cal.get('mounting'):
                raise ValueError('标定至少需要三组实测载荷、参考记录和安装条件')
            previous = None
            directions = []
            for v, f in points:
                if not ch['min_v'] <= number(v) <= ch['max_v'] or number(f) < 0:
                    raise ValueError('标定电压或力值超范围')
                if previous:
                    if v <= previous[0] or f == previous[1]:
                        raise ValueError('标定点电压须严格递增，力值须严格单调')
                    directions.append(f > previous[1])
                previous = (v, f)
            if len(set(directions)) != 1:
                raise ValueError('标定关系不单调，需检查测量与安装')
    return c


def interpolate(voltage, points):
    if not points or not points[0][0] <= voltage <= points[-1][0]:
        return None
    for (v0, f0), (v1, f1) in zip(points, points[1:]):
        if v0 <= voltage <= v1:
            return f0 + (voltage - v0) * (f1 - f0) / (v1 - v0)
    return None


def convert(raw, ch):
    if type(raw) is not int:
        raise ValueError('ADC 原码必须是整数')
    v = (raw + ch['offset_raw']) * ch['scale_mv'] / 1000
    if not ch['min_v'] <= v <= ch['max_v'] or not ch['raw_min'] <= raw <= ch['raw_max']:
        return v, None, 'out_of_range'
    if raw in (ch['raw_min'], ch['raw_max']):
        return v, None, 'adc_rail'
    points = ch.get('calibration', {}).get('points', [])
    force = interpolate(v, points)
    return v, force, ('uncalibrated' if not points else 'outside_calibration' if force is None else 'ok')


def verify_device(ch):
    device = Path(ch['device'])
    if (device / 'name').read_text().strip() != ch['device_name']:
        raise ValueError('IIO 设备名称变化，停止读取')
    if str((device / 'of_node').resolve(strict=True)) != ch['of_node']:
        raise ValueError('IIO 设备树身份变化，停止读取')
    prefix = ch['channel']
    def attribute(suffix, absent=None):
        for p in (device / (prefix + suffix), device / ('in_voltage' + suffix)):
            if p.exists():
                return float(p.read_text().strip())
        if absent is not None:
            return absent
        raise ValueError('缺少 ADC 刻度属性')
    if not math.isclose(attribute('_scale'), ch['scale_mv'], rel_tol=1e-8, abs_tol=1e-12):
        raise ValueError('ADC 刻度与已核实记录不一致')
    if not math.isclose(attribute('_offset', 0), ch['offset_raw'], abs_tol=1e-10):
        raise ValueError('ADC 偏置与已核实记录不一致')
    return device / (prefix + '_raw')


def read_board(c):
    validate(c)
    os.nice(10)
    paths = [verify_device(ch) for ch in c['channels']]
    start = due = time.monotonic()
    seq = 0
    while True:
        time.sleep(max(0, due - time.monotonic()))
        begin = time.monotonic()
        raw = [int(path.read_text().strip()) for path in paths]
        end = time.monotonic()
        seq += 1
        print(json.dumps(dict(seq=seq, t_s=end-start, read_span_s=end-begin,
                              wall_time=datetime.now(timezone.utc).isoformat(),
                              source='real_iio', raw=raw)), flush=True)
        due += 1 / c['rate_hz']
        if due <= end:
            due = end + 1 / c['rate_hz']


class Recorder:
    def __init__(self, config_path, records=None):
        self.config_path = Path(config_path)
        self.records = Path(records or ROOT / 'records/analog')
        self.lock = threading.RLock()
        self.lifecycle = threading.Lock()
        self.stop_event = threading.Event()
        self.thread = None
        self.samples = deque(maxlen=6000)
        self.state, self.error = 'waiting', ''
        self.latest = None
        self.received_at = None
        self.first_received = None
        self.config = {}
        self.path = None
        self.session = ''
        self.writer = None
        self.file = None

    def readiness(self):
        try:
            validate(json.loads(self.config_path.read_text()))
            return True, ''
        except (ValueError, OSError, TypeError, KeyError) as exc:
            return False, str(exc)

    def status(self):
        with self.lock:
            ready, reason = self.readiness()
            age = None if self.received_at is None else time.monotonic() - self.received_at
            fresh = self.state == 'running' and age is not None and age <= self.config['stale_s']
            last = self.latest
            channels = []
            for i in (1, 2):
                good = bool(fresh and last and last['quality'+str(i)] in ('ok', 'uncalibrated', 'outside_calibration'))
                channels.append(dict(channel=i, voltage_valid=good,
                    voltage_v=last['voltage'+str(i)+'_v'] if good else None,
                    force_n=last['force'+str(i)+'_n'] if good else None,
                    force_valid=bool(good and last['quality'+str(i)] == 'ok'),
                    quality=last['quality'+str(i)] if fresh and last else 'no_fresh_data'))
            return dict(state=self.state, error=self.error, ready=ready, waiting_reason=reason,
                session=self.session, count=last['seq'] if last else 0, latest=last,
                host_receive_age_s=age, fresh=fresh, channels=channels,
                timing='Sequential sysfs reads; conversion time and absolute network latency not verified',
                robot_control_ready=False, source='real_iio' if last else None,
                csv_name=self.path.name if self.path else '', rate_hz=self.config.get('rate_hz'))

    def ingest(self, item):
        with self.lock:
            if item.get('source') != 'real_iio' or type(item.get('seq')) is not int:
                raise ValueError('不是有效的真实 IIO 数据帧')
            if item['seq'] != (self.latest['seq'] + 1 if self.latest else 1):
                raise ValueError('帧序号不连续')
            t = number(item.get('t_s'))
            span = number(item.get('read_span_s'))
            if t < 0 or span < 0 or span > t or (self.latest and t <= self.latest['t_s']):
                raise ValueError('无效或倒退的采样时间')
            if not isinstance(item.get('wall_time'), str):
                raise ValueError('缺少采样端 UTC 时间')
            raw = item.get('raw')
            if not isinstance(raw, list) or len(raw) != 2:
                raise ValueError('需要两路 ADC 原码')
            now = time.monotonic()
            if self.first_received is None:
                self.first_received = (now, t)
            lag_growth = (now - self.first_received[0]) - (t - self.first_received[1])
            if lag_growth > self.config['stale_s']:
                raise ValueError('相对传输延迟增长超过失效时间，停止向机器人提供有效值')
            record = {k: item[k] for k in ('seq', 't_s', 'wall_time', 'read_span_s', 'source')}
            record['gap'] = int(bool(self.latest) and t - self.latest['t_s'] > 2.5 / self.config['rate_hz'])
            for i, (r, ch) in enumerate(zip(raw, self.config['channels']), 1):
                v, f, q = convert(r, ch)
                record.update({f'raw{i}': r, f'voltage{i}_v': v, f'force{i}_n': f, f'quality{i}': q})
            self.writer.writerow(record)
            self.file.flush()
            self.latest, self.received_at = record, now
            self.samples.append(record)
            self.state = 'running'

    def start(self):
        with self.lifecycle:
            if self.thread and self.thread.is_alive():
                raise ValueError('采集已在运行')
            conf = validate(json.loads(self.config_path.read_text()))
            with self.lock:
                self.config = conf
                self.samples.clear()
                self.latest = self.received_at = self.first_received = None
                self.session = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S_%fZ') + '_real_iio'
                self.records.mkdir(parents=True, exist_ok=True)
                self.path = self.records / (self.session + '.csv')
                self.path.with_suffix('.json').write_text(json.dumps(conf, ensure_ascii=False, indent=2))
                self.file = self.path.open('x', encoding='utf-8-sig', newline='')
                self.writer = csv.DictWriter(self.file, fieldnames=FIELDS)
                self.writer.writeheader()
                self.file.flush()
                self.error, self.state = '', 'starting'
                self.stop_event.clear()
                self.thread = threading.Thread(target=self._run, daemon=True)
                self.thread.start()

    def _run(self):
        proc = None
        stderr = deque(maxlen=20)
        try:
            encoded = base64.b64encode(json.dumps(self.config).encode()).decode()
            cmd = ['ssh', '-F', str(ROOT / 'ssh_config'), '-T', '-o', 'BatchMode=yes',
                   '-o', 'StrictHostKeyChecking=yes', '-o', 'ConnectTimeout=5',
                   '-o', 'ServerAliveInterval=3', '-o', 'ServerAliveCountMax=2', self.config['host'],
                   shlex.join(['python3', '-u', '-', '--reader', encoded])]
            proc = subprocess.Popen(cmd, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            proc.stdin.write(Path(__file__).read_bytes())
            proc.stdin.close()
            def drain():
                for line in proc.stderr:
                    stderr.append(line.decode(errors='replace').strip())
            err_thread = threading.Thread(target=drain, daemon=True)
            err_thread.start()
            pending = b''
            last_rx = time.monotonic()
            with selectors.DefaultSelector() as selector:
                selector.register(proc.stdout, selectors.EVENT_READ)
                while not self.stop_event.is_set():
                    if time.monotonic() - last_rx > max(6, self.config['stale_s'] * 3):
                        raise ValueError('SSH 没有新样本，采集已停止')
                    if not selector.select(.1):
                        continue
                    block = os.read(proc.stdout.fileno(), 65536)
                    if not block:
                        err_thread.join(.3)
                        raise ValueError('采样进程退出：' + ' / '.join(stderr)[-1600:])
                    last_rx = time.monotonic()
                    pending += block
                    if len(pending) > 262144:
                        raise ValueError('接收积压超过上限')
                    while b'\n' in pending:
                        line, pending = pending.split(b'\n', 1)
                        if len(line) > 8192:
                            raise ValueError('数据帧过长')
                        self.ingest(json.loads(line))
        except Exception as exc:
            with self.lock:
                self.error, self.state = str(exc), 'error'
        finally:
            if proc:
                if proc.poll() is None:
                    proc.terminate()
                    try:
                        proc.wait(2)
                    except subprocess.TimeoutExpired:
                        proc.kill()
                        proc.wait(2)
                for stream in (proc.stdin, proc.stdout, proc.stderr):
                    if stream and not stream.closed:
                        stream.close()
            with self.lock:
                if self.file:
                    self.file.close()
                if self.state != 'error':
                    self.state = 'stopped'

    def stop(self):
        with self.lifecycle:
            self.stop_event.set()
            if self.thread:
                self.thread.join(4)


def serve(port, config, open_browser=False):
    recorder = Recorder(config)
    token = secrets.token_urlsafe(32)
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *_):
            pass

        def send(self, value, status=200, kind='application/json; charset=utf-8'):
            body = value if isinstance(value, bytes) else json.dumps(value, ensure_ascii=False, allow_nan=False).encode()
            self.send_response(status)
            self.send_header('Content-Type', kind)
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Cache-Control', 'no-store')
            self.send_header('X-Content-Type-Options', 'nosniff')
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            url = urlparse(self.path)
            if url.path == '/':
                self.send((ROOT / 'web/force.html').read_bytes().replace(b'__TOKEN__', token.encode()), kind='text/html; charset=utf-8')
            elif url.path in ('/api/status', '/api/tactile'):
                self.send(recorder.status())
            elif url.path == '/api/samples':
                try:
                    after = max(0, int(parse_qs(url.query).get('after', ['0'])[0]))
                except ValueError:
                    self.send({'error': '无效序号'}, 400)
                    return
                with recorder.lock:
                    rows = [row for row in recorder.samples if row['seq'] > after]
                    self.send({'session': recorder.session, 'samples': rows})
            elif url.path == '/api/download':
                with recorder.lock:
                    if recorder.path and recorder.path.exists():
                        path = recorder.path
                    else:
                        self.send({'error': '尚无真实连续采样记录'}, 404)
                        return
                # Stream a fixed byte snapshot without retaining a long recording in RAM.
                with path.open('rb') as stream:
                    remaining = os.fstat(stream.fileno()).st_size
                    self.send_response(200)
                    self.send_header('Content-Type', 'text/csv; charset=utf-8')
                    self.send_header('Content-Length', str(remaining))
                    self.send_header('Cache-Control', 'no-store')
                    self.end_headers()
                    while remaining:
                        block = stream.read(min(65536, remaining))
                        if not block:
                            break
                        self.wfile.write(block)
                        remaining -= len(block)
            else:
                self.send({'error': 'not found'}, 404)

        def do_POST(self):
            if self.headers.get('X-Token') != token:
                self.send({'error': '令牌不匹配'}, 403)
                return
            try:
                if self.path == '/api/start':
                    recorder.start()
                elif self.path == '/api/stop':
                    recorder.stop()
                else:
                    self.send({'error': 'not found'}, 404)
                    return
                self.send(recorder.status())
            except (ValueError, OSError, TypeError, KeyError) as exc:
                self.send({'error': str(exc)}, 400)
    server = ThreadingHTTPServer(('127.0.0.1', port), Handler)
    print(f'连续压力采集界面 http://127.0.0.1:{port}/ （默认等待真实模拟通路核实）', flush=True)
    if open_browser:
        webbrowser.open(f'http://127.0.0.1:{port}/')
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        recorder.stop()
        server.server_close()


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--reader')
    parser.add_argument('--port', type=int, default=8765)
    parser.add_argument('--config', default=str(ROOT / 'analog_config.json'))
    parser.add_argument('--open', action='store_true')
    args = parser.parse_args()
    if args.reader:
        try:
            read_board(json.loads(base64.b64decode(args.reader)))
        except BrokenPipeError:
            pass
    else:
        os.nice(10)
        serve(args.port, args.config, args.open)
