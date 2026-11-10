#!/usr/bin/env python3
"""KR260 dual digital threshold recorder. Standard library desktop service."""
import argparse
from collections import deque
import csv
from datetime import datetime, timezone
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import io
import json
import math
from pathlib import Path
import queue
import re
import secrets
import shlex
import subprocess
import sys
import threading
import time
from urllib.parse import parse_qs, urlparse
import webbrowser
from tactile import Debouncer, integration_view

ROOT = Path(__file__).resolve().parent
FIELDS = ['id', 'wall_time', 't_s', 'dt_s', 'do1', 'do2', 'trigger1',
          'trigger2', 'source', 'chip', 'offset1', 'offset2', 'host',
          'polarity1', 'polarity2', 'requested_hz', 'gap', 'contact1', 'contact2',
          'sensor_connected', 'pl_verified', 'transport_lag_growth_ms']


def configuration(data, probe=False):
    source = data.get('source', 'ssh')
    if source not in ('ssh', 'local', 'demo'):
        raise ValueError('数据来源必须为 ssh、local 或 demo')
    host = str(data.get('host', 'pressure-kr260')).strip()
    if source == 'ssh' and not re.fullmatch(r'[A-Za-z0-9_][A-Za-z0-9_.@-]{0,200}', host):
        raise ValueError('SSH 地址应为已配置别名或 user@hostname，不包含命令、空格或端口参数')
    result = dict(source=source, host=host)
    if probe:
        return result
    chip = str(data.get('chip', '')).strip()
    if source != 'demo' and chip != 'auto' and not re.fullmatch(r'/dev/gpiochip[0-9]+', chip):
        raise ValueError('请通过检查结果填写真实 GPIO 控制器，例如 /dev/gpiochip2')
    offsets = data.get('offsets', [0, 1])
    if not isinstance(offsets, list) or len(offsets) != 2 or any(type(x) is not int or not 0 <= x <= 65535 for x in offsets) or offsets[0] == offsets[1]:
        raise ValueError('两路 offset 必须是不同的非负整数，不能填写 J21 物理脚号')
    rate = data.get('rate_hz', 100)
    if type(rate) not in (int, float) or not math.isfinite(rate) or not 1 <= rate <= 500:
        raise ValueError('采样频率范围为 1 至 500 Hz')
    polarity = data.get('polarity', ['unknown', 'unknown'])
    if not isinstance(polarity, list) or len(polarity) != 2 or any(x not in ('unknown', 'low', 'high') for x in polarity):
        raise ValueError('每路触发极性必须为 unknown、low 或 high')
    if source != 'demo' and data.get('mapping_confirmed') is not True:
        raise ValueError('请先核实 PL 映射及 J21 11/12 为输入，再确认映射复选框')
    result.update(chip=chip, offsets=offsets, rate_hz=float(rate), polarity=polarity,
                  mapping_confirmed=data.get('mapping_confirmed') is True)
    verified = data.get('channel_verified', [False, False])
    if not isinstance(verified, list) or len(verified) != 2 or any(type(v) is not bool for v in verified):
        raise ValueError('两路响应核实标记必须为布尔值')
    debounce, stale = data.get('debounce_ms', 30), data.get('stale_ms', 250)
    if type(debounce) not in (int, float) or not math.isfinite(debounce) or not 0 <= debounce <= 1000:
        raise ValueError('消抖时间为 0–1000 ms')
    if type(stale) not in (int, float) or not math.isfinite(stale) or not 50 <= stale <= 5000:
        raise ValueError('数据失效时间为 50–5000 ms')
    if stale < 2000 / rate:
        raise ValueError('失效时间至少覆盖两个采样周期')
    result.update(sensor_connected=data.get('sensor_connected') is True,
                  channel_verified=verified, debounce_ms=debounce, stale_ms=stale)
    enabled = data.get('enabled_channels', [True, True])
    if (not isinstance(enabled, list) or len(enabled) != 2
            or any(type(v) is not bool for v in enabled) or not any(enabled)):
        raise ValueError('请选择至少一个有效传感器通道')
    result['enabled_channels'] = enabled
    return result


def reader_command(config, probe=False):
    args = ['--probe'] if probe else ['--source', 'demo' if config['source'] == 'demo' else 'real',
        '--chip', config['chip'] or '/dev/gpiochip0', '--offsets', *map(str, config['offsets']),
        '--rate-hz', str(config['rate_hz'])]
    if not probe and config['source'] != 'demo':
        args.append('--verify-pl')
    if config['source'] == 'ssh':
        # OpenSSH passes the remote command through a shell; quote each argument.
        remote = shlex.join(['python3', '-u', '-', *args])
        return ['ssh', '-F', str(ROOT / 'ssh_config'), '-T', '-o', 'BatchMode=yes', '-o', 'StrictHostKeyChecking=yes',
                '-o', 'ConnectTimeout=5', '-o', 'ServerAliveInterval=5',
                '-o', 'ServerAliveCountMax=2', config['host'], remote], (ROOT / 'board_reader.py').read_bytes()
    return [sys.executable, '-u', str(ROOT / 'board_reader.py'), *args], None


class Recorder:
    def __init__(self, records=None):
        self.records = Path(records or ROOT / 'records')
        self.lock = threading.RLock()
        self.lifecycle = threading.Lock()
        self.thread = None
        self.stop_event = threading.Event()
        self.samples = deque(maxlen=30000)
        self.state = 'idle'
        self.error = ''
        self.session_id = ''
        self.config = {}
        self.count = 0
        self.latest = None
        self.edges = [0, 0]
        self.active_s = [0., 0.]
        self.csv_path = None
        self.csv_file = None
        self.writer = None
        self.rate_times = deque(maxlen=1000)
        self.debouncers = [Debouncer(), Debouncer()]
        self.pl_mapping = None
        self.received_at = self.first_received = None
        self.transport_lag_growth_ms = 0

    def tactile_status(self):
        with self.lock:
            c, last = self.config, self.latest
            age = None if self.received_at is None else max(0, (time.monotonic() - self.received_at) * 1000)
            limit = c.get('stale_ms', 250)
            fresh = bool(last and self.state == 'running' and age is not None and age <= limit
                         and self.transport_lag_growth_ms <= limit)
            real = c.get('source') in ('ssh', 'local')
            pl = bool(real and self.pl_mapping)
            connected = c.get('sensor_connected', False)
            channels, reasons = [], []
            if not connected: reasons.append('传感器未连接')
            if not fresh: reasons.append('没有新鲜采样')
            if not real: reasons.append('非真实采样')
            if not pl: reasons.append('PL → PS 接口尚未核实')
            if self.transport_lag_growth_ms > limit: reasons.append('传输延迟增长超过限制')
            for i in range(2):
                enabled = c.get('enabled_channels', [True, True])[i]
                polarity = c.get('polarity', ['unknown', 'unknown'])[i]
                checked = c.get('channel_verified', [False, False])[i]
                stable = self.debouncers[i].stable
                valid = bool(enabled and fresh and connected and pl and checked and polarity != 'unknown' and stable is not None)
                raw_contact = None if not last or polarity == 'unknown' else bool(last['trigger'+str(i+1)])
                channels.append(dict(channel=i+1, label='DO'+str(i+1), enabled=enabled, valid=valid,
                    level=last['do'+str(i+1)] if fresh else None,
                    raw_contact=raw_contact if enabled and fresh and connected and pl and checked else None,
                    contact=bool(stable) if valid else None, response_verified=checked, polarity=polarity))
            enabled = c.get('enabled_channels', [True, True])
            if any(e and p == 'unknown' for e,p in zip(enabled,c.get('polarity', ['unknown', 'unknown']))): reasons.append('启用通道的触发极性尚未确认')
            if any(e and not v for e,v in zip(enabled,c.get('channel_verified', [False, False]))): reasons.append('启用通道的响应尚未核实')
            if fresh and any(e and d.stable is None for e,d in zip(enabled,self.debouncers)): reasons.append('等待稳定电平')
            result = dict(schema='kr260-tactile/v1', source='real' if real else 'simulated' if c else None,
                session_id=self.session_id, seq=last['id'] if last else 0,
                sample_t_s=last['t_s'] if last else None, receive_age_ms=age,
                stale_ms=limit, debounce_ms=c.get('debounce_ms', 30),
                transport_lag_growth_ms=self.transport_lag_growth_ms,
                fresh=fresh, sensor_connected=connected, through_pl_ps=pl,
                valid=bool(any(enabled) and all(ch['valid'] for ch in channels if ch['enabled'])), channels=channels,
                reasons=reasons, pl_mapping=self.pl_mapping,
                grasp_verified=False, robot_motion_enabled=False)
            result.update(integration_view(result))
            return result

    def accept_pl_mapping(self, item):
        if (item.get('type') != 'pl_mapping' or item.get('verified') is not True
                or not re.fullmatch(r'/dev/gpiochip[0-9]+', str(item.get('chip', '')))
                or item.get('offsets') != [0, 1] or self.config['offsets'] != [0, 1]
                or item.get('of_node') != '/axi/gpio@a0000000'):
            raise ValueError('PL → PS 映射回执不匹配')
        if self.config['chip'] != 'auto' and self.config['chip'] != item['chip']:
            raise ValueError('PL GPIO 与所选控制器不一致')
        self.pl_mapping = item
        if self.csv_path:
            self.csv_path.with_suffix('.pl.json').write_text(json.dumps(item, ensure_ascii=False, indent=2), encoding='utf-8')

    def status(self):
        with self.lock:
            dt = self.rate_times[-1] - self.rate_times[0] if len(self.rate_times) > 1 else 0
            return dict(state=self.state, error=self.error, session_id=self.session_id,
                config=self.config, source=('simulated' if self.config.get('source') == 'demo' else 'real') if self.config else '',
                count=self.count, elapsed=self.latest['t_s'] if self.latest else 0,
                actual_hz=(len(self.rate_times) - 1) / dt if dt else 0,
                latest=self.latest, stats=dict(edges=self.edges[:], active_s=self.active_s[:]),
                csv_name=self.csv_path.name if self.csv_path else '', tactile=self.tactile_status())

    def start(self, data):
        conf = configuration(data)
        with self.lifecycle:
            if self.thread and self.thread.is_alive():
                raise ValueError('当前采集尚未结束，请先停止')
            with self.lock:
                self.config = conf
                self.samples.clear()
                self.latest = None
                self.count = 0
                self.edges = [0, 0]
                self.active_s = [0., 0.]
                self.rate_times.clear()
                self.pl_mapping = None
                self.received_at = self.first_received = None
                self.transport_lag_growth_ms = 0
                for debouncer in self.debouncers: debouncer.reset()
                self.error = ''
                self.session_id = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S_%fZ') + '_' + conf['source']
                try:
                    self.records.mkdir(parents=True, exist_ok=True)
                    self.csv_path = self.records / (self.session_id + '.csv')
                    self.csv_file = self.csv_path.open('x', encoding='utf-8-sig', newline='')
                    self.writer = csv.DictWriter(self.csv_file, fieldnames=FIELDS)
                    self.writer.writeheader()
                    self.csv_file.flush()
                    (self.records / (self.session_id + '.json')).write_text(json.dumps({
                        'version': 1, 'measurement': 'digital_threshold_only', 'config': conf,
                        'timebase': 'GPIO read completion on acquisition host monotonic clock; UTC wall clock may be unsynchronized',
                        'wiring': {'DO1': 'J21 pin 11 / AB14', 'DO2': 'J21 pin 12 / Y14'},
                        'mapping_verified_by': 'operator checkbox; not independently established by software',
                        'csv_fields': FIELDS}, ensure_ascii=False, indent=2), encoding='utf-8')
                except Exception as exc:
                    if self.csv_file:
                        self.csv_file.close()
                        self.csv_file = None
                    self.writer = None
                    self.state = 'error'
                    self.error = '采集文件准备失败：' + str(exc)
                    raise
                self.state = 'starting'
                self.stop_event.clear()
                self.thread = threading.Thread(target=self._run, daemon=True)
                self.thread.start()
        return self.status()

    def ingest(self, item):
        if not isinstance(item, dict):
            raise ValueError('采样端返回了非对象数据')
        expected = 'simulated' if self.config['source'] == 'demo' else 'real'
        if item.get('source') != expected:
            raise ValueError('采样来源标记与所选模式不一致，已停止')
        if expected == 'real' and not self.pl_mapping:
            raise ValueError('尚未收到已核实的 PL AXI GPIO → PS 映射')
        expected_chip = 'demo' if expected == 'simulated' else self.pl_mapping['chip']
        reported_offsets = item.get('offsets')
        if (item.get('chip') != expected_chip or not isinstance(reported_offsets, list)
                or any(type(offset) is not int for offset in reported_offsets)
                or reported_offsets != self.config['offsets']):
            raise ValueError('采样端 GPIO 控制器或通道顺序与配置不一致，已停止')
        if type(item.get('seq')) is not int or item['seq'] != self.count + 1:
            raise ValueError('采样序号不连续，已停止以保留错误证据')
        if any(type(item.get(k)) is not int or item[k] not in (0, 1) for k in ('do1', 'do2')):
            raise ValueError('数字输入必须为 0 或 1')
        t = item.get('t_s')
        if type(t) not in (int, float) or not math.isfinite(t) or t < 0:
            raise ValueError('无效采样时间戳')
        if not isinstance(item.get('wall_time'), str):
            raise ValueError('缺少 UTC 时间戳')
        if self.latest and t <= self.latest['t_s']:
            raise ValueError('单调时间戳没有递增')
        dt = t - self.latest['t_s'] if self.latest else 0
        gap = int(bool(self.latest) and dt > 2.5 / self.config['rate_hz'])
        now = time.monotonic()
        if self.first_received is None: self.first_received = (now, t)
        lag = max(0, ((now - self.first_received[0]) - (t - self.first_received[1])) * 1000)
        if gap or (self.received_at is not None and (now - self.received_at) * 1000 > self.config['stale_ms']):
            for debouncer in self.debouncers: debouncer.reset()
        triggers = [None if p == 'unknown' else int(item['do' + str(i + 1)] == (0 if p == 'low' else 1))
                    for i, p in enumerate(self.config['polarity'])]
        record = dict(id=item['seq'], t_s=t, wall_time=item['wall_time'], dt_s=dt,
                      do1=item['do1'], do2=item['do2'], trigger1=triggers[0], trigger2=triggers[1],
                      source=expected, chip=expected_chip if expected == 'real' else 'simulated',
                      offset1=self.config['offsets'][0], offset2=self.config['offsets'][1],
                      host=self.config['host'] if self.config['source'] == 'ssh' else 'local',
                      polarity1=self.config['polarity'][0], polarity2=self.config['polarity'][1],
                      requested_hz=self.config['rate_hz'], gap=gap,
                      sensor_connected=self.config['sensor_connected'], pl_verified=bool(self.pl_mapping),
                      transport_lag_growth_ms=lag)
        for i, trigger in enumerate(triggers):
            stable = self.debouncers[i].update(trigger, t, self.config['debounce_ms'])
            record['contact'+str(i+1)] = stable if (expected == 'real' and self.pl_mapping
                and self.config['sensor_connected'] and self.config['channel_verified'][i]
                and self.config['enabled_channels'][i]
                and lag <= self.config['stale_ms']) else None
        # All writes occur before presentation. Disk errors stop acquisition.
        self.writer.writerow(record)
        self.csv_file.flush()
        if self.latest and not gap:
            for i in range(2):
                prev = self.latest['trigger' + str(i + 1)]
                if prev == 0 and triggers[i] == 1:
                    self.edges[i] += 1
                if prev == 1:
                    self.active_s[i] += dt
        self.count += 1
        self.latest = record
        self.samples.append(record)
        self.rate_times.append(t)
        self.received_at, self.transport_lag_growth_ms = now, lag
        self.state = 'running'

    def _run(self):
        proc = None
        incoming = queue.Queue(maxsize=2048)
        errors = deque(maxlen=40)
        drain_stop = threading.Event()
        drain_threads = []
        def drain(stream, stderr=False):
            try:
                while not drain_stop.is_set():
                    line = stream.readline(16385)
                    if not line:
                        break
                    if stderr:
                        errors.append(line.decode('utf-8', errors='replace').strip())
                    else:
                        while not drain_stop.is_set() and not self.stop_event.is_set():
                            try:
                                incoming.put(line, timeout=.2)
                                break
                            except queue.Full:
                                continue
            finally:
                stream.close()
        try:
            cmd, payload = reader_command(self.config)
            proc = subprocess.Popen(cmd, stdin=subprocess.PIPE if payload is not None else subprocess.DEVNULL,
                                    stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            for args in ((proc.stdout,), (proc.stderr, True)):
                worker = threading.Thread(target=drain, args=args, daemon=True)
                drain_threads.append(worker)
                worker.start()
            if payload is not None:
                proc.stdin.write(payload)
                proc.stdin.close()
            last_sample = time.monotonic()
            while not self.stop_event.is_set():
                try:
                    line = incoming.get(timeout=.15)
                except queue.Empty:
                    if proc.poll() is not None:
                        raise RuntimeError('采样进程已退出：' + ('\n'.join(errors) or str(proc.returncode)))
                    if time.monotonic() - last_sample > 12:
                        raise RuntimeError('超过 12 秒没有新样本，连接或采样已中断')
                    continue
                if len(line) > 16384:
                    raise ValueError('采样端数据行过长')
                item = json.loads(line)
                if item.get('type') == 'pl_mapping':
                    with self.lock: self.accept_pl_mapping(item)
                    continue
                if item.get('type') == 'meta':
                    continue
                if item.get('type') == 'error':
                    raise RuntimeError(item.get('error', '采样端失败'))
                with self.lock:
                    self.ingest(item)
                last_sample = time.monotonic()
        except Exception as exc:
            with self.lock:
                if not self.stop_event.is_set():
                    self.state = 'error'
                    self.error = str(exc) + ('\n' + '\n'.join(errors) if errors and str(exc).find('采样进程已退出') < 0 else '')
        finally:
            drain_stop.set()
            if proc is not None and proc.poll() is None:
                proc.terminate()
                try:
                    proc.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    proc.kill()
                    proc.wait()
            for worker in drain_threads:
                worker.join(timeout=1)
            with self.lock:
                if self.csv_file:
                    self.csv_file.close()
                    self.csv_file = None
                if self.state != 'error':
                    self.state = 'stopped'
                if self.session_id:
                    try:
                        (self.records / (self.session_id + '.result.json')).write_text(
                            json.dumps(self.status(), ensure_ascii=False, indent=2), encoding='utf-8')
                    except OSError as exc:
                        self.state = 'error'
                        self.error = '结束记录写入失败：' + str(exc)

    def stop(self):
        with self.lifecycle:
            self.stop_event.set()
            if self.thread:
                self.thread.join(timeout=6)
            if self.thread and self.thread.is_alive():
                raise RuntimeError('采样进程仍在结束，请稍后重试')
        return self.status()

    def probe(self, data):
        conf = configuration(data, probe=True)
        if conf['source'] == 'demo':
            return {'output': '演示数据不读取 GPIO。请选择 SSH 或本机进行设备检查。'}
        cmd, payload = reader_command(conf, probe=True)
        result = subprocess.run(cmd, input=payload, capture_output=True, timeout=20)
        output = (result.stdout + result.stderr).decode('utf-8', errors='replace')
        return {'output': output or '没有返回信息', 'returncode': result.returncode}


def serve(recorder, port, open_browser=False):
    token = secrets.token_urlsafe(32)
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *_):
            pass

        def send(self, body, kind='application/json; charset=utf-8', status=200, extra=None):
            if not isinstance(body, bytes):
                body = json.dumps(body, ensure_ascii=False).encode('utf-8')
            self.send_response(status)
            self.send_header('Content-Type', kind)
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Cache-Control', 'no-store')
            self.send_header('X-Content-Type-Options', 'nosniff')
            self.send_header('X-Frame-Options', 'DENY')
            for key, value in (extra or {}).items():
                self.send_header(key, value)
            self.end_headers()
            try:
                self.wfile.write(body)
            except (BrokenPipeError, ConnectionResetError):
                pass

        def do_GET(self):
            parsed = urlparse(self.path)
            if parsed.path == '/':
                data = (ROOT / 'web/index.html').read_text(encoding='utf-8').replace('__API_TOKEN__', token)
                self.send(data.encode('utf-8'), 'text/html; charset=utf-8')
            elif parsed.path == '/api/status':
                self.send(recorder.status())
            elif parsed.path == '/api/tactile':
                self.send(recorder.tactile_status())
            elif parsed.path == '/api/profile':
                self.send(json.loads((ROOT / 'tactile_config.json').read_text(encoding='utf-8')))
            elif parsed.path == '/api/samples':
                try:
                    after = int(parse_qs(parsed.query).get('after', ['0'])[0])
                except ValueError:
                    return self.send({'error': '无效序号'}, status=400)
                with recorder.lock:
                    payload = dict(session_id=recorder.session_id, samples=[s for s in recorder.samples if s['id'] > after],
                        oldest_id=recorder.samples[0]['id'] if recorder.samples else 0,
                        state=recorder.state, error=recorder.error)
                self.send(payload)
            elif parsed.path == '/api/download':
                with recorder.lock:
                    path = recorder.csv_path
                    if not path or not path.exists():
                        return self.send({'error': '尚无采集文件'}, status=404)
                    if recorder.csv_file:
                        recorder.csv_file.flush()
                    size = path.stat().st_size
                # Stream a fixed snapshot without loading a long session into RAM.
                self.send_response(200)
                self.send_header('Content-Type', 'text/csv; charset=utf-8')
                self.send_header('Content-Length', str(size))
                self.send_header('Content-Disposition', 'attachment; filename="' + path.name + '"')
                self.send_header('Cache-Control', 'no-store')
                self.end_headers()
                try:
                    with path.open('rb') as f:
                        while size:
                            chunk = f.read(min(65536, size))
                            if not chunk:
                                break
                            self.wfile.write(chunk)
                            size -= len(chunk)
                except (BrokenPipeError, ConnectionResetError):
                    pass
            else:
                self.send({'error': '不存在'}, status=404)

        def do_POST(self):
            if self.headers.get('X-API-Token') != token:
                return self.send({'error': '无效控制令牌，请刷新本机页面'}, status=403)
            try:
                length = int(self.headers.get('Content-Length', '0'))
                if not 0 < length <= 16384:
                    raise ValueError('请求长度无效')
                data = json.loads(self.rfile.read(length))
                if not isinstance(data, dict):
                    raise ValueError('请求必须为 JSON 对象')
                if self.path == '/api/start':
                    result = recorder.start(data)
                elif self.path == '/api/stop':
                    result = recorder.stop()
                elif self.path == '/api/probe':
                    result = recorder.probe(data)
                else:
                    return self.send({'error': '不存在'}, status=404)
                self.send(result)
            except (ValueError, OSError, RuntimeError, subprocess.TimeoutExpired) as exc:
                self.send({'error': str(exc)}, status=400)

    httpd = ThreadingHTTPServer(('127.0.0.1', port), Handler)
    url = f'http://127.0.0.1:{httpd.server_port}'
    print('KR260 双路阈值采集界面：' + url, flush=True)
    print('退出请按 Ctrl+C。数据仅在点击开始后采集；演示模式始终标记 simulated。', flush=True)
    if open_browser:
        threading.Timer(.5, lambda: webbrowser.open(url)).start()
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        recorder.stop()
        httpd.server_close()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', type=int, default=8765)
    parser.add_argument('--open', action='store_true', help='打开本机浏览器')
    parser.add_argument('--records', type=Path, default=ROOT / 'records')
    options = parser.parse_args()
    recorder = Recorder(options.records)
    try:
        recorder.config = configuration(json.loads((ROOT / 'tactile_config.json').read_text(encoding='utf-8')))
    except (OSError, ValueError) as exc:
        print('默认配置未加载：' + str(exc), flush=True)
    serve(recorder, options.port, options.open)
