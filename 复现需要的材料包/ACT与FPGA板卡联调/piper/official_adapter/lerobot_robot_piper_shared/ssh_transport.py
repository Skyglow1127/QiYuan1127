"""Receive CAN frames over an authenticated SSH stdio channel; no CAN TX API."""
import json
import os
import selectors
import shlex
import subprocess
import time

PROTOCOL = 'piper-can-rx-v1'


def ssh_command(config, probe=False):
    transport = config['transport']
    target = transport['ssh_host']
    if not isinstance(target, str) or target.startswith('-') or not target:
        raise ValueError('Invalid SSH host')
    command = ['ssh', '-T', '-o', 'BatchMode=yes', '-o', 'StrictHostKeyChecking=yes',
               '-o', 'ConnectTimeout=5', '-o', 'ServerAliveInterval=5',
               '-o', 'ServerAliveCountMax=2', '-o', 'LogLevel=ERROR']
    if transport.get('ssh_hostname'):
        command += ['-o', 'HostName='+transport['ssh_hostname']]
    remote = ['python3', '-u', transport['remote_script'], '--interface', config['interface']]
    if probe:
        remote.append('--probe')
    return command + [target, shlex.join(remote)]


def remote_probe(config):
    result = subprocess.run(ssh_command(config, probe=True), capture_output=True,
                            text=True, timeout=10)
    if result.returncode:
        raise RuntimeError('KR260 CAN 探测失败：'+result.stderr.strip()[-500:])
    message = json.loads(result.stdout)
    if message.get('protocol') != PROTOCOL or message.get('type') != 'hello':
        raise RuntimeError('KR260 CAN 协议不符')
    return message


class SourceClock:
    """Bound remote monotonic time using four timestamps, never arrival freshness.

    The actual remote-minus-host offset lies between lower and upper. Choosing
    upper maps a frame to its earliest possible host time, conservatively aging
    it by network uncertainty. A small drift allowance grows until the next ping.
    """
    def __init__(self, max_rtt_ms=50, max_age_s=6):
        self.max_rtt_ns = int(max_rtt_ms*1e6)
        self.max_age_ns = int(max_age_s*1e9)
        self.upper = self.lower = self.synced_at = None
        self.rtt_ns = None

    def update(self, sent, received, remote_receive, remote_send):
        duration = received-sent
        processing = remote_send-remote_receive
        if not 0 <= processing <= duration or not 0 < duration <= self.max_rtt_ns:
            raise RuntimeError('CAN 时钟校准往返延迟过大或时间无效')
        lower, upper = remote_send-received, remote_receive-sent
        if self.synced_at is not None:
            drift = int((received-self.synced_at)*.0002)+1_000_000
            if lower > self.upper+drift or upper < self.lower-drift:
                raise RuntimeError('KR260 单调时钟发生不连续变化')
        self.lower, self.upper = lower, upper
        self.synced_at, self.rtt_ns = received, duration-processing

    def host_time(self, remote_ns, received_ns):
        if self.synced_at is None or not 0 <= received_ns-self.synced_at <= self.max_age_ns:
            raise RuntimeError('CAN 源时钟未校准或校准已过期')
        drift = int((received_ns-self.synced_at)*.0002)
        mapped = remote_ns-self.upper-drift
        if mapped > received_ns:
            raise RuntimeError('CAN 源时间位于未来')
        return mapped/1e9

    def status(self):
        return {'method': 'SSH four timestamps, earliest possible host time',
                'network_uncertainty_ms': None if self.upper is None else (self.upper-self.lower)/1e6,
                'rtt_ms': None if self.rtt_ns is None else self.rtt_ns/1e6,
                'drift_bound_ppm': 200}


def receive_remote(bus):
    """One SSH generation. Return only on stop, raise on disconnect or invalid data."""
    expected = bus.config['transport'].get('expected_identity')
    if not expected:
        raise RuntimeError('尚未绑定 KR260 接收端身份')
    clock = SourceClock(bus.config['transport'].get('max_clock_rtt_ms', 50))
    proc = subprocess.Popen(ssh_command(bus.config), stdin=subprocess.PIPE,
                            stdout=subprocess.PIPE, stderr=subprocess.PIPE, bufsize=0)
    with bus.lock:
        bus.remote_process = proc
    selector = selectors.DefaultSelector()
    selector.register(proc.stdout, selectors.EVENT_READ, 'stdout')
    selector.register(proc.stderr, selectors.EVENT_READ, 'stderr')
    buffer, diagnostics, pending = b'', b'', {}
    hello = False
    sequence = 0
    last_source = None
    started = last_output = time.monotonic()
    last_link = None
    next_ping = started
    try:
        while not bus.stop_event.is_set():
            now = time.monotonic()
            if (hello and now-last_output > 2) or (not hello and now-started > 5):
                raise TimeoutError('KR260 CAN 数据通道超时')
            if hello and last_link is not None and now-last_link > 1.5:
                raise TimeoutError('KR260 CAN 接口状态已过期')
            if hello and now >= next_ping:
                token = str(time.monotonic_ns())
                pending[token] = time.monotonic_ns()
                proc.stdin.write((json.dumps({'type': 'ping', 'token': token})+'\n').encode())
                proc.stdin.flush()
                next_ping = now+2
            if pending and now-min(pending.values())/1e9 > 1:
                raise TimeoutError('KR260 CAN 时钟校准超时')
            for key, _ in selector.select(.1):
                data = os.read(key.fileobj.fileno(), 65536)
                if key.data == 'stderr':
                    if data:
                        diagnostics = (diagnostics+data)[-2000:]
                    else:
                        selector.unregister(key.fileobj)
                    continue
                if not data:
                    raise RuntimeError('KR260 CAN SSH 连接断开：'+diagnostics.decode(errors='replace')[-500:])
                buffer += data
                if len(buffer) > 1_000_000:
                    raise RuntimeError('KR260 CAN 消息超出长度限制')
                while b'\n' in buffer:
                    line, buffer = buffer.split(b'\n', 1)
                    received = time.monotonic_ns()
                    message = json.loads(line)
                    if message.get('protocol') != PROTOCOL:
                        raise RuntimeError('KR260 CAN 协议不符')
                    last_output = received/1e9
                    kind = message.get('type')
                    if kind == 'hello':
                        if hello or message['identity'] != expected:
                            raise RuntimeError('KR260 CAN 接收端身份已改变，请重新核验')
                        if not message['link'].get('ready'):
                            raise RuntimeError('KR260 can0 需要 UP + LISTEN-ONLY + 1 Mbps')
                        with bus.lock:
                            bus.link = message['link']
                            bus.latest.clear()
                            bus.generation += 1
                            bus.error = '正在校准 KR260 CAN 源时钟'
                        hello, last_link = True, received/1e9
                    elif not hello:
                        raise RuntimeError('KR260 CAN 缺少初始身份信息')
                    elif kind == 'pong':
                        sent = pending.pop(message['token'])
                        clock.update(sent, received, message['remote_receive_ns'], message['remote_send_ns'])
                        with bus.lock:
                            bus.error = None
                            bus.transport_status = dict(clock.status(), source='ssh', identity=expected)
                    elif kind == 'status':
                        with bus.lock:
                            bus.link = message['link']
                        last_link = received/1e9
                        if not message['link'].get('ready'):
                            raise RuntimeError('KR260 CAN 监听模式或接口状态改变')
                    elif kind == 'frames':
                        for index, source_ns, packet in message['frames']:
                            if index != sequence+1 or (last_source is not None and source_ns < last_source-5_000_000):
                                raise RuntimeError('KR260 CAN 源序号或时间不连续')
                            sequence, last_source = index, source_ns
                            if clock.synced_at is None:
                                continue
                            raw = bytes.fromhex(packet)
                            if len(raw) != 16:
                                raise RuntimeError('KR260 CAN 帧长度无效')
                            host_time = clock.host_time(source_ns, received)
                            bus.ingest(raw, host_time, source={
                                'source_sequence': index, 'source_monotonic_ns': source_ns,
                                'host_arrival_monotonic': received/1e9})
                    elif kind == 'error':
                        raise RuntimeError(message.get('message', 'KR260 CAN 接收错误'))
                    else:
                        raise RuntimeError('未知 KR260 CAN 消息')
    finally:
        selector.close()
        if proc.poll() is None:
            proc.terminate()
            try:
                proc.wait(timeout=1)
            except subprocess.TimeoutExpired:
                proc.kill()
                proc.wait(timeout=1)
        for stream in (proc.stdin, proc.stdout, proc.stderr):
            stream.close()
        with bus.lock:
            bus.remote_process = None
