"""Receive-only local or SSH CAN, with the same PiPER decoding and checks."""
import collections
import json
import socket
import subprocess
import threading
import time
import math


from .protocol import decode, unpack_frame, position_target_hold



def link_status(interface):
    result = subprocess.run(['ip', '-details', 'link', 'show', interface],
                            capture_output=True, text=True, timeout=2)
    detail = result.stdout.strip() or result.stderr.strip()
    up = result.returncode == 0 and 'state UP' in detail
    listen_only = 'LISTEN-ONLY' in detail.upper()
    bitrate_ok = 'bitrate 1000000 ' in detail
    return {'up': up, 'listen_only': listen_only, 'bitrate_ok': bitrate_ok, 'detail': detail,
            'ready': up and listen_only and bitrate_ok}


class PassiveCAN:
    def __init__(self, config):
        self.config = config
        self.stop_event = threading.Event()
        self.lock = threading.RLock()
        self.latest = {}
        self.counts = collections.Counter()
        self.recent_ids = {}
        self.error = '等待 CAN 接口'
        self.link = {}
        self.raw_file = None
        self.raw_error = None
        self.generation = 0
        self.transport_status = {'source': 'local'}
        self.remote_process = None
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()

    def run(self):
        while not self.stop_event.is_set():
            try:
                if self.config.get('transport', {}).get('kind', 'local') == 'ssh':
                    from .ssh_transport import receive_remote
                    receive_remote(self)
                    continue
                with self.lock:
                    self.link = link_status(self.config['interface'])
                if not self.link['ready']:
                    raise RuntimeError('can0 未处于 UP + LISTEN-ONLY + 1 Mbps；等待终端启用')
                # Deliberately no connect/query/enable calls from the vendor SDK.
                with socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW) as bus:
                    bus.bind((self.config['interface'],))
                    bus.settimeout(.5)
                    with self.lock:
                        self.error = None
                        self.latest.clear()
                        self.generation += 1
                    checked = time.monotonic()
                    while not self.stop_event.is_set():
                        try:
                            packet = bus.recv(16)
                        except socket.timeout:
                            packet = b''
                        now = time.monotonic()
                        if packet:
                            self.ingest(packet, now)
                        if now - checked > 1:
                            state = link_status(self.config['interface'])
                            with self.lock:
                                self.link = state
                            if not state['ready']:
                                raise RuntimeError('CAN 监听连接已中断或模式改变')
                            checked = now
            except Exception as error:
                with self.lock:
                    self.error = str(error)
                    self.latest.clear()
                    self.link = dict(self.link, ready=False)
                self.stop_event.wait(1)

    def ingest(self, packet, now, source=None):
        frame = unpack_frame(packet)
        if frame is None:
            return
        can_id, payload = frame
        decoded = decode(can_id, payload, self.config['mapping'])
        with self.lock:
            self.counts[can_id] += 1
            self.recent_ids[can_id] = now
            if decoded:
                decoded['receive_monotonic'] = now
                if source:
                    decoded.update(source)
                self.latest[(decoded['kind'], decoded['part'])] = decoded
            if self.raw_file:
                try:
                    self.raw_file.write(json.dumps({'receive_monotonic': now,
                        'id': can_id, 'data': payload.hex(), **(source or {})}, separators=(',', ':')) + '\n')
                except OSError as error:
                    self.raw_error = str(error)

    def attach_raw(self, output):
        with self.lock:
            self.raw_error = None
            self.raw_file = output

    def detach_raw(self):
        with self.lock:
            self.raw_file = None

    def snapshot(self, now=None):
        with self.lock:
            parts = {f'{kind}_{part}': dict(item) for (kind, part), item in self.latest.items()}
            now = time.monotonic() if now is None else now
            result = {'link': dict(self.link), 'error': self.error, 'parts': parts,
                      'generation': self.generation, 'raw_error': self.raw_error,
                      'transport': dict(self.transport_status),
                      'ids': [{'id': f'0x{k:03X}', 'count': v,
                               'age_ms': round((now-self.recent_ids[k])*1000, 1)}
                              for k, v in sorted(self.counts.items())]}
        errors = []
        result['ages_ms'] = {key: max(0, (now-item['receive_monotonic'])*1000)
                             for key, item in parts.items()}
        if result['error'] or not result['link'].get('ready'):
            errors.append(result['error'] or 'CAN 接口未就绪')
        for kind in ('state', 'action'):
            values, times = [], []
            missing = False
            for part in range(4):
                item = parts.get(f'{kind}_{part}')
                if not item:
                    missing = True
                    continue
                values.extend(item['values'])
                times.append(item['receive_monotonic'])
                if (now - times[-1])*1000 > self.config['max_can_age_ms']:
                    errors.append(f'{kind} 第 {part+1} 组数据过期')
            if missing and not result['error']:
                errors.append(f'{kind} 的 6 关节或夹爪数据不完整')
            result[kind] = values if len(values) == 7 else None
            if times and (max(times)-min(times))*1000 > self.config['max_pair_skew_ms']:
                errors.append(f'{kind} 各组反馈时间差过大')
            gripper = parts.get(f'{kind}_3', {})
            result[kind+'_gripper_mode'] = gripper.get('mode')
            if gripper.get('mode') == 'unknown':
                errors.append(f'{kind} 夹爪模式未知')
            if kind == 'state' and gripper.get('fault_bits'):
                errors.append('执行臂夹爪报告故障位')
            if result[kind] and any(v is None or not math.isfinite(v) for v in result[kind]):
                errors.append(f'{kind} 数值无效')
        if (result.get('state_gripper_mode') is not None and
            result.get('action_gripper_mode') is not None and
            result['state_gripper_mode'] != result['action_gripper_mode']):
            errors.append('指令与反馈夹爪单位不同，需要验证转换')
        status = parts.get('status_0')
        result['arm_status'] = status
        if not result['error']:
            if not status or (now-status['receive_monotonic'])*1000 > self.config['max_can_age_ms']:
                errors.append('执行臂状态反馈缺失或过期')
            elif status['arm_status'] or status['err_code']:
                errors.append(f'机械臂状态异常：status={status["arm_status"]} error=0x{status["err_code"]:04X}')
            elif status['ctrl_mode'] != 1 or status['mode_feedback'] != 1:
                errors.append('执行臂未处于 CAN 关节位置控制模式')
        hold = position_target_hold(parts, now, self.config)
        result['action_sampling'] = hold
        if hold['allowed']:
            errors = [error for error in errors if
                not (error.startswith('action 第 ') and error.endswith('组数据过期')) and
                error != 'action 各组反馈时间差过大']
        if result['raw_error']:
            errors.append('原始 CAN 写入失败：'+result['raw_error'])
        result['issues'] = errors
        result['ready'] = not errors
        # Historical position targets can establish a mapping, but never start
        # a recording. The recorder waits for a new complete target after arming.
        result['mapping_issues'] = [error for error in errors
            if not (error.startswith('action 第 ') and error.endswith('组数据过期'))]
        result['mapping_ready'] = not result['mapping_issues']
        result['observation_issues'] = [error for error in errors
            if not error.startswith('action') and error != '指令与反馈夹爪单位不同，需要验证转换']
        return result


    def close(self):
        self.stop_event.set()
        with self.lock:
            if self.remote_process and self.remote_process.poll() is None:
                self.remote_process.terminate()
        self.thread.join(timeout=2)
        self.detach_raw()
