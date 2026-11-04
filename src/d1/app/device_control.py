"""Explicit protocol controls; no position, damping, zero or hold commands.

The archived firmware supports setting the software power flag to 1. Its
enable aliases are disabled and power-off is not implemented by that firmware.
"""
import argparse
import json
import math
from pathlib import Path
import signal
import sys
import threading
import time

APP = Path(__file__).resolve().parents[1]
CONTROL = APP/'runtime/Control'
sys.path.insert(0, str(CONTROL/'ros2_ws/src/d1_pick_place'))
from d1_pick_place.sdk_executor import CliTransport

ACTIONS = ('refresh', 'power_on', 'power_off', 'enable')
STATUS_KEYS = ('power_status', 'enable_status', 'error_status')
ENABLE_UNAVAILABLE = ('已找到 D1 SDK 的 joint_enable_control 示例，但它发送 funcode=5、mode=0，'
                      '现有机载会执行全轴零阻尼并令 enable_status=0，不能作为开启使能。'
                      '无需手动开启 ENABLE：初始 enable_status=0 可以准备路径，确认执行后由首个已批准的非零位置动作启动并核对反馈。'
                      '本次未发送任何指令。')
POWER_OFF_UNAVAILABLE = ('当前已核验的机载固件没有实现下电：funcode=6 不论 power=0/1 都将 power_status 置为 1。'
                         '本次未发送无效下电请求，机械臂未确认下电；如需物理断电，请使用设备规定的电源开关。')


def fresh_status(snapshot, now):
    return all(type(snapshot.get('status', {}).get(key)) is int
               and snapshot['status'][key] in (0, 1)
               and 0 <= now-snapshot.get('status_time', {}).get(key, -math.inf) <= .6
               for key in STATUS_KEYS)


def status_fields(snapshot, now, wall_time=None):
    wall_time = time.time() if wall_time is None else wall_time
    times = [snapshot.get('status_time', {}).get(key, -math.inf) for key in STATUS_KEYS]
    oldest = min(times)
    stamp = wall_time-(now-oldest) if math.isfinite(oldest) and oldest <= now else None
    q = snapshot.get('q')
    valid_q = (q is not None and len(q) == 7 and all(math.isfinite(float(v)) for v in q)
               and 0 <= now-snapshot.get('q_time', -math.inf) <= .6)
    return dict(status={key: snapshot.get('status', {}).get(key) for key in STATUS_KEYS},
                status_fresh=fresh_status(snapshot, now), snapshot_received_at=stamp,
                status_age_s={key: now-value if math.isfinite(value) else None
                              for key, value in zip(STATUS_KEYS, times)},
                sdk_angles=list(map(float, q)) if valid_q else None)


class StatusTransport(CliTransport):
    def power_on(self):
        """The only writable command this adapter exposes, sent once."""
        with self.lock:
            sent = self.now()
            before = self.acks
            self.log.write(json.dumps(dict(t=sent, kind='control_command', action='power_on',
                                           command='power on', delivery='attempt'))+'\n')
            self.log.flush()
            self.commands += 1
            self.proc.stdin.write('power on\n')
            self.proc.stdin.flush()
            return before, sent


def perform(action, transport, cancel=lambda: False):
    """Uses real status timestamps and CLI acknowledgement, mockable offline."""
    if action not in ('refresh', 'power_on'):
        raise ValueError(POWER_OFF_UNAVAILABLE if action == 'power_off' else ENABLE_UNAVAILABLE if action == 'enable' else '不支持的设备操作')

    def check():
        if cancel():
            raise RuntimeError('用户已停止；不再发送后续设备指令')
        snapshot = transport.snapshot()
        if snapshot.get('fault'):
            raise RuntimeError('设备通信失败：'+str(snapshot['fault']))
        return snapshot

    deadline = transport.now()+4.
    while True:
        snapshot = check()
        if fresh_status(snapshot, transport.now()):
            break
        if transport.now() >= deadline:
            raise TimeoutError('未收到新鲜、完整的 power_status / enable_status / error_status 反馈')
        transport.wait(.025)
    if action == 'refresh':
        return dict(completed=True, message='已读取控制协议状态', **status_fields(snapshot, transport.now()))
    if snapshot['status']['error_status'] != 0:
        raise RuntimeError('控制协议 error_status 不为 0，未发送上电请求')
    if snapshot['status']['power_status'] == 1:
        return dict(completed=True, already_set=True, receipt_confirmed=False,
                    message='power_status 已为 1，未重复发送请求', **status_fields(snapshot, transport.now()))
    check()
    before, sent = transport.power_on()
    deadline = transport.now()+2.
    while True:
        snapshot = check()
        if (transport.acks > before and fresh_status(snapshot, transport.now())
                and snapshot['status_time']['power_status'] > sent
                and snapshot['status']['power_status'] == 1):
            if snapshot['status']['error_status'] != 0:
                raise RuntimeError('请求后 error_status 不为 0，请查看反馈日志')
            return dict(completed=True, receipt_confirmed=True,
                        message='上电请求收到回执，power_status=1 已回读；这是协议软件标志',
                        **status_fields(snapshot, transport.now()))
        if transport.now() >= deadline:
            raise TimeoutError('上电请求未得到完整回执及发送后的 power_status=1 反馈；未自动重发')
        transport.wait(.025)


def run(action, output, *, transport_factory=StatusTransport, cancel=lambda: False):
    if action not in ACTIONS:
        raise ValueError('不支持的设备操作')
    output = Path(output)
    output.mkdir(parents=True, exist_ok=False)
    report = dict(action=action, completed=False, mode='live',
                  status={key: None for key in STATUS_KEYS}, status_fresh=False,
                  snapshot_received_at=None, log_dir=str(output), robot_commands_sent=0,
                  position_commands_sent=0, hardware_power_verified=False,
                  hardware_torque_verified=False)
    transport = None
    start = time.monotonic()
    (output/'request.json').write_text(json.dumps(dict(action=action, requested_at=time.time()), indent=2)+'\n')
    try:
        if action == 'enable':
            raise RuntimeError(ENABLE_UNAVAILABLE)
        if action == 'power_off':
            raise RuntimeError(POWER_OFF_UNAVAILABLE)
        if cancel():
            raise RuntimeError('用户已停止；没有发送设备指令')
        transport = transport_factory(CONTROL/'build/arm_control', 'enp99s0', output/'transport.jsonl')
        report.update(perform(action, transport, cancel))
    except Exception as exc:
        report.update(completed=False, error=str(exc))
    finally:
        if transport is not None:
            report['robot_commands_sent'] = transport.commands
            if not report['completed']:
                report.update(status_fields(transport.snapshot(), transport.now()))
            transport.close()
        report['elapsed_s'] = time.monotonic()-start
        (output/'result.json').write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False)+'\n')
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--action', choices=ACTIONS, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    stopped = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stopped.set())
    signal.signal(signal.SIGTERM, lambda *_: stopped.set())
    result = run(args.action, args.output, cancel=stopped.is_set)
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0 if result['completed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
