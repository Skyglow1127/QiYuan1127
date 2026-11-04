"""Bounded read-only checks; never runs enable/power/motion commands."""
import ipaddress
import json
import math
from pathlib import Path
import subprocess
from datetime import datetime, timezone


def select_interface(interfaces, requested=None):
    network = ipaddress.ip_network('192.168.123.0/24')
    usable = []
    for item in interfaces:
        if 'UP' not in item.get('flags', []) or item.get('operstate') != 'UP':
            continue
        for address in item.get('addr_info', []):
            try:
                if address.get('family') == 'inet' and ipaddress.ip_address(address['local']) in network:
                    usable.append(item['ifname'])
                    break
            except (KeyError, ValueError):
                continue
    if requested is not None:
        return requested if requested in usable else None
    return usable[0] if len(usable) == 1 else None


def parse_feedback(output):
    angles, status = [], None
    for line in output.splitlines():
        if not line.startswith('[rx] '):
            continue
        try:
            message = json.loads(line[5:])
            if not isinstance(message, dict) or not isinstance(message.get('data'), dict):
                continue
            if message.get('address') != 2:
                continue
            data = message['data']
            if message.get('funcode') == 1:
                values = [data[f'angle{i}'] for i in range(7)]
                if all(type(v) in [float, int] and math.isfinite(v) for v in values):
                    angles.append(values)
            elif message.get('funcode') == 3:
                if all(type(data.get(k)) is int for k in ['power_status', 'enable_status', 'error_status']):
                    status = {k: data[k] for k in ['power_status', 'enable_status', 'error_status']}
        except (KeyError, ValueError, TypeError):
            continue
    limits = [(-135,135),(-90,90),(-90,90),(-135,135),(-90,90),(-135,135),(0,65)]
    latest = angles[-1] if angles else None
    return {'angle_frames': len(angles), 'latest_angles_sdk_units': latest, 'firmware_status': status,
            'outside_local_limits': [i for i, (lo, hi) in enumerate(limits)
                                     if latest is not None and not lo <= latest[i] <= hi],
            'sample_spread_sdk_units': [max(a[i] for a in angles)-min(a[i] for a in angles)
                                        for i in range(7)] if angles else None,
            'motion_commands_sent': '[tx]' in output}


def live_check(control_root, nic=None):
    result = {'checked_at_utc': datetime.now(timezone.utc).isoformat(), 'read_only': True}
    try:
        ip = subprocess.run(['ip','-j','address','show'], capture_output=True, text=True, timeout=3, check=True)
        interfaces = json.loads(ip.stdout)
        result['interfaces'] = [{'name': x['ifname'], 'state': x.get('operstate'),
            'ipv4': [a['local'] for a in x.get('addr_info',[]) if a.get('family')=='inet']} for x in interfaces]
        selected = select_interface(interfaces, nic)
        result['selected_interface'] = selected
        if selected is None:
            result.update(connection='no_unique_active_robot_subnet',
                          blocker='没有唯一可用的 192.168.123.x 网口；检查网线所在网口及其 IP 配置')
            return result
        binary = Path(control_root)/'build/arm_control'
        probe = subprocess.run([str(binary),'--nic',selected,'--verbose','watch','2'],
                               capture_output=True,text=True,timeout=8)
        feedback = parse_feedback(probe.stdout + probe.stderr)
        result['feedback'] = feedback
        result['probe_exit_code'] = probe.returncode
        result['connection'] = 'feedback_received' if probe.returncode == 0 and feedback['angle_frames'] else 'feedback_unavailable'
        blockers = []
        if result['connection'] != 'feedback_received':
            blockers.append('未获得当前角度反馈')
        status = feedback['firmware_status']
        if status is None:
            blockers.append('未获得当前供电/使能反馈')
        else:
            if status['power_status'] != 1: blockers.append('固件未报告 power_status=1，不能进行运动测试')
            if status['enable_status'] != 1: blockers.append('固件未报告全臂使能；没有单关节已使能的本轮验证')
            if status['error_status'] != 0: blockers.append('固件报告非零错误码')
        if feedback['outside_local_limits']:
            blockers.append('当前角度超出本地限位：需核对零位/映射，不自动放宽限位')
        result['blockers'] = blockers
        if feedback['motion_commands_sent']:
            raise RuntimeError('Read-only probe unexpectedly emitted a command')
    except (OSError, ValueError, subprocess.SubprocessError, RuntimeError) as exc:
        result.update(connection='probe_error', error=str(exc))
    return result
