import json
import subprocess
from types import SimpleNamespace

from d1_pick_place.diagnostics import live_check, parse_feedback, select_interface


def interface(name='enp99s0', state='UP', ip='192.168.123.222'):
    return dict(ifname=name, flags=['UP'], operstate=state,
                addr_info=[dict(family='inet', local=ip)])


def rx(code, data):
    return '[rx] ' + json.dumps(dict(address=2, funcode=code, data=data)) + '\n'


def test_selects_connected_subnet_and_rejects_ambiguity():
    rows = [interface(), interface('enp100s0', 'DOWN'), interface('wifi', ip='172.30.1.73')]
    assert select_interface(rows) == 'enp99s0'
    assert select_interface(rows, 'enp100s0') is None
    rows.append(interface('second'))
    assert select_interface(rows) is None
    assert select_interface(rows, 'second') == 'second'


def test_parser_rejects_corruption_and_reports_actual_status():
    log = '[rx] []\n[rx] null\n[rx] broken\n' + rx(1, {f'angle{i}': 0 for i in range(7)})
    log += rx(1, {f'angle{i}': (-167.8 if i == 5 else 0) for i in range(7)})
    log += rx(1, {f'angle{i}': float('nan') for i in range(7)})
    log += rx(3, dict(power_status=0, enable_status=0, error_status=0))
    feedback = parse_feedback(log)
    assert feedback['angle_frames'] == 2
    assert feedback['outside_local_limits'] == [5]
    assert feedback['firmware_status']['power_status'] == 0
    assert feedback['sample_spread_sdk_units'][5] == 167.8
    assert not feedback['motion_commands_sent']


def test_live_probe_only_watches_and_rejects_power_zero(monkeypatch, tmp_path):
    calls = []
    def run(args, **kwargs):
        calls.append(args)
        assert kwargs['timeout'] <= 8
        if args[0] == 'ip':
            return SimpleNamespace(stdout=json.dumps([interface()]))
        return SimpleNamespace(returncode=0, stderr='', stdout=
            rx(1, {f'angle{i}': 0 for i in range(7)}) +
            rx(3, dict(power_status=0, enable_status=0, error_status=0)))
    monkeypatch.setattr(subprocess, 'run', run)
    result = live_check(tmp_path)
    assert calls[1][1:] == ['--nic', 'enp99s0', '--verbose', 'watch', '2']
    assert result['connection'] == 'feedback_received'
    assert len(result['blockers']) == 2
    assert not result['feedback']['motion_commands_sent']


def test_no_carrier_never_launches_arm_cli(monkeypatch, tmp_path):
    calls = []
    def run(args, **kwargs):
        calls.append(args)
        return SimpleNamespace(stdout=json.dumps([interface(state='DOWN')]))
    monkeypatch.setattr(subprocess, 'run', run)
    assert live_check(tmp_path)['connection'] == 'no_unique_active_robot_subnet'
    assert len(calls) == 1


def test_timeout_fails_without_retry(monkeypatch, tmp_path):
    calls = []
    def run(args, **kwargs):
        calls.append(args)
        if args[0] == 'ip':
            return SimpleNamespace(stdout=json.dumps([interface()]))
        raise subprocess.TimeoutExpired(args, kwargs['timeout'])
    monkeypatch.setattr(subprocess, 'run', run)
    assert live_check(tmp_path)['connection'] == 'probe_error'
    assert len(calls) == 2
