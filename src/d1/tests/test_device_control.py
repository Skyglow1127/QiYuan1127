"""Protocol button checks use virtual time and no device connection."""
import io
import json
from pathlib import Path
import threading
from types import SimpleNamespace

import pytest

from app.device_control import perform, run, status_fields, StatusTransport


class FakeTransport:
    def __init__(self, mode='normal'):
        self.t = 1.
        self.mode = mode
        self.commands = self.acks = 0
        self.sent = None
        self.closed = False

    def now(self): return self.t
    def wait(self, seconds):
        self.t += seconds
        if self.sent is not None and self.mode != 'no_ack': self.acks = 1

    def power_on(self):
        assert self.commands == 0
        self.commands += 1
        self.sent = self.t
        return self.acks, self.sent

    def snapshot(self):
        power = int(self.mode == 'already_set' or (self.sent is not None and self.t > self.sent and self.mode != 'ack_only'))
        stamp = self.sent if self.mode == 'old_frame' and self.sent is not None else self.t
        if self.mode == 'stale': stamp -= 1.
        return dict(status={'power_status': power, 'enable_status': 0, 'error_status': int(self.mode == 'device_error')},
                    status_time={key: stamp for key in ('power_status', 'enable_status', 'error_status')},
                    q=[0.]*7, q_time=self.t, fault='disconnected' if self.mode == 'fault' else None)

    def close(self): self.closed = True


def test_refresh_reads_flags_without_commands():
    t = FakeTransport()
    result = perform('refresh', t)
    assert result['completed'] and result['status'] == dict(power_status=0, enable_status=0, error_status=0)
    assert result['status_fresh'] and t.commands == 0


def test_power_on_requires_ack_and_new_status_without_changing_enable():
    t = FakeTransport()
    result = perform('power_on', t)
    assert result['completed'] and result['receipt_confirmed'] and t.commands == 1
    assert result['status']['power_status'] == 1 and result['status']['enable_status'] == 0


@pytest.mark.parametrize('mode', ['ack_only', 'no_ack', 'old_frame'])
def test_partial_confirmation_never_succeeds_or_retries(mode):
    t = FakeTransport(mode)
    with pytest.raises(TimeoutError): perform('power_on', t)
    assert t.commands == 1


@pytest.mark.parametrize('mode', ['stale', 'device_error', 'fault'])
def test_bad_initial_status_sends_nothing(mode):
    t = FakeTransport(mode)
    with pytest.raises((TimeoutError, RuntimeError)): perform('power_on', t)
    assert t.commands == 0


def test_already_set_does_not_repeat_a_request():
    t = FakeTransport('already_set')
    result = perform('power_on', t)
    assert result['already_set'] and result['completed'] and t.commands == 0


def test_cancel_before_or_after_request_never_adds_commands():
    for after in (False, True):
        t = FakeTransport()
        with pytest.raises(RuntimeError, match='已停止'):
            perform('power_on', t, cancel=lambda: bool(t.commands) if after else True)
        assert t.commands == int(after)


def test_unsupported_enable_does_not_connect_or_mutate_flags(tmp_path):
    def forbidden(*args): pytest.fail('Unsupported enable must never construct a device transport')
    report = run('enable', tmp_path/'enable', transport_factory=forbidden)
    assert not report['completed'] and '阻尼' in report['error']
    assert report['robot_commands_sent'] == 0 and report['status']['enable_status'] is None
    assert json.loads((tmp_path/'enable/result.json').read_text()) == report


def test_unsupported_power_off_does_not_send_a_noop_that_sets_flag_on(tmp_path):
    def forbidden(*args):pytest.fail('Power off is unsupported by audited firmware; no transport allowed')
    report=run('power_off',tmp_path/'off',transport_factory=forbidden)
    assert not report['completed'] and report['robot_commands_sent']==0
    assert report['status']['power_status'] is None
    assert '没有实现下电' in report['error']
    assert not report['hardware_power_verified']


def test_failed_power_request_is_logged_and_transport_closed(tmp_path):
    t = FakeTransport('ack_only')
    report = run('power_on', tmp_path/'power', transport_factory=lambda *args: t)
    assert not report['completed'] and report['robot_commands_sent'] == 1 and t.closed
    assert report['status']['power_status'] == 0 and not report['hardware_power_verified']
    assert report['position_commands_sent'] == 0


def test_transport_has_only_literal_power_on_command():
    transport = object.__new__(StatusTransport)
    transport.lock = threading.RLock()
    transport.now = lambda: 1.
    transport.acks = transport.commands = 0
    transport.log = io.StringIO()
    transport.proc = SimpleNamespace(stdin=io.StringIO())
    assert transport.power_on() == (0, 1.)
    assert transport.proc.stdin.getvalue() == 'power on\n'
    assert json.loads(transport.log.getvalue())['kind'] == 'control_command'


def test_status_age_preserves_oldest_feedback_time():
    t = FakeTransport()
    snapshot = t.snapshot()
    snapshot['status_time']['enable_status'] = .5
    result = status_fields(snapshot, 1., wall_time=100.)
    assert result['snapshot_received_at'] == 99.5 and result['status_fresh']
    assert status_fields(snapshot, 2.)['status_fresh'] is False
