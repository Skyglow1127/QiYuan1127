"""Offline jump-event replay and execution scheduling; no hardware process."""
import io
import json
import math
from pathlib import Path
import sys
import threading
from types import SimpleNamespace

import numpy as np
import pytest

APP = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(APP / 'runtime/Control/ros2_ws/src/d1_pick_place'))
from d1_pick_place.sdk_executor import (
    Cancelled, CliTransport, Mapping, NAMES, execute, healthy, prepare,
)


def angle_message(q):
    return dict(seq=10, address=2, funcode=1,
                data={f'angle{i}': float(value) for i, value in enumerate(q)})


def marker(accepted, q, stamp, pause=0.):
    prefix = 'accepted' if accepted else 'rejected'
    return f'[feedback-{prefix}] ' + json.dumps(dict(
        message=angle_message(q), received_monotonic_s=stamp,
        pause_total_s=pause))


def reader(enabled=True):
    """Use the production reader without constructing a subprocess or thread."""
    t = object.__new__(CliTransport)
    t.lock = threading.RLock()
    t.changed = threading.Condition(t.lock)
    t.log = io.StringIO()
    t.q = t.raw_q = None
    t.q_time = t.raw_q_time = -math.inf
    t.q_seq = t.acks = t.commands = 0
    t.joint_q_time = np.full(7, -math.inf)
    t.joint_q_seq = np.zeros(7, dtype=int)
    t.joint_feedback_pending = np.zeros(7, dtype=bool)
    t.status = {}
    t.status_time = {}
    t.feedback_filter_enabled = enabled
    t.feedback_pending = False
    t.feedback_pause_total_s = 0.
    t.feedback_rejected_count = 0
    t.fault = None
    t.closed = True  # Exhausting an offline fixture is not an actual CLI exit.
    t.last_targets = {}
    t.t = 0.
    t.now = lambda: t.t
    t.proc = SimpleNamespace(stdout=(), stdin=io.StringIO())
    return t


def feed(t, stamp, *lines):
    t.t = stamp
    t.proc.stdout = iter(line + '\n' for line in lines)
    t._read()
    assert t.fault is None


def status_line():
    return '[rx] ' + json.dumps(dict(seq=10, address=2, funcode=3,
        data=dict(power_status=1, enable_status=1, error_status=0)))


@pytest.mark.parametrize('relative,before_line,bad_line', [
    ('04_执行日志与原始采集/D1_Control_logs/preaction_unheld_axis_round2_20261023/'
     'train/leg_06_after_cli_fix/transport.jsonl', 382, 394),
    ('09_一键抓取程序/logs/20261026_134406_4f7a87/'
     'execution/01_pregrasp/executed/transport.jsonl', 21849, 21891),
])
def test_recorded_bad_angles_remain_raw_without_refreshing_valid_state(
        relative, before_line, bad_line):
    path = APP.parent / relative
    original = path.read_bytes()
    rows = original.decode().splitlines()
    good = json.loads(json.loads(rows[before_line - 1])['line'][5:])
    bad = json.loads(json.loads(rows[bad_line - 1])['line'][5:])
    q_good = [good['data'][f'angle{i}'] for i in range(7)]
    q_bad = [bad['data'][f'angle{i}'] for i in range(7)]
    t = reader()
    # Raw JSON is retained for diagnosis but cannot become valid feedback.
    feed(t, 10., '[rx] ' + json.dumps(good), status_line())
    assert t.snapshot()['q'] is None
    feed(t, 10., marker(True, q_good, 10.))
    initial = t.snapshot()
    feed(t, 10.312, '[rx] ' + json.dumps(bad), marker(False, q_bad, 10.312),
         status_line(), '[ok] seq=123 received')
    rejected = t.snapshot()
    assert np.array_equal(rejected['raw_q'], q_bad)
    assert rejected['raw_q_time'] == 10.312
    assert np.array_equal(rejected['q'], initial['q'])
    assert rejected['q_time'] == initial['q_time'] == 10.
    assert rejected['q_seq'] == initial['q_seq'] == 1
    assert rejected['feedback_pending'] is True
    assert rejected['feedback_rejected_count'] == 1
    assert t.acks == 1 and rejected['status_time']['power_status'] == 10.312
    assert np.array_equal(healthy(rejected, 10.312), q_good)
    # A real accepted sample, never a +204.8 correction, clears the pause.
    feed(t, 10.412, marker(True, q_good, 10.412, pause=.1))
    restored = t.snapshot()
    assert restored['feedback_pending'] is False
    assert restored['feedback_pause_total_s'] == .1
    assert restored['q_seq'] == 2 and restored['q_time'] == 10.412
    assert np.array_equal(restored['q'], q_good)
    logged_lines = [json.loads(row)['line'] for row in t.log.getvalue().splitlines()]
    assert '[rx] ' + json.dumps(bad) in logged_lines
    assert path.read_bytes() == original


def test_repeated_rejects_cannot_keep_angles_alive_using_fresh_status():
    t = reader()
    good = [0., 0., 0., 0., 39.4, 0., 30.]
    bad = good.copy()
    bad[4] = -165.3
    feed(t, 10., marker(True, good, 10.), status_line())
    for stamp in (10.2, 10.4, 10.61):
        feed(t, stamp, '[rx] ' + json.dumps(angle_message(bad)),
             marker(False, bad, stamp), status_line())
    s = t.snapshot()
    assert s['q_seq'] == 1 and s['q_time'] == 10.
    assert s['feedback_rejected_count'] == 3 and s['feedback_pending']
    with pytest.raises(RuntimeError, match='Angle feedback stale'):
        healthy(s, 10.61)


def test_accepted_j6_sdk_change_is_not_reinterpreted_as_arm_degrees():
    t = reader()
    q = np.r_[np.zeros(6), 30.]
    feed(t, 10., marker(True, q, 10.), status_line())
    q[6] = 32.
    feed(t, 10.1, marker(True, q, 10.1))
    assert healthy(t.snapshot(), 10.1)[6] == 32.
    assert not t.snapshot()['feedback_pending']
    # An accepted event still cannot bypass the separate J6 absolute limits.
    q[6] = 65.001
    feed(t, 10.2, marker(True, q, 10.2))
    with pytest.raises(RuntimeError, match='J6'):
        healthy(t.snapshot(), 10.2)


def test_unfiltered_readonly_reader_still_exposes_raw_limit_failure():
    t = reader(enabled=False)
    q = [0., 0., 0., 0., -165.3, 0., 30.]
    feed(t, 10., '[rx] ' + json.dumps(angle_message(q)), status_line())
    assert t.snapshot()['q_seq'] == 1
    assert np.array_equal(t.snapshot()['q'], q)
    with pytest.raises(RuntimeError, match='J4'):
        healthy(t.snapshot(), 10.)


def test_driver_timeout_error_survives_later_failure_line_and_cli_exit():
    t = reader()
    q = [0., 0., 0., 0., 0., 0., 30.]
    feed(t, 10., marker(True, q, 10.), status_line())
    t.t = 12.
    t.closed = False  # Exercise the production reader's unexpected-EOF path.
    lines = [
        '[feedback-timeout] ' + json.dumps(dict(
            reason='angle_feedback_recovery_timeout', valid_age_s=2., timeout_s=2.)),
        '[失败] position command stopped; no later command sent',
    ]
    t.proc.stdout = iter(line + '\n' for line in lines)
    t._read()
    snapshot = t.snapshot()
    assert snapshot['fault'] == 'All joint feedback recovery timeout'
    assert snapshot['q_time'] == 10. and snapshot['q_seq'] == 1
    with pytest.raises(RuntimeError, match='^All joint feedback recovery timeout$'):
        healthy(snapshot, t.now())
    logged = [json.loads(row)['line'] for row in t.log.getvalue().splitlines()]
    assert logged[-2:] == lines


class PausingTransport:
    """An accepted stream, one rejected interval, then real accepted recovery."""
    def __init__(self, start=.35, duration=.1, during_send=False):
        self.t = self.q_time = 0.
        self.q = np.r_[np.zeros(6), 30.]
        self.seq = 1
        self.sent = []
        self.pending = False
        self.pause_total = 0.
        self.rejected = 0
        self.start = start
        self.duration = duration
        self.during_send = during_send
        self.injected = False

    def now(self):
        return self.t

    def snapshot(self):
        return dict(q=self.q.copy(), q_time=self.q_time, q_seq=self.seq,
            fault=None, status=dict(power_status=1, enable_status=1, error_status=0),
            status_time={key: self.t for key in
                ('power_status', 'enable_status', 'error_status')},
            feedback_pending=self.pending, feedback_pause_total_s=self.pause_total,
            feedback_rejected_count=self.rejected)

    def wait(self, seconds):
        self.t += seconds if seconds > 0 else .001
        if not self.during_send and not self.injected and self.t >= self.start:
            self.pending = self.injected = True
            self.rejected += 1
        if self.pending and self.t >= self.start + self.duration:
            self.pending = False
            self.pause_total += self.t - self.start
        if not self.pending:
            self.q_time = self.t
            self.seq += 1

    def send(self, q, check):
        # Simulate rejection between the outer schedule check and send's check.
        if self.during_send and not self.injected:
            self.start = self.t
            self.pending = self.injected = True
            self.rejected += 1
        check()
        assert not self.pending
        self.sent.append((self.t, np.asarray(q).copy()))
        self.q = np.asarray(q).copy()
        return 1


def motion_plan():
    return prepare(NAMES, [[0.] * 6, [np.deg2rad(2.), 0., 0., 0., 0., 0.]],
        [0., .2], Mapping(np.ones(6), np.zeros(6)),
        rate=5., speed=5., acceleration=10., respect_source_timing=False)


@pytest.mark.parametrize('during_send', [False, True])
def test_short_rejection_pauses_commands_and_resumes_without_catchup(during_send):
    t = PausingTransport(during_send=during_send)
    plan = motion_plan()
    result = execute(plan, t)
    assert result['completed'] and result['feedback_rejected_count'] == 1
    assert .1 <= result['feedback_pause_s'] <= .111
    assert result['feedback_pause_s'] == pytest.approx(t.pause_total)
    stamps = np.array([stamp for stamp, _ in t.sent])
    assert np.min(np.diff(stamps)) >= 1 / plan['rate'] - 1e-8
    assert stamps[-1] == pytest.approx(plan['duration_s'] + t.pause_total)
    assert not any(t.start <= stamp < t.start + t.duration - 1e-8
                   for stamp in stamps)
    assert all(np.all(np.abs(q[:6]) < 3.) and q[6] == 30.
               for _, q in t.sent)


def test_continuous_rejection_stops_at_hard_recovery_deadline_without_commands():
    t = PausingTransport(duration=math.inf)
    with pytest.raises(RuntimeError, match='All joint feedback recovery timeout') as caught:
        execute(motion_plan(), t)
    assert 2. - 1e-8 <= t.now() - t.q_time <= 2.011
    assert t.sent and all(stamp < t.start for stamp, _ in t.sent)
    assert caught.value.future_commands_stopped
    assert not caught.value.hold_command_received


@pytest.mark.parametrize('guard', ['cancel', 'camera'])
def test_pending_feedback_does_not_disable_cancel_or_camera_guard(guard):
    t = PausingTransport(duration=math.inf)
    def camera_check():
        if guard == 'camera' and t.now() >= .4:
            raise RuntimeError('Camera guard failed')
    error = Cancelled if guard == 'cancel' else RuntimeError
    with pytest.raises(error, match='Cancellation|Camera guard'):
        execute(motion_plan(), t,
            cancel=lambda: guard == 'cancel' and t.now() >= .4,
            external_check=camera_check)
    assert t.now() < .5 and all(stamp < t.start for stamp, _ in t.sent)


def test_reader_rejection_between_send_check_and_lock_cannot_write_bad_target():
    t = reader()
    good = [0., 0., 0., 0., 39.4, 0., 30.]
    bad = good.copy()
    bad[4] = -165.3
    feed(t, 10., marker(True, good, 10.), status_line())
    t.active_joints = (4,)
    writes = []
    class Input:
        def write(self, line):
            assert not t.feedback_pending
            writes.append(line)
        def flush(self):
            t.acks += 1
    t.proc.stdin = Input()
    checks = 0
    def check():
        nonlocal checks
        checks += 1
        if checks == 1:
            healthy(t.snapshot(), t.now())
            # This event races with the completed health check, before write.
            feed(t, 10.1, '[rx] ' + json.dumps(angle_message(bad)),
                 marker(False, bad, 10.1))
        elif t.feedback_pending:
            feed(t, 10.2, marker(True, good, 10.2, pause=.1))
        healthy(t.snapshot(), t.now())
    target = np.array(good)
    target[4] = 39.5
    assert t.send(target, check) == 1
    assert checks >= 2
    assert writes == ['j 4 39.500 6\n']
    assert '-165' not in ''.join(writes)
