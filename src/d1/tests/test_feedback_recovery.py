"""Soft angle gaps pause commands; hard gaps and other guards still stop them."""
import importlib.util
import math
from pathlib import Path
import sys

import numpy as np
import pytest

CONTROL = Path(__file__).resolve().parents[1] / 'runtime/Control'
sys.path.insert(0, str(CONTROL / 'ros2_ws/src/d1_pick_place'))
sys.path.insert(0, str(CONTROL / 'tools'))
from d1_pick_place.sdk_executor import (
    Cancelled, FeedbackRecovery, Mapping, NAMES, execute, prepare,
)

_spec = importlib.util.spec_from_file_location(
    'feedback_recovery_gripper', CONTROL / 'tools/gripper_stage.py')
gripper = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(gripper)


class GapTransport:
    def __init__(self, duration=1.2, *, pending=False, recovered_change=None):
        self.t = self.valid_time = 0.
        self.q = np.r_[np.zeros(6), 30.]
        self.physical_q = self.q.copy()
        self.seq = 1
        self.acks = self.commands = 0
        self.sent = []
        self.gap_start = .35
        self.gap_end = self.gap_start + duration
        self.pending_mode = pending
        self.pause_total = 0.
        self.pending = self.started = self.recovered = False
        self.recovered_change = recovered_change
        self.status_failure = None
        self.failure_at = .99
        self.initial_stale = False

    def now(self):
        return self.t

    def snapshot(self):
        status = dict(power_status=1, enable_status=1, error_status=0)
        times = {key: self.t for key in status}
        if self.status_failure and self.t >= self.failure_at:
            if self.status_failure == 'stale_status':
                times['power_status'] = self.t - .7
            else:
                key, value = self.status_failure
                status[key] = value
        return dict(q=self.q.copy(), q_seq=self.seq,
            q_time=self.t - 1. if self.initial_stale else self.valid_time,
            status=status, status_time=times, fault=None,
            feedback_pending=self.pending, feedback_pause_total_s=self.pause_total,
            feedback_rejected_count=int(self.started and self.pending_mode))

    def wait(self, seconds):
        self.t += seconds if seconds > 0 else .001
        if not self.started and self.t >= self.gap_start:
            self.started = True
            self.pending = self.pending_mode
        if self.started and not self.recovered and self.t >= self.gap_end:
            self.recovered = True
            self.pending = False
            if self.pending_mode:
                self.pause_total = self.t - self.gap_start
            if self.recovered_change:
                joint, value = self.recovered_change
                self.physical_q[joint] = value
        if not self.started or self.recovered:
            self.q = self.physical_q.copy()
            self.valid_time = self.t
            self.seq += 1

    def record_send(self):
        age = self.t - self.valid_time
        assert age <= .6 + 1e-9 and not self.pending
        self.sent.append((self.t, age, self.q.copy()))
        self.commands += 1
        self.acks += 1

    def send(self, target, check):
        check()
        self.record_send()
        self.physical_q = np.asarray(target).copy()
        return 1

    def send_target(self, target, check):
        check()
        self.record_send()
        self.physical_q[6] = target
        return True


def arm_plan():
    return prepare(NAMES, [[0.] * 6, [np.deg2rad(6.), 0., 0., 0., 0., 0.]],
        [0., .2], Mapping(np.ones(6), np.zeros(6)), rate=5., speed=5., acceleration=10.,
        respect_source_timing=False)


def run(kind, transport, **kwargs):
    if kind == 'arm':
        return execute(arm_plan(), transport, **kwargs)
    return gripper.execute_segment(25., [0.] * 6 + [30.], transport, rate=5., speed=5., **kwargs)


@pytest.mark.parametrize('kind', ['arm', 'gripper'])
@pytest.mark.parametrize('duration', [.604, 1.2])
def test_raw_angle_gap_can_recover_but_never_send_stale_targets(kind, duration):
    t = GapTransport(duration)
    report = run(kind, t)
    assert report['completed']
    # A 604-ms gap may end between two checks; a longer gap must enter waiting.
    if duration > .7:
        assert report['feedback_gap_recoveries'] >= 1
        assert report['feedback_gap_wait_s'] > 0
    assert report['feedback_gap_timeout_s'] == 2.
    stamps = [stamp for stamp, _, _ in t.sent]
    assert all(age <= .6 + 1e-9 for _, age, _ in t.sent)
    assert any(stamp >= t.gap_end for stamp in stamps)
    assert min(b - a for a, b in zip(stamps, stamps[1:])) >= .2 - 1e-8


@pytest.mark.parametrize('kind', ['arm', 'gripper'])
def test_raw_gap_cannot_be_kept_alive_by_fresh_status_or_receipts(kind):
    t = GapTransport(math.inf)
    with pytest.raises(RuntimeError, match='All joint feedback recovery timeout'):
        run(kind, t)
    assert 2. - 1e-8 <= t.t - t.valid_time <= 2.011
    assert t.sent and all(age <= .6 + 1e-9 for _, age, _ in t.sent)
    assert max(stamp for stamp, _, _ in t.sent) < t.valid_time + .6 + 1e-8


@pytest.mark.parametrize('kind', ['arm', 'gripper'])
def test_initial_stale_angles_are_not_allowed_to_enter_recovery(kind):
    t = GapTransport()
    t.initial_stale = True
    with pytest.raises(RuntimeError, match='Angle feedback stale'):
        run(kind, t)
    assert t.t == 0. and t.sent == []


@pytest.mark.parametrize('failure,match', [
    (('power_status', 0), 'power_status'),
    (('enable_status', 0), 'enable_status'),
    (('error_status', 1), 'error_status'),
    ('stale_status', 'power_status'),
])
def test_status_fault_or_staleness_during_soft_wait_is_immediate(failure, match):
    t = GapTransport(math.inf)
    t.status_failure = failure
    with pytest.raises(RuntimeError, match=match):
        run('arm', t)
    assert t.failure_at <= t.t <= t.failure_at + .011
    assert t.t - t.valid_time < 1. and all(age <= .6 + 1e-9 for _, age, _ in t.sent)


@pytest.mark.parametrize('guard', ['cancel', 'camera'])
def test_soft_wait_keeps_cancel_and_camera_checks_active(guard):
    t = GapTransport(math.inf)
    def external_check():
        if guard == 'camera' and t.t >= .99:
            raise RuntimeError('Camera guard failed')
    with pytest.raises(Cancelled if guard == 'cancel' else RuntimeError,
                       match='Cancellation|Camera guard'):
        run('arm', t, cancel=lambda: guard == 'cancel' and t.t >= .99,
            external_check=external_check)
    assert .99 <= t.t <= 1.001


@pytest.mark.parametrize('change,match', [
    ((4, -165.3), 'joint outside limits'),
    ((1, .7), 'Uncommanded joint drift'),
    ((0, 10.), 'Tracking error'),
])
def test_recovered_angles_must_pass_original_limits_and_motion_guards(change, match):
    t = GapTransport(recovered_change=change)
    with pytest.raises(RuntimeError, match=match) as caught:
        run('arm', t)
    assert t.recovered
    assert caught.value.feedback_diagnostics['final_valid_feedback_age_s'] <= .01
    assert all(stamp < t.gap_end for stamp, _, _ in t.sent)


def test_driver_pending_pause_is_not_counted_again_as_soft_gap_pause():
    t = GapTransport(duration=.9, pending=True)
    helper = FeedbackRecovery(t)
    while t.t < .36:
        t.wait(.01)
    q, snapshot = helper.read()
    assert t.recovered and not snapshot['feedback_pending']
    assert np.array_equal(q, t.q)
    assert t.pause_total >= .9 and helper.extra_pause_s == pytest.approx(0.)


def test_helper_observing_601ms_old_angles_waits_for_604ms_recovery():
    t = GapTransport()
    t.t = .601
    t.valid_time = 0.
    t.started = True
    t.gap_end = .604
    helper = FeedbackRecovery(t)
    q, snapshot = helper.read()
    assert t.recovered and snapshot['q_time'] > .604
    assert helper.gap_recoveries == 1 and helper.gap_wait_s > 0
    assert np.array_equal(q, t.q) and t.sent == []
