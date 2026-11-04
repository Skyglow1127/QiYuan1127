"""ACK and bootstrap delays use virtual time; no CLI or hardware is started."""
import importlib.util
from pathlib import Path
import sys

import numpy as np
import pytest

CONTROL = Path(__file__).resolve().parents[1] / 'runtime/Control'
sys.path.insert(0, str(CONTROL / 'ros2_ws/src/d1_pick_place'))
sys.path.insert(0, str(CONTROL / 'tools'))
from d1_pick_place.sdk_executor import Mapping, NAMES, execute, prepare

_spec = importlib.util.spec_from_file_location(
    'command_ack_gripper', CONTROL / 'tools/gripper_stage.py')
gripper = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(gripper)

# 171057 transport rows 19 and 30: J6 command to final bootstrap [ok].
BOOTSTRAP_WAIT_S = 289295.388094178 - 289295.065431306


class SlowAckTransport:
    def __init__(self, *, gripper_mode=False, feedback_pause=0., freeze=False):
        self.t = self.q_time = 0.
        self.q = np.r_[np.zeros(6), 36.2 if gripper_mode else 30.]
        self.q_seq = 1
        self.acks = self.commands = 0
        self.group_starts = []
        self.group_returns = []
        self.writes = []
        self.feedback_pause = feedback_pause
        self.pause_total = 0.
        self.pending = False
        self.pause_start = self.pause_end = None
        self.freeze = freeze
        self.frozen = False

    def now(self):
        return self.t

    def snapshot(self):
        return dict(q=self.q.copy(), q_time=self.q_time, q_seq=self.q_seq,
            fault=None, status=dict(power_status=1, enable_status=1, error_status=0),
            status_time={key: self.t for key in
                ('power_status', 'enable_status', 'error_status')},
            feedback_pending=self.pending, feedback_pause_total_s=self.pause_total,
            feedback_rejected_count=int(self.pause_start is not None))

    def wait(self, seconds):
        self.t += seconds if seconds > 0 else .001
        if self.pending and self.t >= self.pause_end - 1e-12:
            self.pending = False
            self.pause_total += self.t - self.pause_start
        if not self.frozen and not self.pending:
            self.q_time = self.t
            self.q_seq += 1

    def block_for_ack(self, duration, check):
        end = self.t + duration
        while self.t < end - 1e-12:
            self.wait(min(.01, end - self.t))
            check()
        self.acks += 1

    def start_group(self):
        self.group_starts.append(self.t)
        if len(self.group_starts) == 1:
            if self.feedback_pause:
                self.pending = True
                self.pause_start = self.t
                self.pause_end = self.t + self.feedback_pause
            self.frozen = self.freeze

    def send_target(self, target, check):
        check()
        self.start_group()
        self.q[6] = target
        self.commands += 1
        self.writes.append((self.t, 6, target))
        delay = (1. if self.freeze else
                 BOOTSTRAP_WAIT_S if len(self.group_starts) == 1 else .01)
        self.block_for_ack(delay, check)
        self.group_returns.append(self.t)
        return True

    def send(self, target, check):
        check()
        self.start_group()
        for joint in self.active_joints:
            check()
            self.q[joint] = target[joint]
            self.commands += 1
            self.writes.append((self.t, joint, target[joint]))
            self.block_for_ack(1. if self.freeze else .12, check)
        self.group_returns.append(self.t)
        return len(self.active_joints)


def arm_plan():
    return prepare(NAMES, [[0.] * 6, np.deg2rad([2., 1., 0., 0., 1.5, 0.])],
        [0., .2], Mapping(np.ones(6), np.zeros(6)), rate=5., speed=5., acceleration=10.,
        respect_source_timing=False)


def assert_no_backlog_burst(transport, period=.2):
    assert len(transport.group_starts) >= 3
    for returned, following in zip(transport.group_returns, transport.group_starts[1:]):
        assert following - returned >= period - 1e-8
    assert all(np.isfinite(value) and abs(value) < 100.
               for _, _, value in transport.writes)


@pytest.mark.parametrize('pause', [0., .1])
def test_recorded_gripper_bootstrap_wait_does_not_miss_next_tick(pause):
    t = SlowAckTransport(gripper_mode=True, feedback_pause=pause)
    report = gripper.execute_segment(36., t.q.tolist(), t, rate=5., speed=.5)
    assert report['completed'] and t.q[6] == 36.
    assert t.group_returns[0] - t.group_starts[0] == pytest.approx(BOOTSTRAP_WAIT_S)
    assert_no_backlog_burst(t)
    blocking = sum(b - a for a, b in zip(t.group_starts, t.group_returns))
    # Feedback pauses already shifted the clock; command waits add the remainder.
    assert report['command_wait_pause_s'] == pytest.approx(blocking - t.pause_total)
    assert t.pause_total == pytest.approx(pause)
    assert [joint for _, joint, _ in t.writes] == [6] * 4


@pytest.mark.parametrize('pause', [0., .1])
def test_multi_axis_ack_batch_longer_than_period_keeps_next_tick_spacing(pause):
    t = SlowAckTransport(feedback_pause=pause)
    result = execute(arm_plan(), t)
    assert result['completed']
    assert t.group_returns[0] - t.group_starts[0] >= .36 - 1e-8
    assert_no_backlog_burst(t)
    blocking = sum(b - a for a, b in zip(t.group_starts, t.group_returns))
    assert result['command_wait_pause_s'] == pytest.approx(blocking - t.pause_total)
    assert result['feedback_pause_s'] == pytest.approx(t.pause_total)
    assert t.pause_total == pytest.approx(pause)
    assert set(joint for _, joint, _ in t.writes) == {0, 1, 4}


@pytest.mark.parametrize('kind', ['arm', 'gripper'])
def test_missing_angles_during_ack_wait_still_stops_at_hard_deadline(kind):
    t = SlowAckTransport(gripper_mode=kind == 'gripper', freeze=True)
    report = {}
    with pytest.raises(RuntimeError, match='All joint feedback recovery timeout'):
        if kind == 'gripper':
            gripper.execute_segment(36., t.q.tolist(), t, speed=.5, report=report)
        else:
            execute(arm_plan(), t)
    assert 2. - 1e-8 <= t.now() - t.q_time <= 2.011
    assert len(t.group_starts) == 1 and t.group_returns == []
    assert t.commands == 1 and t.acks == 0
    if kind == 'gripper':
        assert report['future_commands_stopped'] and not report['completed']
