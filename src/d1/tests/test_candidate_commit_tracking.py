"""An unsent reference cannot become the tracking fault reference."""
from pathlib import Path
import io
import json
import sys
import threading
from types import SimpleNamespace

import numpy as np
import pytest

CONTROL = Path(__file__).resolve().parents[1] / 'runtime/Control'
sys.path.insert(0, str(CONTROL / 'ros2_ws/src/d1_pick_place'))
from d1_pick_place.sdk_executor import CliTransport, execute
from test_feedback_jump_filter import feed, marker, reader, status_line


LAST_SENT = -84.980
NEXT_UNSENT = -84.481651
ACTUAL_LAGGING = -87.900


def approved_steps(axis=1):
    start = np.zeros(6)
    start[axis] = -87.4
    first, final = start.copy(), start.copy()
    first[axis], final[axis] = LAST_SENT, NEXT_UNSENT
    return dict(sdk=[start.tolist(), first.tolist(), final.tolist()],
        ticks=[1, 1], rate=10., duration_s=.2, active_joints=[axis],
        timing_profile='edge_quintic_v1')


class CommittedRobot:
    """Fresh feedback with the exact 202549 sent/candidate residuals."""
    def __init__(self, *, axis=1, recover=True, tracking_fault=False):
        self.axis = axis
        self.t = 0.
        self.q = np.r_[np.asarray(approved_steps(axis)['sdk'][0]), 30.]
        self.seq = 1
        self.acks = self.commands = 0
        self.last_targets = {}
        self.writes = []
        self.recover = recover
        self.tracking_fault = tracking_fault

    def now(self):
        return self.t

    def snapshot(self):
        return dict(q=self.q.copy(), q_time=self.t, q_seq=self.seq, fault=None,
            status=dict(power_status=1, enable_status=1, error_status=0),
            status_time={key: self.t for key in
                ('power_status', 'enable_status', 'error_status')})

    def wait(self, seconds):
        self.t += seconds if seconds > 0 else .001
        self.seq += 1
        if self.last_targets.get(self.axis) == LAST_SENT:
            self.q[self.axis] = -86.9 if self.recover and self.t >= .5 else ACTUAL_LAGGING

    def send(self, target, check):
        for joint in self.active_joints:
            check()
            value = round(float(target[joint]), 3)
            candidate_error = abs(value - self.q[joint])
            assert candidate_error <= 2.5 + 1e-5
            self.writes.append((self.t, joint, value, self.q[joint]))
            self.last_targets[joint] = value
            self.commands += 1
            self.acks += 1
            if value == LAST_SENT:
                self.q[joint] = -88.1 if self.tracking_fault else ACTUAL_LAGGING
            else:
                self.q[joint] = value
            self.seq += 1
            check()  # ACK wait also rechecks the last actually committed target.
        return len(self.active_joints)


@pytest.mark.parametrize('axis', [1, 5])
def test_next_unsent_reference_waits_for_feedback_without_false_tracking_fault(axis):
    robot = CommittedRobot(axis=axis)
    assert abs(LAST_SENT - ACTUAL_LAGGING) == pytest.approx(2.920)
    assert abs(NEXT_UNSENT - ACTUAL_LAGGING) > 3.
    result = execute(approved_steps(axis), robot)
    assert result['completed']
    assert [value for _, _, value, _ in robot.writes] == [LAST_SENT, round(NEXT_UNSENT, 3)]
    assert robot.writes[1][0] >= .5
    assert all(abs(value - actual) <= 2.5 + 1e-5 for _, _, value, actual in robot.writes)


def test_stalled_unsent_candidate_has_bounded_wait_and_last_committed_diagnostic():
    robot = CommittedRobot(recover=False)
    with pytest.raises(TimeoutError, match='progress|catch up|wait') as caught:
        execute(approved_steps(), robot)
    assert len(robot.writes) == 1 and 2. <= robot.now() <= 2.3
    diagnostic = caught.value.execution_diagnostics
    assert diagnostic['reference_sdk'][1] == pytest.approx(LAST_SENT)
    assert diagnostic['max_error_deg'] == pytest.approx(2.920)
    assert diagnostic['next_reference_sdk'][1] == pytest.approx(NEXT_UNSENT)
    assert diagnostic['next_reference_max_error_deg'] > 3.
    assert diagnostic['tracking_reference_basis'] == 'last_published_joint_targets'
    assert robot.last_targets[1] == LAST_SENT


def test_real_tracking_error_against_a_committed_target_still_stops():
    robot = CommittedRobot(tracking_fault=True)
    with pytest.raises(RuntimeError, match='Tracking error >3 degrees') as caught:
        execute(approved_steps(), robot)
    assert len(robot.writes) == 1
    diagnostic = caught.value.execution_diagnostics
    assert diagnostic['reference_sdk'][1] == pytest.approx(LAST_SENT)
    assert diagnostic['max_error_deg'] == pytest.approx(3.120)


def test_published_targets_advance_only_for_the_axis_seen_on_actual_tx():
    robot = reader()
    robot.last_targets = {1: LAST_SENT, 5: round(NEXT_UNSENT, 3)}
    robot.published_targets = {1: -87.4, 5: -88.4}
    old_published = robot.published_targets.copy()
    q = [0., -87.4, 0., 0., 0., -88.4, 30.]
    # Stdin acceptance and new feedback do not prove a driver TX occurred.
    feed(robot, 10., marker(True, q, 10.), status_line(), '[ok] prior command received')
    assert robot.published_targets == old_published
    wire = dict(seq=123, address=1, funcode=1,
                data=dict(id=1, angle=LAST_SENT, delay_ms=1))
    feed(robot, 10.1, '[tx] ' + json.dumps(wire))
    assert robot.published_targets == {1: LAST_SENT, 5: old_published[5]}
    assert robot.last_targets[5] != robot.published_targets[5]


@pytest.mark.parametrize('recover', [True, False])
def test_real_transport_rechecks_later_axis_before_writing_new_target(recover):
    class BatchRobot(CliTransport):
        def __init__(self):
            # Exercise the actual per-axis send loop with a virtual writer.
            self.lock = threading.RLock()
            self.log = io.StringIO()
            self.t = self.q_time = 0.
            self.q_seq = 1
            self.q = np.r_[np.zeros(6), 30.]
            self.q[[1, 5]] = -87.4
            self.last_targets = {}
            self.acks = self.commands = 0
            self.writes = []
            self.first_write_at = None
            self.finished = False
            robot = self
            class Writer:
                def write(self, line):
                    _, joint, value, _ = line.split()
                    joint, value = int(joint), float(value)
                    actual = float(robot.q[joint])
                    assert abs(value - actual) <= 2.5 + 1e-5
                    robot.writes.append((robot.t, joint, value, actual))
                    if joint == 1:
                        robot.first_write_at = robot.t
                        robot.q[1] = ACTUAL_LAGGING  # Sent J1 is still <3° away.
                        robot.q[5] = -88.4  # Unsent J5 candidate is now 3.42° away.
                    else:
                        robot.finished = True
                        robot.q[1] = robot.last_targets[1]
                        robot.q[5] = value
                    robot.q_time = robot.t
                    robot.q_seq += 1
                def flush(self):
                    robot.acks += 1
            self.proc = SimpleNamespace(stdin=Writer())

        def now(self):
            return self.t

        def snapshot(self):
            return dict(q=self.q.copy(), q_time=self.q_time, q_seq=self.q_seq,
                fault=None, status=dict(power_status=1, enable_status=1, error_status=0),
                status_time={key: self.t for key in
                    ('power_status', 'enable_status', 'error_status')})

        def wait(self, seconds):
            self.t += seconds if seconds > 0 else .001
            self.q_time = self.t
            self.q_seq += 1
            if (recover and self.first_write_at is not None and not self.finished
                    and self.t - self.first_write_at >= .3):
                self.q[5] = -86.9

    robot = BatchRobot()
    start = robot.q[:6].copy()
    final = start.copy()
    final[[1, 5]] = LAST_SENT
    plan = dict(sdk=[start.tolist(), final.tolist()], ticks=[1], rate=10.,
        duration_s=.1, active_joints=[1, 5], timing_profile='edge_quintic_v1')
    if recover:
        result = execute(plan, robot)
        assert result['completed']
        assert [joint for _, joint, _, _ in robot.writes] == [1, 5]
        assert robot.writes[1][0] - robot.writes[0][0] >= .3
    else:
        with pytest.raises(TimeoutError, match='progress|catch up|wait') as caught:
            execute(plan, robot)
        assert [joint for _, joint, _, _ in robot.writes] == [1]
        assert robot.t - robot.first_write_at <= 2.011
        diagnostic = caught.value.execution_diagnostics
        assert diagnostic['reference_sdk'][1] == pytest.approx(LAST_SENT)
        assert diagnostic['reference_sdk'][5] == pytest.approx(start[5])
        assert diagnostic['next_reference_joint'] == 5
        assert diagnostic['next_reference_max_error_deg'] > 3.
    assert all(abs(value - actual) <= 2.5 + 1e-5
               for _, _, value, actual in robot.writes)
