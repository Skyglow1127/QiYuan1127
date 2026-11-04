"""Per-axis acceptance and whole-trajectory waiting, using offline fixtures."""
import io
import json
import math

import numpy as np
import pytest

from test_feedback_jump_filter import angle_message, feed, reader, status_line
from test_feedback_recovery import arm_plan, gripper
from d1_pick_place.sdk_executor import Cancelled, FeedbackRecovery, execute, healthy


def joint_marker(raw, merged, times, sequences, pending, stamp, *, pause=0., raw_message=None):
    accepted = [joint for joint, blocked in enumerate(pending) if not blocked]
    rejected = [joint for joint, blocked in enumerate(pending) if blocked]
    event = dict(message=angle_message(raw) if raw_message is None else raw_message,
        received_monotonic_s=stamp,
        pause_total_s=pause, joint_times_s=times, joint_sequences=sequences,
        joint_pending=pending, merged_angles=merged, accepted_joints=accepted,
        rejected_joints=rejected,
        joint_rejections=[dict(joint=joint, reason='angular_jump') for joint in rejected])
    tag = 'accepted' if accepted else 'rejected'
    return f'[feedback-{tag}] ' + json.dumps(event)


def test_partial_accepted_marker_updates_good_axes_but_preserves_bad_j4():
    t = reader()
    initial = [0., 0., 0., 0., 39.4, 0., 30.]
    feed(t, 10., joint_marker(initial, initial, [10.] * 7, [1] * 7,
                             [False] * 7, 10.), status_line())
    pending = [False] * 7
    pending[4] = True
    for index, stamp in enumerate((10.1, 10.2), start=2):
        raw = initial.copy()
        raw[0], raw[1], raw[4], raw[6] = .1 * index, -.1 * index, -165.3, 29.8
        merged = raw.copy()
        merged[4] = 39.4
        times = [stamp] * 7
        times[4] = 10.
        sequences = [index] * 7
        sequences[4] = 1
        feed(t, stamp, '[rx] ' + json.dumps(angle_message(raw)),
             joint_marker(raw, merged, times, sequences, pending, stamp))
        s = t.snapshot()
        assert np.array_equal(s['q'], merged)
        assert s['raw_q'][4] == -165.3 and s['q'][4] == 39.4
        assert np.array_equal(s['joint_q_time'], times)
        assert np.array_equal(s['joint_q_seq'], sequences)
        assert np.array_equal(s['joint_feedback_pending'], pending)
        # An accepted tag with six good axes must not clear the seventh's pause.
        assert s['feedback_pending'] is True
    restored = merged.copy()
    restored[4] = 39.5
    sequences = [4] * 7
    sequences[4] = 2
    feed(t, 10.3, joint_marker(restored, restored, [10.3] * 7, sequences,
                              [False] * 7, 10.3, pause=.2))
    s = t.snapshot()
    assert s['q'][4] == 39.5 and s['joint_q_time'][4] == 10.3
    assert s['joint_q_seq'][4] == 2 and not s['feedback_pending']


@pytest.mark.parametrize('bad_kind', ['null', 'missing', 'string'])
def test_malformed_raw_j4_keeps_reader_alive_and_other_axes_accepted(bad_kind):
    t = reader()
    initial = [0., 0., 0., 0., 39.4, 0., 30.]
    feed(t, 10., joint_marker(initial, initial, [10.] * 7, [1] * 7,
                             [False] * 7, 10.), status_line())
    raw = angle_message([.2, -.1, 0., 0., 39.4, .1, 29.8])
    if bad_kind == 'missing':
        del raw['data']['angle4']
    else:
        raw['data']['angle4'] = None if bad_kind == 'null' else 'invalid-angle'
    merged = [.2, -.1, 0., 0., 39.4, .1, 29.8]
    times, sequences, pending = [10.1] * 7, [2] * 7, [False] * 7
    times[4], sequences[4], pending[4] = 10., 1, True
    feed(t, 10.1, '[rx] ' + json.dumps(raw), joint_marker(
        initial, merged, times, sequences, pending, 10.1, raw_message=raw))
    s = t.snapshot()
    assert s['fault'] is None and np.isnan(s['raw_q'][4])
    assert np.array_equal(s['q'], merged)
    assert s['joint_q_time'][4] == 10. and s['joint_q_seq'][4] == 1
    assert s['joint_feedback_pending'][4] and s['feedback_pending']
    for joint in (0, 1, 2, 3, 5, 6):
        assert s['joint_q_time'][joint] == 10.1 and s['joint_q_seq'][joint] == 2


def test_unknown_initial_axis_does_not_create_complete_pose_until_accepted():
    t = reader()
    initial = [0., 0., 0., 0., 39.4, 0., 30.]
    raw = angle_message(initial)
    raw['data']['angle4'] = None
    merged = initial.copy()
    merged[4] = None
    times, sequences, pending = [10.] * 7, [1] * 7, [False] * 7
    times[4], sequences[4], pending[4] = None, 0, True
    feed(t, 10., '[rx] ' + json.dumps(raw), joint_marker(
        initial, merged, times, sequences, pending, 10., raw_message=raw), status_line())
    s = t.snapshot()
    assert s['fault'] is None and s['q'] is None
    assert s['joint_q_time'][4] == -math.inf and s['joint_q_seq'][4] == 0
    assert all(s['joint_q_time'][joint] == 10. for joint in range(7) if joint != 4)
    with pytest.raises(RuntimeError, match='Angle feedback stale'):
        healthy(s, 10.)
    sequences = [2] * 7
    sequences[4] = 1
    feed(t, 10.1, joint_marker(initial, initial, [10.1] * 7, sequences,
                              [False] * 7, 10.1))
    s = t.snapshot()
    assert np.array_equal(healthy(s, 10.1), initial)
    assert not s['feedback_pending'] and s['joint_q_seq'][4] == 1


@pytest.mark.parametrize('tamper', ['time', 'held_angle', 'sequence'])
def test_rejected_axis_cannot_refresh_time_or_change_held_valid_state(tamper):
    t = reader()
    initial = [0., 0., 0., 0., 39.4, 0., 30.]
    feed(t, 10., joint_marker(initial, initial, [10.] * 7, [1] * 7,
                             [False] * 7, 10.), status_line())
    merged = initial.copy()
    merged[0] = .2
    times, sequences, pending = [10.1] * 7, [2] * 7, [False] * 7
    times[4], sequences[4], pending[4] = 10., 1, True
    if tamper == 'time':
        times[4] = 10.1
    elif tamper == 'held_angle':
        merged[4] = 39.5
    else:
        sequences[4] = 2
    line = joint_marker(initial, merged, times, sequences, pending, 10.1)
    t.t = 10.1
    t.proc.stdout = iter([line + '\n'])
    t._read()
    s = t.snapshot()
    assert s['fault'] == 'Rejected joint refreshed valid feedback'
    assert np.array_equal(s['q'], initial)
    assert np.array_equal(s['joint_q_time'], [10.] * 7)
    assert np.array_equal(s['joint_q_seq'], [1] * 7)


def test_initial_readiness_uses_every_joint_age_not_only_global_timestamp():
    q = [0., 0., 0., 0., 39.4, 0., 30.]
    times = [10.] * 7
    times[4] = 9.2
    snapshot = dict(q=np.array(q), q_time=10., q_seq=1, joint_q_time=times,
        joint_q_seq=[1] * 7, joint_feedback_pending=[False] * 7, fault=None,
        status=dict(power_status=1, enable_status=1, error_status=0),
        status_time={key: 10. for key in ('power_status', 'enable_status', 'error_status')})
    with pytest.raises(RuntimeError, match='stale|fresh|J4'):
        healthy(snapshot, 10.)


class PerJointTransport:
    """Some axes stop yielding valid samples while the others remain fresh."""
    def __init__(self, stuck=(4,), recover_at=math.inf, recovered_change=None):
        self.t = 0.
        self.q = np.r_[np.zeros(6), 30.]
        self.physical_q = self.q.copy()
        self.times = np.zeros(7)
        self.sequences = np.ones(7, dtype=int)
        self.pending = np.zeros(7, dtype=bool)
        self.stuck = tuple(stuck)
        self.start = .35
        self.recover_at = recover_at
        self.recovered_change = recovered_change
        self.started = self.recovered = False
        self.pause_total = 0.
        self.acks = self.commands = 0
        self.sent = []
        self.log = io.StringIO()

    def now(self):
        return self.t

    def snapshot(self):
        return dict(q=self.q.copy(), q_time=float(np.min(self.times)),
            q_seq=int(np.max(self.sequences)), joint_q_time=self.times.copy(),
            joint_q_seq=self.sequences.copy(), joint_feedback_pending=self.pending.copy(),
            feedback_pending=bool(np.any(self.pending)),
            feedback_pause_total_s=self.pause_total,
            feedback_rejected_count=int(self.started), fault=None,
            status=dict(power_status=1, enable_status=1, error_status=0),
            status_time={key: self.t for key in
                ('power_status', 'enable_status', 'error_status')})

    def wait(self, seconds):
        self.t += seconds if seconds > 0 else .001
        if not self.started and self.t >= self.start:
            self.started = True
        if self.started and not self.recovered and self.t >= self.recover_at:
            self.recovered = True
            self.pause_total = self.t - self.start
            if self.recovered_change:
                joint, value = self.recovered_change
                self.physical_q[joint] = value
        for joint in range(7):
            blocked = self.started and not self.recovered and joint in self.stuck
            self.pending[joint] = blocked
            if not blocked:
                self.q[joint] = self.physical_q[joint]
                self.times[joint] = self.t
                self.sequences[joint] += 1

    def record_send(self):
        assert not np.any(self.pending)
        assert np.max(self.t - self.times) <= .6 + 1e-9
        self.sent.append(self.t)
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


def run(kind, t, **kwargs):
    if kind == 'arm':
        return execute(arm_plan(), t, **kwargs)
    return gripper.execute_segment(25., [0.] * 6 + [30.], t, rate=5., speed=5., **kwargs)


def timeout_events(t):
    return [json.loads(row) for row in t.log.getvalue().splitlines()
            if json.loads(row).get('kind') == 'joint_feedback_timeout']


@pytest.mark.parametrize('kind', ['arm', 'gripper'])
def test_one_bad_axis_warns_without_all_axis_timeout_or_partial_path_motion(kind):
    t = PerJointTransport()
    with pytest.raises(Cancelled):
        run(kind, t, cancel=lambda: t.t >= 10.)
    assert 10. <= t.t <= 10.011
    assert t.sent and all(stamp < t.start for stamp in t.sent)
    assert t.times[4] < t.start and all(t.times[joint] >= 10. for joint in range(7) if joint != 4)
    events = timeout_events(t)
    assert len(events) == 1
    assert events[0]['event'] == 'stale' and events[0]['joint'] == 4
    assert events[0]['age_s'] >= .6


@pytest.mark.parametrize('kind', ['arm', 'gripper'])
def test_only_all_seven_stale_axes_trigger_whole_feedback_timeout(kind):
    t = PerJointTransport(stuck=range(7))
    with pytest.raises(RuntimeError, match='^All joint feedback recovery timeout$') as caught:
        run(kind, t)
    assert np.min(t.t - t.times) >= 2. - 1e-8
    assert np.min(t.t - t.times) <= 2.011
    assert all(stamp < t.start for stamp in t.sent)
    diagnostic = caught.value.feedback_diagnostics
    assert diagnostic['stale_feedback_joints'] == list(range(7))
    assert len(diagnostic['joint_feedback_age_s']) == 7
    assert {event['joint'] for event in timeout_events(t) if event['event'] == 'stale'} == set(range(7))


def test_joint_warning_and_recovery_are_logged_once_per_transition():
    t = PerJointTransport(recover_at=1.45)
    while t.t < .36:
        t.wait(.01)
    helper = FeedbackRecovery(t)
    q, snapshot = helper.read()
    events = helper.metrics()['feedback_joint_timeout_events']
    assert [(event['joint'], event['event']) for event in events] == [(4, 'stale'), (4, 'recovered')]
    assert events[0]['age_s'] >= .6 and events[1]['t'] > events[0]['t']
    assert [(event['joint'], event['event']) for event in timeout_events(t)] == [(4, 'stale'), (4, 'recovered')]
    assert np.array_equal(q, t.q) and not np.any(snapshot['joint_feedback_pending'])
    helper.read()
    assert len(helper.metrics()['feedback_joint_timeout_events']) == 2


@pytest.mark.parametrize('change,match', [
    ((4, -165.3), 'joint outside limits'),
    ((1, .7), 'Uncommanded joint drift'),
    ((0, 10.), 'Tracking error'),
])
def test_per_joint_recovery_rechecks_limits_tracking_and_drift(change, match):
    t = PerJointTransport(recover_at=1.45, recovered_change=change)
    with pytest.raises(RuntimeError, match=match):
        run('arm', t)
    assert t.recovered and all(stamp < t.recover_at for stamp in t.sent)


def test_out_of_range_j6_is_a_hard_fault_even_when_marked_pending():
    t = PerJointTransport(stuck=(6,))
    t.q[6] = 65.01
    t.pending[6] = True
    with pytest.raises(RuntimeError, match='joint outside limits.*J6'):
        FeedbackRecovery(t).read()
    assert t.t == 0. and t.sent == []


@pytest.mark.parametrize('whole_gap_happened', [False, True])
def test_late_consumer_uses_driver_all_axis_gap_history(whole_gap_happened):
    class LateWakeTransport(PerJointTransport):
        def __init__(self):
            super().__init__()
            while self.t < .36:
                super().wait(.01)
            self.all_gap_count = 0
            self.last_gap_start = self.last_gap_end = None
            self.woke = False

        def snapshot(self):
            result = super().snapshot()
            result.update(all_feedback_gap_count=self.all_gap_count,
                last_all_feedback_gap_start_s=self.last_gap_start,
                last_all_feedback_gap_end_s=self.last_gap_end)
            return result

        def wait(self, seconds):
            # Python is not scheduled until 3.5 s. The driver still observes
            # samples and records any whole-arm gap independently of Python.
            assert not self.woke
            self.woke = True
            self.t = 3.5
            self.times[:] = 3.49
            self.sequences += 100
            self.pending[:] = False
            self.recovered = True
            if whole_gap_happened:
                self.all_gap_count = 1
                self.last_gap_start = .5
                self.last_gap_end = 3.49

    t = LateWakeTransport()
    helper = FeedbackRecovery(t)
    if whole_gap_happened:
        # Fresh samples at wakeup cannot erase a recorded all-axis timeout.
        with pytest.raises(RuntimeError, match='^All joint feedback recovery timeout$'):
            helper.read()
    else:
        # The old consumer-side deadline must not reject a continuously fresh
        # driver stream merely because Python wakes up after that deadline.
        q, snapshot = helper.read()
        assert np.array_equal(q, t.q)
        assert snapshot['all_feedback_gap_count'] == 0
        assert np.max(t.t - np.asarray(snapshot['joint_q_time'])) < .02
    assert t.woke and t.sent == []


@pytest.mark.parametrize('kind,frozen_joint', [('arm', 4), ('gripper', 6)])
def test_endpoint_requires_new_samples_from_the_axis_being_verified(kind, frozen_joint):
    class RepeatedEndpointSample(PerJointTransport):
        def __init__(self):
            super().__init__(stuck=())
            self.frozen_sequence = None
            self.frozen_at = None

        def freeze_endpoint_sample(self):
            self.frozen_sequence = int(self.sequences[frozen_joint])
            self.frozen_at = self.t

        def wait(self, seconds):
            super().wait(seconds)
            if self.frozen_sequence is not None:
                # Other axes, timestamps, and the aggregate sequence keep
                # updating, but this axis has no independently new sample.
                self.sequences[frozen_joint] = self.frozen_sequence

        def send(self, target, check):
            count = super().send(target, check)
            if np.allclose(np.asarray(target)[:6], arm_plan()['sdk'][-1], atol=1e-10):
                self.freeze_endpoint_sample()
            return count

        def send_target(self, target, check):
            result = super().send_target(target, check)
            if abs(target - 25.) < 1e-10:
                self.freeze_endpoint_sample()
            return result

    t = RepeatedEndpointSample()
    with pytest.raises(TimeoutError, match='Endpoint position|J6 endpoint'):
        run(kind, t)
    assert t.frozen_at is not None and t.t - t.frozen_at >= 5. - 1e-8
    assert t.sequences[frozen_joint] == t.frozen_sequence
    assert all(t.sequences[joint] > t.frozen_sequence
               for joint in range(7) if joint != frozen_joint)
    assert np.max(t.t - t.times) <= .6 and not np.any(t.pending)
    assert timeout_events(t) == []
