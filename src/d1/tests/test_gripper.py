"""J6 execution tests using virtual time and mock feedback only."""
import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace
import sys

import numpy as np
import pytest

CONTROL = Path(__file__).resolve().parents[1] / 'runtime/Control'
sys.path.insert(0, str(CONTROL / 'tools'))
_spec = importlib.util.spec_from_file_location('one_click_gripper', CONTROL / 'tools/gripper_stage.py')
gripper = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(gripper)

START = [0., 0., 0., 0., 0., 0., 30.]


class FakeTransport:
    def __init__(self, mode='follow'):
        self.t = 0.
        self.q = np.asarray(START).copy()
        self.seq = 1
        self.sent = []
        self.commands = 0
        self.acks = 0
        self.mode = mode

    def now(self):
        return self.t

    def snapshot(self):
        return dict(q=self.q.copy(), q_time=self.t - (1. if self.mode == 'stale' else 0.),
                    q_seq=self.seq, status=dict(power_status=1, enable_status=1, error_status=0),
                    status_time={k: self.t for k in ('power_status', 'enable_status', 'error_status')},
                    fault=None)

    def wait(self, seconds):
        self.t += max(seconds, .001)
        if self.mode != 'frozen_sequence':
            self.seq += 1

    def send_target(self, target, check):
        check()
        self.sent.append(target)
        self.commands += 1
        self.acks += 1
        if self.mode != 'stalled':
            self.q[6] = target

    def close(self):
        pass


class RejectedFeedbackTransport(FakeTransport):
    """A rejected frame preserves accepted timestamps until a new valid frame."""
    def __init__(self, duration=.25):
        super().__init__()
        self.reject_at = .45
        self.resume_at = self.reject_at + duration
        self.pending = self.triggered = False
        self.valid_time = 0.
        self.pause_total = 0.
        self.send_times = []

    def snapshot(self):
        result = super().snapshot()
        result.update(q_time=self.valid_time, feedback_pending=self.pending,
                      feedback_pause_total_s=self.pause_total)
        return result

    def wait(self, seconds):
        self.t += max(seconds, .001)
        if not self.triggered and self.t >= self.reject_at:
            self.triggered = self.pending = True
        if self.pending and self.t >= self.resume_at:
            self.pending = False
            self.pause_total = self.resume_at-self.reject_at
        if not self.pending:
            self.seq += 1
            self.valid_time = self.t

    def send_target(self, target, check):
        check()
        assert not self.pending
        self.send_times.append(self.t)
        return super().send_target(target, check)


def test_rejected_arm_feedback_pauses_gripper_then_resumes_without_catchup():
    transport = RejectedFeedbackTransport()
    report = gripper.execute_segment(28., START, transport, rate=5., speed=5.)
    assert report['completed'] and transport.pause_total == pytest.approx(.25)
    assert all(not transport.reject_at <= stamp < transport.resume_at
               for stamp in transport.send_times)
    assert transport.send_times[1]-transport.send_times[0] == pytest.approx(.45, abs=.025)
    assert all(b-a >= .19 for a,b in zip(transport.send_times,transport.send_times[1:]))
    assert report['execution_duration_s'] >= report['actual_segment_plan']['duration_s']+.25
    assert transport.sent[-1] == 28.


def test_continuous_rejected_feedback_still_expires_without_next_gripper_command():
    transport = RejectedFeedbackTransport(duration=3.)
    report = {}
    with pytest.raises(RuntimeError, match='All joint feedback recovery timeout'):
        gripper.execute_segment(28., START, transport, rate=5., speed=5., report=report)
    assert len(transport.sent) == 1
    assert 2. - 1e-8 <= transport.t-transport.valid_time <= 2.011
    assert report['future_commands_stopped'] and not report['completed']


def test_cancellation_during_feedback_wait_does_not_resume_gripper_commands():
    transport = RejectedFeedbackTransport()
    with pytest.raises(gripper.Cancelled):
        gripper.execute_segment(28., START, transport, rate=5., speed=5.,
                                cancel=lambda: transport.pending)
    assert len(transport.sent) == 1


def test_gripper_write_rechecks_pending_feedback_under_reader_lock():
    import io
    import threading
    transport = gripper.GripperTransport.__new__(gripper.GripperTransport)
    transport.lock = threading.RLock()
    transport.log = io.StringIO()
    transport.acks = transport.commands = 0
    state = dict(t=0., pending=False, snapshots=0, pause=0.)
    transport.now = lambda: state['t']
    def snapshot():
        state['snapshots'] += 1
        if state['snapshots'] == 2:
            state['pending'] = True  # Reader rejected a frame after outer check.
        return dict(q=np.asarray(START), q_time=state['t'], feedback_pending=state['pending'],
                    feedback_pause_total_s=state['pause'])
    transport.snapshot = snapshot
    def wait(seconds):
        state.update(t=state['t']+.2, pending=False, pause=.2)
    transport.wait = wait
    class Input(io.StringIO):
        def write(self, value):
            assert not state['pending']
            transport.acks += 1
            return super().write(value)
    stream = Input()
    transport.proc = SimpleNamespace(stdin=stream)
    assert transport.send_target(28., lambda: None) is True
    assert stream.getvalue() == 'j 6 28.000\n'
    assert state['t'] >= .2 and transport.commands == 1


def test_gripper_receipt_timeout_excludes_completed_feedback_pause():
    import io
    import threading
    transport = gripper.GripperTransport.__new__(gripper.GripperTransport)
    transport.lock = threading.RLock()
    transport.log = io.StringIO()
    transport.acks = transport.commands = 0
    state = dict(t=0.)
    transport.now = lambda: state['t']
    def snapshot():
        return dict(q=np.asarray(START), q_time=state['t'], feedback_pending=.1 <= state['t'] < .5,
                    feedback_pause_total_s=.4 if state['t'] >= .5 else 0.)
    transport.snapshot = snapshot
    def wait(seconds):
        state['t'] += seconds
        if state['t'] >= .75:
            transport.acks = 1
    transport.wait = wait
    transport.proc = SimpleNamespace(stdin=io.StringIO())
    assert transport.send_target(28., lambda: None) is True
    assert state['t'] >= .75 and transport.commands == 1


def test_speed_removes_old_eight_second_and_half_unit_limits():
    old = gripper.plan(28., START, rate=5.)
    faster = gripper.plan(28., START, rate=5., speed=5.)
    assert faster['duration_s'] < old['duration_s'] < 8.
    assert faster['peak_reference_velocity_sdk_units_s'] <= 5.
    assert gripper.plan(28., START, rate=5., speed=1000.)['duration_s'] == .2


@pytest.mark.parametrize('value', [0., -1., float('nan'), float('inf'), True])
@pytest.mark.parametrize('key', ['speed', 'rate'])
def test_invalid_timing_rejected(value, key):
    with pytest.raises(ValueError):
        gripper.plan(28., START, **{key: value})


def test_segment_requires_measured_completion_and_only_moves_j6():
    t = FakeTransport()
    report = gripper.execute_segment(28., START, t, speed=5.)
    assert report['completed'] and report['position_reached']
    assert report['grasp_verified'] is False and report['contact_detected'] is False
    assert report['endpoint']['feedback_count'] >= 3
    assert report['actual_segment_plan']['speed_sdk_s'] == 5.
    assert t.sent[0] < 30.  # No deliberate initial same-position command.
    assert t.sent[-1] == 28.
    assert np.array_equal(t.q[:6], np.zeros(6))


def test_zero_move_uses_feedback_without_same_position_command():
    t = FakeTransport()
    report = gripper.execute_segment(30., START, t, speed=5.)
    assert report['completed'] and report['already_at_target']
    assert not t.sent


@pytest.mark.parametrize('start', [6.5, 43.9, 36.])
def test_initial_gripper_adjustment_accepts_current_position_and_verifies_segments(start):
    expected = START.copy()
    expected[6] = start
    t = FakeTransport()
    t.q = np.asarray(expected)
    planned = gripper.plan(36., expected, speed=10., segmented=True)
    report = gripper.execute_segmented(36., expected, t, speed=10.)
    assert report['completed'] and report['position_reached']
    assert planned['units'] == 'uncalibrated_sdk_position'
    assert all(abs(item['delta_sdk_units']) <= 10 for item in planned['segments'])
    assert all(abs(item['actual_segment_plan']['delta_sdk_units']) <= 10
               for item in report['segment_results'])
    assert len(report['completed_segments']) == len(planned['segments'])
    assert t.q[6] == 36.
    assert np.array_equal(t.q[:6], np.zeros(6))
    if start == 36.:
        assert report['already_at_target'] and not t.sent
    else:
        assert t.sent and t.sent[0] != start and t.sent[-1] == 36.


def test_initial_gripper_adjustment_stops_between_segments_without_retry_or_open():
    expected = START.copy()
    expected[6] = 6.5
    t = FakeTransport()
    t.q = np.asarray(expected)
    report = {}
    with pytest.raises(gripper.Cancelled):
        gripper.execute_segmented(36., expected, t, speed=10., report=report,
            cancel=lambda: len(report.get('completed_segments', [])) == 1)
    assert len(report['completed_segments']) == 1
    assert len(report['segment_results']) == 2
    assert t.sent[-1] == report['segmented_plan']['segments'][0]['target_sdk_units']
    assert t.sent[-1] < 36.
    assert report['future_commands_stopped'] and not report['completed']
    assert report['hold_command_received'] is False


def test_initial_adjustment_does_not_remove_single_segment_or_position_limits():
    expected = START.copy()
    expected[6] = 6.5
    with pytest.raises(ValueError, match='at most 10'):
        gripper.plan(36., expected)
    for start, target in [(-1., 36.), (51., 36.), (6.5, 51.)]:
        expected[6] = start
        with pytest.raises(ValueError):
            gripper.plan(target, expected, segmented=True)


@pytest.mark.parametrize('current,target,last', [(30., 30.0001, None), (29., 30.0001, 30.)])
def test_real_transport_omits_quantized_same_position_and_duplicate_commands(current, target, last):
    # No process/log/lock is installed: any attempted write makes the test fail.
    transport = gripper.GripperTransport.__new__(gripper.GripperTransport)
    q = np.asarray(START)
    q[6] = current
    transport.snapshot = lambda: {'q': q, 'q_time': 0.}
    transport._last_j6_target = last
    checks = []
    assert transport.send_target(target, lambda: checks.append(True)) is False
    assert checks == [True]


@pytest.mark.parametrize('target', [6.5005, 6.4995, 13.3355, 36.0005])
def test_gripper_half_boundary_uses_same_quantized_value_for_wire_and_log(target):
    import io
    import threading
    transport = gripper.GripperTransport.__new__(gripper.GripperTransport)
    q = np.asarray(START)
    q[6] = 6.49
    transport.snapshot = lambda: {'q': q, 'q_time': 0.}
    transport.now = lambda: 0.
    transport.lock = threading.RLock()
    transport.log = io.StringIO()
    transport.acks = transport.commands = 0
    class Input(io.StringIO):
        def write(self, value):
            transport.acks += 1
            return super().write(value)
    stream = Input()
    transport.proc = SimpleNamespace(stdin=stream)
    assert transport.send_target(target, lambda: None) is True
    rounded = round(target, 3)
    assert stream.getvalue() == f'j 6 {rounded:.3f}\n'
    assert json.loads(transport.log.getvalue())['target_sdk_units'] == rounded
    assert transport._last_j6_target == rounded
    # Repeating a nearby raw value that quantizes identically never writes again.
    assert transport.send_target(rounded, lambda: None) is False
    assert transport.commands == 1


def test_gripper_half_boundary_that_quantizes_to_feedback_sends_nothing():
    transport = gripper.GripperTransport.__new__(gripper.GripperTransport)
    q = np.asarray(START)
    q[6] = round(6.5005, 3)
    transport.snapshot = lambda: {'q': q}
    assert transport.send_target(6.5005, lambda: None) is False


def test_gripper_can_start_from_protocol_enable_zero_and_requires_one_after_first_receipt():
    class BootstrapTransport(FakeTransport):
        def snapshot(self):
            snapshot = super().snapshot()
            snapshot['status']['enable_status'] = 0 if self.acks == 0 else 1
            return snapshot
    transport = BootstrapTransport()
    report = gripper.execute_segment(28., START, transport, speed=5., allow_position_start=True)
    assert report['completed'] and transport.commands > 0


def test_enable_zero_after_first_receipt_stops_next_gripper_command():
    class DisabledTransport(FakeTransport):
        def snapshot(self):
            snapshot = super().snapshot()
            snapshot['status']['enable_status'] = 0
            return snapshot
    transport = DisabledTransport()
    with pytest.raises(RuntimeError):
        gripper.execute_segment(28., START, transport, speed=5., allow_position_start=True)
    assert transport.commands == 1


def test_enable_zero_without_explicit_initial_start_permission_sends_nothing():
    class DisabledTransport(FakeTransport):
        def snapshot(self):
            snapshot = super().snapshot()
            snapshot['status']['enable_status'] = 0
            return snapshot
    transport = DisabledTransport()
    with pytest.raises(RuntimeError):
        gripper.execute_segment(28., START, transport, speed=5.)
    assert transport.commands == 0


def test_receipts_without_motion_do_not_claim_completion():
    t = FakeTransport('stalled')
    report = {}
    with pytest.raises(TimeoutError, match='endpoint'):
        gripper.execute_segment(28., START, t, speed=5., report=report)
    assert t.commands > 0 and t.commands == t.acks
    assert not report['completed'] and not report['position_reached']


def test_cancellation_never_adds_a_hold_or_open_command():
    t = FakeTransport()
    report = {}
    with pytest.raises(gripper.Cancelled):
        gripper.execute_segment(28., START, t, speed=1., cancel=lambda: t.now() > .75, report=report)
    assert 0 < len(t.sent)
    assert all(b < a for a, b in zip(t.sent, t.sent[1:]))
    assert report['hold_command_received'] is False
    assert report['physical_stop_verified'] is False
    assert report['future_commands_stopped'] is True


def test_non_target_joint_drift_stops_next_commands():
    t = FakeTransport()
    def drift():
        if t.now() > .5:
            t.q[0] = .6
    with pytest.raises(RuntimeError, match='Non-target joint drift'):
        gripper.execute_segment(28., START, t, speed=1., external_check=drift)
    assert t.sent[-1] > 28.


@pytest.mark.parametrize('mode,error', [('stale', RuntimeError), ('frozen_sequence', TimeoutError)])
def test_invalid_start_feedback_sends_nothing(mode, error):
    t = FakeTransport(mode)
    with pytest.raises(error):
        gripper.execute_segment(28., START, t)
    assert not t.sent


def test_wrong_start_and_limits_are_rejected_before_motion():
    t = FakeTransport()
    wrong_start = START.copy()
    wrong_start[0] = 1.
    with pytest.raises(ValueError, match='reviewed start'):
        gripper.execute_segment(28., wrong_start, t)
    assert not t.sent
    with pytest.raises(ValueError, match='range'):
        gripper.plan(66., START)


@pytest.mark.parametrize('actual_j1', [-90.1, -90.2, float(np.float32(-90.2))])
def test_gripper_accepts_arm_feedback_jitter_without_commanding_or_clamping_arm(actual_j1):
    expected = START.copy()
    expected[1] = -90.
    transport = FakeTransport()
    transport.q = np.asarray(expected)
    transport.q[1] = actual_j1
    report = gripper.execute_segment(28., expected, transport, speed=5.)
    assert report['completed']
    assert report['measured_start'][1] == actual_j1
    assert report['actual_segment_plan']['reviewed_start'][1] == actual_j1
    assert report['actual_segment_plan']['feedback_joint_tolerance_deg'] == .2
    assert transport.q[1] == actual_j1
    assert transport.sent[-1] == 28.


def test_gripper_rejects_arm_feedback_beyond_point_two_before_any_command():
    expected = START.copy()
    expected[1] = -90.
    transport = FakeTransport()
    transport.q = np.asarray(expected)
    transport.q[1] = -90.201
    with pytest.raises((ValueError, RuntimeError)):
        gripper.execute_segment(28., expected, transport, speed=5.)
    assert not transport.sent


@pytest.mark.parametrize('j6', [-.001, 65.001])
def test_arm_feedback_tolerance_never_expands_j6_feedback_range(j6):
    values = START.copy()
    values[6] = j6
    with pytest.raises(ValueError):
        gripper.check_angles(values)


def test_arm_feedback_tolerance_keeps_gripper_target_and_segment_limits_strict():
    expected = START.copy()
    expected[1] = -90.2
    with pytest.raises(ValueError, match='range'):
        gripper.plan(50.001, expected)
    expected[6] = 50.001
    with pytest.raises(ValueError, match='start'):
        gripper.plan(49., expected)
    expected[6] = 30.
    with pytest.raises(ValueError, match='at most 10'):
        gripper.plan(40.001, expected)


def test_arm_feedback_tolerance_does_not_relax_reviewed_start_difference():
    expected = START.copy()
    expected[1] = -90.
    measured = expected.copy()
    measured[1] = -89.6
    with pytest.raises(ValueError, match='reviewed start'):
        gripper.checked_start(measured, expected, 28.)


def test_stop_file_before_start_writes_summary_without_opening_hardware(tmp_path, monkeypatch):
    stop = tmp_path / 'STOP'
    stop.touch()
    args = SimpleNamespace(output=tmp_path/'run', stop_file=stop, nic='unused',
                           target=28., expected_angles=START, speed_sdk_s=5., rate_hz=5.)
    def forbidden(*args, **kwargs):
        raise AssertionError('Hardware client must not be constructed')
    monkeypatch.setattr(gripper, 'GripperTransport', forbidden)
    result = gripper.run(args, gripper.plan(28., START, speed=5.))
    summary = json.loads((args.output/'summary.json').read_text())
    assert result == 1
    assert summary['commands'] == 0 and summary['hardware_commands_sent'] is False
    assert summary['physical_stop_verified'] is False
    assert 'cancellation' in summary['error']


def test_sigterm_handler_stops_mock_run_and_restores_handlers(tmp_path, monkeypatch):
    import handeye_joint_step
    calls = []
    original_handlers = {gripper.signal.SIGINT: object(), gripper.signal.SIGTERM: object()}
    handlers = dict(original_handlers)
    def set_handler(sig, handler):
        old = handlers[sig]
        handlers[sig] = handler
        return old
    class FakeCameras:
        ready = True
        error = None
        def __init__(self, folder):
            folder.mkdir()
        def check(self):
            calls.append(1)
            if len(calls) == 6:
                handlers[gripper.signal.SIGTERM](gripper.signal.SIGTERM, None)
        def close(self):
            pass
    t = FakeTransport()
    monkeypatch.setattr(gripper.signal, 'signal', set_handler)
    monkeypatch.setattr(handeye_joint_step, 'Cameras', FakeCameras)
    monkeypatch.setattr(gripper, 'GripperTransport', lambda *args, **kwargs: t)
    args = SimpleNamespace(output=tmp_path/'run', stop_file=None, nic='unused',
                           target=28., expected_angles=START, speed_sdk_s=1., rate_hz=5.)
    assert gripper.run(args, gripper.plan(28., START)) == 1
    summary = json.loads((args.output/'summary.json').read_text())
    assert summary['future_commands_stopped'] is True
    assert summary['physical_stop_verified'] is False
    assert handlers == original_handlers
