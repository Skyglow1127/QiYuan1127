"""Offline speed/feedback regression tests; never construct a hardware client."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace
import sys

import numpy as np
import pytest

CONTROL = Path(__file__).resolve().parents[1] / 'runtime/Control'
sys.path.insert(0, str(CONTROL / 'ros2_ws/src/d1_pick_place'))
from d1_pick_place.sdk_executor import Cancelled, Mapping, NAMES, CliTransport, execute, healthy, prepare, references

_spec = importlib.util.spec_from_file_location('one_click_arm_stage', CONTROL / 'ros2_ws/tools/run_arm_stage.py')
arm_stage = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(arm_stage)


def make_plan(speed=.5, acceleration=1., source_time=120., **kwargs):
    return prepare(NAMES, [[0.] * 6, [np.deg2rad(6.), 0., 0., 0., 0., 0.]],
                   [0., source_time], Mapping(np.ones(6), np.zeros(6)),
                   speed=speed, acceleration=acceleration, **kwargs)


def test_user_speed_retimes_geometry_without_old_half_degree_ceiling():
    slow = make_plan(respect_source_timing=False)
    fast = make_plan(speed=5., acceleration=10., respect_source_timing=False)
    inherited = make_plan(speed=5., acceleration=10., respect_source_timing=True)
    assert fast['duration_s'] < slow['duration_s'] < inherited['duration_s']
    assert inherited['duration_s'] == 120.
    assert fast['sdk'] == slow['sdk'] == inherited['sdk']
    samples = list(references(fast))
    t = np.asarray([stamp for stamp, _ in samples])
    q = np.asarray([value for _, value in samples])
    peak = np.max(np.abs(np.diff(q, axis=0) / np.diff(t)[:, None]))
    assert .5 < peak <= 5. + 1e-8
    assert np.allclose(q[-1], [6., 0., 0., 0., 0., 0.])


def test_new_timing_preserves_actual_corner():
    points = np.deg2rad([[0., 0., 0., 0., 0., 0.],
                        [5., 0., 0., 0., 0., 0.],
                        [5., 4., 0., 0., 0., 0.]])
    p = prepare(NAMES, points, [0., 100., 200.], Mapping(np.ones(6), np.zeros(6)),
                speed=8., acceleration=20., respect_source_timing=False)
    assert p['stopped_corners'] == 1
    assert np.allclose(p['sdk'], np.rad2deg(points))
    assert p['waypoint_velocities_deg_s'][1] == [0.] * 6


def test_dense_collinear_waypoints_do_not_reintroduce_a_slow_time_floor():
    points = np.zeros((101, 6))
    points[:, 0] = np.deg2rad(np.linspace(0., 6., 101))
    dense = prepare(NAMES, points, np.linspace(0., 120., 101),
                    Mapping(np.ones(6), np.zeros(6)), speed=5., acceleration=10.,
                    respect_source_timing=False)
    sparse = make_plan(speed=5., acceleration=10., respect_source_timing=False)
    assert dense['duration_s'] == sparse['duration_s']


@pytest.mark.parametrize('value', [0., -1., float('nan'), float('inf'), True])
@pytest.mark.parametrize('parameter', ['speed', 'acceleration', 'rate'])
def test_invalid_timing_rejected(value, parameter):
    kwargs = {parameter: value}
    with pytest.raises(ValueError):
        make_plan(**kwargs)


def test_plan_metadata_controls_preview_and_execution_speed():
    timing = dict(rate=5., speed=7., acceleration=12., respect_source_timing=False)
    source = {'execution_timing': timing}
    assert arm_stage.execution_timing(source, SimpleNamespace()) == timing
    assert arm_stage.execution_timing(source, SimpleNamespace(speed_deg_s=7.)) == timing
    with pytest.raises(ValueError, match='differs'):
        arm_stage.execution_timing(source, SimpleNamespace(speed_deg_s=8.))


class FakeTransport:
    def __init__(self, mode='follow'):
        self.t = 0.
        self.q = np.array([0., 0., 0., 0., 0., 0., 30.])
        self.seq = 1
        self.sent = []
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
        self.seq += 1

    def send(self, q, check):
        check()
        self.sent.append(np.asarray(q).copy())
        if self.mode != 'stalled':
            self.q = np.asarray(q).copy()


def test_cancellation_stops_future_commands_without_same_position_hold():
    t = FakeTransport()
    with pytest.raises(Cancelled) as caught:
        execute(make_plan(respect_source_timing=False), t, cancel=lambda: t.now() > .7)
    assert caught.value.hold_command_received is False
    assert caught.value.physical_stop_verified is False
    assert caught.value.future_commands_stopped is True
    assert len(t.sent) >= 2
    assert not np.array_equal(t.sent[-1], t.sent[-2])
    assert t.sent[-1][0] < 6.


def test_stale_feedback_still_sends_no_command_at_user_speed():
    t = FakeTransport('stale')
    with pytest.raises(RuntimeError, match='stale'):
        execute(make_plan(speed=8., acceleration=20., respect_source_timing=False), t)
    assert not t.sent


def test_joint_limit_still_rejected_at_user_speed():
    with pytest.raises(ValueError, match='joint limit'):
        prepare(NAMES, [[0.] * 6, [np.deg2rad(136.), 0., 0., 0., 0., 0.]],
                [0., .2], Mapping(np.ones(6), np.zeros(6)), speed=8., acceleration=20.)


def test_receipt_without_motion_still_fails():
    t = FakeTransport('stalled')
    with pytest.raises(TimeoutError, match='Accepted commands but no measured joint progress') as caught:
        execute(make_plan(speed=5., acceleration=10., respect_source_timing=False), t)
    assert caught.value.future_commands_stopped and t.q[0] == 0.
    assert t.sent and np.max(np.abs(np.asarray(t.sent)[:, :6])) <= 2.5 + 1e-5


@pytest.mark.parametrize('offset,accepted', [(.482, True), (1., True), (1.001, False)])
def test_fixed_stage_requires_arrival_within_next_stage_start_tolerance(offset,accepted):
    class OffsetTransport(FakeTransport):
        def send(self,q,check):
            super().send(q,check)
            self.q[0]-=offset
    plan=make_plan(speed=5.,acceleration=10.,respect_source_timing=False)
    if not accepted:
        with pytest.raises(TimeoutError,match='Endpoint position'):
            execute(plan,OffsetTransport(),endpoint_tolerance_deg=arm_stage.ENDPOINT_TOLERANCE_DEG)
    else:
        completed=execute(plan,OffsetTransport(),endpoint_tolerance_deg=arm_stage.ENDPOINT_TOLERANCE_DEG)
        assert completed['completed'] and completed['endpoint_tolerance_deg']==1.


@pytest.mark.parametrize('offset,accepted', [(1.,True),(1.001,False)])
def test_executor_stage_start_uses_one_degree_tolerance(offset,accepted):
    transport=FakeTransport()
    transport.q[0]=offset
    plan=make_plan(speed=5.,acceleration=10.,respect_source_timing=False)
    if accepted:
        assert execute(plan,transport)['completed']
    else:
        with pytest.raises(ValueError,match='Plan start'):
            execute(plan,transport)
        assert transport.sent==[]


def test_approved_plan_digest_is_checked_before_loading_trajectory():
    import hashlib
    raw=b'{"immutable": "approved trajectory"}'
    digest=hashlib.sha256(raw).hexdigest()
    assert arm_stage.check_approved_plan_digest(raw,digest)==digest
    with pytest.raises(ValueError,match='Approved trajectory file changed'):
        arm_stage.check_approved_plan_digest(raw+b' ',digest)


def test_measured_completion_works_at_user_speed():
    t = FakeTransport()
    result = execute(make_plan(speed=5., acceleration=10., respect_source_timing=False), t)
    assert result['completed']
    assert np.allclose(result['actual_sdk'][:6], [6., 0., 0., 0., 0., 0.])
    assert all(q[6] == 30. for q in t.sent)


class StartingTransport(FakeTransport):
    def __init__(self, lose=False):
        super().__init__()
        self.acks=0
        self.lose=lose

    def snapshot(self):
        result=super().snapshot()
        result['status']['enable_status']=int(self.acks>0 and not(self.lose and self.acks>=2))
        return result

    def send(self,q,check):
        super().send(q,check)
        self.acks+=1


def test_position_start_permission_is_explicit_and_does_not_send_initial_hold():
    t=StartingTransport()
    with pytest.raises(RuntimeError,match='enable_status'):
        execute(make_plan(speed=5.,acceleration=10.,respect_source_timing=False),t)
    assert not t.sent
    result=execute(make_plan(speed=5.,acceleration=10.,respect_source_timing=False),t,allow_position_start=True)
    assert result['completed'] and t.sent[0][0] > 0


def test_enable_drop_after_first_command_is_fault_not_another_start():
    t=StartingTransport(lose=True)
    with pytest.raises(RuntimeError,match='enable_status'):
        execute(make_plan(speed=5.,acceleration=10.,respect_source_timing=False),t,allow_position_start=True)
    assert len(t.sent)==2


@pytest.mark.parametrize('field,value', [('power_status',0),('error_status',1),('enable_status',2)])
def test_startup_permission_does_not_allow_bad_protocol_state(field,value):
    t=StartingTransport()
    snapshot=t.snapshot();snapshot['status'][field]=value
    with pytest.raises(RuntimeError): healthy(snapshot,t.now(),allow_unstarted_position=True)
    snapshot=t.snapshot();snapshot['status_time']['enable_status']=-1
    with pytest.raises(RuntimeError): healthy(snapshot,t.now(),allow_unstarted_position=True)


def test_wire_transport_sends_only_changed_active_joints_and_never_j6_hold():
    import io,json,threading
    t=object.__new__(CliTransport)
    t.lock=threading.RLock();t.log=io.StringIO();t.acks=t.commands=0
    t.q=np.array([0.,0.,0.,0.,0.,0.,36.]);t.q_time=1.;t.last_targets={};t.active_joints=(0,2)
    t.now=lambda: 1.
    lines=[]
    class Stdin:
        def write(self,line): lines.append(line)
        def flush(self): t.acks+=1
    t.proc=SimpleNamespace(stdin=Stdin())
    assert t.send([.0004,30.,.0004,0.,0.,0.,29.6],lambda:None)==0
    q=[.1254,30.,.2504,0.,0.,0.,29.6]
    assert t.send(q,lambda:None)==2
    assert lines==['j 0 0.125 8\n','j 2 0.250 16\n']
    assert t.send(q,lambda:None)==0  # Does not resend a lagging target.
    records=[json.loads(line) for line in t.log.getvalue().splitlines()]
    assert [r['joint'] for r in records]==[0,2] and t.commands==2
