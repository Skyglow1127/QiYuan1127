"""Method 01 uses a virtual robot and clock; it never starts DDS or a CLI."""
from pathlib import Path
import sys

import numpy as np
import pytest

CONTROL = Path(__file__).resolve().parents[1] / 'runtime/Control'
sys.path.insert(0, str(CONTROL / 'ros2_ws/src/d1_pick_place'))
from d1_pick_place.sdk_executor import Cancelled, execute


def approved_plan():
    return dict(sdk=[[0.] * 6, [0.8, 0.4, 0., 0., 0., 0.]], ticks=[4],
                rate=10., active_joints=[0, 1], duration_s=.4,
                timing_profile='edge_quintic_v1')


class AckRobot:
    def __init__(self, ack_per_axis=.01, pause_s=0., oversleep_s=0., frozen_axis=None):
        self.t=self.q_time=0.;self.q_seq=1
        self.q=np.r_[np.zeros(6), 36.];self.target=self.q.copy()
        self.acks=self.commands=0;self.ack_per_axis=ack_per_axis
        self.pause_s=pause_s;self.pause_total=0.;self.pending=False
        self.pause_start=self.pause_end=None;self.oversleep_s=oversleep_s
        self.frozen_axis=frozen_axis;self.group_starts=[];self.group_returns=[]
        self.writes=[];self.receipts=[];self.sent_targets=[]

    def now(self):return self.t

    def snapshot(self):
        return dict(q=self.q.copy(), q_time=self.q_time, q_seq=self.q_seq, fault=None,
            status=dict(power_status=1, enable_status=1, error_status=0),
            status_time={k:self.t for k in ('power_status','enable_status','error_status')},
            feedback_pending=self.pending, feedback_pause_total_s=self.pause_total)

    def wait(self, seconds):
        seconds=seconds if seconds>0 else .0001
        if self.oversleep_s:
            seconds+=self.oversleep_s;self.oversleep_s=0.
        if self.pending:seconds=min(seconds,max(0.,self.pause_end-self.t))
        self.t+=seconds
        if self.pending and self.t>=self.pause_end-1e-10:
            self.pending=False;self.pause_total+=self.t-self.pause_start
        if not self.pending:
            if self.frozen_axis is not None:
                # Another commanded axis keeps moving while the blocking axis
                # is stationary. Its movement cannot certify joint 1 progress.
                self.q[0]+=np.clip(self.target[0]-self.q[0],-.3*seconds,.3*seconds)
            self.q_time=self.t;self.q_seq+=1

    def send(self, target, check):
        check();self.group_starts.append(self.t);self.sent_targets.append(target.copy())
        for joint in self.active_joints:
            check();self.target[joint]=target[joint]
            if self.frozen_axis is None:self.q[joint]=target[joint]
            self.writes.append((self.t,joint,float(target[joint])));self.commands+=1
            if len(self.writes)==1 and self.pause_s:
                self.pending=True;self.pause_start=self.t;self.pause_end=self.t+self.pause_s
            deadline=self.t+self.ack_per_axis
            while self.t<deadline-1e-10:
                self.wait(min(.005,deadline-self.t));check()
            self.acks+=1;self.receipts.append((self.t,joint))
        self.group_returns.append(self.t)
        return len(self.active_joints)


def run(robot, **kwargs):
    return execute(approved_plan(),robot,progress_policy='feedback_paced',
                   scheduling_policy='ack_overlap_v1',**kwargs)


@pytest.mark.parametrize('axis_ack', [.01,.075])
def test_ack_is_inside_period_and_long_batches_shift_without_backlog(axis_ack):
    robot=AckRobot(ack_per_axis=axis_ack);result=run(robot)
    interval=max(.1,2*axis_ack)
    assert robot.group_starts==pytest.approx([.1+i*interval for i in range(4)])
    assert np.diff(robot.group_starts)==pytest.approx([interval]*3)
    assert result['scheduling_policy']=='ack_overlap_v1'
    assert result['command_wait_pause_s']==0.
    assert result['scheduling_overrun_s']==pytest.approx(3*max(0.,2*axis_ack-.1))
    # Keep the approved order and individual receipts, even for long batches.
    assert [j for _,j,_ in robot.writes]==[0,1]*4
    assert [j for _,j in robot.receipts]==[0,1]*4
    for next_start,previous_return in zip(robot.group_starts[1:],robot.group_returns):
        assert next_start>=previous_return-1e-9
    events=result['command_group_schedule']
    assert [e['source_elapsed_s'] for e in events]==pytest.approx([.1,.2,.3,.4])
    assert [e['group_started_monotonic_s'] for e in events]==pytest.approx(robot.group_starts)
    assert [e['group_completed_monotonic_s'] for e in events]==pytest.approx(robot.group_returns)
    assert all(e['completed'] and e['commands']==2 for e in events)


def test_default_policy_preserves_ack_then_full_period():
    robot=AckRobot(ack_per_axis=.01)
    result=execute(approved_plan(),robot,progress_policy='feedback_paced')
    assert result['scheduling_policy']=='ack_then_period_v1'
    assert robot.group_starts==pytest.approx([.1,.22,.34,.46])
    assert result['command_wait_pause_s']==pytest.approx(.08)


def test_feedback_freeze_moves_clock_once_during_ack_wait():
    robot=AckRobot(pause_s=.08);result=run(robot)
    assert result['feedback_pause_s']==pytest.approx(.08)
    assert robot.group_starts==pytest.approx([.1,.28,.38,.48])
    assert result['scheduling_overrun_s']==pytest.approx(0.,abs=1e-9)
    assert result['command_wait_pause_s']==0.


def test_scheduler_oversleep_reanchors_once_without_replaying_due_queue():
    robot=AckRobot(oversleep_s=.26);result=run(robot)
    assert robot.group_starts==pytest.approx([.28,.38,.48,.58])
    assert result['scheduling_overrun_s']==pytest.approx(.18)
    assert len(robot.sent_targets)==4
    assert [q[0] for q in robot.sent_targets]==sorted(q[0] for q in robot.sent_targets)


def test_adaptive_subdivision_keeps_source_intervals_after_actual_group_start():
    from test_adaptive_pacing import ResidualRobot, approved
    robot=ResidualRobot(receipt_wait=.03)
    with pytest.raises(TimeoutError,match='Endpoint position not reached') as caught:
        execute(approved([[0.],[20.]]),robot,progress_policy='feedback_paced',
                scheduling_policy='ack_overlap_v1')
    events=caught.value.command_group_schedule
    assert caught.value.adaptive_reference_steps
    source=np.array([e['source_elapsed_s'] for e in events])
    starts=np.array([e['group_started_monotonic_s'] for e in events])
    assert np.all(np.diff(source)>0.)
    assert np.all(np.diff(starts)+1e-8>=np.diff(source))
    assert np.all(np.diff(starts)>=.03-1e-8)
    assert robot.target[0]==pytest.approx(20.)
    assert all(q[6]==36. for _,q in robot.sent)


def test_stationary_blocking_axis_is_not_masked_by_another_moving_axis():
    robot=AckRobot(frozen_axis=1)
    plan=dict(sdk=[[0.]*6,[2.,4.8,0.,0.,0.,0.]],ticks=[2],rate=10.,
              active_joints=[0,1],duration_s=.2,timing_profile='edge_quintic_v1')
    with pytest.raises(TimeoutError,match='no measured joint progress') as caught:
        execute(plan,robot,progress_policy='feedback_paced',scheduling_policy='ack_overlap_v1')
    d=caught.value.execution_diagnostics
    assert d['measured_change_deg']>.2
    assert d['blocking_joint']==1 and d['next_reference_joint']==1
    assert d['blocking_joint_change_deg']==0.
    assert d['blocking_joint_error_progress_deg']==0.
    assert d['blocking_joint_error_deg']==pytest.approx(d['blocking_joint_initial_error_deg'])
    assert caught.value.future_commands_stopped
    assert all(e['completed'] for e in caught.value.command_group_schedule)


def test_overlap_requires_explicit_feedback_paced_opt_in():
    robot=AckRobot()
    with pytest.raises(ValueError,match='requires.*fixed-pose'):
        execute(approved_plan(),robot,scheduling_policy='ack_overlap_v1')
    with pytest.raises(ValueError,match='Unknown command scheduling'):
        execute(approved_plan(),robot,progress_policy='feedback_paced',scheduling_policy='other')
    assert robot.writes==[]


def test_cancel_during_per_axis_ack_stops_future_axes_and_records_incomplete_group():
    robot=AckRobot(ack_per_axis=.075)
    with pytest.raises(Cancelled) as caught:
        run(robot,cancel=lambda:robot.now()>.12)
    assert [j for _,j,_ in robot.writes]==[0]
    assert caught.value.future_commands_stopped
    event=caught.value.command_group_schedule[0]
    assert not event['completed'] and event['commands'] is None
    assert event['group_completed_monotonic_s']==pytest.approx(robot.now())


@pytest.mark.parametrize('guard', ['external','tracking'])
def test_overlap_retains_external_and_three_degree_tracking_guards(guard):
    robot=AckRobot()
    def external():
        if robot.now()>=.15:
            if guard=='external':raise RuntimeError('Collision scene invalid')
            robot.q[1]=4.
    with pytest.raises(RuntimeError,match='Collision scene' if guard=='external' else 'Tracking error >3'):
        run(robot,external_check=external)
    assert len(robot.group_starts)==1
