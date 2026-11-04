"""Fixed-pose startup lag must not be mistaken for trajectory progress."""
import json
from pathlib import Path

import numpy as np
import pytest

from app.backend import _executor_reason
from d1_pick_place.sdk_executor import Cancelled, Mapping, NAMES, execute, prepare, paced_references


def plan(distance=6.,speed=20.):
    points=np.zeros((max(2,int(distance/10)+1),6))
    points[:,1]=np.linspace(0.,distance,len(points))
    return prepare(NAMES,np.deg2rad(points),np.arange(len(points),dtype=float),
        Mapping(np.ones(6),np.zeros(6)),speed=speed,acceleration=40.,rate=10.,respect_source_timing=False)


class DelayedRobot:
    """Fresh 10 Hz feedback, but motion starts late and has a finite real speed."""
    def __init__(self,startup_s=1.,speed=4.):
        self.t=0.;self.physical=np.zeros(7);self.physical[6]=36.
        self.q=self.physical.copy();self.target=self.q.copy()
        self.startup_s=startup_s;self.speed=speed;self.seq=1
        self.sample_time=0.;self.sent=[];self.errors=[];self.acks=0

    def now(self):return self.t

    def snapshot(self):
        return dict(q=self.q.copy(),q_time=self.sample_time,q_seq=self.seq,
            status=dict(power_status=1,enable_status=1,error_status=0),
            status_time={key:self.t for key in ('power_status','enable_status','error_status')},fault=None)

    def wait(self,seconds):
        end=self.t+max(seconds,.001)
        while self.t<end-1e-10:
            dt=min(.01,end-self.t);self.t+=dt
            if self.t>=self.startup_s:
                self.physical[:6]+=np.clip(self.target[:6]-self.physical[:6],-self.speed*dt,self.speed*dt)
            if self.t-self.sample_time>=.1-1e-8:
                self.q=np.round(self.physical,1);self.sample_time=self.t;self.seq+=1

    def send(self,target,check):
        check()
        self.target=np.asarray(target).copy();self.acks+=1
        self.sent.append((self.t,self.target.copy()))
        self.errors.append(float(np.max(abs(self.target[:6]-self.q[:6]))))


class ResidualRobot(DelayedRobot):
    """A servo that stops 1.4 degrees short of each intermediate command."""
    def __init__(self,finish_endpoint):
        super().__init__(startup_s=0.,speed=40.)
        self.finish_endpoint=finish_endpoint

    def wait(self,seconds):
        end=self.t+max(seconds,.001)
        while self.t<end-1e-10:
            dt=min(.01,end-self.t);self.t+=dt
            residual=0. if self.finish_endpoint and abs(self.target[1]-6.)<1e-8 else 1.4
            delta=self.target[:6]-self.physical[:6]
            self.physical[:6]+=np.sign(delta)*np.minimum(np.maximum(abs(delta)-residual,0.),self.speed*dt)
            if self.t-self.sample_time>=.1-1e-8:
                self.q=np.round(self.physical,1);self.sample_time=self.t;self.seq+=1


def single_step_plan(axis,distance):
    start=np.zeros(6);end=start.copy();end[axis]=distance
    return dict(sdk=[start.tolist(),end.tolist()],ticks=[1],rate=10.,
                active_joints=[axis],duration_s=.1,timing_profile='edge_quintic_v1')


def test_intermediate_1_4_degree_residual_does_not_prevent_approved_path_progress():
    robot=ResidualRobot(finish_endpoint=True)
    result=execute(plan(),robot,progress_policy='feedback_paced')
    assert result['completed'] and robot.target[1]==pytest.approx(6.)
    assert max(robot.errors)<=2.5+1e-5
    assert max(abs(value) for value in result['endpoint_error_deg'])<=1.


def test_final_1_4_degree_residual_still_fails_user_one_degree_arrival():
    robot=ResidualRobot(finish_endpoint=False)
    with pytest.raises(TimeoutError,match='Endpoint position not reached'):
        execute(plan(),robot,progress_policy='feedback_paced')
    assert robot.target[1]==pytest.approx(6.) and robot.q[1]==pytest.approx(4.6)
    assert max(robot.errors)<=2.5+1e-5


@pytest.mark.parametrize('axis',range(6))
@pytest.mark.parametrize('distance',[2.5,2.51])
def test_candidate_admission_boundary_applies_to_every_arm_axis(axis,distance):
    robot=DelayedRobot(startup_s=100.)
    snapshot=robot.snapshot
    def retreating_snapshot():
        value=snapshot()
        if robot.now()>=.05:value['q'][axis]=1.4-distance
        return value
    robot.snapshot=retreating_snapshot
    with pytest.raises(TimeoutError) as caught:
        execute(single_step_plan(axis,1.4),robot,progress_policy='feedback_paced')
    if distance==2.5:
        assert len(robot.sent)==1 and robot.target[axis]==pytest.approx(1.4)
        assert 'Endpoint position not reached' in str(caught.value)
    else:
        assert len(robot.sent)==1 and robot.target[axis]==pytest.approx(1.389)
        assert robot.now()<2.3
        for _,target in robot.sent:
            assert 0.<=target[axis]<1.4
            assert abs(target[axis]-(1.4-distance))<=2.5+1e-5
            assert np.allclose(np.delete(target[:6],axis),0.) and target[6]==36.
        diagnostic=caught.value.execution_diagnostics
        assert diagnostic['next_reference_joint']==axis
        assert diagnostic['next_reference_max_error_deg']==pytest.approx(2.51)
        assert diagnostic['reference_sdk'][axis]==pytest.approx(robot.target[axis])
        assert diagnostic['next_reference_sdk'][axis]==pytest.approx(1.4)
        assert diagnostic['progression_limit_deg']==2.5


@pytest.mark.parametrize('tracking_fault',[False,True])
def test_candidate_is_checked_after_due_time_against_latest_feedback(tracking_fault):
    robot=DelayedRobot(startup_s=100.)
    snapshot=robot.snapshot
    def retreating_snapshot():
        value=snapshot()
        if robot.now()>=.05:value['q'][1]=-1.2
        if tracking_fault and robot.now()>=.5:value['q'][1]=-3.1
        return value
    robot.snapshot=retreating_snapshot
    expected=RuntimeError if tracking_fault else TimeoutError
    with pytest.raises(expected,match='Tracking error' if tracking_fault else 'no measured joint progress') as caught:
        execute(single_step_plan(1,1.4),robot,progress_policy='feedback_paced')
    assert len(robot.sent)==1 and robot.target[1]==pytest.approx(1.299)
    assert robot.target[1]-(-1.2)<=2.5
    for stamp,target in robot.sent:
        latest_actual=-3.1 if tracking_fault and stamp>=.5 else -1.2
        assert 0.<=target[1]<1.4 and abs(target[1]-latest_actual)<=2.5+1e-5
        assert np.allclose(target[[0,2,3,4,5]],0.) and target[6]==36.
    if tracking_fault:
        assert robot.now()<=.511
        assert caught.value.execution_diagnostics['max_error_deg']>3.
        assert caught.value.execution_diagnostics['reference_sdk'][1]==pytest.approx(robot.sent[-1][1][1])
    else:
        assert robot.now()<2.3


def test_saved_172255_stall_allows_only_the_next_reference_with_tracking_margin():
    root=Path(__file__).resolve().parents[1]
    source=root/'logs/20261026_172255_d66d7b/execution/01_fixed_pregrasp/executed'
    before={name:(source/name).read_bytes() for name in ('plan.json','result.json')}
    approved=json.loads(before['plan.json'])['sdk_plan'];recorded=json.loads(before['result.json'])
    reference=np.asarray(recorded['execution_diagnostics']['reference_sdk'])[:6]
    actual=np.asarray(recorded['final_angles'])[:6]
    refs=list(paced_references(approved))
    index=min(range(len(refs)),key=lambda i:float(np.max(abs(refs[i][1]-reference))))
    assert max(abs(actual-reference))==pytest.approx(1.4077952553)
    assert max(abs(actual-refs[index+1][1]))==pytest.approx(2.1173110685)
    assert max(abs(actual-refs[index+1][1]))<=2.5
    assert max(abs(actual-refs[index+2][1]))>2.5
    assert {name:(source/name).read_bytes() for name in before}==before


def test_same_delayed_robot_waits_safely_with_both_trajectory_policies():
    approved=plan()
    for policy in ('scheduled','feedback_paced'):
        robot=DelayedRobot()
        result=execute(approved,robot,progress_policy=policy)
        assert result['completed'] and result['progress_policy']==policy
        assert result['feedback_pacing_wait_s']>.3 and result['feedback_pacing_waits']>0
        assert max(robot.errors)<=2.5+1e-5
        assert abs(result['endpoint_error_deg'][1])<=1.
        assert robot.target[1]==pytest.approx(6.) and robot.target[6]==36.
        assert not hasattr(robot,'hold')


def test_receipt_without_motion_stops_with_specific_evidence_instead_of_advancing():
    robot=DelayedRobot(startup_s=100.)
    with pytest.raises(TimeoutError,match='no measured joint progress') as caught:
        execute(plan(),robot,progress_policy='feedback_paced')
    diagnostic=caught.value.execution_diagnostics
    assert 2.<diagnostic['wait_s']<2.1 and diagnostic['measured_change_deg']==0.
    assert diagnostic['joint']==1 and diagnostic['actual_sdk'][1]==0.
    assert max(robot.errors)<=2.5+1e-5 and robot.target[1]<3.
    assert caught.value.future_commands_stopped and not caught.value.physical_stop_verified


def test_paced_references_subdivide_large_steps_without_changing_path_or_endpoint():
    approved=plan(50.,speed=500.)
    samples=list(paced_references(approved))
    assert max(float(np.max(abs(b-a))) for (_t,a),(_u,b) in zip(samples,samples[1:]))<=1.5+1e-8
    assert all(np.allclose(q[[0,2,3,4,5]],0.) for _,q in samples)
    assert samples[0][1][1]==0. and samples[-1][1][1]==50.
    assert all(b[0]>a[0] for a,b in zip(samples,samples[1:]))


def test_cancellation_during_wait_sends_no_recovery_or_replay():
    robot=DelayedRobot(startup_s=100.)
    with pytest.raises(Cancelled):
        execute(plan(),robot,progress_policy='feedback_paced',cancel=lambda:robot.now()>.8)
    assert robot.target[1]<3. and len(robot.sent)==len(set(q[1] for _,q in robot.sent))


@pytest.mark.parametrize('failure',['stale','camera','enable'])
def test_feedback_wait_keeps_freshness_camera_and_control_state_checks(failure):
    robot=DelayedRobot(startup_s=100.)
    snapshot=robot.snapshot
    def bad_snapshot():
        value=snapshot()
        if robot.now()>.8:
            if failure=='stale':value['q_time']=-10.
            if failure=='enable':value['status']['enable_status']=0
        return value
    def camera():
        if failure=='camera' and robot.now()>.8:raise RuntimeError('Camera disconnected')
    robot.snapshot=bad_snapshot
    with pytest.raises(RuntimeError,match={'stale':'All joint feedback recovery timeout','camera':'Camera','enable':'enable_status'}[failure]):
        execute(plan(),robot,progress_policy='feedback_paced',external_check=camera)
    assert robot.target[1]<3.


def test_continuing_slow_progress_has_bounded_total_wait():
    robot=DelayedRobot(startup_s=0.,speed=1.)
    with pytest.raises(TimeoutError,match='wait budget exceeded'):
        execute(plan(80.),robot,progress_policy='feedback_paced')
    assert robot.now()<45. and max(robot.errors)<=2.5+1e-5


def test_specific_failure_ui_shows_the_measured_axis_reference_and_wait(tmp_path):
    diagnostic=dict(actual_sdk=[0.]*6+[36.],reference_sdk=[0.,1.4,0.,0.,0.,0.,36.],wait_s=2.01,measured_change_deg=.1)
    report=dict(error='Accepted commands but no measured joint progress',execution_diagnostics=diagnostic)
    reason=_executor_reason(report,tmp_path/'result.json')
    for text in ('设备已接收指令','未观察到有效关节运动','J1','参考 1.400°','实际反馈 0.000°','等待 2.01 秒','变化 0.100°'):
        assert text in reason
    report['error']='Tracking error >3 degrees';diagnostic['reference_sdk'][1]=3.4
    assert 'J1' in _executor_reason(report,tmp_path/'result.json')


def test_full_cycle_default_remains_scheduled():
    robot=DelayedRobot(startup_s=0.,speed=100.)
    result=execute(plan(),robot)
    assert result['completed'] and result['progress_policy']=='scheduled'
    assert result['feedback_pacing_wait_s']==0.


def test_saved_user_trajectory_and_static_feedback_reproduce_failure_read_only():
    root=Path(__file__).resolve().parents[1]
    source=root/'logs/20261026_170613_da3b42/execution/01_fixed_pregrasp/executed'
    before={name:(source/name).read_bytes() for name in ('plan.json','result.json')}
    approved=json.loads(before['plan.json'])['sdk_plan']
    actual=json.loads(before['result.json'])['measured_start']
    def static_robot():
        robot=DelayedRobot(startup_s=100.)
        robot.q=np.asarray(actual);robot.physical=robot.q.copy();robot.target=robot.q.copy()
        return robot
    scheduled_robot=static_robot()
    with pytest.raises(TimeoutError,match='Accepted commands but no measured joint progress') as scheduled:
        execute(approved,scheduled_robot,gripper_target=36.)
    assert scheduled.value.future_commands_stopped
    assert max(scheduled_robot.errors)<=2.5+1e-5
    robot=static_robot()
    with pytest.raises(TimeoutError,match='no measured joint progress') as caught:
        execute(approved,robot,gripper_target=36.,progress_policy='feedback_paced')
    assert caught.value.execution_diagnostics['joint']==1
    assert robot.now()<5. and max(robot.errors)<=2.5+1e-5
    assert {name:(source/name).read_bytes() for name in before}==before
