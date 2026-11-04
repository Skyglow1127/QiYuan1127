"""Only the reviewed fixed-pose endpoint may be retried, using actual feedback."""
import io
import json
from pathlib import Path
import threading
from types import SimpleNamespace

import numpy as np
import pytest

from d1_pick_place.endpoint_control import ENDPOINT_RETRY_POLICY
from d1_pick_place.sdk_executor import Cancelled, CliTransport, Mapping, NAMES, execute, prepare


def approved_plan():
    points=np.zeros((2,6));points[-1,4]=3.
    return prepare(NAMES,np.deg2rad(points),[0.,1.],Mapping(np.ones(6),np.zeros(6)),
                   rate=10.,speed=20.,acceleration=40.,respect_source_timing=False)


class EndpointRobot:
    """A real-time-independent robot with a terminal position residual."""
    def __init__(self,residual=1.128,improvement=.3):
        self.t=0.;self.q=np.r_[np.zeros(6),36.];self.seq=1;self.sample_time=0.
        self.residual=residual;self.improvement=improvement
        self.sent=[];self.retries=[];self.acks=0;self.commands=0
        self.seek_start=None;self.failure=None;self.freeze_sequence=False

    def now(self):return self.t

    def snapshot(self):
        value=dict(q=self.q.copy(),q_time=self.t,q_seq=self.seq,fault=None,
            status=dict(power_status=1,enable_status=1,error_status=0),
            status_time={key:self.t for key in ('power_status','enable_status','error_status')})
        if self.seek_start is not None and self.t-self.seek_start>1.5:
            if self.failure in value['status']:value['status'][self.failure]=0 if self.failure!='error_status' else 1
            if self.failure=='stale':value['q_time']=self.t-3.
            if self.failure=='limit':value['q'][4]=91.
            if self.failure=='tracking':value['q'][4]=6.1
        return value

    def wait(self,seconds):
        self.t+=max(seconds,.001)
        if self.t-self.sample_time>=.1-1e-8:
            self.sample_time=self.t
            if not self.freeze_sequence:self.seq+=1

    def send(self,target,check):
        check();self.sent.append(np.asarray(target).copy());self.commands+=1;self.acks+=1
        self.q=np.asarray(target).copy()
        if abs(target[4]-3.)<1e-8:
            self.q[4]=target[4]-self.residual;self.seek_start=self.t
        return 1

    def retry_endpoint(self,target,check,tolerance):
        check()
        assert self.t-self.snapshot()['q_time']<=.6
        outstanding=[j for j in self.active_joints if j<6 and abs(target[j]-self.q[j])>tolerance+1e-5]
        for j in outstanding:
            assert target[j]==pytest.approx(3.) and j==4
            self.retries.append((self.t,j,float(target[j]),float(self.q[j]),self.seq))
            self.q[j]=target[j]-max(0.,abs(target[j]-self.q[j])-self.improvement)
            self.commands+=1;self.acks+=1
        return len(outstanding)


def run(robot,policy=ENDPOINT_RETRY_POLICY,**kwargs):
    return execute(approved_plan(),robot,gripper_target=36.,progress_policy='feedback_paced',
                   endpoint_policy=policy,**kwargs)


def test_1_128_degree_residual_can_finish_after_reissuing_same_approved_endpoint():
    robot=EndpointRobot()
    result=run(robot)
    assert result['completed'] and result['endpoint_retry_rounds']==result['endpoint_retry_commands']==1
    assert result['endpoint_initial_error_deg']==pytest.approx(1.128)
    assert abs(result['endpoint_error_deg'][4])<1.
    assert robot.retries[0][2]==pytest.approx(approved_plan()['sdk'][-1][4])
    assert result['endpoint_seek_elapsed_s']>=1.2
    assert robot.q[6]==36.


def test_persistent_residual_stops_after_five_seconds_without_effective_improvement():
    robot=EndpointRobot(improvement=0.)
    with pytest.raises(TimeoutError,match='Endpoint convergence stalled') as caught:
        run(robot)
    diagnostic=caught.value.endpoint_diagnostics
    assert 5.<=diagnostic['endpoint_seek_elapsed_s']<5.05
    assert diagnostic['endpoint_retry_rounds']==diagnostic['endpoint_retry_commands']==4
    assert diagnostic['endpoint_best_error_deg']==pytest.approx(1.128)
    assert caught.value.execution_diagnostics['joint']==4
    assert all(b[0]-a[0]>=1.-1e-8 for a,b in zip(robot.retries,robot.retries[1:]))
    assert caught.value.future_commands_stopped and not caught.value.physical_stop_verified


def test_continuing_small_progress_still_has_a_thirty_second_budget():
    robot=EndpointRobot(residual=2.8,improvement=.05)
    with pytest.raises(TimeoutError,match='Endpoint convergence time budget exceeded') as caught:
        run(robot)
    diagnostic=caught.value.endpoint_diagnostics
    assert 30.<=diagnostic['endpoint_seek_elapsed_s']<30.05
    assert diagnostic['endpoint_retry_rounds']>20
    assert diagnostic['endpoint_no_progress_s']<5.
    assert robot.q[4]<2.


def test_already_arrived_and_stable_axes_need_no_retry():
    robot=EndpointRobot(residual=.7)
    result=run(robot)
    assert result['completed'] and result['endpoint_retry_commands']==0
    assert robot.retries==[]


def test_without_approved_retry_policy_the_original_passive_settle_is_preserved():
    robot=EndpointRobot()
    with pytest.raises(TimeoutError,match='Endpoint position not reached'):
        run(robot,policy=None)
    assert robot.retries==[] and 5.<robot.now()-robot.seek_start<5.1


def test_endpoint_policy_cannot_enable_retries_on_a_clock_driven_grasp():
    robot=EndpointRobot()
    with pytest.raises(ValueError,match='reviewed fixed-pose'):
        execute(approved_plan(),robot,endpoint_policy=ENDPOINT_RETRY_POLICY)
    assert robot.sent==robot.retries==[]


def test_each_retry_round_requires_new_feedback_sequence():
    robot=EndpointRobot(improvement=0.)
    def freeze_after_first_retry():
        if robot.retries:robot.freeze_sequence=True
    with pytest.raises(TimeoutError,match='Endpoint convergence stalled'):
        run(robot,external_check=freeze_after_first_retry)
    assert len(robot.retries)==1


@pytest.mark.parametrize('failure',['stale','limit','tracking','power_status','enable_status','error_status','camera','cancel'])
def test_every_original_guard_remains_active_during_endpoint_retries(failure):
    robot=EndpointRobot(improvement=0.);robot.failure=failure
    def due():return robot.seek_start is not None and robot.now()-robot.seek_start>1.5
    def external():
        if failure=='camera' and due():raise RuntimeError('Camera disconnected')
    with pytest.raises(Cancelled if failure=='cancel' else RuntimeError):
        run(robot,cancel=lambda:failure=='cancel' and due(),external_check=external)
    assert len(robot.retries)==1
    assert robot.retries[-1][0]<robot.seek_start+1.5


def wire_transport():
    robot=CliTransport.__new__(CliTransport)
    robot.lock=threading.RLock();robot.q=np.r_[np.zeros(6),36.];robot.q[4]=1.872
    robot.q_time=0.;robot.acks=robot.commands=0;robot.last_targets={4:3.}
    robot.log=io.StringIO();robot.now=lambda:0.;robot.active_joints=tuple(range(7))
    robot.position_delay_policy='sdk_minimum';lines=[]
    class Writer:
        def write(self,line):lines.append(line);robot.acks+=1
        def flush(self):pass
    robot.proc=SimpleNamespace(stdin=Writer())
    return robot,lines


def test_wire_retry_bypasses_duplicate_only_for_still_outstanding_arm_axes():
    robot,lines=wire_transport();target=np.r_[np.zeros(6),36.];target[4]=3.
    assert robot.send(target,lambda:None)==0
    assert robot.retry_endpoint(target,lambda:None,1.)==1
    assert lines==['j 4 3.000 1\n']
    event=json.loads(robot.log.getvalue())
    assert event['joint']==4 and event['endpoint_retry'] is True
    assert event['endpoint_error_deg']==pytest.approx(1.128)
    assert event['measured_before_sdk_units']==1.872
    assert event['target_sdk_units']==3. and robot.last_targets[4]==3.


def test_feedback_is_rechecked_before_retry_write_and_arrived_axis_is_skipped():
    robot,lines=wire_transport();target=np.r_[np.zeros(6),36.];target[4]=3.
    def arrive():robot.q[4]=2.5
    assert robot.retry_endpoint(target,arrive,1.)==0 and lines==[]


def test_exact_one_degree_error_is_not_a_retry_target():
    robot,lines=wire_transport();target=np.r_[np.zeros(6),36.];target[4]=3.;robot.q[4]=2.
    assert robot.retry_endpoint(target,lambda:None,1.)==0 and lines==[]


def test_gripper_is_never_resent_even_when_its_feedback_differs_from_target():
    robot,lines=wire_transport();target=np.r_[np.zeros(6),36.];target[4]=3.
    robot.q[6]=34.
    assert robot.retry_endpoint(target,lambda:None,1.)==1
    assert lines==['j 4 3.000 1\n']


def test_partial_retry_failure_preserves_attempt_count_without_sending_recovery():
    robot=EndpointRobot(improvement=0.)
    def fail_after_publish(target,check,tolerance):
        check();robot.commands+=1
        raise RuntimeError('Receipt timeout; no next command')
    robot.retry_endpoint=fail_after_publish
    with pytest.raises(RuntimeError,match='Receipt timeout') as caught:
        run(robot)
    diagnostic=caught.value.endpoint_diagnostics
    assert diagnostic['endpoint_retry_commands']==diagnostic['endpoint_retry_rounds']==1
    assert diagnostic['endpoint_retry_events'][0]['command_group_completed'] is False
    assert caught.value.future_commands_stopped and not caught.value.hold_command_received


def test_retry_internal_wait_cannot_publish_a_later_axis_after_seek_budget():
    robot=EndpointRobot(improvement=0.);published=[]
    policy={**ENDPOINT_RETRY_POLICY,'max_seek_s':3.,'stall_timeout_s':3.}
    def slow_retry(target,check,tolerance):
        check();robot.commands+=1;published.append(robot.now())
        robot.wait(2.1)
        check()  # A new axis/command must not pass after the seek budget.
        robot.commands+=1;published.append(robot.now())
        return 2
    robot.retry_endpoint=slow_retry
    with pytest.raises(TimeoutError,match='time budget exceeded') as caught:
        run(robot,policy=policy)
    assert len(published)==1
    assert caught.value.endpoint_diagnostics['endpoint_retry_commands']==1


def test_new_progress_during_receipt_wait_is_observed_before_stall_decision():
    robot=EndpointRobot(improvement=0.)
    def slow_retry(target,check,tolerance):
        check();robot.commands+=1
        robot.wait(4.1);robot.q[4]=2.3
        check()
        return 1
    robot.retry_endpoint=slow_retry
    result=run(robot)
    assert result['completed'] and result['endpoint_retry_commands']==1
    assert result['endpoint_seek_elapsed_s']>5.


def test_new_frame_between_guard_and_write_is_checked_before_any_retry_command():
    robot,lines=wire_transport();robot.q_seq=1
    target=np.r_[np.zeros(6),36.];target[4]=3.
    actual_lock=robot.lock
    class InjectFrameLock:
        injected=False
        def __enter__(self):
            actual_lock.acquire()
            if not self.injected:
                self.injected=True;robot.q_seq+=1;robot.q[4]=6.1
        def __exit__(self,*exc):actual_lock.release()
    robot.lock=InjectFrameLock()
    def check():
        if max(abs(robot.q[:6]-target[:6]))>3:raise RuntimeError('Tracking error >3 degrees')
    with pytest.raises(RuntimeError,match='Tracking error'):
        robot.retry_endpoint(target,check,1.)
    assert lines==[] and robot.commands==0


def test_latest_saved_endpoint_residual_is_reproduced_without_changing_log_bytes():
    source=Path(__file__).resolve().parents[1]/'logs/20261026_183952_a0d7c6/execution/01_fixed_pregrasp/executed'
    before={name:(source/name).read_bytes() for name in ('plan.json','result.json')}
    recorded=json.loads(before['result.json']);plan=json.loads(before['plan.json'])['sdk_plan']
    errors=abs(np.asarray(recorded['final_angles'])[:6]-np.asarray(plan['sdk'][-1]))
    assert np.argmax(errors)==4 and errors[4]==pytest.approx(1.1278844717)
    assert recorded['commands']==recorded['receipts']==30
    assert recorded['feedback_rejected_count']==0
    assert {name:(source/name).read_bytes() for name in before}==before
