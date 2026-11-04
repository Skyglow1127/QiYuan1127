"""Shared-phase behavior with a virtual clock and robot; no hardware/network."""
from pathlib import Path
import sys
import numpy as np
import pytest

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'runtime/Control/ros2_ws/src/d1_pick_place')]
from d1_pick_place.sdk_executor import execute,prepare,Mapping,NAMES,LIMITS
from d1_pick_place.shared_progress import SourcePath,PhaseGovernor,MAX_PHASE_RATE


def plan(points=None):
    points=np.asarray(points if points is not None else [[0.]*6,[8.,4.,0.,0.,0.,0.]])
    return prepare(NAMES,np.deg2rad(points),np.arange(len(points),dtype=float),
        Mapping(np.ones(6),np.zeros(6)),rate=10.,speed=5.,acceleration=10.,respect_source_timing=False)


class Robot:
    def __init__(self,speeds=None,ack=.005,residual=0.,bad_feedback=None):
        self.t=0.;self.q=np.r_[np.zeros(6),36.];self.physical=self.q.copy();self.target=self.q.copy()
        self.speeds=np.asarray(speeds if speeds is not None else [100.]*6)
        self.ack_delay=ack;self.residual=residual;self.bad_feedback=bad_feedback
        self.qt=np.zeros(7);self.seq=np.ones(7,dtype=int);self.q_seq=1
        self.last_sample=0.;self.pending=[False]*7;self.pause_total=0.
        self.commands=self.acks=0;self.published_targets={}
        self.groups=[];self.writes=[];self.group_returns=[]

    def now(self):return self.t

    def snapshot(self):
        return dict(q=self.q.copy(),q_time=float(min(self.qt)),q_seq=self.q_seq,
            joint_q_time=self.qt.tolist(),joint_q_seq=self.seq.tolist(),
            joint_feedback_pending=self.pending.copy(),feedback_pending=any(self.pending),
            feedback_pause_total_s=self.pause_total,
            status=dict(power_status=1,enable_status=1,error_status=0),
            status_time={k:self.t for k in ('power_status','enable_status','error_status')},fault=None)

    def wait(self,duration):
        end=self.t+(duration if duration>0 else .0001)
        while self.t<end-1e-12:
            dt=min(.005,end-self.t);self.t+=dt
            delta=self.target[:6]-self.physical[:6]
            self.physical[:6]+=np.sign(delta)*np.minimum(np.maximum(abs(delta)-self.residual,0.),self.speeds*dt)
            if self.t-self.last_sample>=.05-1e-9:
                self.last_sample=self.t
                for joint in range(7):
                    bad=(joint==1 and self.bad_feedback is not None and self.bad_feedback[0]<=self.t<self.bad_feedback[1])
                    self.pending[joint]=bad
                    if not bad:
                        self.q[joint]=round(self.physical[joint],3);self.qt[joint]=self.t;self.seq[joint]+=1
                self.q_seq+=1
        self.t=end

    def send(self,target,check):
        assert not hasattr(check,'admit_joint')  # No normal per-axis blocking gate.
        self.groups.append((self.t,target.copy(),dict(self.reference_context)))
        for joint in self.active_joints:
            q=check()
            guard=getattr(check,'admit_group',None)
            if guard:guard(target,q,joint)
            self.target[joint]=target[joint];self.published_targets[joint]=target[joint]
            self.writes.append((self.t,joint,float(target[joint]),self.q.copy(),dict(self.reference_context)))
            self.commands+=1;until=self.t+self.ack_delay
            while self.t<until-1e-12:
                self.wait(min(.005,until-self.t));check()
            self.acks+=1
        self.group_returns.append(self.t)
        return len(self.active_joints)


def run(robot,approved=None,**kwargs):
    return execute(approved or plan(),robot,progress_policy='feedback_paced',
                   scheduling_policy='shared_progress_v1',**kwargs)


def test_slow_axis_changes_one_phase_and_preserves_multiaxis_path():
    robot=Robot(speeds=[100.,1.,100.,100.,100.,100.]);approved=plan()
    result=run(robot,approved)
    assert result['completed']
    events=result['phase_events'];rates=[e['phase_rate'] for e in events]
    assert max(rates)<=MAX_PHASE_RATE+1e-9
    assert any(b<a-.001 for a,b in zip(rates,rates[1:]))
    assert max(e['tracking_lag_deg'] for e in events)>.5
    assert all(abs(q[0]-2*q[1])<1e-8 for _,q,_ in robot.groups)
    assert all(max(abs(q[:6]-actual[:6]))<=2.5+1e-5 for _,_,_,actual,context in robot.writes
               for _,q,c in robot.groups if c['reference_id']==context['reference_id'])
    assert events[-1]['source_elapsed_s']==pytest.approx(approved['duration_s'])
    assert result['reference_epoch_monotonic_s']==0.
    assert len({c['original_reference_epoch_monotonic_s'] for _,_,c in robot.groups})==1
    assert robot.shared_progress_state['source_elapsed_s']==pytest.approx(approved['duration_s'])


def test_ack_overrun_is_measured_and_does_not_create_catchup_bursts():
    robot=Robot(ack=.08);result=run(robot,plan([[0.]*6,[2.,1.,0.,0.,0.,0.]]))
    starts=[t for t,_,_ in robot.groups]
    assert min(np.diff(starts))>=.16-1e-8
    assert result['shared_progress_metrics']['schedule_overrun_ticks']>0
    assert result['scheduling_overrun_s']>.05
    phase=np.array([e['source_elapsed_s'] for e in result['phase_events']])
    assert max(np.diff(phase))<=.1*MAX_PHASE_RATE+1e-8
    assert all(e['integration_dt_s']<=.1+1e-9 for e in result['phase_events'])


def test_chain_rule_budget_includes_phase_acceleration_and_visits_corners():
    approved=plan([[0.]*6,[2.,1.,0.,0.,0.,0.],[2.,-1.,0.,0.,0.,0.]])
    path=SourcePath(approved,LIMITS);governor=PhaseGovernor(path)
    phases=[]
    for _ in range(2000):
        q=path.state(governor.phase)[0]
        step=governor.propose(q,.1)
        assert max(step['reference_velocity_peak_deg_s'])<=path.speed_limit+1e-7
        assert max(step['reference_acceleration_peak_deg_s2'])<=path.acceleration_limit+1e-7
        assert step['phase_acceleration_peak']<=governor.acceleration+1e-8
        governor.accept(step);phases.append(governor.phase)
        if governor.phase==path.duration:break
    assert path.duration==pytest.approx(phases[-1])
    assert all(any(abs(boundary-phase)<1e-8 for phase in phases) for boundary in path.ends)
    assert max(path.acceleration_bound*MAX_PHASE_RATE**2+
               path.velocity_bound*governor.acceleration)<=path.acceleration_limit+1e-8


def test_edge_quintic_sampler_derivatives_match_finite_difference():
    approved=dict(sdk=[[0.]*6,[1.,2.,0.,0.,0.,0.]],ticks=[20],rate=10.,
                  active_joints=[0,1],duration_s=2.,timing_profile='edge_quintic_v1')
    path=SourcePath(approved,LIMITS);h=1e-5
    q,v,a=path.state(.7);left=path.state(.7-h);right=path.state(.7+h)
    assert (right[0]-left[0])/(2*h)==pytest.approx(v,rel=1e-7,abs=1e-8)
    assert (right[1]-left[1])/(2*h)==pytest.approx(a,rel=1e-7,abs=1e-8)


@pytest.mark.parametrize('field',['start','end'])
def test_run_geometry_cannot_replace_the_reviewed_sdk_path(field):
    approved=plan();approved['runs'][0][field][0]+=.1
    with pytest.raises(ValueError,match='geometry differs'):SourcePath(approved,LIMITS)


@pytest.mark.parametrize('damage',['distance','start_time','indices','corner'])
def test_damaged_shared_run_metadata_rejected_before_any_command(damage):
    approved=plan([[0.]*6,[2.,1.,0.,0.,0.,0.],[4.,2.,0.,0.,0.,0.]])
    if damage=='distance':approved['runs'][0]['distance']+=1.
    elif damage=='start_time':approved['runs'][0]['start_time_s']=1.
    elif damage=='indices':approved['runs'][0]['start_index']=-1
    else:approved['sdk'][1][1]+=.2  # Same endpoints but a real unapproved bend.
    robot=Robot()
    with pytest.raises(ValueError,match='Shared run'):run(robot,approved)
    assert robot.commands==0


def test_real_tracking_fault_stops_future_commands():
    robot=Robot()
    def fault():
        if robot.now()>.8:robot.q[0]=-5.
    with pytest.raises(RuntimeError,match='Tracking error >3') as caught:run(robot,external_check=fault)
    assert caught.value.future_commands_stopped and not caught.value.physical_stop_verified
    assert robot.t<.9


def test_stable_servo_residual_is_not_integrated_or_accepted_as_endpoint():
    robot=Robot(residual=1.4)
    with pytest.raises(TimeoutError,match='Endpoint position not reached') as caught:
        run(robot,plan([[0.]*6,[3.,1.5,0.,0.,0.,0.]]))
    assert max(abs(robot.target[:6]-[3.,1.5,0.,0.,0.,0.]))<1e-8
    assert abs(robot.q[0]-3.)>1.
    assert caught.value.shared_progress_metrics['physical_derivatives_guaranteed'] is False


def test_frozen_servo_fails_boundedly_without_advancing_outside_admission_margin():
    robot=Robot(speeds=[100.,0.,100.,100.,100.,100.])
    with pytest.raises((TimeoutError,RuntimeError),match='stalled|margin lost') as caught:run(robot)
    assert robot.t<15.
    assert robot.target[1]<=2.5+1e-5
    assert caught.value.future_commands_stopped


def test_stable_two_point_five_degree_residual_cannot_create_unbounded_microsteps():
    robot=Robot(residual=2.5)
    with pytest.raises((TimeoutError,RuntimeError),match='stalled|margin lost') as caught:run(robot)
    assert robot.t<15. and robot.target[0]<=2.5+1e-5
    assert caught.value.shared_progress_metrics['source_elapsed_s']<plan()['duration_s']


@pytest.mark.parametrize('bad_feedback,success',[((.8,1.15),True),((.8,100.),False)])
def test_feedback_recovery_is_bounded_for_each_required_axis(bad_feedback,success):
    robot=Robot(bad_feedback=bad_feedback)
    if success:
        result=run(robot)
        assert result['completed']
        assert result['shared_progress_metrics']['phase_pause_resets']>=1
        assert result['feedback_timeout_policy']=='required_joint_deadline'
    else:
        with pytest.raises(RuntimeError,match='Required joint feedback recovery timeout: J1') as caught:run(robot)
        assert robot.t<3.
        assert caught.value.future_commands_stopped


def test_shared_source_limit_and_wrong_progress_policy_rejected_without_commands():
    robot=Robot();bad=plan();bad['sdk'][-1][0]=136.
    with pytest.raises(ValueError,match='joint limits'):run(robot,bad)
    with pytest.raises(ValueError,match='fixed-pose'):
        execute(plan(),robot,scheduling_policy='shared_progress_v1')
    assert robot.commands==0


def test_mid_group_margin_change_aborts_later_axis_without_waiting_or_rephasing():
    robot=Robot();original_send=robot.send
    def disturbed_send(target,check):
        real_guard=check.admit_group;calls=[]
        def guard(candidate,actual,joint=None):
            calls.append(joint)
            if len(calls)==2:actual[1]=candidate[1]-2.6
            real_guard(candidate,actual,joint)
        check.admit_group=guard
        return original_send(target,check)
    robot.send=disturbed_send
    with pytest.raises(RuntimeError,match='Shared group margin lost') as caught:run(robot)
    assert [joint for _,joint,_,_,_ in robot.writes]==[0]
    assert caught.value.phase_events[-1]['completed'] is False
    assert caught.value.phase_events[-1]['commands_attempted']==1
    assert caught.value.phase_events[-1]['commands_acknowledged']==1
    assert robot.t<.2


def test_real_cli_transport_correlates_each_submission_and_ack_to_immutable_phase():
    import io,json,threading
    from types import SimpleNamespace
    from d1_pick_place.sdk_executor import CliTransport
    robot=CliTransport.__new__(CliTransport)
    robot.lock=threading.RLock();robot.log=io.StringIO();robot.q=np.r_[np.zeros(6),36.]
    robot.q_time=0.;robot.q_seq=1;robot.commands=robot.acks=0
    robot.last_targets={};robot.active_joints=(0,1);robot.now=lambda:0.
    robot.reference_context=dict(reference_id=7,source_elapsed_s=.125,
        original_reference_epoch_monotonic_s=123.,original_plan_start_sdk=[0.]*6)
    writes=[]
    class Writer:
        def write(self,line):writes.append(line)
        def flush(self):robot.acks+=1
    robot.proc=SimpleNamespace(stdin=Writer())
    check=lambda:robot.q.copy()
    admissions=[]
    check.admit_group=lambda candidate,actual,joint:admissions.append(joint)
    assert robot.send(np.array([1.,.5,0.,0.,0.,0.,36.]),check)==2
    assert admissions==[0,1]
    events=[json.loads(line) for line in robot.log.getvalue().splitlines()]
    assert [e['kind'] for e in events]==['command','command_ack','command','command_ack']
    assert all(e['reference_id']==7 and e['source_elapsed_s']==.125 and
               e['original_reference_epoch_monotonic_s']==123. for e in events)
    assert [e['command_ordinal'] for e in events]==[1,1,2,2]


@pytest.mark.parametrize('pause_at_group',[1,4])
def test_ack_internal_feedback_freeze_is_not_counted_as_stall_time(pause_at_group):
    robot=Robot();send=robot.send
    def pausing_send(target,check):
        if len(robot.groups)+1==pause_at_group:
            robot.t+=2.1;robot.pause_total+=2.1
            robot.qt[:]=robot.t  # Represents multiple recovered short gaps.
        return send(target,check)
    robot.send=pausing_send
    result=run(robot,plan([[0.]*6,[1.,.5,0.,0.,0.,0.]]))
    assert result['completed']
    assert result['shared_progress_metrics']['phase_pause_resets']>=1
