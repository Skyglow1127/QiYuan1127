import sys
from pathlib import Path
import numpy as np
import pytest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from d1_pick_place.sdk_executor import Mapping,NAMES,prepare,references,execute,Cancelled


class FakeTransport:
    def __init__(self,mode='follow'):
        self.t=0.;self.q=np.array([0.,0.,0.,0.,0.,0.,30.]);self.seq=1;self.sent=[];self.mode=mode
    def now(self):return self.t
    def snapshot(self):
        return dict(q=self.q.copy(),q_time=self.t if self.mode!='stale' else self.t-1,
                    q_seq=self.seq,status=dict(power_status=1,enable_status=1,error_status=0),
                    status_time={k:self.t for k in ['power_status','enable_status','error_status']},fault=None)
    def wait(self,seconds):self.t+=max(seconds,.001);self.seq+=1
    def send(self,q,check):
        check();self.sent.append(np.array(q))
        if self.mode!='stalled':self.q=np.array(q)


def plan(target=2.):
    mapping=Mapping(np.ones(6),np.zeros(6))
    return prepare(NAMES,[[0]*6,[np.deg2rad(target),0,0,0,0,0]],[0,.2],mapping)


def test_mapping_bidirectional_and_joint_order():
    m=Mapping(np.array([-1,1,1,-1,1,1]),np.deg2rad([1,2,3,4,5,6]))
    raw=np.array([10,-20,30,40,50,-60]);assert np.allclose(m.to_sdk(m.to_model(raw)),raw)
    names=list(reversed(NAMES));a=m.to_model(raw);b=m.to_model(raw+[1,0,0,0,0,0])
    p=prepare(names,[list(reversed(a)),list(reversed(b))],[0,1],m)
    assert np.allclose(p['sdk'][0],raw)


def test_reference_speed_and_path_preserved():
    p=plan();points=list(references(p));q=np.array([v for _,v in points]);ts=np.array([t for t,_ in points])
    assert np.max(abs(np.diff(q,axis=0)/np.diff(ts)[:,None]))<=.5+1e-8
    assert q[0,0]==0 and q[-1,0]==2 and np.all(np.diff(q[:,0])>=0)
    assert np.max(abs(q[:,1:]))==0


@pytest.mark.parametrize('target',[float('nan'),136.,-136.])
def test_invalid_path_never_reaches_transport(target):
    with pytest.raises(ValueError):plan(target)


def test_arrival_uses_feedback_and_preserves_gripper_target():
    t=FakeTransport();result=execute(plan(),t)
    assert result['completed'] and result['actual_sdk'][0]==2
    assert all(x[6]==30 for x in t.sent)


def test_receipt_without_motion_is_not_success():
    t=FakeTransport('stalled')
    with pytest.raises(TimeoutError,match='Endpoint'):execute(plan(),t)


def test_stale_start_sends_nothing():
    t=FakeTransport('stale')
    with pytest.raises(RuntimeError,match='stale'):execute(plan(),t)
    assert not t.sent


def test_cancellation_stops_future_commands_without_claiming_a_physical_hold():
    t=FakeTransport()
    with pytest.raises(Cancelled) as caught:execute(plan(),t,cancel=lambda:t.now()>.7)
    assert caught.value.future_commands_stopped
    assert caught.value.hold_command_received is False
    assert caught.value.physical_stop_verified is False
    assert t.sent[-1][0]<2 and not np.array_equal(t.sent[-1],t.sent[-2])


def test_non_target_drift_stops_next_commands():
    t=FakeTransport()
    def disturb():
        if t.now()>.5:t.q[3]=.6
    with pytest.raises(RuntimeError,match='Uncommanded'):execute(plan(),t,external_check=disturb)
    assert t.sent[-1][0]<2


class GrippedTransport(FakeTransport):
    def __init__(self):
        super().__init__()
        self.q[6]=30.6
    def send(self,q,check):
        check();self.sent.append(np.array(q))
        self.q[:6]=np.asarray(q)[:6]  # Gripped J6 cannot follow the smaller command.


def test_explicit_grip_command_survives_arm_motion():
    t=GrippedTransport()
    result=execute(plan(),t,gripper_target=29.6)
    assert result['completed'] and result['actual_sdk'][6]==30.6
    assert result['gripper_target_sdk']==29.6 and result['gripper_target_source']=='explicit'
    assert all(q[6]==29.6 for q in t.sent)


@pytest.mark.parametrize('target',[-1.,66.,float('nan'),float('inf'),26.])
def test_invalid_explicit_grip_rejected_before_send(target):
    t=GrippedTransport()
    with pytest.raises(ValueError,match='Gripper target'):
        execute(plan(),t,gripper_target=target)
    assert not t.sent


def test_explicit_grip_does_not_relax_actual_drift_guard():
    t=GrippedTransport()
    def disturb():
        if t.now()>.5:t.q[6]=31.2
    with pytest.raises(RuntimeError,match='Uncommanded joint drift'):
        execute(plan(),t,external_check=disturb,gripper_target=29.6)
    assert t.sent[-1][0]<2
