"""Feedback-adaptive references stay on every approved segment and source time."""
import numpy as np
import pytest

from d1_pick_place.sdk_executor import Mapping,NAMES,execute,prepare,paced_references
from test_fixed_feedback_pacing import DelayedRobot


class ResidualRobot(DelayedRobot):
    def __init__(self,receipt_wait=0.):
        super().__init__(startup_s=0.,speed=40.)
        self.receipt_wait=receipt_wait

    def wait(self,seconds):
        end=self.t+max(seconds,.001)
        while self.t<end-1e-10:
            dt=min(.01,end-self.t);self.t+=dt
            delta=self.target[:6]-self.physical[:6]
            self.physical[:6]+=np.sign(delta)*np.minimum(np.maximum(abs(delta)-2.1,0.),self.speed*dt)
            if self.t-self.sample_time>=.1-1e-8:
                self.q=np.round(self.physical,1);self.sample_time=self.t;self.seq+=1

    def send(self,target,check):
        super().send(target,check)
        if self.receipt_wait:self.wait(self.receipt_wait)


def approved(points):
    q=np.zeros((len(points),6))
    for i,row in enumerate(points):q[i,:len(row)]=row
    return prepare(NAMES,np.deg2rad(q),np.arange(len(q),dtype=float),
        Mapping(np.ones(6),np.zeros(6)),rate=10.,speed=20.,acceleration=40.,respect_source_timing=False)


def run_to_terminal_failure(plan,robot):
    with pytest.raises(TimeoutError,match='Endpoint position not reached') as caught:
        execute(plan,robot,progress_policy='feedback_paced')
    return caught.value


@pytest.mark.parametrize('receipt_wait',[0.,.12])
def test_large_intermediate_residual_advances_to_original_end_without_catchup(receipt_wait):
    plan=approved([[0.],[20.]])
    robot=ResidualRobot(receipt_wait)
    run_to_terminal_failure(plan,robot)
    assert robot.target[0]==pytest.approx(20.)
    assert robot.q[0]==pytest.approx(17.9)  # Still fails the final 1-degree test.
    assert max(robot.errors)<=2.5+1e-5
    refs=list(paced_references(plan));index=0
    # Every original waypoint is retained, including the one that was split.
    for _,q in refs[1:]:
        while not np.allclose(robot.sent[index][1][:6],q,rtol=0,atol=1e-8):index+=1
        index+=1
    assert len(robot.sent)>len(refs)-1
    source_q=np.asarray([q[0] for _,q in refs]);source_t=np.asarray([t for t,_ in refs])
    sent_source_times=[]
    for _,q in robot.sent:
        j=min(max(1,int(np.searchsorted(source_q,q[0],side='left'))),len(refs)-1)
        u=(q[0]-source_q[j-1])/(source_q[j]-source_q[j-1])
        sent_source_times.append(source_t[j-1]+u*(source_t[j]-source_t[j-1]))
    assert np.all(np.diff(sent_source_times)>0)
    assert np.all(np.diff([t for t,_ in robot.sent])+1e-8>=np.diff(sent_source_times))
    assert all(q[6]==36. for _,q in robot.sent)


def test_adaptive_steps_keep_multiaxis_corners_and_reverse_segment_order():
    points=[[0.,0.],[8.,0.],[8.,-8.],[0.,-8.]]
    plan=approved(points);robot=ResidualRobot()
    run_to_terminal_failure(plan,robot)
    assert np.allclose(robot.target[:2],points[-1])
    assert max(robot.errors)<=2.5+1e-5
    seen_corner=0
    for _,q in robot.sent:
        a=np.asarray(points[seen_corner]);b=np.asarray(points[seen_corner+1]);delta=b-a
        moving=int(np.argmax(abs(delta)));u=(q[moving]-a[moving])/delta[moving]
        assert -1e-8<=u<=1+1e-8
        assert np.allclose(q[:2],a+u*delta,rtol=0,atol=1e-8)
        if np.allclose(q[:2],b,rtol=0,atol=1e-8) and seen_corner<2:seen_corner+=1
    assert seen_corner==2


def test_static_robot_still_stops_without_replaying_or_microstepping_forever():
    robot=DelayedRobot(startup_s=100.)
    with pytest.raises(TimeoutError,match='no measured joint progress') as caught:
        execute(approved([[0.],[20.]]),robot,progress_policy='feedback_paced')
    assert robot.now()<5. and max(robot.errors)<=2.5+1e-5
    assert robot.target[0]<2.5
    steps=caught.value.execution_diagnostics['adaptive_reference_steps']
    assert steps and all(step['max_error_deg']<=2.5 for step in steps)
    assert len(robot.sent)<12
