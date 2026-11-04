"""Replay actual boundary jitter without connecting to any hardware."""
import json
from pathlib import Path
import sys

import numpy as np
import pytest

APP=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(APP/'runtime/Control/ros2_ws/src/d1_pick_place'))
sys.path.insert(0,str(APP/'runtime/Control/tools'))
from d1_pick_place.sdk_executor import (LIMITS, Mapping, NAMES, feedback_limit_errors,
    nominal_planning_start, healthy, prepare)
from prepare_accel_grasp import robot_readiness_errors


def snapshot(q):
    return dict(q=np.array(q),q_time=10.,fault=None,
        status=dict(power_status=1,enable_status=0,error_status=0),
        status_time={k:10. for k in ('power_status','enable_status','error_status')})


@pytest.mark.parametrize('joint',range(6))
@pytest.mark.parametrize('side',(0,1))
def test_feedback_tolerance_is_separate_from_command_limits(joint,side):
    q=np.zeros(7);q[6]=6.5
    sign=-1 if side==0 else 1
    q[joint]=np.float32(LIMITS[joint,side]+sign*.2)
    raw=q.copy()
    assert feedback_limit_errors(q)==[]
    assert robot_readiness_errors(snapshot(q),10.)==[]
    assert np.array_equal(healthy(snapshot(q),10.,allow_unstarted_position=True),raw)
    reference=nominal_planning_start(q)
    assert reference[joint]==LIMITS[joint,side]
    assert np.array_equal(q,raw)
    q[joint]=LIMITS[joint,side]+sign*.201
    assert feedback_limit_errors(q)
    with pytest.raises(ValueError):nominal_planning_start(q)
    # Even a within-tolerance *target* remains prohibited.
    positions=np.zeros((2,6));positions[1,joint]=np.deg2rad(LIMITS[joint,side]+sign*.1)
    with pytest.raises(ValueError,match='joint limit'):
        prepare(NAMES,positions,[0.,1.],Mapping(np.ones(6),np.zeros(6)))


def test_actual_failure_readbacks_now_pass_without_rewriting_history():
    path=APP/'logs/20261026_000512_8af2c3/execution/00_prepare_gripper/executed/transport.jsonl'
    original=path.read_bytes();count=0
    for line in original.decode().splitlines():
        row=json.loads(line)
        if '[rx] ' not in row.get('line',''):continue
        payload=json.loads(row['line'].split('[rx] ',1)[1])
        if payload.get('funcode')!=1:continue
        q=[payload['data'][f'angle{i}'] for i in range(7)]
        assert feedback_limit_errors(q)==[]
        assert np.array_equal(healthy(snapshot(q),10.,allow_unstarted_position=True),q)
        count+=1
    assert count==4
    assert path.read_bytes()==original


@pytest.mark.parametrize('value',[-.001,65.001,float('nan')])
def test_j6_does_not_receive_angular_feedback_tolerance(value):
    assert feedback_limit_errors([0.,0.,0.,0.,0.,0.,value])


def test_readback_tolerance_keeps_status_and_freshness_guards():
    s=snapshot([0.,-90.1,0.,0.,0.,0.,6.5])
    with pytest.raises(RuntimeError,match='enable_status'):healthy(s,10.)
    with pytest.raises(RuntimeError,match='stale'):healthy(s,11.,allow_unstarted_position=True)
    s['status']['power_status']=0
    with pytest.raises(RuntimeError,match='power_status'):healthy(s,10.,allow_unstarted_position=True)
