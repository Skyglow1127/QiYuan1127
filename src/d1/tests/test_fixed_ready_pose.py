"""Fixed Gazebo ready posture must never be replaced by a cube-relative goal."""
import ast
import hashlib
import json
import math
from pathlib import Path
from unittest.mock import Mock

import pytest

from app import backend as bm
from app.backend import Backend, DEFAULTS
from app.execution_scope import FIXED_PREGRASP, select_execution_stages
from app.gui import GraspApp, pregrasp_summary
from app.fixed_pose import FIXED_MOTION_DEFAULTS, fixed_motion_checked

APP=Path(__file__).resolve().parents[1]
PROFILE=json.loads((APP/'app/fixed_ready_pose.json').read_text())


def test_ready_source_is_exact_gazebo_console_constant_in_radians():
    source=Path(PROFILE['source']['path'])
    assert hashlib.sha256(source.read_bytes()).hexdigest()==PROFILE['source']['sha256']
    tree=ast.parse(source.read_text())
    branch=next(node for node in ast.walk(tree) if isinstance(node,ast.If)
        and ast.unparse(node.test)=="action == 'ready'")
    targets=next(node for node in ast.walk(branch.body[0]) if isinstance(node,ast.List)
        and len(node.elts)==6)
    assert ast.literal_eval(targets)==PROFILE['model_target_rad']==[0.,-.55,.8,0.,-.25,0.]
    assert [round(math.degrees(v),2) for v in PROFILE['model_target_rad']]==[0.,-31.51,45.84,0.,-14.32,0.]


def test_fixed_scope_rejects_cube_relative_path_even_if_it_has_pregrasp():
    with pytest.raises(ValueError,match='独立的固定姿态计划'):
        select_execution_stages([dict(name='prepare_gripper'),dict(name='pregrasp')],FIXED_PREGRASP)


def test_demo_fixed_ready_is_independent_of_full_grasp_and_stops_at_constant(tmp_path,monkeypatch):
    monkeypatch.setattr(bm,'APP',tmp_path)
    backend=Backend(mode='demo');backend._run=Mock(side_effect=AssertionError('No device subprocess in demo'))
    prepared=backend.prepare(DEFAULTS,fixed_pregrasp=True)
    assert prepared['ready'] and prepared['operation']==FIXED_PREGRASP
    assert [stage['name'] for stage in prepared['stages']]==['prepare_gripper',FIXED_PREGRASP]
    endpoint=prepared['stages'][-1]['sdk_plan']['sdk'][-1]
    assert endpoint==[math.degrees(v) for v in PROFILE['model_target_rad']]
    assert '固定预抓取终点' in pregrasp_summary(prepared)
    with pytest.raises(ValueError,match='不能混用'):
        backend.execute(prepared,'A fixed pose approval cannot start a grasp')
    result=backend.execute(prepared,'Approve displayed fixed ready route; demo only',execution_scope=FIXED_PREGRASP)
    assert result['status']=='pregrasp_completed' and result['pregrasp_reached']
    assert result['grasp_result']==result['cycle_result']=='not_attempted'
    assert list(result['stage_times'])==['prepare_gripper',FIXED_PREGRASP]
    assert result['command_count']==result['actual_feedback_samples']==0
    assert result['final_angles'][:6]==endpoint
    backend._run.assert_not_called()


def test_live_fixed_prepare_uses_feedback_and_static_table_without_recognition(tmp_path,monkeypatch):
    backend=Backend()
    monkeypatch.setattr(bm,'check_environment',lambda:[])
    q=[0.,-20.,40.,0.,20.,0.,36.]
    commands=[]
    def run(argv,*args):
        commands.append(argv)
        output=Path(argv[argv.index('--output')+1]);output.mkdir(parents=True)
        assert '--execute' not in argv
        if Path(argv[0]).name=='observe_grasp.py':
            (output/'wrist_observation.json').write_text(json.dumps(dict(sdk_angles=q,status={})))
        else:
            assert Path(argv[0]).name=='run_grasp_cycle.py' and '--fixed-ready' in argv
            (output/'manifest.json').write_text(json.dumps(dict(target={'description':'fixed ready'},
                spec=DEFAULTS,expires_at=1.,stages=[dict(name=FIXED_PREGRASP,sdk_plan={'duration_s':1.})])))
    monkeypatch.setattr(backend,'_run',run)
    prepared=backend._prepare_fixed_pregrasp(DEFAULTS,tmp_path)
    assert prepared['operation']==FIXED_PREGRASP
    assert [Path(argv[0]).name for argv in commands]==['observe_grasp.py','run_grasp_cycle.py']
    assert not any('prepare_accel_grasp.py' in str(argv) or '--recognition-report' in argv for argv in commands)


def test_full_review_click_prepares_fixed_ready_without_motion():
    gui=GraspApp.__new__(GraspApp)
    gui.busy='';gui.closing=False;gui.review=None;gui.review_used=False;gui.parameters_changed=False
    gui.prepare=Mock();gui._execute_review=Mock()
    gui.enter_pregrasp()
    gui.prepare.assert_called_once_with(fixed_pregrasp=True)
    gui._execute_review.assert_not_called()


def test_fixed_speed_is_independent_of_grasp_settings_and_has_no_speed_ceiling(tmp_path,monkeypatch):
    monkeypatch.setattr(bm,'APP',tmp_path)
    backend=Backend(mode='demo')
    review=backend.prepare({**DEFAULTS,'speed_deg_s':.01,'acceleration_deg_s2':.02,'gripper_speed_sdk_s':.03},fixed_pregrasp=True)
    for key,value in FIXED_MOTION_DEFAULTS.items():
        assert review['parameters'][key]==value
    custom=backend.prepare(DEFAULTS,fixed_pregrasp=True,fixed_motion={'speed_deg_s':50.,'acceleration_deg_s2':100.})
    assert custom['parameters']['speed_deg_s']==50.
    full=backend.prepare(DEFAULTS)
    assert full['parameters']['speed_deg_s']==DEFAULTS['speed_deg_s']
    assert fixed_motion_checked({'speed_deg_s':10000.})['speed_deg_s']==10000.


@pytest.mark.parametrize('key,value',[('speed_deg_s',0),('acceleration_deg_s2',float('nan')),('gripper_speed_sdk_s',-1),('rate_hz',11)])
def test_fixed_speed_invalid_values_are_rejected(key,value):
    with pytest.raises(ValueError):fixed_motion_checked({key:value})


@pytest.mark.parametrize('policy,expected_delay',[(None,166),('sdk_minimum',1)])
def test_minimum_sdk_delay_changes_only_fixed_policy_not_target(policy,expected_delay):
    import io
    import threading
    from types import SimpleNamespace
    import numpy as np
    from d1_pick_place.sdk_executor import CliTransport
    transport=CliTransport.__new__(CliTransport)
    transport.lock=threading.RLock();transport.q=np.zeros(7);transport.q_time=0.;transport.acks=0
    transport.commands=0;transport.last_targets={};transport.log=io.StringIO()
    transport.now=lambda:0.;transport.active_joints=[1]
    lines=[]
    class Writer:
        def write(self,line):lines.append(line);transport.acks+=1
        def flush(self):pass
    transport.proc=SimpleNamespace(stdin=Writer())
    if policy:transport.position_delay_policy=policy
    target=np.zeros(7);target[1]=2.5
    assert transport.send(target,lambda:None)==1
    assert lines==[f'j 1 2.500 {expected_delay}\n']
    assert json.loads(transport.log.getvalue())['requested_delay_ms']==expected_delay
