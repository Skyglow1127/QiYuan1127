"""One-click pregrasp uses reviewed poses; all tests are offline or synthetic."""
import copy
import json
import time
from pathlib import Path
from unittest.mock import Mock

import pytest

from app import backend as bm
from app.backend import Backend, DEFAULTS
from app.execution_scope import select_execution_stages
from app.gui import GraspApp, pregrasp_summary


@pytest.mark.parametrize('names', [[], ['approach'], ['close', 'pregrasp'], ['approach', 'pregrasp']])
def test_prefix_selector_rejects_missing_or_prior_grasp_motion(names):
    with pytest.raises(ValueError):
        select_execution_stages([{'name': name} for name in names], 'pregrasp')


def test_unknown_scope_cannot_silently_fall_back_to_full_cycle():
    with pytest.raises(ValueError, match='未知执行范围'):
        select_execution_stages([], 'typo')


def review():
    return dict(ready=True, approval_sha256='a'*64, expires_at=time.time()+600,
        parameters=DEFAULTS.copy(), stages=[
            dict(name='prepare_gripper', expected_angles=[0]*6+[6.5], gripper_target=36),
            dict(name='pregrasp', expected_angles=[0]*6+[36], gripper_target=36,
                 sdk_plan=dict(sdk=[[0]*6, [1, -2, 3, -4, 5, -6]], duration_s=5)),
            dict(name='approach')])


def stub():
    app=GraspApp.__new__(GraspApp)
    app.review=review();app.review_used=False;app.parameters_changed=False
    app.review['operation']='fixed_pregrasp';app.review['stages'][1]['name']='fixed_pregrasp'
    app.review['stages']=app.review['stages'][:2]
    app.busy='';app.closing=False
    for name in ['state_label','notebook','_append_log','_start_job','_refresh_controls','backend']:
        setattr(app,name,Mock())
    return app


def test_gui_shows_exact_pregrasp_endpoint_and_gripper_target():
    text=pregrasp_summary(review())
    assert '[1.00, -2.00, 3.00, -4.00, 5.00, -6.00]' in text
    assert '36 SDK' in text and '到位后停止' in text
    assert pregrasp_summary(None)==''
    assert pregrasp_summary({'stages':[{'name':'pregrasp'}]})==''


def test_gui_click_records_prefix_approval_without_preparing_new_target():
    app=stub();original=copy.deepcopy(app.review)
    app.enter_pregrasp()
    assert app.review==original and app.review_used
    kind,job=app._start_job.call_args.args
    assert kind=='execute'
    app.backend.execute.assert_not_called()
    job()
    args,kwargs=app.backend.execute.call_args
    assert args[0]==original and args[0] is not app.review
    assert kwargs['execution_scope']=='fixed_pregrasp'
    assert '仅执行至固定预抓取姿态并停止' in kwargs['approval_text']
    app.backend.prepare.assert_not_called()


@pytest.mark.parametrize('flag,value', [('busy','execute'),('closing',True)])
def test_gui_pregrasp_obeys_existing_review_and_busy_guards(flag,value):
    app=stub();setattr(app,flag,value)
    app.enter_pregrasp()
    app._start_job.assert_not_called()


@pytest.mark.parametrize('flag', ['review_used','parameters_changed'])
def test_used_or_changed_fixed_pose_starts_preparation_without_motion(flag):
    app=stub();setattr(app,flag,True);app.prepare=Mock()
    app.enter_pregrasp()
    app.prepare.assert_called_once_with(fixed_pregrasp=True)
    app.backend.execute.assert_not_called()


def test_gui_pregrasp_without_displayable_endpoint_cannot_move():
    app=stub();app.review['stages'][1].pop('sdk_plan')
    app.enter_pregrasp()
    app._start_job.assert_not_called()
    assert not app.review_used


def test_full_cycle_button_keeps_original_execution_api():
    app=stub();app.review=review();app.execute()
    app._start_job.call_args.args[1]()
    assert 'execution_scope' not in app.backend.execute.call_args.kwargs
    assert '完整路径' in app.backend.execute.call_args.kwargs['approval_text']


def test_demo_pregrasp_reports_only_two_stages_and_zero_robot_commands(tmp_path,monkeypatch):
    monkeypatch.setattr(bm,'APP',tmp_path)
    backend=Backend(mode='demo')
    real_io=Mock(side_effect=AssertionError('Demo must never create a device subprocess'))
    monkeypatch.setattr(backend,'_run',real_io)
    prepared=backend.prepare(DEFAULTS)
    result=backend.execute(prepared,'offline test of approved pregrasp prefix',execution_scope='pregrasp')
    assert result['status']=='pregrasp_completed' and result['pregrasp_reached']
    assert result['grasp_result']==result['cycle_result']=='not_attempted'
    assert not result['trajectory_completed'] and result['actual_feedback_samples']==0
    assert list(result['stage_times'])==['prepare_gripper','pregrasp']
    assert result['command_count']==0
    folder=Path(prepared['run_dir'])
    saved=json.loads((folder/'result.json').read_text())
    assert saved['status']=='pregrasp_completed'
    approval=json.loads((folder/'user_approval.json').read_text())
    assert approval['execution_scope']=='pregrasp' and approval['requested_stages']==['prepare_gripper','pregrasp']
    assert '已到预抓取点' in (folder/'report.html').read_text()
    with pytest.raises(ValueError,match='已使用'):
        backend.execute(prepared,'cannot reuse prefix approval for grasp')
    real_io.assert_not_called()


@pytest.mark.parametrize('reported', ['missing_completion','full_cycle_completion'])
def test_backend_never_counts_exit_code_or_full_result_as_pregrasp_completion(tmp_path,monkeypatch,reported):
    backend=Backend()
    monkeypatch.setattr(backend,'_validate_review',lambda *a:None)
    monkeypatch.setattr(backend,'_claim_device',lambda:None)
    monkeypatch.setattr(backend,'_release_device',lambda:None)
    prepared=review();prepared.update(run_dir=str(tmp_path),manifest=str(tmp_path/'unused.json'))
    def run(argv,*args):
        assert argv[argv.index('--execution-scope')+1]=='pregrasp'
        folder=tmp_path/'execution';folder.mkdir()
        data={} if reported=='missing_completion' else dict(completed=True,trajectory_completed=True,grasp_result='success',cycle_result='success')
        (folder/'result.json').write_text(json.dumps(data))
    monkeypatch.setattr(backend,'_run',run)
    result=backend.execute(prepared,'offline mock approval',execution_scope='pregrasp')
    assert result['status']=='failed' and not result['pregrasp_reached']
    assert result['grasp_result']==result['cycle_result']=='not_attempted'
    assert '未确认已到达预抓取点' in result['error']
