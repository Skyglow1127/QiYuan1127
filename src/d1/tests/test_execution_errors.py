"""Saved execution failures are diagnostic evidence, never new device actions."""
import json
from pathlib import Path
import sys

import pytest

APP=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(APP))
from app.backend import Backend, ProcessFailure
from app import reporting

SOURCE=APP/'logs/20261026_000512_8af2c3'
FILES=['execution/result.json','execution/events.jsonl',
       'execution/00_prepare_gripper/executed/summary.json',
       'execution/00_prepare_gripper/executed/transport.jsonl']
CLOCK={'monotonic':100.,'unix':1700000000.}

def save(path,value):
    path.parent.mkdir(parents=True,exist_ok=True)
    path.write_text(json.dumps(value,ensure_ascii=False))

@pytest.fixture
def saved_run(tmp_path):
    before={name:(SOURCE/name).read_bytes() for name in FILES}
    for name,data in before.items():
        path=tmp_path/name;path.parent.mkdir(parents=True,exist_ok=True);path.write_bytes(data)
    (tmp_path/'execution.log').write_text('run_grasp_cycle.py: error: gripper_stage.py failed (1)\n')
    yield tmp_path
    assert {name:(SOURCE/name).read_bytes() for name in FILES}==before

def result_for(run):
    return dict(mode='live',status='failed',grasp_result='unknown',cycle_result='not_completed',
                total_time_s=1.,details=reporting.load(run/'execution/result.json',{}))

def test_saved_limit_failure_surfaces_actual_joint_and_preserves_historical_reason(saved_run):
    failure=ProcessFailure(['run_grasp_cycle.py','--output',saved_run/'execution'],2,saved_run/'execution.log')
    assert 'J1=-90.100°' in failure.reasons[0] and '[-90, 90]°' in failure.reasons[0]
    assert '当次执行器的限位失败记录' in failure.reasons[0]
    assert '实际关节位置超出现有限位' in str(failure)
    assert str(saved_run/'execution/00_prepare_gripper/executed/summary.json') in str(failure)
    assert 'camera_error' not in str(failure)

def test_backend_failure_reports_no_grasp_when_every_entered_stage_explicitly_sent_zero(saved_run,monkeypatch):
    backend=Backend()
    monkeypatch.setattr(backend,'_validate_review',lambda *args:None)
    monkeypatch.setattr(backend,'_claim_device',lambda:None)
    monkeypatch.setattr(backend,'_release_device',lambda:None)
    def fail(argv,log_path,timeout):
        raise ProcessFailure(argv,2,log_path)
    monkeypatch.setattr(backend,'_run',fail)
    review=dict(run_dir=str(saved_run),manifest=str(saved_run/'unused.json'),approval_sha256='test',parameters={})
    result=backend.execute(review,'mock-only test approval; no device client exists')
    assert result['grasp_result']==result['cycle_result']=='not_attempted'
    assert result['robot_commands_sent']==result['command_count']==0
    assert result['execution_command_evidence']['no_commands_proven'] is True
    assert 'J1=-90.100°' in result['error']
    assert '完整抓放结果：未执行' in (saved_run/'report.html').read_text()

@pytest.mark.parametrize('count',[1,5,None,'0',False])
def test_positive_or_missing_command_count_is_never_relabelled_not_attempted(saved_run,count):
    path=saved_run/'execution/00_prepare_gripper/executed/summary.json'
    summary=reporting.load(path);summary['commands']=count;save(path,summary)
    result=reporting.export_report(saved_run,result_for(saved_run),CLOCK)
    assert result['grasp_result']=='unknown' and result['cycle_result']=='not_completed'
    assert result['execution_command_evidence']['no_commands_proven'] is False
    assert '完整抓放结果：未完成' in (saved_run/'report.html').read_text()

def test_zero_first_stage_does_not_cover_missing_later_execution_summary(saved_run):
    (saved_run/'execution/01_pregrasp/executed').mkdir(parents=True)
    with (saved_run/'execution/events.jsonl').open('a') as stream:
        stream.write(json.dumps(dict(state='executing',stage='pregrasp',time=1800000000.))+'\n')
    result=reporting.export_report(saved_run,result_for(saved_run),CLOCK)
    assert result['grasp_result']=='unknown'
    assert not result['execution_command_evidence']['no_commands_proven']

def test_logged_send_attempt_overrides_zero_summary(saved_run):
    path=saved_run/'execution/00_prepare_gripper/executed/transport.jsonl'
    with path.open('a') as stream:
        stream.write(json.dumps(dict(kind='command',t=101.,joint=6,target_sdk_units=7.))+'\n')
    result=reporting.export_report(saved_run,result_for(saved_run),CLOCK)
    assert result['command_count']==1 and result['grasp_result']=='unknown'
    assert not result['execution_command_evidence']['no_commands_proven']


def test_parent_explicit_command_count_conflicts_with_zero_stage(saved_run):
    original=result_for(saved_run);original['details']['robot_commands_sent']=1
    result=reporting.export_report(saved_run,original,CLOCK)
    assert result['grasp_result']=='unknown'
    assert not result['execution_command_evidence']['no_commands_proven']

def test_absence_of_executor_reports_is_not_zero_command_proof(tmp_path):
    result=reporting.export_report(tmp_path,result_for(tmp_path),CLOCK)
    assert result['command_count']==0 and result['grasp_result']=='unknown'
    assert not result['execution_command_evidence']['no_commands_proven']

def test_arm_structured_camera_error_is_visible_and_external_stage_is_not_read(tmp_path):
    run=tmp_path/'run';output=run/'execution';stage=output/'01_pregrasp/executed'
    save(stage/'result.json',dict(completed=False,error='Camera check failed',camera_error='wrist frame stale: 3.2 s',commands=0))
    outside=tmp_path/'outside/executed';save(outside/'summary.json',dict(completed=False,error='EXTERNAL SECRET',commands=0))
    (output/'02_close').symlink_to(outside.parent,target_is_directory=True)
    log=run/'execution.log';log.write_text('executor: error: failed; see '+str(outside/'summary.json'))
    failure=ProcessFailure(['run_grasp_cycle.py','--output',output],2,log)
    assert 'wrist frame stale: 3.2 s' in str(failure)
    assert 'EXTERNAL SECRET' not in str(failure)
    result=reporting.export_report(run,result_for(run),CLOCK)
    assert not result['execution_command_evidence']['no_commands_proven']


def close_failure(run):
    source=APP/'logs/20261026_093003_e6fb39'
    files=['execution/result.json','execution/03_close/executed/summary.json']
    before={name:(source/name).read_bytes() for name in files}
    for name,data in before.items():
        path=run/name;path.parent.mkdir(parents=True,exist_ok=True);path.write_bytes(data)
    log=run/'execution.log';log.write_text('run_grasp_cycle.py failed (1)')
    failure=ProcessFailure(['run_grasp_cycle.py','--output',run/'execution'],1,log)
    assert {name:(source/name).read_bytes() for name in files}==before
    return failure


def test_real_close_failure_shows_target_feedback_and_tolerance_in_chinese(tmp_path):
    failure=close_failure(tmp_path)
    message=str(failure)
    assert message.startswith('执行阶段 闭合夹爪：夹爪未到达目标位置')
    for text in ['目标 29.60 SDK','最终反馈 30.20 SDK','偏差 0.60 SDK','≤0.50 SDK','是否夹住物体尚未确认']:
        assert text in message
    assert '退出 1' not in message and 'failed (1)' not in message
    assert failure.exit_code==1 and failure.technical_error=='run_grasp_cycle.py 退出 1'
    assert '执行阶段报告：' in message and '完整日志：' in message


def test_backend_and_exported_report_use_specific_close_reason(tmp_path,monkeypatch):
    failure=close_failure(tmp_path)
    backend=Backend()
    monkeypatch.setattr(backend,'_validate_review',lambda *a:None)
    monkeypatch.setattr(backend,'_claim_device',lambda:None)
    monkeypatch.setattr(backend,'_release_device',lambda:None)
    def fail(*a,**kw):raise failure
    monkeypatch.setattr(backend,'_run',fail)
    result=backend.execute(dict(run_dir=str(tmp_path),manifest=str(tmp_path/'unused.json'),
        approval_sha256='offline',parameters={}),'offline test only')
    assert '最终反馈 30.20 SDK' in result['error']
    assert '退出 1' not in result['error']
    assert result['technical_error']=='run_grasp_cycle.py 退出 1'
    assert result['grasp_result']=='unknown'
    assert 'J6 endpoint not reached/stable' in (tmp_path/'execution/03_close/executed/summary.json').read_text()
    assert '最终反馈 30.20 SDK' in (tmp_path/'report.html').read_text()


def test_saved_arm_endpoint_failure_uses_recorded_tolerance(tmp_path):
    source=APP/'logs/20261026_092451_d203a6/execution/01_pregrasp/executed'
    stage=tmp_path/'execution/01_pregrasp/executed';stage.mkdir(parents=True)
    for name in ['result.json','plan.json']:(stage/name).write_bytes((source/name).read_bytes())
    log=tmp_path/'execution.log';log.write_text('failed')
    failure=ProcessFailure(['run_grasp_cycle.py','--output',tmp_path/'execution'],1,log)
    assert '移动至预抓取点：机械臂未到位' in str(failure)
    assert '允许偏差 ≤0.3°' in str(failure)  # Do not reinterpret old failures using today's 1 degree.
    assert 'J0 目标 -10.082°，最终反馈 -9.600°，偏差 0.482°' in str(failure)


def stale_feedback_failure(run, fields):
    report=dict(completed=False,error='Angle feedback stale',commands=1,
                feedback_pending_at_end=True,feedback_rejected_count=5,
                final_raw_feedback_age_s=.1,final_valid_feedback_age_s=.7)
    if 'feedback_pending' in fields and 'feedback_pending_at_end' not in fields:
        report.pop('feedback_pending_at_end')
    report.update(fields)
    save(run/'execution/01_pregrasp/executed/result.json',report)
    log=run/'execution.log';log.write_text('executor failed')
    return ProcessFailure(['run_grasp_cycle.py','--output',run/'execution'],1,log)


@pytest.mark.parametrize('pending_key',['feedback_pending_at_end','feedback_pending'])
def test_stale_valid_feedback_with_fresh_rejected_raw_is_not_labeled_disconnected(tmp_path,pending_key):
    failure=stale_feedback_failure(tmp_path,{pending_key:True})
    message=str(failure)
    assert '原始反馈仍在到达，但连续未通过速度筛选，有效反馈超过0.6秒；检查筛选诊断' in message
    assert '检查机械臂通信' not in message
    assert 'Angle feedback stale' in (tmp_path/'execution/01_pregrasp/executed/result.json').read_text()


@pytest.mark.parametrize('fields',[
    {'feedback_pending_at_end':False},
    {'feedback_pending_at_end':None},
    {'final_raw_feedback_age_s':None},
    {'final_raw_feedback_age_s':.60001},
    {'final_raw_feedback_age_s':-.1},
    {'final_raw_feedback_age_s':'0.1'},
    {'final_raw_feedback_age_s':True},
    {'feedback_rejected_count':0},
    {'feedback_rejected_count':True},
    {'feedback_rejected_count':None},
])
def test_stale_feedback_without_complete_filter_evidence_keeps_communication_reason(tmp_path,fields):
    failure=stale_feedback_failure(tmp_path,fields)
    assert '关节反馈缺失或超过 0.6 秒；检查机械臂通信后重新准备' in str(failure)
    assert '原始反馈仍在到达' not in str(failure)


def test_feedback_recovery_timeout_with_raw_still_arriving_names_filter_exhaustion(tmp_path):
    failure=stale_feedback_failure(tmp_path,dict(error='Angle feedback recovery timeout',
        feedback_gap_timeout_s=2.,final_valid_feedback_age_s=2.01,
        feedback_gap_recoveries=2,feedback_gap_wait_s=.8))
    message=str(failure)
    assert '原始反馈仍在到达，但连续未通过速度筛选' in message
    assert '距最后有效关节反馈已超过 2 秒且未恢复' in message
    assert '已终止本次执行，暂停期间未发送新目标' in message
    assert '原始关节反馈已停更' not in message and '检查机械臂通信' not in message


def test_feedback_recovery_timeout_with_raw_stopped_names_missing_raw_updates(tmp_path):
    failure=stale_feedback_failure(tmp_path,dict(error='Angle feedback recovery timeout',
        feedback_pending_at_end=False,feedback_rejected_count=0,
        final_raw_feedback_age_s=2.01,final_valid_feedback_age_s=2.01,
        feedback_gap_timeout_s=2.))
    message=str(failure)
    assert '原始关节反馈已停更' in message
    assert '距最后有效关节反馈已超过 2 秒且未恢复' in message
    assert '暂停期间未发送新目标' in message and '速度筛选' not in message


def test_feedback_recovery_timeout_without_raw_evidence_does_not_invent_cause(tmp_path):
    failure=stale_feedback_failure(tmp_path,dict(error='Angle feedback recovery timeout',
        final_raw_feedback_age_s=None,feedback_gap_timeout_s=2.))
    message=str(failure)
    assert '距最后有效关节反馈已超过 2 秒且未恢复' in message
    assert '原始关节反馈已停更' not in message and '原始反馈仍在到达' not in message


@pytest.mark.parametrize('filtered',[False,True])
def test_recovery_timeout_uses_failure_snapshot_when_finally_has_new_fresh_feedback(tmp_path,filtered):
    failure=stale_feedback_failure(tmp_path,dict(error='Angle feedback recovery timeout',
        final_raw_feedback_age_s=.001,final_valid_feedback_age_s=.001,
        feedback_pending_at_end=False,feedback_rejected_count=0,feedback_gap_timeout_s=9.,
        feedback_failure_diagnostics=dict(final_raw_feedback_age_s=.1 if filtered else 2.01,
            final_valid_feedback_age_s=2.01,feedback_pending_at_end=filtered,
            feedback_rejected_count=5 if filtered else 0,feedback_gap_timeout_s=2.)))
    message=str(failure)
    assert '距最后有效关节反馈已超过 2 秒且未恢复' in message
    assert '暂停期间未发送新目标' in message
    assert ('原始反馈仍在到达，但连续未通过速度筛选' in message) is filtered
    assert ('原始关节反馈已停更' in message) is not filtered


def test_all_joint_feedback_timeout_reports_per_axis_ages_and_keeps_failure_snapshot(tmp_path):
    failure=stale_feedback_failure(tmp_path,dict(error='All joint feedback recovery timeout',
        joint_feedback_age_s=[.01]*7,feedback_failure_diagnostics=dict(
            joint_feedback_age_s=[2.01,2.02,2.03,2.04,2.05,2.06,2.07],
            stale_feedback_joints=list(range(7)),feedback_gap_timeout_s=2.,
            feedback_joint_timeout_events=[dict(event='stale',joint=4,age_s=.7,t=100.)])))
    message=str(failure)
    assert '全部 J0–J6 的有效反馈均超过 2 秒且未恢复' in message
    assert 'J0=2.01s' in message and 'J6=2.07s' in message
    assert '等待恢复期间未发送新目标' in message
    assert '检查机械臂通信' not in message


def test_all_joint_feedback_timeout_preserves_recorded_gap_even_when_failure_snapshot_is_fresh(tmp_path):
    failure=stale_feedback_failure(tmp_path,dict(error='All joint feedback recovery timeout',
        joint_feedback_age_s=[.01]*7,feedback_failure_diagnostics=dict(
            joint_feedback_age_s=[.02]*7,stale_feedback_joints=[],feedback_gap_timeout_s=2.,
            all_feedback_gap_count=1,last_all_feedback_gap_start_s=98.,last_all_feedback_gap_end_s=100.12)))
    message=str(failure)
    assert '全部 J0–J6 曾连续 2.120 秒无有效反馈' in message
    assert '已超过 2 秒；后到帧不能撤销超时' in message
    assert '等待恢复期间未发送新目标' in message
    assert '且未恢复' not in message and 'J0=0.02s' not in message


def test_missing_gripper_feedback_does_not_invent_values_or_contact(tmp_path):
    save(tmp_path/'execution/03_close/executed/summary.json',
         dict(completed=False,error='J6 endpoint not reached/stable; receipt was not completion',
              target_sdk_units=29.6,final_angles=None))
    log=tmp_path/'execution.log';log.write_text('failed')
    failure=ProcessFailure(['run_grasp_cycle.py','--output',tmp_path/'execution'],1,log)
    assert '夹爪未满足到位或稳定性要求' in str(failure)
    assert '最终反馈' not in str(failure) and '接触成功' not in str(failure)


def test_fallback_python_exception_is_not_replaced_by_exit_code(tmp_path):
    log=tmp_path/'execution.log';log.write_text('Traceback (most recent call last):\nValueError: bad configuration\n')
    failure=ProcessFailure(['run_grasp_cycle.py'],1,log)
    assert 'ValueError: bad configuration' in str(failure)
    assert not str(failure).startswith('run_grasp_cycle.py')


def test_pre_motion_target_recheck_is_localized_and_not_a_grasp_attempt(tmp_path):
    source=APP/'logs/20261026_134130_314dc2'
    for name in ['execution/result.json','execution/events.jsonl',
                 'execution/00_prepare_gripper/target_recheck/result.json']:
        path=tmp_path/name;path.parent.mkdir(parents=True,exist_ok=True)
        path.write_bytes((source/name).read_bytes())
    log=tmp_path/'execution.log';log.write_text('failed')
    failure=ProcessFailure(['run_grasp_cycle.py','--output',tmp_path/'execution'],1,log)
    assert '执行前目标复核未通过：红块分割轮廓有缺口' in str(failure)
    assert 'Fresh target' not in str(failure) and 'Foreground is' not in str(failure)
    result=reporting.export_report(tmp_path,result_for(tmp_path),CLOCK)
    assert result['grasp_result']==result['cycle_result']=='not_attempted'
    assert result['execution_command_evidence']['readonly_target_recheck_only']
    # An actual send attempt always overrides the pre-motion declaration.
    path=tmp_path/'execution/00_prepare_gripper/target_recheck/readonly.jsonl'
    path.write_text(json.dumps(dict(kind='command',t=101.,joint=6,target_sdk_units=7.))+'\n')
    result=reporting.export_report(tmp_path,result_for(tmp_path),CLOCK)
    assert not result['execution_command_evidence']['no_commands_proven']
    assert result['grasp_result']=='unknown'
