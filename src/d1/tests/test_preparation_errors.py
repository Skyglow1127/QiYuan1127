"""Preparation failures stay actionable; all subprocesses are mocked."""
import json
from pathlib import Path
import shutil
import sys

import pytest

APP=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(APP))
from app import backend as module
from app.backend import Backend, ProcessFailure, preparation_diagnostics, DEFAULTS


def save(path,value):
    path.parent.mkdir(parents=True,exist_ok=True)
    path.write_text(json.dumps(value,ensure_ascii=False))


def fake_failure(monkeypatch,text=''):
    class Process:
        def __init__(self,argv,stdout,**kwargs):
            stdout.write(text);stdout.flush()
        def poll(self): return 1
        def wait(self,*args,**kwargs): return 1
    monkeypatch.setattr(module.subprocess,'Popen',Process)


@pytest.mark.parametrize('filename',['result.json','summary.json'])
def test_nonzero_prefers_structured_cause_over_timing_tail(tmp_path,monkeypatch,filename):
    output=tmp_path/'output';log=tmp_path/'stage.log'
    save(output/filename,dict(error='目标接触图像边界',
        blockers=[dict(code='vision',message='目标位于画面底边',action='查看诊断图后重新准备')],
        robot_readiness_errors=['J1=-90.100° 超出现有限位 [-90, 90]']))
    fake_failure(monkeypatch,'ValueError: unrelated old debug\n'+json.dumps({'timing':{'spans':[{'duration':1}] }},indent=2))
    with pytest.raises(ProcessFailure) as caught:
        Backend()._run(['prepare_accel_grasp.py','--output',output],log)
    message=str(caught.value)
    assert '目标接触图像边界' in message and '目标位于画面底边' in message
    assert '查看诊断图后重新准备' in message and 'J1=-90.100' in message
    assert str(log) in message and 'duration' not in message and 'unrelated old debug' not in message


@pytest.mark.parametrize('text,expected',[
    ('Traceback (most recent call last):\n  File "a.py", line 2\nValueError: no usable image\n'+ '  }\n'*9,'ValueError: no usable image'),
    ('usage: prepare\nprepare: error: --output is required\n','--output is required'),
    ('{\n  "timing": 2\n}\n','未提供可解析的失败原因'),
])
def test_nonzero_log_fallback_finds_error_without_json_tail(tmp_path,monkeypatch,text,expected):
    fake_failure(monkeypatch,text)
    with pytest.raises(RuntimeError) as caught:
        Backend()._run(['script.py','--output='+str(tmp_path/'absent')],tmp_path/'error.log')
    assert expected in str(caught.value)
    assert '完整日志：' in str(caught.value) and '"timing"' not in str(caught.value)


def test_bad_structured_json_does_not_hide_traceback(tmp_path,monkeypatch):
    output=tmp_path/'output';output.mkdir()
    (output/'result.json').write_text('{')
    (output/'summary.json').write_text('[]')
    fake_failure(monkeypatch,'FileNotFoundError: calibration.json\n')
    with pytest.raises(ProcessFailure,match='FileNotFoundError: calibration.json'):
        Backend()._run(['script.py','--output',output],tmp_path/'stage.log')


def test_nested_planner_failure_exposes_real_start_bounds_without_mutating_source(tmp_path):
    source=APP/'logs/20261025_233218_604166/plan/01_pregrasp/planned'
    before={name:(source/name).read_bytes() for name in ('plan.json','moveit.log')}
    planned=tmp_path/'plan/01_pregrasp/planned';planned.mkdir(parents=True)
    for name,contents in before.items(): (planned/name).write_bytes(contents)
    log=tmp_path/'planning.log'
    log.write_text('run_grasp_cycle.py: error: plan_grasp_stage.py failed (2); see arbitrary-log-path\n')
    failure=ProcessFailure(['run_grasp_cycle.py','--output',tmp_path/'plan'],2,log)
    assert "Joint 'Joint2'" in failure.reasons[0]
    assert 'outside bounds' in failure.reasons[0]
    assert '[-1.5708 ]' in failure.reasons[0] and '[-1.57 ]' in failure.reasons[0]
    message=str(failure)
    assert 'pregrasp: MoveIt error 99999' in message
    assert str(planned/'plan.json') in message and str(planned/'moveit.log') in message
    assert 'No controller_names specified' not in message and 'No 3D sensor' not in message
    assert 'failed (2)' not in message and 'arbitrary-log-path' not in message
    assert {name:(source/name).read_bytes() for name in before}==before


@pytest.mark.parametrize('detail',[
    "Start state is in collision with the planning scene.",
    "Found a contact between 'Link4' and 'table'.",
    "IK failed: no solution found for target pose.",
    "Unable to sample any valid states for goal tree",
])
def test_nested_planner_specific_collision_and_ik_reasons(tmp_path,detail):
    planned=tmp_path/'plan/02_approach/planned'
    save(planned/'plan.json',dict(completed=False,stage='approach',error='MoveIt error -1'))
    (planned/'moveit.log').write_text('[move_group-1] [INFO] [100] [planner]: '+detail+'\n')
    log=tmp_path/'planning.log';log.write_text('planner: error: stage failed\n')
    failure=ProcessFailure(['run_grasp_cycle.py','--output',tmp_path/'plan'],2,log)
    assert failure.reasons[0]=='规划阶段 approach：'+detail


def test_real_moveit_start_collision_contact_list_is_readable_and_preserves_logs(tmp_path):
    source=APP/'logs/20261025_233919_fa9636/plan/01_pregrasp/planned'
    before={name:(source/name).read_bytes() for name in ('plan.json','moveit.log')}
    planned=tmp_path/'plan/01_pregrasp/planned';planned.mkdir(parents=True)
    for name,contents in before.items(): (planned/name).write_bytes(contents)
    log=tmp_path/'planning.log';log.write_text('planner: error: plan_grasp_stage.py failed (2)\n')
    failure=ProcessFailure(['run_grasp_cycle.py','--output',tmp_path/'plan'],2,log)
    assert failure.reasons[0].startswith('规划阶段 pregrasp：当前姿态的模型检测到 Link2 与 Link5 碰撞；')
    assert '1 contact(s) detected : Link2 - Link5' in failure.reasons[0]
    assert str(planned/'moveit.log') in str(failure)
    assert {name:(source/name).read_bytes() for name in before}==before


def test_nested_planner_does_not_follow_external_paths_or_stage_symlinks(tmp_path):
    run=tmp_path/'run';run.mkdir();output=run/'plan';output.mkdir()
    external=tmp_path/'outside/planned'
    save(external/'plan.json',dict(completed=False,stage='external',error='DO NOT READ OUTSIDE RUN'))
    (external/'moveit.log').write_text('Start state out of bounds SECRET\n')
    (output/'01_pregrasp').symlink_to(external.parent,target_is_directory=True)
    log=run/'planning.log';log.write_text('planner: error: failed; see '+str(external/'moveit.log'))
    for target in (output,external):
        failure=ProcessFailure(['run_grasp_cycle.py','--output',target],2,log)
        assert 'DO NOT READ' not in str(failure) and 'SECRET' not in str(failure)
        assert not any(reason.startswith('规划阶段') for reason in failure.reasons)


def test_nested_planner_ignores_successful_earlier_stage(tmp_path):
    output=tmp_path/'plan'
    successful=output/'01_pregrasp/planned';failed=output/'02_approach/planned'
    save(successful/'plan.json',dict(completed=True,stage='pregrasp',error='OLD RECOVERED ERROR'))
    save(failed/'plan.json',dict(completed=False,stage='approach',error='目标不可达'))
    log=tmp_path/'planning.log';log.write_text('planner: error: stage failed\n')
    failure=ProcessFailure(['run_grasp_cycle.py','--output',output],2,log)
    assert failure.reasons[0]=='规划阶段 approach：目标不可达'
    assert 'OLD RECOVERED ERROR' not in str(failure)


def test_original_failure_fixture_exposes_all_blockers_without_mutating_logs(tmp_path,monkeypatch):
    original=APP/'logs/20261025_224213_276e77/perception'
    assert original.is_dir(), 'User failure fixture must remain available'
    names=['result.json','foreground.json','wrist_observation.json','wrist.jpg']
    before={name:(original/name).read_bytes() for name in names}
    run=tmp_path/'new_run';perception=run/'perception';perception.mkdir(parents=True)
    for name in names: shutil.copy2(original/name,perception/name)
    events=[];backend=Backend(emit=events.append)
    monkeypatch.setattr(backend,'_new_run',lambda:run)
    monkeypatch.setattr(backend,'_claim_device',lambda:None)
    monkeypatch.setattr(backend,'_release_device',lambda:None)
    monkeypatch.setattr(module,'check_environment',lambda:[])
    calls=[]
    def fail(argv,log_path,timeout):
        calls.append(argv)
        Path(log_path).write_text('timing-only tail\n')
        raise ProcessFailure(argv,1,log_path)
    monkeypatch.setattr(backend,'_run',fail)
    review=backend.prepare({})
    error='\n'.join(review['blockers'])
    assert not review['ready'] and len(calls)==1 and '--capture-live' in calls[0]
    assert '目标接触图像边界' in error
    assert '实际关节位置超出现有限位' in error  # Preserve the saved pre-tolerance failure.
    assert review['observed_joint_angles'][1]==pytest.approx(-90.1)
    assert 'power_status=0' in error and 'enable_status=0' not in error
    notices='\n'.join(review['notices'])
    assert '43.90 SDK' in notices and '36.00 SDK' in notices
    assert '无需手工' in notices and 'enable_status=0 可准备路径' in notices
    assert review['image']==str(perception/'wrist.jpg')
    assert [event for event in events if event['kind']=='joint'][0]['q']==json.loads(before['wrist_observation.json'])['sdk_angles']
    result=json.loads((run/'result.json').read_text())
    assert result['cycle_result']=='not_attempted' and result['grasp_result']=='not_attempted'
    assert result['robot_commands_sent']==0 and result['command_count']==0
    assert str(run/'perception.log') in result['error']
    assert (run/'report.html').is_file()
    assert {name:(original/name).read_bytes() for name in names}==before


def test_diagnostic_overlay_and_multiple_limit_violations(tmp_path):
    perception=tmp_path/'perception'
    save(perception/'wrist_observation.json',dict(sdk_angles=[136.,-91.,0.,0.,0.,0.,36.],
        status=dict(power_status=1,enable_status=1,error_status=0),
        robot_readiness_errors=['反馈已超时，请检查通信']))
    (perception/'wrist.jpg').touch();(perception/'diagnostic_overlay.jpg').touch()
    result=preparation_diagnostics(tmp_path,DEFAULTS,RuntimeError('识别失败'))
    reasons='\n'.join(result['blockers'])
    assert 'J0 实际反馈 136.000°' in reasons and 'J1 实际反馈 -91.000°' in reasons
    assert '反馈已超时' in reasons
    assert result['image']==str(perception/'diagnostic_overlay.jpg')
    assert '夹爪当前' not in reasons


def test_invalid_observed_q_does_not_emit_fabricated_joint_positions(tmp_path):
    save(tmp_path/'perception/wrist_observation.json',dict(sdk_angles=[0.,0.,0.],status={}))
    result=preparation_diagnostics(tmp_path,DEFAULTS,RuntimeError('no observation'))
    assert result['observed_joint_angles'] is None
    assert result['image'] is None
    assert any('power_status=None' in item for item in result['blockers'])


def test_new_structured_diagnostics_only_show_each_cause_once(tmp_path):
    perception=tmp_path/'perception'
    vision='目标接触图像边界'
    readiness=[
        'J1 实际反馈 -90.100° 超出当前范围 [-90, 90]°；需核实并恢复有效初始姿态',
        '控制协议软件标志 power_status=0，执行器要求 1；需恢复并核实控制状态，软件标志不证明硬件扭矩',
        '控制协议软件标志 enable_status=0，执行器要求 1；需恢复并核实控制状态，软件标志不证明硬件扭矩',
    ]
    joined='；'.join(readiness)
    observation=dict(sdk_angles=[2.,-90.1,87.9,10.1,13.3,-7.3,43.9],
        status=dict(power_status=0,enable_status=0,error_status=0),
        robot_readiness_errors=readiness,robot_readiness_error=joined)
    save(perception/'wrist_observation.json',observation)
    save(perception/'result.json',dict(error=vision,robot_readiness_errors=readiness,
        robot_readiness_error=joined,blockers=[
            dict(code='vision_rejected',message=vision,action='查看诊断图并重新准备'),
            *[dict(code='robot_not_ready',message=message,action='恢复有效初态并核实控制状态后重新准备') for message in readiness],
        ]))
    save(perception/'foreground.json',dict(rejection_reason=vision))
    # The eligible=false path uses the legacy joined readiness error directly.
    result=preparation_diagnostics(tmp_path,DEFAULTS,RuntimeError(joined))
    errors='\n'.join(result['blockers'])
    assert len(result['blockers'])==4  # Legacy vision/joint/flags retained; never add an initial-gripper blocker.
    for marker in ('目标接触图像边界','J1 ','power_status=0','enable_status=0'):
        assert errors.count(marker)==1
    assert '夹爪当前' not in errors
    assert result['blockers'][0]=='目标接触图像边界；查看诊断图并重新准备'
    assert all('恢复有效初态并核实控制状态后重新准备' in item for item in result['blockers'][1:4])


def test_camera_failure_uses_saved_robot_status_feedback(tmp_path):
    perception=tmp_path/'perception'
    q=[0.,-91.,0.,0.,0.,0.,36.]
    save(perception/'robot_status.json',dict(sdk_angles=q,
        status=dict(power_status=0,enable_status=0,error_status=0),
        robot_readiness_errors=['关节实际反馈缺失或超过 0.6 秒']))
    result=preparation_diagnostics(tmp_path,DEFAULTS,RuntimeError('Unexpected wrist camera'))
    errors='\n'.join(result['blockers'])
    assert result['observed_joint_angles']==q and result['image'] is None
    assert '关节实际反馈缺失或超过 0.6 秒' in errors and 'J1 实际反馈 -91.000°' in errors
    assert 'power_status=0' in errors and 'enable_status=0' not in errors
    # A complete wrist observation remains authoritative when both exist.
    fresh=[0.,0.,0.,0.,0.,0.,36.]
    save(perception/'wrist_observation.json',dict(sdk_angles=fresh,
        status=dict(power_status=1,enable_status=1,error_status=0),robot_readiness_errors=[]))
    result=preparation_diagnostics(tmp_path,DEFAULTS,RuntimeError('识别失败'))
    assert result['observed_joint_angles']==fresh and result['blockers']==['识别失败']


def test_valid_arbitrary_gripper_start_and_enable_zero_are_not_new_blockers(tmp_path):
    save(tmp_path/'perception/wrist_observation.json',dict(sdk_angles=[0.,0.,0.,0.,0.,0.,6.5],
        status=dict(power_status=1,enable_status=0,error_status=0),robot_readiness_errors=[]))
    result=preparation_diagnostics(tmp_path,DEFAULTS,RuntimeError('目标未找到'))
    assert result['blockers']==['目标未找到']
    assert '6.50 SDK → 抓取前张开目标 36.00 SDK' in result['notices'][0]
    assert '批准执行' in result['notices'][0]
    assert 'enable_status=0 可准备路径' in result['notices'][1]


@pytest.mark.parametrize('j6',[-.01,65.01])
def test_arbitrary_gripper_start_still_requires_valid_limits(tmp_path,j6):
    save(tmp_path/'perception/wrist_observation.json',dict(sdk_angles=[0.,0.,0.,0.,0.,0.,j6],
        status=dict(power_status=1,enable_status=0,error_status=0),robot_readiness_errors=[]))
    result=preparation_diagnostics(tmp_path,DEFAULTS,RuntimeError('准备未通过'))
    assert any('J6 实际反馈' in item and '超出范围' in item for item in result['blockers'])
    assert not any('夹爪实际' in item for item in result['notices'])


def test_live_prepare_preserves_real_gripper_start_for_reviewed_adjustment(tmp_path,monkeypatch):
    run=tmp_path/'run';run.mkdir();q=[0.,-20.,40.,0.,20.,0.,6.5]
    backend=Backend();calls=[]
    monkeypatch.setattr(backend,'_new_run',lambda:run)
    monkeypatch.setattr(backend,'_claim_device',lambda:None)
    monkeypatch.setattr(backend,'_release_device',lambda:None)
    monkeypatch.setattr(module,'check_environment',lambda:[])
    def simulate(argv,log_path,timeout):
        calls.append(argv)
        if len(calls)==1:
            save(run/'perception/result.json',dict(completed=True,execution_candidate_eligible=True))
            save(run/'perception/pose.json',dict(sdk_start=q))
            save(run/'perception/wrist_observation.json',dict(sdk_angles=q,
                status=dict(power_status=1,enable_status=0,error_status=0)))
        else:
            save(run/'plan/manifest.json',dict(target={},spec=DEFAULTS,expires_at=module.time.time()+600,
                stages=[dict(name='prepare_gripper',expected_angles=q,
                    gripper_plan=dict(target_sdk_units=36.,duration_s=59.))]))
    monkeypatch.setattr(backend,'_run',simulate)
    review=backend.prepare({})
    assert review['ready'] and not review['blockers'] and len(calls)==2
    start=calls[1].index('--expected-angles')+1
    assert calls[1][start:start+7]==q
    assert review['observed_joint_angles']==q
    assert review['stages'][0]['name']=='prepare_gripper'
    assert review['stages'][0]['expected_angles'][6]==6.5
    assert '6.50 SDK → 抓取前张开目标 36.00 SDK' in review['notices'][0]


def test_demo_includes_reviewable_initial_gripper_stage(tmp_path):
    review=Backend('demo')._demo_prepare(DEFAULTS,tmp_path)
    assert review['observed_joint_angles'][6]==6.5
    first,second=review['stages'][:2]
    assert first['name']=='prepare_gripper'
    assert first['expected_angles'][6]==6.5
    assert first['gripper_plan']['target_sdk_units']==36.
    assert second['expected_angles'][6]==36.


@pytest.mark.parametrize('j1,allowed',[(-90.1,True),(-90.2,True),(-90.21,False)])
def test_preparation_diagnostics_reuses_approved_arm_feedback_tolerance(tmp_path,j1,allowed):
    save(tmp_path/'perception/wrist_observation.json',dict(sdk_angles=[0.,j1,0.,0.,0.,0.,6.5],
        status=dict(power_status=1,enable_status=0,error_status=0),robot_readiness_errors=[]))
    result=preparation_diagnostics(tmp_path,DEFAULTS,RuntimeError('目标未找到'))
    assert any('J1 实际反馈' in item for item in result['blockers']) is not allowed
