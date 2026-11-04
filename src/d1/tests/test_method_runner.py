"""One review per method; all tests use synthetic data or read-only file validation."""
import importlib.util
import json
import math
from pathlib import Path
import signal
import time
from unittest.mock import Mock

import pytest
import yaml

from app import backend as bm
from app import method_runner as runner
from app.backend import Backend, DEFAULTS, digest
from app.execution_scope import FIXED_PREGRASP
from app.reporting import load, write_json


@pytest.fixture
def demo(tmp_path,monkeypatch):
    monkeypatch.setattr(bm,'APP',tmp_path)
    monkeypatch.setattr(Backend,'_run',Mock(side_effect=AssertionError('No hardware or subprocess in method tests')))
    return runner.prepare_method(mode='demo')


def test_prepare_is_read_only_and_summarizes_every_approved_waypoint(demo):
    folder=Path(demo['run_dir'])
    assert demo['synthetic'] and not demo['physical_motion_validated']
    assert demo['parameters']['speed_deg_s']==5.
    assert demo['parameters']['acceleration_deg_s2']==10.
    assert demo['parameters']['gripper_speed_sdk_s']==10.
    assert demo['parameters']['open_target']==36.
    assert demo['startup_actions']==runner.STARTUP_ACTIONS
    assert demo['method_approval_sha256']==runner._method_digest(demo)
    text=(folder/'summary.txt').read_text()
    for point in demo['stages'][-1]['sdk_plan']['sdk']:
        assert json.dumps(point,ensure_ascii=False) in text
    assert '不将结果归因于 ACK 单项' in text
    assert '协议上电' in text and '不证明实机' in text
    assert not (folder/'execution').exists()
    assert not (folder/'method_approval.json').exists()
    Backend._run.assert_not_called()


def test_demo_execute_is_automatic_and_cannot_replay_even_in_new_process(demo):
    folder=Path(demo['run_dir']);path=folder/'method_review.json'
    summary=runner.execute_method(path,'批准上述方法一和已展示目标、路径、上电与参数；仅演示',mode='demo')
    assert summary['completed'] is True and summary['synthetic'] is True
    assert summary['physical_motion_validated'] is False
    assert summary['max_endpoint_error_deg']==pytest.approx(0.)
    assert summary['commands']==summary['receipts']==0
    assert summary['startup_result']['completed'] and summary['startup_result']['synthetic']
    assert summary['camera_startup_result']['completed'] and summary['camera_startup_result']['synthetic']
    assert load(folder/'user_approval.json')['text'].startswith('批准上述方法一')
    assert (folder/'method_review.json.consumed').exists()
    for name in ('method_summary.json','result.json','report.html','joint_positions.csv','commands.csv'):
        assert (folder/name).is_file()
    with pytest.raises((ValueError,FileExistsError),match='已使用|File exists'):
        runner.execute_method(path,'不能重放',mode='demo')
    Backend._run.assert_not_called()


def test_startup_failure_consumes_method_and_never_enters_motion(demo,monkeypatch):
    folder=Path(demo['run_dir']);execute=Mock(side_effect=AssertionError('No motion after startup failure'))
    monkeypatch.setattr(Backend,'execute',execute)
    monkeypatch.setattr(Backend,'control_device',lambda *a:dict(completed=False,error='power request failed'))
    summary=runner.execute_method(folder/'method_review.json','批准一次方法一，包括协议上电',mode='demo')
    assert not summary['completed'] and 'power request failed' in summary['stop_reason']
    assert not (folder/'execution').exists()
    assert (folder/'method_summary.json').is_file()
    with pytest.raises(FileExistsError):
        runner.execute_method(folder/'method_review.json','失败后不能重放',mode='demo')
    execute.assert_not_called()


def test_camera_startup_failure_consumes_approval_before_any_power_or_motion(demo,monkeypatch):
    monkeypatch.setattr(Backend,'start_cameras',lambda *a:dict(completed=False,error='camera service failed'))
    power=Mock(side_effect=AssertionError('Camera failure must prevent power request'))
    monkeypatch.setattr(Backend,'control_device',power)
    folder=Path(demo['run_dir']);path=folder/'method_review.json'
    result=runner.execute_method(path,'批准双相机启动、协议上电和一次演示移动',mode='demo')
    assert not result['completed'] and 'camera service failed' in result['stop_reason']
    assert result['startup_result'] is None
    assert (folder/'method_review.json.consumed').exists()
    with pytest.raises(FileExistsError):runner.execute_method(path,'失败不能重放',mode='demo')
    power.assert_not_called()


def test_cancel_during_startup_stops_before_motion_and_restores_handlers(demo,monkeypatch):
    previous={sig:signal.getsignal(sig) for sig in (signal.SIGINT,signal.SIGTERM)}
    def startup(instance,action):
        signal.raise_signal(signal.SIGINT)
        return dict(completed=True,status_fresh=True,status={'power_status':1,'error_status':0})
    execute=Mock(side_effect=AssertionError('Cancellation cannot enter motion'))
    monkeypatch.setattr(Backend,'control_device',startup);monkeypatch.setattr(Backend,'execute',execute)
    folder=Path(demo['run_dir'])
    result=runner.execute_method(folder/'method_review.json','批准一次演示方法',mode='demo')
    assert result['status']=='cancelled' and not result['completed']
    assert (folder/'method_review.json.consumed').exists()
    assert (folder/'STOP').exists()
    assert {sig:signal.getsignal(sig) for sig in previous}==previous
    execute.assert_not_called()


def test_post_startup_resume_failure_is_recorded_and_cannot_replay(demo,monkeypatch):
    original=Backend.resume_review;calls=[]
    def resume(instance,path):
        calls.append(path)
        if len(calls)==2: raise ValueError('source changed while powering on')
        return original(instance,path)
    execute=Mock(side_effect=AssertionError('Invalidated review cannot execute'))
    monkeypatch.setattr(Backend,'resume_review',resume);monkeypatch.setattr(Backend,'execute',execute)
    folder=Path(demo['run_dir']);path=folder/'method_review.json'
    result=runner.execute_method(path,'批准一次演示方法',mode='demo')
    assert 'source changed' in result['stop_reason']
    assert not (folder/'execution').exists()
    with pytest.raises(FileExistsError):runner.execute_method(path,'不得重放失败方法',mode='demo')
    execute.assert_not_called()


def test_summary_keeps_real_schedule_and_raw_log_paths(demo):
    folder=Path(demo['run_dir']);stage=folder/'execution/01_fixed_pregrasp/executed';stage.mkdir(parents=True)
    schedule=[dict(start_s=1.,end_s=1.06,completed=True,basis='transport_send_call')]
    write_json(stage/'result.json',dict(commands=6,receipts=6,scheduling_policy='ack_overlap_v1',
        scheduling_overrun_s=.02,command_group_schedule=schedule,completed=False,error='blocked'))
    summary=runner._method_summary(demo,{'status':'failed','error':'blocked'},None)
    assert summary['commands']==summary['receipts']==6
    assert summary['stage_reports'][0]['command_group_schedule']==schedule
    assert summary['stage_reports'][0]['scheduling_overrun_s']==.02
    assert summary['stage_reports'][0]['raw_transport']==str(stage/'transport.jsonl')


def test_empty_approval_and_mode_switch_cannot_start_devices(demo,monkeypatch):
    control=Mock(side_effect=AssertionError('No control without matching approval'))
    monkeypatch.setattr(Backend,'control_device',control)
    path=Path(demo['run_dir'])/'method_review.json'
    with pytest.raises(ValueError,match='实际明确授权'):
        runner.execute_method(path,'  ',mode='demo')
    with pytest.raises(ValueError,match='模式'):
        runner.execute_method(path,'演示不能切换为实机',mode='live')
    with pytest.raises(ValueError,match='模式'):
        Backend(mode='live').resume_review(path.parent/'review.json')
    assert not Path(str(path)+'.consumed').exists()
    control.assert_not_called()


def test_method_cannot_change_target_even_after_rehash(demo):
    path=Path(demo['run_dir'])/'method_review.json'
    changed=load(path);changed['target']['description']='changed after review'
    changed['method_approval_sha256']=runner._method_digest(changed);write_json(path,changed)
    with pytest.raises(ValueError,match='摘要'):
        runner.execute_method(path,'原批准不能覆盖修改后的内容',mode='demo')
    assert not Path(str(path)+'.consumed').exists()


@pytest.mark.parametrize('mutation',['outside','symlink','manifest','expiry','used','run_dir'])
def test_resume_rejects_wrong_location_changes_and_consumed_review(demo,tmp_path,mutation):
    folder=Path(demo['run_dir']);path=folder/'review.json';review=load(path)
    if mutation=='outside':
        path=tmp_path/'outside/review.json';path.parent.mkdir();write_json(path,review)
    elif mutation=='symlink':
        outside=tmp_path/'outside.json';write_json(outside,review)
        path.unlink();path.symlink_to(outside)
    elif mutation=='manifest':
        Path(review['manifest']).write_text('{}')
    elif mutation=='expiry':
        review['expires_at']=time.time()-1;write_json(path,review)
    elif mutation=='used':
        Path(review['manifest']+'.consumed').write_text('{}')
    elif mutation=='run_dir':
        review['run_dir']=str(tmp_path);write_json(path,review)
    backend=Backend(mode='demo')
    with pytest.raises((ValueError,OSError)):
        backend.resume_review(path)
    assert backend._review is None


def test_live_resume_reuses_full_manifest_and_source_hash_validation(tmp_path,monkeypatch):
    """A realistic file-only live manifest validates without constructing any transport."""
    monkeypatch.setattr(bm,'APP',tmp_path)
    spec=importlib.util.spec_from_file_location('method_runner_cycle',bm.TOOLS/'run_grasp_cycle.py')
    cycle=importlib.util.module_from_spec(spec);spec.loader.exec_module(cycle)
    cfg=yaml.safe_load(cycle.CONFIG.read_text())
    profile_path=cycle.ROOT.parents[1]/'app/fixed_ready_pose.json';profile=load(profile_path)
    from d1_pick_place.sdk_executor import Mapping
    target=Mapping.from_candidate(cfg['hardware_mapping_candidate']).to_sdk(profile['model_target_rad']).tolist()
    folder=tmp_path/'logs/live_review';plan=folder/'plan';plan.mkdir(parents=True)
    source=plan/'arm.json';write_json(source,dict(scheduling_policy=runner.SCHEDULING_POLICY,
        progress_policy='feedback_paced',endpoint_policy=None))
    parameters=dict(DEFAULTS,**runner.FIXED_MOTION,fixed_ready=True,candidate=cfg['hardware_mapping_candidate'],
        scheduling_policy=runner.SCHEDULING_POLICY)
    initial=[0.,-20.,40.,0.,20.,0.,36.]
    stages=[dict(name='prepare_gripper',expected_angles=initial,gripper_target=36.,gripper_plan={'duration_s':.1}),
        dict(name=FIXED_PREGRASP,expected_angles=initial,gripper_target=36.,plan_file=str(source),
            sdk_plan=dict(sdk=[initial[:6],target],duration_s=3.),progress_policy='feedback_paced',
            scheduling_policy=runner.SCHEDULING_POLICY,endpoint_policy=None)]
    manifest=dict(schema_version=1,created_at=time.time()-1,expires_at=time.time()+600,
        initial_angles=initial,target={'description':'fixed'},spec=parameters,limits=cycle.approved_limits(parameters),
        stages=stages,continuation_envelope_deg=1.,continuation_policy='reviewed_path',
        arm_execution_policy=cycle.ARM_EXECUTION_POLICY,accept_stable_close_shortfall=False,
        physical_grasp_verification='automatic_opencv_two_hypothesis_v1',
        fixed_model_target_rad=profile['model_target_rad'],fixed_sdk_target_deg=target,operation=FIXED_PREGRASP,
        files={str(profile_path.resolve()):digest(profile_path),str(source):digest(source),
            str((bm.TOOLS/'run_grasp_cycle.py').resolve()):digest(bm.TOOLS/'run_grasp_cycle.py')},
        replay_only=False,execution_candidate_eligible=True,endpoint_policy=None)
    manifest_path=plan/'manifest.json';write_json(manifest_path,manifest)
    review=dict(mode='live',ready=True,run_dir=str(folder),manifest=str(manifest_path),
        approval_sha256=digest(manifest_path),expires_at=manifest['expires_at'],target=manifest['target'],
        parameters=parameters,observed_joint_angles=initial,operation=FIXED_PREGRASP,
        scheduling_policy=runner.SCHEDULING_POLICY,startup_actions=runner.STARTUP_ACTIONS,
        stages=[dict(item,duration_s=item.get('sdk_plan',item.get('gripper_plan'))['duration_s']) for item in stages])
    path=folder/'review.json';write_json(path,review)
    instance=Backend();assert instance.resume_review(path)==review
    source.write_text('changed implementation after review')
    with pytest.raises(ValueError,match='Reviewed file changed'):
        Backend().resume_review(path)


def test_live_prepare_passes_read_only_unpowered_flag_and_scheduling_policy(tmp_path,monkeypatch):
    monkeypatch.setattr(bm,'check_environment',lambda:[])
    commands=[];q=[0.,-20.,40.,0.,20.,0.,36.]
    def run(argv,*unused):
        commands.append(argv);assert '--execute' not in argv
        output=Path(argv[argv.index('--output')+1]);output.mkdir(parents=True)
        if Path(argv[0]).name=='observe_grasp.py':
            assert '--allow-unpowered-observation' in argv
            assert '--arm-only-observation' in argv
            write_json(output/'wrist_observation.json',dict(sdk_angles=q,status={'power_status':0}))
        else:
            assert argv[argv.index('--scheduling-policy')+1]=='ack_overlap_v1'
            assert argv[argv.index('--valid-for-s')+1]==7200.
            write_json(output/'manifest.json',dict(target={'description':'fixed'},spec=DEFAULTS,
                stages=[dict(name=FIXED_PREGRASP,sdk_plan={'duration_s':1.})],expires_at=time.time()+7200))
    instance=Backend();monkeypatch.setattr(instance,'_run',run)
    review=instance._prepare_fixed_pregrasp(DEFAULTS,tmp_path,
        scheduling_policy='ack_overlap_v1',valid_for_s=7200.)
    assert len(commands)==2 and review['startup_actions']==runner.STARTUP_ACTIONS


def test_unknown_method_and_invalid_lifetime_do_not_prepare(monkeypatch):
    prepare=Mock(side_effect=AssertionError('Unknown method must never prepare'))
    monkeypatch.setattr(Backend,'prepare',prepare)
    with pytest.raises(ValueError,match='未知'):
        runner.prepare_method('other')
    prepare.assert_not_called()
