"""Approval binding and policy routing, with no device subprocesses."""
import importlib.util
import json
from pathlib import Path
from unittest.mock import Mock

import pytest

from app import backend as bm
from app import closed_loop_runner as runner
from app.backend import Backend

ROOT = Path(__file__).resolve().parents[1]
CONTROL = ROOT / 'runtime/Control'


def load_module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


arm = load_module('closed_loop_arm', CONTROL/'ros2_ws/tools/run_arm_stage.py')
cycle = load_module('closed_loop_cycle', CONTROL/'tools/run_grasp_cycle.py')


@pytest.fixture
def demo(tmp_path, monkeypatch):
    (tmp_path/'execution_contract.json').write_bytes((ROOT/'execution_contract.json').read_bytes())
    monkeypatch.setattr(bm, 'APP', tmp_path)
    monkeypatch.setattr(Backend, '_run', Mock(side_effect=AssertionError('No external device command')))
    return runner.prepare_method(mode='demo')


def test_prepare_binds_shared_policy_and_unverified_hardware_contract(demo):
    assert demo['method_id'] == 'method02_shared_progress'
    assert demo['scheduling_policy'] == 'shared_progress_v1'
    assert demo['execution_contract']['deployment_verified'] is False
    assert demo['physical_motion_validated'] is False
    assert demo['execution_contract_sha256'] == bm.digest(bm.APP/'execution_contract.json')
    text = (Path(demo['run_dir'])/'summary.txt').read_text()
    assert '共享轨迹相位' in text and '非硬件同步' in text
    assert not (Path(demo['run_dir'])/'method_approval.json').exists()
    Backend._run.assert_not_called()


def test_contract_change_prevents_startup_and_motion(demo, monkeypatch):
    path = bm.APP/'execution_contract.json'
    contract = json.loads(path.read_text())
    contract['deployment_verified'] = True
    path.write_text(json.dumps(contract))
    startup = Mock(side_effect=AssertionError('Changed contract cannot start devices'))
    monkeypatch.setattr(Backend, 'start_cameras', startup)
    with pytest.raises(ValueError, match='不一致'):
        runner.execute_method(Path(demo['run_dir'])/'method_review.json', 'synthetic test only', mode='demo')
    startup.assert_not_called()


def test_demo_completion_is_explicitly_synthetic_and_one_shot(demo):
    path = Path(demo['run_dir'])/'method_review.json'
    result = runner.execute_method(path, '纯软件合成测试，无实物授权', mode='demo')
    assert result['completed'] and result['synthetic']
    assert result['physical_motion_validated'] is False
    assert result['commands'] == result['receipts'] == 0
    with pytest.raises((ValueError, FileExistsError)):
        runner.execute_method(path, 'pure software replay must fail', mode='demo')
    Backend._run.assert_not_called()


def test_shared_policy_is_bound_to_fixed_arm_source(tmp_path):
    source = tmp_path/'plan.json'
    source.write_text(json.dumps(dict(scheduling_policy='shared_progress_v1', progress_policy='feedback_paced')))
    manifest = dict(spec=dict(fixed_ready=True, scheduling_policy='shared_progress_v1'), stages=[
        dict(name='prepare_gripper'),
        dict(name='fixed_pregrasp', plan_file=str(source), scheduling_policy='shared_progress_v1')])
    cycle.validate_scheduling_policies(manifest)
    assert arm.source_scheduling_policy(json.loads(source.read_text()), 'fixed_pregrasp') == 'shared_progress_v1'
    manifest['spec']['fixed_ready'] = False
    with pytest.raises(ValueError):
        cycle.validate_scheduling_policies(manifest)
    with pytest.raises(ValueError):
        arm.source_scheduling_policy(json.loads(source.read_text()), 'approach')
    manifest['spec']['fixed_ready'] = True
    manifest['stages'][0]['scheduling_policy'] = 'shared_progress_v1'
    with pytest.raises(ValueError, match='gripper'):
        cycle.validate_scheduling_policies(manifest)


def test_shared_source_cannot_silently_fall_back_to_old_executor(tmp_path):
    source = tmp_path/'plan.json'
    source.write_text(json.dumps(dict(scheduling_policy='ack_overlap_v1', progress_policy='feedback_paced')))
    manifest = dict(spec=dict(fixed_ready=True, scheduling_policy='shared_progress_v1'), stages=[
        dict(name='fixed_pregrasp', plan_file=str(source), scheduling_policy='shared_progress_v1')])
    with pytest.raises(ValueError, match='differs'):
        cycle.validate_scheduling_policies(manifest)


def test_failed_execution_still_generates_automatic_metrics_without_altering_result(tmp_path, monkeypatch):
    tool=tmp_path/'tools/analyze_execution.py'
    tool.parent.mkdir()
    tool.write_bytes((ROOT/'tools/analyze_execution.py').read_bytes())
    monkeypatch.setattr(bm,'APP',tmp_path)
    folder=tmp_path/'logs/synthetic_run'
    stage=folder/'execution/01_fixed_pregrasp/executed'
    stage.mkdir(parents=True)
    plan=dict(sdk_plan=dict(sdk=[[0.]*6,[1.,0.,0.,0.,0.,0.]],ticks=[10],rate=10.,
                           duration_s=1.,timing_profile='edge_quintic_v1'))
    result=dict(completed=False,error='Endpoint position not reached',
                reference_epoch_monotonic_s=10.,command_group_schedule=[])
    (stage/'plan.json').write_text(json.dumps(plan))
    (stage/'result.json').write_text(json.dumps(result))
    (stage/'transport.jsonl').write_text('')
    original=(stage/'result.json').read_bytes()
    summary=dict(completed=False,stage_reports=[dict(path=str(stage/'result.json'))])
    reports=runner.collect_execution_metrics(dict(run_dir=str(folder)),summary)
    assert len(reports)==1 and reports[0]['completed'] is False
    assert Path(reports[0]['report']).is_file()
    assert json.loads(Path(reports[0]['metrics']).read_text())['completed'] is False
    assert summary['completed'] is False
    assert (stage/'result.json').read_bytes()==original
    again=runner.collect_execution_metrics(dict(run_dir=str(folder)),summary)
    assert 'error' in again[0]  # Existing analysis is preserved, not silently overwritten.


def test_reference_speed_reserve_is_visible_in_review(demo):
    control=demo['progress_controller']
    assert control['max_phase_rate']==pytest.approx(.8)
    assert control['effective_reference_peak_speed_upper_bound_deg_s']==pytest.approx(4.)
    text=(Path(demo['run_dir'])/'summary.txt').read_text()
    assert '参考峰值速度上界 4°/s' in text


def test_parent_timeout_covers_reviewed_shared_progress_budget():
    stage=dict(name='fixed_pregrasp',scheduling_policy='shared_progress_v1')
    assert cycle.stage_process_timeout(stage,120.)>=4*120.+30.+5.
    assert cycle.stage_process_timeout(dict(name='fixed_pregrasp'),120.)==165.


def test_unavailable_live_interface_prevents_sdk_preparation(monkeypatch):
    def down():raise RuntimeError('D1通信网口 enp99s0 未连接')
    monkeypatch.setattr(runner,'check_live_interface',down)
    prepare=Mock(side_effect=AssertionError('No SDK work when the interface is unavailable'))
    monkeypatch.setattr(Backend,'prepare',prepare)
    with pytest.raises(RuntimeError,match='网口 enp99s0 未连接'):
        runner.prepare_method(mode='live')
    prepare.assert_not_called()
