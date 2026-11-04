"""Integration contracts exercised with mocked observers/planners/transports.

No subprocess, camera, network or robot client is created by these tests.
"""
import copy
import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace
import sys
import time

import pytest

APP = Path(__file__).resolve().parents[1]
CONTROL = APP / 'runtime/Control'
sys.path.insert(0, str(CONTROL/'tools'))
sys.path.insert(0, str(APP))
_spec = importlib.util.spec_from_file_location('one_click_cycle', CONTROL/'tools/run_grasp_cycle.py')
cycle = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(cycle)
import gripper_stage
from app.backend import Backend, Cancelled as BackendCancelled


def transform(z=0.):
    return [[1., 0., 0., 0.], [0., 1., 0., 0.], [0., 0., 1., z], [0., 0., 0., 1.]]


@pytest.fixture
def workflow(tmp_path, monkeypatch):
    spec = dict(include_pregrasp=True, include_prepare_gripper=True, speed_deg_s=3., acceleration_deg_s2=6.,
                gripper_speed_sdk_s=2., rate_hz=5., lift_height_m=.03,
                retreat_height_m=.08, keep_start_orientation=True,
                open_target=30., close_target=28., pose=str(tmp_path/'pose.json'),
                yaw_deg=90., style='top', grasp_height_offset_m=.01,
                timing_profile='collinear_bounded_v1', candidate=str(tmp_path/'mock_mapping.json'))
    q = [0., 0., 0., 0., 0., 0., 30.]
    stages = []
    geometry = dict(pregrasp=(.15, .1), approach=(.1, .02), lift=(.02, .05),
                    lower=(.05, .023), retreat=(.023, .103))
    for name in cycle.stage_order(spec):
        item = dict(name=name, expected_angles=q.copy())
        if name in cycle.GRIPPER_STAGES:
            item['gripper_target'] = spec['close_target' if name == 'close' else 'open_target']
            item['gripper_plan'] = gripper_stage.plan(item['gripper_target'], q, speed=2., rate=5., segmented=name == 'prepare_gripper')
            q[6] = item['gripper_target']
        else:
            endpoint = q[:6].copy()
            endpoint[0] += 1.
            plan_file = tmp_path/(name+'_plan.json')
            plan_file.write_text(json.dumps({'completed': True, 'stage': name}))
            start_z, end_z = geometry[name]
            item.update(gripper_target=q[6], plan_file=str(plan_file),
                sdk_plan=dict(sdk=[q[:6].copy(), endpoint], rate=5., max_velocity_deg_s=3.,
                              max_acceleration_deg_s2=6., duration_s=1.,
                              timing_profile='collinear_bounded_v1'),
                cube=transform(.02), table=transform(), start_tcp=transform(start_z),
                target_tcp=transform(end_z), attachment=transform() if name in ('lift', 'lower') else None)
            q[:6] = endpoint
        stages.append(item)
    immutable = tmp_path/'source.txt'
    immutable.write_text('bound source')
    manifest = dict(schema_version=1, created_at=time.time()-1., expires_at=time.time()+600.,
        spec=spec, limits=cycle.approved_limits(spec), stages=stages,
        initial_angles=[0., 0., 0., 0., 0., 0., 30.],
        target={'T_base_cube': transform(.02), 'T_base_table': transform()},
        place_release_gap_m=.003, continuation_policy='reviewed_path', continuation_envelope_deg=1.,
        accept_stable_close_shortfall=False, replay_only=False, execution_candidate_eligible=True,
        physical_grasp_verification='automatic_opencv_two_hypothesis_v1',
        files={str(immutable): cycle.digest(immutable),
               **{item['plan_file']: cycle.digest(item['plan_file'])
                  for item in stages if 'plan_file' in item}})
    path = tmp_path/'manifest.json'
    cycle.save(path, manifest)
    args = SimpleNamespace(manifest=path, approval_sha256=cycle.digest(path),
        approval_text='I approve this displayed target, path and parameters',
        output=tmp_path/'execution', stop_file=tmp_path/'STOP', approval_wait_json=None)
    state = SimpleNamespace(q=manifest['initial_angles'].copy(), commands=[], plans=[], observations=[],
        lift_status='success', return_status='success', return_calls=0,
        fail_stage=None, cancel_after=None, immutable=immutable,
        manifest=manifest, args=args, stages=stages, enable_status=None, disable_after=None,
        observation_offsets={}, mutate_plan_on_observe=None, lift_plan_paths=[], lift_verification_args=[])
    by_name = {item['name']: item for item in stages}
    state.by_name = by_name

    def observe(folder, allow_position_start=False):
        if state.enable_status == 0 and not allow_position_start:
            raise RuntimeError('Invalid/stale enable_status')
        folder = Path(folder)
        folder.mkdir(parents=True, exist_ok=True)
        path = folder/'wrist_observation.json'
        observed = state.q.copy()
        name = folder.name.split('_', 1)[-1]
        for joint, offset in state.observation_offsets.get(name, {}).items():
            observed[joint] += offset
        if name == state.mutate_plan_on_observe:
            Path(by_name['pregrasp']['plan_file']).write_text('changed after initial validation')
        observation = {'sdk_angles': observed, 'hardware_commands_sent': 0}
        if state.enable_status is not None:
            observation.update(status_fresh=True,
                status={'power_status': 1, 'enable_status': state.enable_status, 'error_status': 0})
        cycle.save(path, observation)
        state.observations.append(folder)
        return observed, path

    def plan_arm(name, angles, spec, folder, attachment=None):
        state.plans.append((name, copy.deepcopy(spec)))
        raise AssertionError('Execution must use the approved plan, never invoke planning again')

    def run_command(argv, folder, timeout, allow_failure=False):
        command = list(argv)
        assert '--execute' in command, 'Unexpected real planning/observation subprocess'
        if Path(command[0]) == cycle.ARM:
            name = command[command.index('--stage')+1]
        else:
            assert Path(command[0]) == cycle.GRIPPER
            target = command[command.index('--target')+1]
            name = ('prepare_gripper' if '--segmented' in command else
                    'close' if target == spec['close_target'] else 'open')
        state.commands.append((name, command))
        item = by_name[name]
        commands = 0 if name == 'prepare_gripper' and state.q[6] == item['gripper_target'] else 1
        if name in cycle.GRIPPER_STAGES:
            state.q[6] = item['gripper_target']
        else:
            state.q[:6] = item['sdk_plan']['sdk'][-1]
        output = Path(command[command.index('--output')+1])
        output.mkdir(parents=True)
        filename = 'summary.json' if name in cycle.GRIPPER_STAGES else 'result.json'
        cycle.save(output/filename, dict(completed=name != state.fail_stage,
            final_angles=state.q.copy(), commands=commands,
            error='mock stage failure' if name == state.fail_stage else None))
        if name == state.disable_after:
            state.enable_status = 0
        if name == state.cancel_after:
            args.stop_file.touch()
        if name == state.fail_stage:
            raise RuntimeError('Mock execution failed: '+name)
        return 0

    def verify_lift(*args, **kwargs):
        state.lift_plan_paths.append(kwargs.get('lift_plan_path'))
        state.lift_verification_args.append(kwargs)
        return {'status': state.lift_status, 'reason': 'mock visual evidence'}

    def verify_return(*args, **kwargs):
        state.return_calls += 1
        return {'status': state.return_status, 'reason': 'mock visual evidence'}

    monkeypatch.setattr(cycle, 'observe', observe)
    monkeypatch.setattr(cycle, 'plan_arm', plan_arm)
    monkeypatch.setattr(cycle, 'run_command', run_command)
    monkeypatch.setattr(cycle, 'verify_lift', verify_lift)
    monkeypatch.setattr(cycle, 'verify_return', verify_return)
    monkeypatch.setattr(cycle, '_STOP_FILE', None)
    return state


def test_full_cycle_order_without_manual_gate_and_with_speed_contract(workflow):
    result = cycle.execute_cycle(workflow.args)
    assert result['completed'] and result['trajectory_completed']
    assert result['physical_grasp_verified'] and result['physical_cycle_verified']
    assert [name for name, _ in workflow.commands] == cycle.stage_order(workflow.manifest['spec'])
    assert not hasattr(cycle, 'await_visual_gate') and not hasattr(cycle, 'verify_gate')
    assert workflow.plans == []
    approved = {item['name']: item for item in workflow.stages}
    for name, command in workflow.commands:
        assert command[command.index('--stop-file')+1] == workflow.args.stop_file
        if name in cycle.GRIPPER_STAGES:
            assert command[command.index('--speed-sdk-s')+1] == 2.
            assert command[command.index('--arm-speed-deg-s')+1] == 3.
            assert command[command.index('--rate-hz')+1] == 5.
        else:
            assert Path(command[command.index('--plan')+1]) == Path(approved[name]['plan_file'])
            assert command[command.index('--approved-plan-sha256')+1] == workflow.manifest['files'][approved[name]['plan_file']]
            assert command[command.index('--candidate')+1] == workflow.manifest['spec']['candidate']
            assert approved[name]['sdk_plan']['max_velocity_deg_s'] == 3.
            assert approved[name]['sdk_plan']['max_acceleration_deg_s2'] == 6.
    assert workflow.lift_plan_paths == [Path(approved['lift']['plan_file'])]
    lift_start = workflow.lift_verification_args[0]['lift_start_observation_path']
    assert cycle.read(lift_start)['sdk_angles'] == approved['lift']['expected_angles']


@pytest.mark.parametrize('arguments,expected',[([],10.),(['--rate-hz','5'],5.)])
def test_cycle_cli_rate_defaults_to_ten_without_overriding_explicit_rate(tmp_path,monkeypatch,arguments,expected):
    rates=[]
    monkeypatch.setattr(cycle,'prepare',lambda args:rates.append(args.rate_hz) or dict(completed=True))
    assert cycle.main(['--output',str(tmp_path/'plan'),'--expected-angles',*['0']*7,*arguments])==0
    assert rates==[expected]
    assert cycle.LIMITS['rate_hz']==10.


def test_pregrasp_scope_executes_only_approved_prefix_then_stops(workflow):
    workflow.args.execution_scope = 'pregrasp'
    result = cycle.execute_cycle(workflow.args)
    assert result['completed'] and result['pregrasp_reached']
    assert result['execution_scope'] == 'pregrasp'
    assert result['completed_stages'] == result['requested_stages'] == ['prepare_gripper', 'pregrasp']
    assert [name for name, _ in workflow.commands] == ['prepare_gripper', 'pregrasp']
    assert result['final_angles'] == workflow.q
    assert not result['trajectory_completed'] and not result['physical_grasp_verified']
    assert result['grasp_result'] == result['cycle_result'] == 'not_attempted'
    assert not workflow.plans and not workflow.lift_verification_args and workflow.return_calls == 0
    approved = next(stage for stage in workflow.stages if stage['name'] == 'pregrasp')
    command = workflow.commands[-1][1]
    assert command[command.index('--plan')+1] == approved['plan_file']
    assert command[command.index('--approved-plan-sha256')+1] == workflow.manifest['files'][approved['plan_file']]
    assert cycle.read(workflow.args.output/'result.json')['timing']
    assert cycle.read(Path(str(workflow.args.manifest)+'.consumed'))['execution_scope'] == 'pregrasp'


@pytest.mark.parametrize('stage', ['prepare_gripper', 'pregrasp'])
def test_pregrasp_scope_failure_never_reaches_later_stages(workflow, stage):
    workflow.args.execution_scope = 'pregrasp'
    workflow.fail_stage = stage
    result = cycle.execute_cycle(workflow.args)
    assert not result['completed'] and not result['pregrasp_reached']
    assert result['grasp_result'] == result['cycle_result'] == 'not_attempted'
    assert [name for name, _ in workflow.commands] == (['prepare_gripper'] if stage == 'prepare_gripper' else ['prepare_gripper', 'pregrasp'])
    assert not workflow.lift_verification_args and not workflow.return_calls


@pytest.mark.parametrize('stage', ['prepare_gripper', 'pregrasp'])
def test_pregrasp_scope_honors_stop_even_at_last_stage(workflow, stage):
    workflow.args.execution_scope = 'pregrasp'
    workflow.cancel_after = stage
    result = cycle.execute_cycle(workflow.args)
    assert result['cancelled'] and not result['pregrasp_reached'] and not result['completed']
    assert [name for name, _ in workflow.commands] == (['prepare_gripper'] if stage == 'prepare_gripper' else ['prepare_gripper', 'pregrasp'])


def test_pregrasp_claim_cannot_be_reused_for_full_cycle(workflow):
    workflow.args.execution_scope = 'pregrasp'
    assert cycle.execute_cycle(workflow.args)['completed']
    workflow.args.execution_scope = 'full_cycle'
    with pytest.raises(FileExistsError):
        cycle.execute_cycle(workflow.args)
    assert [name for name, _ in workflow.commands] == ['prepare_gripper', 'pregrasp']


def test_pregrasp_requires_a_manifest_with_pregrasp_before_claim_or_motion(workflow):
    workflow.manifest['spec']['include_pregrasp'] = False
    workflow.manifest['stages'] = [stage for stage in workflow.stages if stage['name'] != 'pregrasp']
    cycle.save(workflow.args.manifest, workflow.manifest)
    workflow.args.approval_sha256 = cycle.digest(workflow.args.manifest)
    workflow.args.execution_scope = 'pregrasp'
    with pytest.raises(ValueError, match='缺少预抓取阶段'):
        cycle.execute_cycle(workflow.args)
    assert not workflow.commands
    assert not Path(str(workflow.args.manifest)+'.consumed').exists()


def test_pregrasp_scope_still_rejects_changed_approved_file(workflow):
    workflow.args.execution_scope = 'pregrasp'
    workflow.mutate_plan_on_observe = 'pregrasp'
    result = cycle.execute_cycle(workflow.args)
    assert not result['completed'] and 'Reviewed file changed' in result['error']
    assert [name for name, _ in workflow.commands] == ['prepare_gripper']


def make_fixed_ready(workflow):
    import math
    profile_path=APP/'app/fixed_ready_pose.json'
    profile=cycle.read(profile_path)
    candidate=Path(workflow.manifest['spec']['candidate'])
    cycle.save(candidate,dict(mapping=dict(order=[f'J{i}->Joint{i+1}' for i in range(6)],
        signs=[-1,1,1,-1,1,1],zero_offsets_deg=[0]*6)))
    target=[math.degrees(value) for value in profile['model_target_rad']]
    arm=workflow.stages[1];arm['name']='fixed_pregrasp';arm['sdk_plan']['sdk'][-1]=target
    workflow.by_name['fixed_pregrasp']=workflow.by_name.pop('pregrasp')
    manifest=workflow.manifest
    manifest['spec']['fixed_ready']=True
    manifest.update(operation='fixed_pregrasp',stages=workflow.stages[:2],
        fixed_sdk_target_deg=target,fixed_model_target_rad=profile['model_target_rad'])
    manifest['files'].update({str(profile_path.resolve()):cycle.digest(profile_path),str(candidate):cycle.digest(candidate)})
    workflow.args.execution_scope='fixed_pregrasp'
    return target


def save_fixed_ready(workflow):
    cycle.save(workflow.args.manifest,workflow.manifest)
    workflow.args.approval_sha256=cycle.digest(workflow.args.manifest)


def test_fixed_ready_executes_only_fixed_joint_goal_then_stops(workflow):
    target=make_fixed_ready(workflow);save_fixed_ready(workflow)
    result=cycle.execute_cycle(workflow.args)
    assert result['completed'] and result['pregrasp_reached']
    assert result['final_angles'][:6]==target
    assert [name for name,_ in workflow.commands]==['prepare_gripper','fixed_pregrasp']
    assert result['grasp_result']==result['cycle_result']=='not_attempted'
    assert not workflow.plans and not workflow.lift_verification_args and not workflow.return_calls


def test_fixed_ready_rejects_wrong_fixed_endpoint_before_any_command(workflow):
    make_fixed_ready(workflow)
    workflow.stages[1]['sdk_plan']['sdk'][-1][1]+=.02
    save_fixed_ready(workflow)
    with pytest.raises(ValueError,match='Gazebo ready'):
        cycle.execute_cycle(workflow.args)
    assert not workflow.commands


def test_fixed_ready_manifest_cannot_start_full_grasp(workflow):
    make_fixed_ready(workflow);save_fixed_ready(workflow)
    workflow.args.execution_scope='full_cycle'
    with pytest.raises(ValueError,match='不能混用'):
        cycle.execute_cycle(workflow.args)
    assert not workflow.commands


def test_already_at_fixed_ready_checks_fresh_feedback_without_arm_commands(workflow):
    target=make_fixed_ready(workflow)
    q=target+[workflow.q[6]];workflow.q=q.copy();workflow.manifest['initial_angles']=q.copy()
    for stage in workflow.manifest['stages']:stage['expected_angles']=q.copy()
    arm=workflow.stages[1];arm.update(already_at_fixed_target=True)
    arm['sdk_plan']['sdk']=[target.copy(),target.copy()]
    save_fixed_ready(workflow)
    result=cycle.execute_cycle(workflow.args)
    assert result['completed'] and result['pregrasp_reached']
    assert [name for name,_ in workflow.commands]==['prepare_gripper']
    assert result['stage_outcomes']['fixed_pregrasp']['already_at_fixed_target']


@pytest.mark.parametrize('stage', ['pregrasp', 'approach', 'lift', 'lower', 'retreat'])
def test_arm_stage_small_start_difference_uses_original_approved_plan_without_replanning(workflow, stage):
    workflow.observation_offsets[stage] = {0: 1.0}
    result = cycle.execute_cycle(workflow.args)
    assert result['completed'], result.get('error')
    assert workflow.plans == []
    approved = next(item for item in workflow.stages if item['name'] == stage)
    command = next(argv for name, argv in workflow.commands if name == stage)
    assert Path(command[command.index('--plan')+1]) == Path(approved['plan_file'])
    assert command[command.index('--expected-angles')+1] == pytest.approx(approved['expected_angles'][0]+1.0)


@pytest.mark.parametrize('stage', ['pregrasp', 'approach', 'lift', 'lower', 'retreat'])
def test_arm_stage_start_difference_over_one_degree_stops_before_that_stage(workflow, stage):
    workflow.observation_offsets[stage] = {0: 1.001}
    result = cycle.execute_cycle(workflow.args)
    assert not result['completed']
    assert 'start' in result['error'].lower()
    assert workflow.plans == []
    order = cycle.stage_order(workflow.manifest['spec'])
    assert [name for name, _ in workflow.commands] == order[:order.index(stage)]


def test_gripper_difference_above_point_five_stops_before_approved_arm_stage(workflow):
    workflow.observation_offsets['approach'] = {6: .51}
    result = cycle.execute_cycle(workflow.args)
    assert not result['completed']
    assert [name for name, _ in workflow.commands] == ['prepare_gripper', 'pregrasp']
    assert workflow.plans == []


def test_bound_plan_modified_after_initial_validation_is_rejected_before_sending(workflow):
    workflow.mutate_plan_on_observe = 'prepare_gripper'
    result = cycle.execute_cycle(workflow.args)
    assert not result['completed']
    assert 'changed' in result['error'].lower()
    assert workflow.commands == []
    assert workflow.plans == []


def test_legacy_task_geometry_cannot_trigger_replanning_or_execute(workflow):
    manifest = cycle.read(workflow.args.manifest)
    manifest['continuation_policy'] = 'task_geometry'
    cycle.save(workflow.args.manifest, manifest)
    workflow.args.approval_sha256 = cycle.digest(workflow.args.manifest)
    with pytest.raises(ValueError):
        cycle.execute_cycle(workflow.args)
    assert workflow.commands == []
    assert workflow.plans == []


@pytest.mark.parametrize('status', ['failed', 'unknown'])
def test_lift_visual_failure_or_unknown_stops_before_lower_without_retry(workflow, status):
    workflow.lift_status = status
    result = cycle.execute_cycle(workflow.args)
    assert [name for name, _ in workflow.commands] == ['prepare_gripper', 'pregrasp', 'approach', 'close', 'lift']
    assert not result['completed'] and not result['trajectory_completed']
    assert result['grasp_result'] == status and result['physical_grasp_verified'] is False
    assert workflow.return_calls == 0
    assert result['state'] == 'stopped_no_recovery'


@pytest.mark.parametrize('status', ['failed', 'unknown'])
def test_return_visual_failure_or_unknown_does_not_claim_completed_cycle(workflow, status):
    workflow.return_status = status
    result = cycle.execute_cycle(workflow.args)
    assert result['trajectory_completed'] is True and result['completed'] is False
    assert result['grasp_result'] == 'success' and result['cycle_result'] == status
    assert result['physical_cycle_verified'] is False
    assert 'Return visual result' in result['error']
    assert len(workflow.commands) == 8


def test_failed_stage_has_no_retry_and_no_following_stage(workflow):
    workflow.fail_stage = 'lift'
    result = cycle.execute_cycle(workflow.args)
    assert [name for name, _ in workflow.commands] == ['prepare_gripper', 'pregrasp', 'approach', 'close', 'lift']
    assert result['completed'] is False and result['physical_grasp_verified'] is False
    assert workflow.return_calls == 0


def test_failed_initial_gripper_adjustment_never_starts_arm(workflow):
    workflow.fail_stage = 'prepare_gripper'
    result = cycle.execute_cycle(workflow.args)
    assert [name for name, _ in workflow.commands] == ['prepare_gripper']
    assert not workflow.plans
    assert result['completed'] is False
    assert workflow.return_calls == 0


def test_initial_noop_preserves_start_permission_only_until_first_real_motion(workflow):
    result = cycle.execute_cycle(workflow.args)
    assert result['completed']
    permissions = {name: '--allow-position-start' in argv for name, argv in workflow.commands}
    assert permissions['prepare_gripper'] and permissions['pregrasp']
    assert all(not permission for name, permission in permissions.items()
               if name not in ('prepare_gripper', 'pregrasp'))
    assert result['position_start_pending'] is False


def test_fresh_enabled_status_consumes_start_permission_before_any_motion(workflow):
    workflow.enable_status = 1
    assert cycle.execute_cycle(workflow.args)['completed']
    assert all('--allow-position-start' not in argv for _, argv in workflow.commands)


def test_enable_drop_after_first_real_motion_cannot_restart_position_mode(workflow):
    workflow.disable_after = 'pregrasp'
    result = cycle.execute_cycle(workflow.args)
    assert not result['completed']
    assert [name for name, _ in workflow.commands] == ['prepare_gripper', 'pregrasp']
    assert 'enable_status' in result['error']
    assert result['position_start_pending'] is False


def test_start_allowance_is_consumed_by_real_command_log_even_when_result_count_missing(tmp_path):
    log = tmp_path/'transport.jsonl'
    log.write_text(json.dumps({'kind': 'output', 'line': '[rx] status'})+'\n')
    assert not cycle.position_commands_recorded({}, tmp_path)
    with log.open('a') as stream:
        stream.write(json.dumps({'kind': 'command', 'joint': 6, 'target_sdk_units': 6.6})+'\n')
    assert cycle.position_commands_recorded({}, tmp_path)


def test_approved_target_is_rechecked_before_initial_gripper_adjustment(workflow, monkeypatch):
    manifest = cycle.read(workflow.args.manifest)
    perception = workflow.args.manifest.parent/'perception.json'
    cycle.save(perception, {'source_kind': 'live_camera', 'execution_candidate_eligible': True})
    manifest.update(recognition_report=str(perception), recognition_refresh={'enabled': True},
                    target_recheck=cycle.TARGET_RECHECK)
    cycle.save(workflow.args.manifest, manifest)
    workflow.args.approval_sha256 = cycle.digest(workflow.args.manifest)
    monkeypatch.setattr(cycle, 'import_recognition', lambda *args: None)
    calls = []
    def recheck(*args):
        assert not workflow.commands
        calls.append(args[1].name)
        return workflow.q.copy(), perception, {'target_updated': False}
    monkeypatch.setattr(cycle, 'recheck_target', recheck)
    result = cycle.execute_cycle(workflow.args)
    assert result['completed']
    assert calls == ['00_prepare_gripper']
    assert workflow.commands[0][0] == 'prepare_gripper'


@pytest.mark.parametrize('initial_j6', [6.5, 43.9, 36.])
def test_prepare_manifest_reviews_initial_adjustment_before_arm_without_commands(tmp_path, monkeypatch, initial_j6):
    config = tmp_path/'config.yaml'
    config.write_text('cube_size_m: 0.04\nhardware_mapping_candidate: unused\nplace_release_gap_m: 0.003\n')
    pose = tmp_path/'pose.json'
    cycle.save(pose, {'cube_size_m': .04, 'T_base_cube': transform(.02),
                      'T_base_table': transform(), 'source_kind': 'saved_image'})
    monkeypatch.setattr(cycle, 'CONFIG', config)
    monkeypatch.setattr(cycle, 'digest', lambda path: 'mock-read-only-source-hash')
    planned_starts = []
    def plan_arm(name, current, spec, folder, attachment=None):
        planned_starts.append((name, list(current)))
        assert current[6] == (29.6 if name in ('lift', 'lower') else 36.)
        return dict(name=name, expected_angles=current.copy(), gripper_target=current[6],
                    plan_file=str(tmp_path/(name+'.json')),
                    sdk_plan={'sdk': [current[:6], current[:6]], 'duration_s': 1.})
    monkeypatch.setattr(cycle, 'plan_arm', plan_arm)
    def no_process(*args, **kwargs):
        raise AssertionError('Planning must not create any execution subprocess')
    monkeypatch.setattr(cycle, 'run_command', no_process)
    args = SimpleNamespace(expected_angles=[0., 0., 0., 0., 0., 0., initial_j6],
        pose=pose, close_target=29.6, open_target=36., valid_for_s=600.,
        verification_timeout_s=30., continuation_envelope_deg=1., retreat_height_m=.08,
        lift_height_m=.03, output=tmp_path/'review', yaw_deg=90., style='top',
        grasp_height_offset_m=.01, keep_start_orientation=True, speed_deg_s=3.,
        acceleration_deg_s2=6., rate_hz=5., gripper_speed_sdk_s=2.,
        include_pregrasp=True, accept_stable_close_shortfall=False)
    result = cycle.prepare(args)
    manifest = cycle.read(result['manifest'])
    first = manifest['stages'][0]
    assert first['name'] == 'prepare_gripper'
    assert first['expected_angles'][6] == initial_j6
    assert first['gripper_plan']['target_sdk_units'] == 36.
    assert first['gripper_plan']['segmented'] is True
    assert planned_starts[0][1][6] == 36.
    assert result['hardware_commands_sent'] is False


def test_stop_file_stops_before_next_stage_and_keeps_failure_log(workflow):
    workflow.cancel_after = 'close'
    result = cycle.execute_cycle(workflow.args)
    assert [name for name, _ in workflow.commands] == ['prepare_gripper', 'pregrasp', 'approach', 'close']
    assert result['cancelled'] and result['physical_stop_verified'] is False
    assert cycle.read(workflow.args.output/'result.json')['completed'] is False


@pytest.mark.parametrize('mutation', ['manifest', 'source', 'plan', 'expired', 'shortfall'])
def test_invalid_manifest_cannot_execute(workflow, mutation):
    if mutation == 'manifest':
        workflow.args.manifest.write_text(workflow.args.manifest.read_text()+'\n')
    elif mutation == 'source':
        workflow.immutable.write_text('changed source')
    elif mutation == 'plan':
        Path(next(item['plan_file'] for item in workflow.stages if 'plan_file' in item)).write_text('changed approved path')
    else:
        value = cycle.read(workflow.args.manifest)
        if mutation == 'expired':
            value['created_at'] = time.time()-100.
            value['expires_at'] = time.time()-1.
        else:
            value['accept_stable_close_shortfall'] = True
        cycle.save(workflow.args.manifest, value)
        workflow.args.approval_sha256 = cycle.digest(workflow.args.manifest)
    with pytest.raises(ValueError):
        cycle.execute_cycle(workflow.args)
    assert not workflow.commands


def test_used_manifest_cannot_execute_again(workflow):
    assert cycle.execute_cycle(workflow.args)['completed']
    count = len(workflow.commands)
    workflow.args.output = workflow.args.output.parent/'second_execution'
    with pytest.raises(FileExistsError):
        cycle.execute_cycle(workflow.args)
    assert len(workflow.commands) == count


def test_planner_call_receives_adjustable_speed_parameters(tmp_path, monkeypatch):
    commands = []
    def fake_run(argv, folder, timeout, allow_failure=False):
        argv = list(argv)
        commands.append(argv)
        output = Path(argv[argv.index('--output')+1])
        output.mkdir(parents=True)
        if Path(argv[0]) == cycle.PLANNER:
            cycle.save(output/'plan.json', dict(completed=True, T_base_target_tcp=transform(.05),
                T_base_start_tcp=transform(.02), T_tcp_cube=transform(),
                T_base_cube=transform(.02), T_base_table=transform()))
        else:
            cycle.save(output/'plan.json', {'sdk_plan': {'duration_s': 1.}})
    monkeypatch.setattr(cycle, 'run_command', fake_run)
    spec = dict(pose='unused', yaw_deg=90., style='top', grasp_height_offset_m=.01,
        lift_height_m=.03, retreat_height_m=.08, keep_start_orientation=True,
        open_target=30., close_target=28., speed_deg_s=8., acceleration_deg_s2=12.,
        gripper_speed_sdk_s=4., rate_hz=10.)
    cycle.plan_arm('lift', [0., 0., 0., 0., 0., 0., 28.], spec, tmp_path/'plan')
    assert commands[0][commands[0].index('--speed-deg-s')+1] == 8.
    assert commands[0][commands[0].index('--acceleration-deg-s2')+1] == 12.
    assert commands[0][commands[0].index('--rate-hz')+1] == 10.


def test_backend_rejects_modified_expired_and_reused_review(tmp_path):
    backend = Backend('live')
    backend.run_dir = tmp_path
    path = tmp_path/'manifest.json'
    path.write_text('{}')
    review = dict(mode='live', ready=True, manifest=str(path), approval_sha256=cycle.digest(path),
                  expires_at=time.time()+600., run_dir=str(tmp_path))
    backend._review = copy.deepcopy(review)
    backend._validate_review(review, 'approve displayed path')
    changed = dict(review, ready=False)
    with pytest.raises(ValueError):
        backend._validate_review(changed, 'approve')
    backend._used.add(review['approval_sha256'])
    with pytest.raises(ValueError, match='已使用'):
        backend._validate_review(review, 'approve')
    backend._used.clear()
    backend._review['expires_at'] = time.time()-1.
    with pytest.raises(ValueError, match='过期'):
        backend._validate_review(copy.deepcopy(backend._review), 'approve')


@pytest.fixture
def backend_run(tmp_path, monkeypatch):
    backend = Backend('live')
    backend.run_dir = tmp_path
    path = tmp_path/'manifest.json'
    path.write_text('{}')
    review = dict(mode='live', ready=True, manifest=str(path), approval_sha256=cycle.digest(path),
                  expires_at=time.time()+600., run_dir=str(tmp_path), parameters={})
    backend._review = copy.deepcopy(review)
    monkeypatch.setattr(backend, '_claim_device', lambda: None)
    monkeypatch.setattr(backend, '_release_device', lambda: None)
    return backend, review


@pytest.mark.parametrize('raw,expected', [
    ({}, 'failed'),
    ({'completed': False, 'trajectory_completed': True, 'grasp_result': 'success',
      'cycle_result': 'unknown'}, 'unconfirmed'),
    ({'completed': True, 'trajectory_completed': True, 'grasp_result': 'success',
      'cycle_result': 'unknown'}, 'unconfirmed'),
])
def test_backend_exit_zero_empty_or_unknown_result_is_never_completed(backend_run, monkeypatch, raw, expected):
    backend, review = backend_run
    folder = Path(review['run_dir'])
    def fake_run(argv, log_path, timeout):
        execution = folder/'execution'
        execution.mkdir()
        cycle.save(execution/'result.json', raw)
        # Successful return simulates a child that exited with code zero.
    monkeypatch.setattr(backend, '_run', fake_run)
    result = backend.execute(review, 'approve this displayed plan')
    assert result['status'] == expected
    assert result['status'] != 'completed'
    assert cycle.read(folder/'result.json')['status'] == expected
    assert (folder/'report.html').is_file()


def test_backend_stop_file_cancellation_preserves_actual_partial_logs(backend_run, monkeypatch):
    backend, review = backend_run
    folder = Path(review['run_dir'])
    def fake_run(argv, log_path, timeout):
        execution = folder/'execution'
        stage = execution/'02_close/executed'
        stage.mkdir(parents=True)
        raw = dict(completed=False, trajectory_completed=False, cancelled=True,
                   state='stopped_no_recovery', grasp_result='unknown', physical_stop_verified=False)
        cycle.save(execution/'result.json', raw)
        snapshot = {'address': 2, 'funcode': 1,
                    'data': {f'angle{i}': value for i, value in enumerate([1., 2., 3., 4., 5., 6., 29.])}}
        (stage/'transport.jsonl').write_text(json.dumps(dict(
            kind='output', t=time.monotonic(), line='[rx] '+json.dumps(snapshot)))+'\n')
        backend.cancel()
        assert (folder/'STOP').is_file()
        raise BackendCancelled('cancelled by stop file')
    monkeypatch.setattr(backend, '_run', fake_run)
    result = backend.execute(review, 'approve this displayed plan')
    assert result['status'] == 'cancelled'
    assert result['feedback_samples'] == 1
    assert result['physical_stop_verified'] is False
    assert result['command_count'] == 0
    assert (folder/'execution/02_close/executed/transport.jsonl').is_file()
    assert '29.0' in (folder/'joint_positions.csv').read_text(encoding='utf-8-sig')
    assert cycle.read(folder/'result.json')['status'] == 'cancelled'
    assert (folder/'report.html').is_file()
