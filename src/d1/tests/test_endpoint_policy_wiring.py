"""Endpoint retries must be fixed-only, reviewed, frozen, and budgeted."""
import copy
import base64
import importlib.util
import json
import math
from pathlib import Path
import sys
from types import SimpleNamespace

import pytest

from app import backend as bm
from app.backend import Backend, DEFAULTS, fixed_endpoint_notices
from d1_pick_place.endpoint_control import ENDPOINT_RETRY_POLICY, checked_endpoint_policy

APP=Path(__file__).resolve().parents[1]
CONTROL=APP/'runtime/Control'
sys.path.insert(0,str(CONTROL/'tools'))


def module(name,path):
    spec=importlib.util.spec_from_file_location(name,path)
    result=importlib.util.module_from_spec(spec);spec.loader.exec_module(result)
    return result


arm=module('endpoint_wiring_arm',CONTROL/'ros2_ws/tools/run_arm_stage.py')
cycle=module('endpoint_wiring_cycle',CONTROL/'tools/run_grasp_cycle.py')


def test_policy_defaults_are_exact_and_validation_returns_an_independent_copy():
    expected=dict(mode='retry_approved_target_v1',retry_interval_s=1.,max_seek_s=30.,
                  stall_timeout_s=5.,min_progress_deg=.2)
    assert ENDPOINT_RETRY_POLICY==expected
    assert checked_endpoint_policy(None) is None
    result=checked_endpoint_policy(expected)
    assert result==expected and result is not expected
    result['max_seek_s']=60.
    assert ENDPOINT_RETRY_POLICY['max_seek_s']==30.


@pytest.mark.parametrize('key',['retry_interval_s','max_seek_s','stall_timeout_s','min_progress_deg'])
@pytest.mark.parametrize('value',[True,False,0,-1,float('inf'),float('nan'),'1',None])
def test_policy_rejects_non_positive_non_finite_and_non_numeric_values(key,value):
    policy=dict(ENDPOINT_RETRY_POLICY,**{key:value})
    with pytest.raises(ValueError):checked_endpoint_policy(policy)


@pytest.mark.parametrize('policy',[{},[],True,dict(ENDPOINT_RETRY_POLICY,mode='force'),
    dict(ENDPOINT_RETRY_POLICY,extra=1),dict(ENDPOINT_RETRY_POLICY,retry_interval_s=6),
    dict(ENDPOINT_RETRY_POLICY,stall_timeout_s=31)])
def test_policy_rejects_unknown_shape_mode_keys_and_invalid_time_order(policy):
    with pytest.raises(ValueError):checked_endpoint_policy(policy)


@pytest.mark.parametrize('stage',['pregrasp','approach','lift','lower','retreat','close'])
def test_arm_source_cannot_enable_endpoint_retry_for_a_grasp_stage(stage):
    source=dict(progress_policy='feedback_paced',endpoint_policy=ENDPOINT_RETRY_POLICY)
    with pytest.raises(ValueError,match='fixed-pregrasp'):arm.source_endpoint_policy(source,stage)


def test_arm_policy_requires_feedback_paced_fixed_source_and_keeps_legacy_passive():
    assert arm.source_endpoint_policy({},'pregrasp') is None
    source=dict(progress_policy='scheduled',endpoint_policy=ENDPOINT_RETRY_POLICY)
    with pytest.raises(ValueError,match='feedback-paced'):arm.source_endpoint_policy(source,'fixed_pregrasp')
    source['progress_policy']='feedback_paced'
    assert arm.source_endpoint_policy(source,'fixed_pregrasp')==ENDPOINT_RETRY_POLICY


def retry_manifest(tmp_path):
    source=tmp_path/'plan.json'
    source.write_text(json.dumps(dict(progress_policy='feedback_paced',endpoint_policy=ENDPOINT_RETRY_POLICY)))
    stage=dict(name='fixed_pregrasp',plan_file=str(source),progress_policy='feedback_paced',
               endpoint_policy=copy.deepcopy(ENDPOINT_RETRY_POLICY))
    return dict(spec={'fixed_ready':True},endpoint_policy=copy.deepcopy(ENDPOINT_RETRY_POLICY),
                stages=[dict(name='prepare_gripper'),stage])


def test_manifest_source_stage_and_top_level_policy_agree(tmp_path):
    manifest=retry_manifest(tmp_path)
    cycle.validate_endpoint_policies(manifest)
    manifest['stages'][-1]['endpoint_policy']['retry_interval_s']=2.
    with pytest.raises(ValueError,match='differs from source'):cycle.validate_endpoint_policies(manifest)


def test_manifest_top_level_cannot_silently_change_retry_policy(tmp_path):
    manifest=retry_manifest(tmp_path);manifest['endpoint_policy']['max_seek_s']=31.
    with pytest.raises(ValueError,match='reviewed manifest'):cycle.validate_endpoint_policies(manifest)


def test_full_grasp_and_gripper_cannot_receive_retry_policy(tmp_path):
    manifest=retry_manifest(tmp_path);manifest['spec']['fixed_ready']=False
    with pytest.raises(ValueError,match='restricted'):cycle.validate_endpoint_policies(manifest)
    manifest=retry_manifest(tmp_path);manifest['stages'][0]['endpoint_policy']=dict(ENDPOINT_RETRY_POLICY)
    with pytest.raises(ValueError,match='gripper'):cycle.validate_endpoint_policies(manifest)


def test_fixed_child_process_budget_includes_approved_seek_time_only():
    assert cycle.stage_process_timeout(dict(name='fixed_pregrasp',endpoint_policy=ENDPOINT_RETRY_POLICY),3.8)==78.8
    assert cycle.stage_process_timeout(dict(name='pregrasp'),3.8)==48.8
    assert cycle.stage_process_timeout(dict(name='prepare_gripper',segments=[{},{}]),3.)==58.
    with pytest.raises(ValueError,match='restricted'):
        cycle.stage_process_timeout(dict(name='lift',endpoint_policy=ENDPOINT_RETRY_POLICY),3.)


def test_fixed_demo_review_binds_and_describes_policy_without_subprocesses(tmp_path,monkeypatch):
    monkeypatch.setattr(bm,'APP',tmp_path)
    backend=Backend(mode='demo')
    monkeypatch.setattr(backend,'_run',lambda *args,**kwargs:pytest.fail('Demo may not create a subprocess'))
    review=backend.prepare(DEFAULTS,fixed_pregrasp=True)
    assert review['ready'] and review['endpoint_policy']==ENDPOINT_RETRY_POLICY
    manifest=json.loads(Path(review['manifest']).read_text())
    assert manifest['endpoint_policy']==manifest['stages'][-1]['endpoint_policy']==ENDPOINT_RETRY_POLICY
    assert 'endpoint_policy' not in manifest['stages'][0]
    notice=''.join(review['notices'])
    for value in ['每 1 秒','≤1°','连续3次','5 秒','≥0.2°','30 秒','J6']:
        assert value in notice
    assert fixed_endpoint_notices(None)==[]


def serialized_fixed_source(tmp_path):
    from rclpy.serialization import serialize_message
    from moveit_msgs.msg import RobotTrajectory
    from trajectory_msgs.msg import JointTrajectoryPoint
    from d1_pick_place.hardware_bridge import trajectory_digest
    from d1_pick_place.sdk_executor import NAMES
    trajectory=RobotTrajectory();trajectory.joint_trajectory.joint_names=list(NAMES)
    for second,target in [(0,0.),(1,.04),(2,.08)]:
        point=JointTrajectoryPoint();point.positions=[target,0.,0.,0.,0.,0.]
        point.time_from_start.sec=second;trajectory.joint_trajectory.points.append(point)
    source=dict(completed=True,progress_policy='feedback_paced',position_delay_policy='sdk_minimum',
        endpoint_policy=copy.deepcopy(ENDPOINT_RETRY_POLICY),timing_profile='edge_quintic_v1',
        execution_timing=dict(rate=5.,speed=3.,acceleration=6.,respect_source_timing=False),
        stages=[dict(name='fixed_pregrasp',sha256=trajectory_digest(trajectory),
            trajectory_cdr_base64=base64.b64encode(serialize_message(trajectory)).decode())])
    source_path=tmp_path/'source.json';source_path.write_text(json.dumps(source))
    candidate=tmp_path/'mapping.json'
    candidate.write_text(json.dumps(dict(mapping=dict(order=[f'J{i}->Joint{i+1}' for i in range(6)],
        signs=[1]*6,zero_offsets_deg=[0]*6))))
    return source_path,candidate


def test_real_ros_preview_records_approved_policy_without_device_construction(tmp_path,monkeypatch):
    import d1_pick_place.sdk_executor as executor
    source,candidate=serialized_fixed_source(tmp_path)
    monkeypatch.setattr(executor,'CliTransport',lambda *args,**kwargs:pytest.fail('Preview opened a device'))
    output=tmp_path/'preview'
    assert arm.main(['--plan',str(source),'--stage','fixed_pregrasp','--candidate',str(candidate),
        '--expected-angles','0','0','0','0','0','0','36','--gripper-target','36','--output',str(output)])==0
    for filename in ['plan.json','result.json']:
        assert json.loads((output/filename).read_text())['endpoint_policy']==ENDPOINT_RETRY_POLICY
    report=json.loads((output/'result.json').read_text())
    assert report['plan_only'] and not report['hardware_commands_sent']


def test_endpoint_retry_cannot_finish_at_an_approved_prefix_instead_of_original_endpoint(tmp_path,capsys):
    source,candidate=serialized_fixed_source(tmp_path)
    with pytest.raises(SystemExit):
        arm.main(['--plan',str(source),'--stage','fixed_pregrasp','--candidate',str(candidate),
            '--expected-angles','0','0','0','0','0','0','36','--max-duration','2',
            '--output',str(tmp_path/'preview')])
    assert 'complete approved fixed-pregrasp endpoint' in capsys.readouterr().err


def test_read_only_already_at_fixed_target_preparation_freezes_policy_and_module(tmp_path,monkeypatch):
    profile=json.loads((APP/'app/fixed_ready_pose.json').read_text())
    target=[math.degrees(value) for value in profile['model_target_rad']]
    candidate=tmp_path/'candidate.json'
    candidate.write_text(json.dumps(dict(mapping=dict(order=[f'J{i}->Joint{i+1}' for i in range(6)],
        signs=[1]*6,zero_offsets_deg=[0]*6))))
    config=tmp_path/'config.yaml'
    config.write_text(f'cube_size_m: 0.04\nhardware_mapping_candidate: {candidate}\nplace_release_gap_m: 0.003\n')
    identity=[[1.,0.,0.,0.],[0.,1.,0.,0.],[0.,0.,1.,0.],[0.,0.,0.,1.]]
    pose=tmp_path/'pose.json';pose.write_text(json.dumps(dict(cube_size_m=.04,T_base_cube=identity,T_base_table=identity)))
    monkeypatch.setattr(cycle,'CONFIG',config)
    monkeypatch.setattr(cycle,'run_command',lambda *args,**kwargs:pytest.fail('Already-at-target preparation made a subprocess'))
    args=SimpleNamespace(expected_angles=target+[36.],pose=pose,close_target=29.6,open_target=36.,
        valid_for_s=600.,verification_timeout_s=30.,continuation_envelope_deg=1.,retreat_height_m=.08,
        lift_height_m=.03,output=tmp_path/'review',yaw_deg=90.,style='top',grasp_height_offset_m=.01,
        keep_start_orientation=True,speed_deg_s=20.,acceleration_deg_s2=40.,rate_hz=10.,
        gripper_speed_sdk_s=10.,include_pregrasp=False,accept_stable_close_shortfall=False,fixed_ready=True)
    result=cycle.prepare(args);manifest=cycle.read(result['manifest'])
    stage=manifest['stages'][-1];source=cycle.read(stage['plan_file'])
    assert stage['already_at_fixed_target']
    assert manifest['endpoint_policy']==stage['endpoint_policy']==source['endpoint_policy']==ENDPOINT_RETRY_POLICY
    policy_source=str((cycle.PACKAGE/'endpoint_control.py').resolve())
    assert manifest['files'][policy_source]==cycle.digest(policy_source)
    assert not result['hardware_commands_sent']
    cycle.validate_endpoint_policies(manifest)
