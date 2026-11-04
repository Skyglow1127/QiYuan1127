"""Local review identity and passive pre-start checks; no actuation API."""
from __future__ import annotations

import copy
from datetime import datetime, timezone
import hashlib
import importlib.metadata
import json
import math
from pathlib import Path

HERE = Path(__file__).resolve().parent
MUTABLE_HASH_FIELDS = ('readiness_sha256', 'j_hold_review_sha256', 'receiver_policy_sha256')
REVIEW_ARTIFACTS = ('readiness.json', 'j_hold_review.json', 'pi05_trial_policy.json')
REVIEW_CODE = tuple(HERE / name for name in ('client.py', 'launch_grasp.py', 'trial.py',
    'prepare_trial.py', 'start_trial.py', 'plan_workflow.py', 'preview.sh', 'run_preview.sh',
    'refresh_plan.sh', 'start_trial.sh', 'trial.sh', 'rollout_expert.py', 'nativequeuepolicy.py', 'newconditioning.py', 'offline_verification.py',
    'camera_recovery.py', 'test_camera_recovery.py', 'control_logging.py', 'test_control_logging.py',
    'test_lossless_transport.py', 'connection_warmth.py', 'test_connection_warmth.py',
    'visual_gripper_gate.py', 'test_visual_gripper_gate.py',
    'visual_gripper_runtime.py', 'test_visual_gripper_runtime.py',
    'closure_sync.py', 'test_closure_sync.py', 'test_closure_sync_integration.py')) + tuple(
    Path('/home/aup/Lerobot/projects/piper/deployment/lerobot_robot_piper_executor') / name
    for name in ('robot.py', 'guard.py', 'sdk_codec.py', 'remote_bridge.py', 'policy_target_trace.py',
                 'policy_bounds.py', 'target_rate.py', 'backend.py')) + (
    Path('/home/aup/Lerobot/projects/piper/official_adapter/lerobot_robot_piper_shared/devices.py'),
    Path('/home/aup/Lerobot/projects/piper/official_adapter/rollout_act_piper.py'))



DEFAULT_DURATION_S = 0.
MIN_DURATION_S = 1.
MAX_DURATION_S = 300.


def validate_duration(value):
    """Official rollout semantics:0 means indefinite; otherwise finite1..300s."""
    if (isinstance(value, bool) or not isinstance(value, (int, float))
            or not math.isfinite(value) or not (value == 0 or MIN_DURATION_S <= value <= MAX_DURATION_S)):
        raise ValueError('duration_s must be0(unlimited) or finite seconds in[1,300]; no bool,NaN or inf')
    return float(value)


def duration_description(value):
    duration = validate_duration(value)
    return '持续至Ctrl+C/保护停止' if duration == 0 else f'最长{duration:g}秒'


def duration_stop_description(value):
    duration = validate_duration(value)
    return 'Ctrl+C或保护触发结束' if duration == 0 else f'{duration:g}秒到时或保护触发结束'


def duration_argument(value):
    """argparse converter; JSON and programmatic callers use validate_duration."""
    if isinstance(value, bool):
        raise ValueError('duration cannot be bool')
    return validate_duration(float(value))


def validate_duration_binding(plan, deployment):
    duration = validate_duration(plan.get('duration_s'))
    configured = validate_duration(deployment.get('rollout_duration_s'))
    if configured != duration:
        raise ValueError('Plan duration_s must exactly match execute_config.rollout_duration_s')
    return duration


def read(path):
    value = json.loads(Path(path).read_text(encoding='utf-8'))
    if not isinstance(value, dict):
        raise ValueError(f'Expected JSON object: {path}')
    return value


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def canonical_digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, ensure_ascii=False,
        separators=(',', ':'), allow_nan=False).encode('utf-8')).hexdigest()


def code_review_hashes():
    interface = read(HERE / 'inference_config.json')
    service = Path(interface['runtime_config']).parent
    official = importlib.metadata.distribution('lerobot')
    native = tuple(Path(official.locate_file('lerobot/' + name)) for name in (
        'policies/pi05/modeling_pi05.py', 'policies/pi05/configuration_pi05.py',
        'rollout/configs.py', 'rollout/inference/sync.py', 'rollout/inference/factory.py',
        'rollout/strategies/base.py', 'rollout/strategies/core.py'))
    paths = REVIEW_CODE + native + tuple(service / name for name in ('server.py', 'runtime.py', 'pi05_low_memory.py'))
    if read(Path(interface['runtime_config'])).get('production_image_codec') == 'png-rgb-u8-v1':
        paths += (service / 'wire_codec.py',)
    return {str(path): digest(path) for path in paths}


def approval_neutral_artifact(name, value):
    value = copy.deepcopy(value)
    if name == 'readiness.json':
        value['approved_goal_path_parameters'] = ''
    elif name == 'j_hold_review.json':
        value['approval_note'] = ''
        value['support_and_disable_on_fault_reviewed'] = False
    elif name == 'pi05_trial_policy.json':
        value['readiness']['approved_goal_path_parameters'] = ''
    else:
        raise ValueError(f'Unexpected review artifact: {name}')
    return value


def artifact_review_hashes(trial):
    return {name: canonical_digest(approval_neutral_artifact(name, read(Path(trial) / name)))
            for name in REVIEW_ARTIFACTS}


def review_content_id(plan):
    """Only recorded approval changes are omitted; poses/scene/config stay pinned."""
    value = copy.deepcopy(plan)
    value.pop('review_content_sha256', None)
    value['readiness']['approved_goal_path_parameters'] = ''
    value['physical_approval_pending'] = True
    value['configuration_does_not_authorize_physical_grasp'] = True
    for field in MUTABLE_HASH_FIELDS:
        value.pop(field, None)
    for name in ('j_hold_review.json', 'pi05_trial_policy.json'):
        value['staging_files'][name].pop('sha256', None)
    # Approval-neutral fingerprints of these exact files replace the removed
    # byte hashes, preserving every non-approval field in the reviewed content.
    if set(value.get('review_artifact_sha256', {})) != set(REVIEW_ARTIFACTS):
        raise ValueError('计划缺少批准中立的配置指纹；请先运行 refresh_plan.sh。')
    return canonical_digest(value)


def verify_review_content(plan):
    from nativequeuepolicy import QUEUE_CONTRACT, RUNTIME_ACTION_STEPS
    from newconditioning import CONDITIONING_ID, profile_sha256
    validate_duration(plan.get('duration_s'))
    actual = review_content_id(plan)
    if plan.get('review_content_sha256') != actual:
        raise ValueError('计划的姿态、场景或参数已经变化；必须重新审阅并批准。')
    if artifact_review_hashes(plan['trial_directory']) != plan['review_artifact_sha256']:
        raise ValueError('启动/保护配置的非批准内容已经变化；必须重新审阅并批准。')
    if (plan.get('schema_version') != 'pi05-expert-newcloud-act-bridge-plan/v1'
            or plan.get('trial_kind') != 'lerobot_native_queue_act_bridge_trial'
            or plan.get('queue_contract') != QUEUE_CONTRACT
            or plan.get('n_action_steps') != RUNTIME_ACTION_STEPS
            or plan.get('action_indices') != list(range(RUNTIME_ACTION_STEPS))
            or plan.get('command_conditioning_id') != CONDITIONING_ID
            or plan.get('command_conditioning_sha256') != profile_sha256(read(HERE/'pi05_execution_defaults.json')['command_conditioning'])
            or 'action_index' in plan):
        raise ValueError('该计划不属于用户指定runtime20的官方PI05队列，旧action0或整形方案不可复用。')
    from launch_grasp import reject_custom_conditioning
    reject_custom_conditioning(plan)
    if plan.get('command_conditioning') != read(HERE/'pi05_execution_defaults.json')['command_conditioning']:
        raise ValueError('ACT硬件适配配置与当前固定配置不同；必须重新审阅。')
    if plan.get('reviewed_code_sha256') != code_review_hashes():
        raise ValueError('新云端模型/预览/启动代码已变化；必须重新预览和批准。')
    if digest(HERE / 'inference_config.json') != plan.get('inference_config_sha256'):
        raise ValueError('新云端推理服务配置已变化；必须重新预览和批准。')
    interface = read(HERE / 'inference_config.json')
    if digest(interface['runtime_config']) != plan.get('runtime_config_sha256'):
        raise ValueError('完整模型或推理参数已变化；必须重新预览和批准。')
    evidence = plan['native_queue_offline_evidence']
    if (Path(evidence['report']).resolve() != (HERE / 'offline_verification_result.json').resolve()
            or digest(evidence['report']) != evidence['sha256']
            or read(evidence['report']).get('command_conditioning_id') != CONDITIONING_ID
            or read(evidence['report']).get('command_conditioning_sha256') != plan['command_conditioning_sha256']
            or read(evidence['report']).get('tested_code_sha256') != code_review_hashes()):
        raise ValueError('官方队列离线验证结果已变化或已过期；必须重新验证、预览和批准。')
    defaults_path = Path(plan['trial_directory']).parent / 'pi05_execution_defaults.json'
    if digest(defaults_path) != plan.get('execution_defaults_sha256'):
        raise ValueError('π0.5独立执行默认配置已变化；必须刷新计划并重新批准。')
    base_io_path = Path(read(defaults_path)['base_io_config'])
    if digest(base_io_path) != plan.get('base_io_defaults_sha256'):
        raise ValueError('π0.5独立IO默认配置已变化；必须刷新计划并重新批准。')
    observe_path = Path(plan['trial_directory']).parent / 'pi05_observe_config.json'
    if digest(observe_path) != plan.get('observe_config_sha256'):
        raise ValueError('π0.5只读观测配置已变化；必须重新预览、刷新计划并重新批准。')
    scene = plan['preview_evidence']
    if digest(scene['last_scene_image']) != scene.get('last_scene_image_sha256'):
        raise ValueError('已审阅场景图像的字节已变化；必须重新预览和批准。')
    return actual


def preview_reports(here=HERE):
    """Include failed latest attempts so an older success cannot hide them."""
    here = Path(here)
    reports = [*here.glob('previews/*/report.json')]
    if (here / 'live_preview/report.json').is_file():
        reports.append(here / 'live_preview/report.json')
    pointer = here / 'latest_preview_attempt.json'
    if pointer.exists():
        attempted_report = Path(read(pointer)['report']).resolve()
        if not attempted_report.is_file():
            raise ValueError('最新预览尚未完成或未产生报告；旧批准不可执行。请重新运行 preview.sh。')
        reports.append(attempted_report)
    def key(path):
        value = read(path)
        return (datetime.fromisoformat(value['created_utc']).timestamp(), str(path))
    return sorted(reports, key=key)


def begin_preview_attempt(output, here=HERE):
    """Invalidate old review before read-only acquisition, including failures."""
    output, here = Path(output).resolve(), Path(here)
    if output.exists():
        raise ValueError('预览必须使用新目录，不能覆盖或复用已有场景证据。')
    value = {'started_utc': datetime.now(timezone.utc).isoformat(),
             'report': str(output / 'report.json'), 'purpose': 'latest_read_only_preview_attempt'}
    pointer = here / 'latest_preview_attempt.json'
    temporary = pointer.with_suffix('.tmp')
    temporary.write_text(json.dumps(value, indent=2, ensure_ascii=False) + '\n', encoding='utf-8')
    temporary.replace(pointer)
    return value


def latest_preview(here=HERE):
    reports = preview_reports(here)
    if not reports:
        raise ValueError('尚无预览报告；先运行 preview.sh。')
    return reports[-1].resolve()


def verify_latest_preview(plan, here=HERE):
    from nativequeuepolicy import QUEUE_CONTRACT
    from newconditioning import CONDITIONING_ID, profile_sha256
    evidence = plan['preview_evidence']
    preview = Path(evidence['report']).resolve()
    if preview != latest_preview(here):
        raise ValueError('已有更新的预览，旧批准已失效；运行 refresh_plan.sh 审阅最新结果。')
    if digest(preview) != evidence['sha256']:
        raise ValueError('预览报告已变化；必须刷新计划并重新审阅。')
    if plan.get('execution_ready') is True:
        report = read(preview)
        requests = report.get('requests', [])
        if (report.get('purpose') != 'pi05_expert_newcloud_act_bridge_read_only_preview'
                or report.get('visual_gripper_gate_sha256') != canonical_digest(plan['vla_visual_gripper_gate'])
                or report.get('queue_contract') != QUEUE_CONTRACT
                or report.get('robot_commands_sent') != 0 or report.get('status') != 'preview_complete'
                or len(requests) < 5 or any(item.get('source_kind') != 'live'
                    or item.get('guard_assessment', {}).get('preview_numeric_and_source_checks_pass') is not True
                    or item.get('guard_assessment', {}).get('queue_contract') != QUEUE_CONTRACT
                    or item.get('guard_assessment', {}).get('vla_visual_gripper_gate', {}).get('config_sha256') != report.get('visual_gripper_gate_sha256')
                    or item.get('guard_assessment', {}).get('command_conditioning_id') != CONDITIONING_ID
                    or item.get('guard_assessment', {}).get('command_conditioning_sha256') != profile_sha256(read(HERE/'pi05_execution_defaults.json')['command_conditioning'])
                    or item.get('guard_assessment', {}).get('future_queue_feedback_or_commit_history_verified') is not False
                    for item in requests)):
            raise ValueError('最新真实预览并未全部通过，execution_ready不能用于执行。')
    return preview


def compare_current_snapshot(plan, source_metadata):
    """Check actual healthy unarmed feedback against the reviewed pose in place.

    source_metadata is PassiveSource.capture()['source_metadata']. This function
    never replaces model source timestamps, changes targets or sends commands.
    """
    snapshot = source_metadata['snapshot']
    if snapshot.get('healthy') is not True or snapshot.get('armed') is not False:
        raise ValueError('当前只读反馈不健康或已 armed；未启动试跑。')
    if source_metadata['receiver_identity'] != plan['preview_evidence']['receiver_identity']:
        raise ValueError('当前机械臂/CAN身份与审阅预览不一致；未启动。')
    current = snapshot['state']
    reviewed = plan['startup']['current_pose_deg_m']
    if (len(current) != 7 or any(isinstance(v, bool) or not isinstance(v, (int, float))
            or not math.isfinite(v) for v in current)):
        raise ValueError('当前只读反馈必须为7个有限数值。')
    failures = [f'关节{i+1}：审阅 {old:.3f}°，当前 {new:.3f}°'
                for i, (old, new) in enumerate(zip(reviewed[:6], current[:6]))
                if abs(old-new) > plan['startup']['start_pose_tolerance_deg']]
    if abs(reviewed[6]-current[6]) > plan['startup']['start_width_tolerance_m']:
        failures.append(f'夹爪：审阅 {reviewed[6]*1000:.1f} mm，当前 {current[6]*1000:.1f} mm，'
                        f'允许差值 {plan["startup"]["start_width_tolerance_m"]*1000:.1f} mm')
    if failures:
        raise ValueError('当前姿态与已审阅计划不一致：' + '；'.join(failures) +
                         '。未合拢夹爪、未发送动作；请 preview.sh 后重新审阅。')
    return {'passed': True, 'current_state_deg_m': current,
            'review_content_sha256': verify_review_content(plan), 'commands_sent': 0}


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description=__doc__)
    action = parser.add_mutually_exclusive_group(required=True)
    action.add_argument('--begin-preview', type=Path,
                        help='只标记新预览目录，使旧批准失效；不连接硬件')
    action.add_argument('--validate-duration', type=duration_argument,
                        help='只校验0无限或有限1..300秒时长；不读写计划、不连接硬件')
    args = parser.parse_args()
    if args.begin_preview is not None:
        begin_preview_attempt(args.begin_preview)
