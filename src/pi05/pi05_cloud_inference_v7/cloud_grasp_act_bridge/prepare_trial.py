#!/usr/bin/env python3
"""Prepare local PI05 trial files from the completed passive preview.

This command does not connect to cloud, cameras, SSH or CAN. It does not grant
physical approval. An approval string is accepted only when explicitly supplied
by the operator together with the onsite-condition declaration.
"""
from __future__ import annotations

import argparse
import copy
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import shlex
import shutil
import sys
import tempfile

from plan_workflow import (artifact_review_hashes, latest_preview, review_content_id,
                           verify_review_content, code_review_hashes, validate_duration, duration_argument, DEFAULT_DURATION_S, duration_description, duration_stop_description)
from nativequeuepolicy import QUEUE_CONTRACT, RUNTIME_ACTION_STEPS, CHUNK_SIZE, DATASET_FPS
from newconditioning import CONDITIONING_ID, profile_sha256, _profile

HERE = Path(__file__).resolve().parent
DEPLOYMENT = Path('/home/aup/Lerobot/projects/piper/deployment')
DEFAULTS = HERE / 'pi05_execution_defaults.json'
REMOTE = '/home/ubuntu/kr260-camera/piper_pi05_expert_act_bridge_v8_executor'
TASK = 'Pick up the red cube and place it inside the black square on the left side of the table.'
CHECKPOINT = HERE.parent.parent / 'piper_pi05_expert_cloud_backup_v6/pi05_expert_only_v6_r09/best_model'
STAGED_HELPERS = {
    'remote_bridge.py': DEPLOYMENT / 'lerobot_robot_piper_executor/remote_bridge.py',
    'guard.py': DEPLOYMENT / 'lerobot_robot_piper_executor/guard.py',
    'reset_piper_once.py': DEPLOYMENT / 'reset_piper_once.py',
    'preflight_j_hold.py': DEPLOYMENT / 'preflight_j_hold.py',
}


def read(path):
    value = json.loads(Path(path).read_text(encoding='utf-8'))
    if not isinstance(value, dict):
        raise ValueError(f'Expected JSON object: {path}')
    return value


def digest(path):
    with Path(path).open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def save(path, value):
    payload = json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + '\n'
    path = Path(path)
    if path.is_file() and path.read_text(encoding='utf-8') == payload:
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + '.tmp')
    temporary.write_text(payload, encoding='utf-8')
    temporary.replace(path)


def arguments():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration', type=duration_argument, default=DEFAULT_DURATION_S,
        help='本次待审运行时长，0为持续至Ctrl+C/保护停止（默认）；有限值1..300秒；不批准、不执行，也不修改已有计划，刷新须显式--refresh')
    parser.add_argument('--preview', type=Path, help='默认使用最新预览，禁止回选旧预览')
    parser.add_argument('--refresh', action='store_true', help='归档原计划、配置和日志，生成最新待审计划')
    parser.add_argument('--approve-review-id', help='批准转换必须明确绑定当前稳定审阅内容ID')
    parser.add_argument('--approval-note', help='Complete explicit user approval for the displayed goal/path/parameters; never inferred')
    parser.add_argument('--onsite-conditions-confirmed', action='store_true',
        help='Affirm unchanged power, no other CAN control traffic before execution, continued source supervision, empty supported follower, hands clear, onsite power cutoff, and unchanged reviewed scene')
    return parser.parse_args()


def archive_existing(here=HERE):
    """Copy byte-exact previous evidence, keeping the active generation intact."""
    here = Path(here)
    plan, trial = here / 'grasp_plan.json', here / 'trial'
    if not plan.exists() and not trial.exists():
        return None
    archive = here / 'plan_history' / datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S_%fZ')
    archive.mkdir(parents=True, exist_ok=False)
    record = {'reason': 'new preview requires a new review and explicit approval', 'files': {}}
    for name in ('grasp_plan.json', 'approval_record.json'):
        path = here / name
        if path.exists():
            record['files'][name] = digest(path)
            shutil.copy2(path, archive / name)
    if trial.exists():
        shutil.copytree(trial, archive / 'trial', symlinks=True)
    save(archive / 'archive_manifest.json', record)
    return str(archive)


def publish_candidate(stage, *, approval):
    """Publish only validated files; rollback leaves the previous bytes intact."""
    stage = Path(stage)
    archived = archive_existing(HERE) if not approval else None
    trial, plan = HERE / 'trial', HERE / 'grasp_plan.json'
    saved_trial = stage / 'previous_trial'
    if approval and trial.exists():
        # Recording approval changes only managed configuration files. Keep any
        # existing run logs/evidence in the same generation byte-for-byte.
        for path in trial.iterdir():
            destination = stage / 'trial' / path.name
            if not destination.exists():
                if path.is_dir():
                    shutil.copytree(path, destination, symlinks=True)
                else:
                    shutil.copy2(path, destination)
    old_files = {name: (HERE / name).read_bytes() if (HERE / name).exists() else None
                 for name in ('grasp_plan.json', 'approval_record.json')}
    trial_replaced = False
    try:
        if trial.exists():
            trial.rename(saved_trial)
        (stage / 'trial').rename(trial)
        trial_replaced = True
        # Configuration is committed before the plan, so any concurrent reader
        # sees either matching generations or a failed exact-byte integrity gate.
        approval_path = HERE / 'approval_record.json'
        if (stage / 'approval_record.json').exists():
            (stage / 'approval_record.json').replace(approval_path)
        elif approval_path.exists():
            approval_path.unlink()  # Exact old bytes already kept in archive.
        (stage / 'grasp_plan.json').replace(plan)
    except BaseException:
        if trial_replaced and trial.exists():
            trial.rename(stage / 'uncommitted_trial')
        if saved_trial.exists():
            saved_trial.rename(trial)
        for name, payload in old_files.items():
            path = HERE / name
            if payload is None:
                if path.exists():
                    path.unlink()
            else:
                temporary = path.with_name(path.name + '.rollback')
                temporary.write_bytes(payload)
                temporary.replace(path)
        raise
    return archived


def build_candidate(args, trial, final_trial, approval, preview, requests):
    duration = validate_duration(args.duration)
    duration_text = duration_description(duration)
    last = requests[-1]
    if any(item.get('source_kind') != 'live' for item in requests):
        raise ValueError('实物待审方案只能来自实时只读预览，不能使用录制数据。')
    source, snapshot = last['source_metadata'], last['source_metadata']['snapshot']
    identity = copy.deepcopy(source['receiver_identity'])
    if snapshot.get('healthy') is not True or snapshot.get('armed') is not False:
        raise ValueError('The last preview snapshot must be healthy and unarmed')
    state = snapshot['state']
    if (len(state) != 7 or any(isinstance(v, bool) or not isinstance(v, (int, float))
            or not math.isfinite(v) for v in state) or not 0 <= state[6] <= .1):
        raise ValueError('Expected seven finite current state values and gripper width within [0,0.1]m')
    defaults = read(DEFAULTS)
    profile = _profile(defaults['command_conditioning'])
    cloud = read(HERE / 'inference_config.json')
    runtime_hash = digest(cloud['runtime_config'])
    if (preview.get('runtime_config_sha256') != runtime_hash
            or preview.get('inference_config_sha256') != digest(HERE / 'inference_config.json')
            or preview.get('server_health', {}).get('model_sha256') != cloud['expected_model_sha256']):
        raise ValueError('最新预览的新云端完整 B 模型或推理配置与当前不一致，必须重新预览。')
    if preview.get('visual_gripper_gate_sha256') != profile_sha256(defaults['vla_visual_gripper_gate']):
        raise ValueError('最新预览没有覆盖当前VLA视觉闭爪候选配置，必须重新预览。')
    io = read(defaults['base_io_config'])
    old_identity = copy.deepcopy(io['transport']['expected_identity'])
    observed_identity = copy.deepcopy(identity)
    old_identity.pop('ifindex', None)
    observed_identity.pop('ifindex', None)
    if old_identity != observed_identity:
        raise ValueError('Preview physical CAN/USB/board identity does not match the existing pinned identity')
    io['transport']['expected_identity'] = identity
    io['transport']['ssh_hostname'] = defaults['host']
    save(trial / 'io_config.json', io)
    readiness = {
        'leader_isolation': 'Keep the existing power state as requested by the operator. Existing preflight requires no other CAN control traffic; execution ends if supervision detects another control source. Observed control silence does not prove physical isolation or that the master is powered off. The onsite declaration still covers the unchanged reviewed scene, empty supported follower, hands clear and immediate power cutoff available.',
        'firmware_and_j_mode': 'The independent RX bridge must observe this trial startup preflight exact CAN/J/POS_VEL 5% request; its fresh healthy current-pose hold result is saved in trial/runs/<timestamp>/startup_result.json before arming.',
        'hardware_and_fixture_limits': 'Pinned controller limits plus the task-workspace candidate from the successful ACT physical task: Guard J1[-60,40],J2[0,135],J3[-120,0],J4[-45,45],J5[-60,70],J6[-80,60]degrees and grip[0,0.1]metres. Only upperJ1/J2/J5 differ from the earlier short-trial envelope;2-degree inward targets now end at38/133/68. This new candidate requires explicit approval of the current concrete scene/plan; earlier approvals do not transfer. Official PI05 select_action consumes the first20 targets from each50-target chunk under the explicit user-authorized runtime reduction20(checkpoint saved10) in order, then predicts again. The existing ACT PiPER hardware adapter applies 2-degree inward joint margins,1mm gripper margin,this-cycle-input-feedback-relative2-degree/3mm cap and0.9 target-rate fraction; no alpha smoothing,release extension or raw tracking gate. The2-degree/3mm cap is relative to the current rollout input,not a total-trial cumulative limit or a guarantee relative to hardware feedback after inference. The receiver still checks actual current feedback using unchanged8-degree/15mm tracking limits. All actual adapted targets must pass both physical guards using the same candidate bounds and unchanged step/rate/tracking/time checks. The operator must review the red cube, left black square, table and complete arm path. Preview checks only adapted first-command numerical feasibility; raw all50 bounds are diagnostics, not future feedback or continuous-control proof. This software is not a collision checker.',
        'signed_gripper_mapping': 'Six joint targets remain absolute degrees and signed raw gripper feedback remains metres. This trial requires current width within[0,0.1]m, holds exactly that width at1N during takeover, and never zeros/offsets/opens a negative width.',
        'verified_stop_strategy': 'Existing independent healthy-feedback current-target hold once at1N; software disarm does not prove a physical stop. Fault/stale feedback never triggers guessed holding targets. Onsite power cutoff remains required.',
        'robot_side_lease_and_watchdog': 'Separate PI05 receiver directory; pinned CAN identity, exclusive owner, ordered complete targets,120ms delivery cap,250ms receiver cap,750ms lease. Current feedback age stays independently bounded. Cached target slot is original chunk capture plus index/20Hz, with at most400ms lateness; original chunk age is separately logged and may exceed400ms. No timestamp is replaced by a new observation or receipt time.',
        'approved_goal_path_parameters': approval,
    }
    save(trial / 'readiness.json', readiness)
    policy = copy.deepcopy(defaults['policy'])
    policy['readiness'] = readiness
    save(trial / 'pi05_trial_policy.json', policy)
    config = {
        'execute': True, 'rollout_duration_s': duration, 'io_config': str(final_trial / 'io_config.json'),
        'expected_ifindex': identity['ifindex'], 'remote_bridge': REMOTE + '/remote_bridge.py',
        'remote_rx_helper': '/home/ubuntu/kr260-camera/remote_can_source.py',
        'remote_execution_policy': REMOTE + '/pi05_trial_policy.json',
        'guard_limits': copy.deepcopy(policy['limits']), 'normal_gripper_force_n': 1.,
        'tx_ttl_s': .12, 'rpc_timeout_s': 1.,
        'policy_target_trace_path': str(final_trial / 'policy_target_trace.jsonl'),
    }
    config.update(copy.deepcopy(profile))
    config['vla_visual_gripper_gate'] = copy.deepcopy(defaults['vla_visual_gripper_gate'])
    config['vla_closure_sync'] = copy.deepcopy(defaults['vla_closure_sync'])
    save(trial / 'execute_config.json', config)
    review = {
        'expected_identity': identity, 'joint_bounds_deg': defaults['verified_controller_joint_bounds_deg'],
        'expected_pose_deg_m': state, 'start_joint_tolerance_deg': 1.,
        'start_gripper_tolerance_m': .002, 'max_deviation_deg': 2.,
        'support_and_disable_on_fault_reviewed': bool(approval and args.onsite_conditions_confirmed),
        'approval_note': approval, 'reuse_enabled': True, 'warm_gripper_target_m': state[6],
    }
    save(trial / 'j_hold_review.json', review)
    # This only constructs command text. No SSH subprocess is started here.
    startup = ['ssh', '-T', '-o', 'BatchMode=yes', '-o', 'StrictHostKeyChecking=yes',
        '-o', 'ConnectTimeout=5', '-o', 'HostName=' + defaults['host'], io['transport']['ssh_host'],
        shlex.join(['python3', '-S', REMOTE + '/preflight_j_hold.py',
                    '--prepare-j-hold-supported-reviewed', '--review-json', REMOTE + '/j_hold_review.json'])]
    save(trial / 'startup_command.json', startup)
    staging = {name: {'local_path': str(path), 'sha256': digest(path)} for name, path in STAGED_HELPERS.items()}
    for name in ('j_hold_review.json', 'pi05_trial_policy.json'):
        staging[name] = {'local_path': str(final_trial / name), 'sha256': digest(trial / name)}
    if preview.get('queue_contract') != QUEUE_CONTRACT:
        raise ValueError('Preview does not bind the explicit runtime20 PI05 queue contract')
    verification_path = HERE / 'offline_verification_result.json'
    verification = read(verification_path)
    if (verification.get('success') is not True or verification.get('network_calls') != 0
            or verification.get('robot_commands_sent') != 0
            or verification.get('command_conditioning_id') != CONDITIONING_ID
            or verification.get('command_conditioning_sha256') != profile_sha256(profile)
            or verification.get('queue_contract') != QUEUE_CONTRACT
            or verification.get('tested_code_sha256') != code_review_hashes()):
        raise ValueError('Current native queue/physical guard offline verification is missing or stale')
    for item in requests:
        if (item.get('guard_assessment', {}).get('queue_contract') != QUEUE_CONTRACT
                or item['guard_assessment'].get('vla_visual_gripper_gate', {}).get('config_sha256') != profile_sha256(defaults['vla_visual_gripper_gate'])
                or item['guard_assessment'].get('future_queue_feedback_or_commit_history_verified') is not False
                or item['guard_assessment'].get('command_conditioning_id') != CONDITIONING_ID
                or item['guard_assessment'].get('command_conditioning_sha256') != profile_sha256(profile)):
            raise ValueError('Preview must distinguish static prefix checks from unverified future feedback')
    summaries = [{'index': i, 'request_id': item['request_id'],
        'numeric_source_checks_pass': item['guard_assessment']['preview_numeric_and_source_checks_pass'],
        'first_action_static_checks_pass': item['guard_assessment']['first_action_static_checks_pass'],
        'consumed_prefix_bounds_pass': item['guard_assessment']['consumed_prefix_bounds_pass'],
        'full_chunk_bounds_pass': item['guard_assessment']['full_chunk_bounds']['all_50_within_hard_bounds'],
        'rejection_reasons': item['guard_assessment']['rejection_reasons'],
        'source_to_validated_action_s': item['guard_assessment']['source_to_validated_action_s']}
        for i, item in enumerate(requests)]
    passes = sum(item['numeric_source_checks_pass'] is True for item in summaries)
    execution_ready = (preview.get('status') == 'preview_complete' and len(summaries) >= 5 and passes == len(summaries))
    blockers = []
    if passes != len(summaries):
        blockers.append(f'最新完整预览有{len(summaries)-passes}/{len(summaries)}次预测被拦截，禁止挑选成功样本；' +
            '；'.join(dict.fromkeys(reason for item in summaries for reason in item['rejection_reasons'])))
    if preview.get('status') != 'preview_complete':
        blockers.append('最新预览未完整完成：' + str(preview.get('error', preview.get('status'))))
    if len(summaries) < 5:
        blockers.append('官方队列试跑需要最新完整至少5次独立新动作块只读预览，且整批全部通过。')
    plan = {
        'schema_version': 'pi05-expert-newcloud-act-bridge-plan/v1', 'task': TASK,
        'command_conditioning': copy.deepcopy(profile), 'command_conditioning_id': CONDITIONING_ID,
        'command_conditioning_sha256': profile_sha256(profile),
        'vla_visual_gripper_gate': copy.deepcopy(defaults['vla_visual_gripper_gate']),
        'vla_closure_sync': copy.deepcopy(defaults['vla_closure_sync']),
        'trial_kind': 'lerobot_native_queue_act_bridge_trial', 'queue_contract': copy.deepcopy(QUEUE_CONTRACT),
        'checkpoint': str(CHECKPOINT), 'checkpoint_config_sha256': digest(CHECKPOINT / 'config.json'),
        'inference_config_sha256': digest(HERE / 'inference_config.json'),
        'execution_defaults_sha256': digest(DEFAULTS),
        'base_io_defaults_sha256': digest(defaults['base_io_config']),
        'observe_config_sha256': digest(HERE / 'pi05_observe_config.json'),
        'expected_model_sha256': cloud['expected_model_sha256'],
        'expected_model_bytes': cloud['expected_model_bytes'],
        'runtime_config_sha256': runtime_hash,
        'native_queue_offline_evidence': {'report': str(verification_path), 'sha256': digest(verification_path)},
        'reviewed_code_sha256': code_review_hashes(),
        'inference_backend': 'new cloud CUDA model server over dedicated localhost SSH tunnel',
        'duration_s': duration, 'action_indices': list(range(RUNTIME_ACTION_STEPS)),
        'n_action_steps': RUNTIME_ACTION_STEPS, 'chunk_size': CHUNK_SIZE,
        'execution_ready': execution_ready, 'blocking_reasons': blockers,
        'guard_limits': copy.deepcopy(policy['limits']),
        'deployment_config_sha256': digest(trial / 'execute_config.json'),
        'readiness_sha256': digest(trial / 'readiness.json'),
        'startup_command_sha256': digest(trial / 'startup_command.json'),
        'io_config_sha256': digest(trial / 'io_config.json'),
        'j_hold_review_sha256': digest(trial / 'j_hold_review.json'),
        'receiver_policy_sha256': digest(trial / 'pi05_trial_policy.json'),
        'readiness': readiness,
        'trial_directory': str(final_trial), 'remote_directory': REMOTE,
        'staging_files': staging,
        'goal': f'{duration_text}由现场监督完成一次桌面小红块抓取、放入左侧黑框、退到采集示范最后的收爪位置；完整抓放撤回尚未验证。',
        'path_intent': ['已展示当前姿态', '接近桌面小红块', '在接近姿态保持六轴并按反馈完成闭爪候选，重新预测后抬起', '移动到桌面左侧黑框', '放下红块并松爪', '按模型示范撤回到最后收爪位置', '监督确认完整一轮后停止', duration_stop_description(duration)],
        'return_semantics': '用户明确要求退到示范最后的收爪位置，由模型继续输出示范撤回动作；不额外执行六轴归零或自动home。软件未实现可靠视觉任务完成判定，必须结合现场/相机确认放置及撤回后结束本轮。',
        'path_semantics': f'采用用户明确运行时覆盖n_action_steps=20（checkpoint保存值10不变）及官方PI05Policy.select_action/reset，新观测预测50步、官方FIFO依序消费前20步后再预测。原始FIFO七维目标与本地视觉闭爪否决后的目标分别记录；闭合阶段本地协调目标与原始七维预测分开留痕。张开意图出现后，未通过连续2个不同腕部图像的经验接近条件、实际抓取关节窗和原始六轴目标/反馈3°一致性时，只暂缓闭爪并保持不小于当前反馈/上次确认的夹爪目标；首次否决后8秒仍不能开始闭合则停止。视觉条件不能证明三维对齐、接触或抓住。允许闭合后，本地反馈阶段控制保持当时实测六轴、锁存模型最小闭合意图并保留其原来源；实际开口42.5～45.5mm连续0.30秒极差≤0.5mm，且上次ACK目标比实际开口小至少2mm时仅判为夹持候选。4秒仍未满足、空闭到35mm以下、实际关节偏离锚点3°、故障或反馈过时则停。ACK后丢弃未执行FIFO，必须用之后新观测重新推理；夹持后固定确认时ACK持有上限并保持至少2mm内压差，放置窗之外要求超出此固定上限即停。每次发送前只读补取反馈，相机与原始模型时间均不刷新，反馈力只记录。之后仍由现有ACT硬件适配执行2°关节内边距、1mm夹爪内边距、相对本周期输入反馈2°/3mm限幅和0.9速率整形；不加alpha平滑、抓放续步或原始8°预测拒绝。每条仍接受主机/接收端全部物理保护。{len(requests)}次静态预览不是20步真实未来反馈或无碰撞轨迹的证明。',
        'startup': {'reuse_enabled_only': True, 'hold_six_current_joint_targets': True,
            'hold_gripper_width_m': state[6], 'joint_mode_speed_percent': 5,
            'gripper_force_n': 1., 'enable_reset_home_calibrate': False,
            'current_pose_deg_m': state, 'start_pose_tolerance_deg': 1., 'start_width_tolerance_m': .002},
        'timing': {'target_fps_scheduling_ceiling': 20., 'actual_20hz_verified': False,
            'fps_source': 'recorded dataset20Hz; generic LeRobot RolloutConfig default is30Hz',
            'max_current_feedback_observation_age_s': .4, 'max_original_slot_lateness_s': .4,
            'chunk_original_source_age_reported_separately': True,
            'scheduled_slot': 'original_chunk_capture_monotonic + action_index / 20',
            'timing_contract_change_requires_new_approval': True,
            'tx_ttl_s': .12, 'receiver_max_ttl_s': .25, 'lease_s': .75,
            'on_guard_failure': '立即结束本次试跑并撤销执行租约；不放宽硬保护门限、不自动重试。'},
        'preview_evidence': {'report': str(args.preview.resolve()), 'sha256': digest(args.preview),
            'created_utc': preview['created_utc'], 'request_count': len(summaries),
            'numeric_source_checks_pass_count': passes, 'rejected_count': len(summaries) - passes,
            'requests': summaries, 'last_request_index': len(requests) - 1,
            'last_request_id': last['request_id'],
            'last_scene_image': str((args.preview.resolve().parent / last['camera_pair_image']).resolve()),
            'last_scene_image_sha256': digest(args.preview.resolve().parent / last['camera_pair_image']),
            'current_state_from_last_request': state, 'receiver_identity': identity,
            'limitations': f'{len(summaries)}次独立新块预览中{len(summaries)-passes}次被适配后首动作数值可行性/时效检查拒绝；最后一次' +
                ('通过。' if last['guard_assessment']['preview_numeric_and_source_checks_pass'] else '未通过。') +
                '后续队列动作没有虚构未来反馈、提交历史、速率或跟踪通过证据。原始全部50步边界仅诊断；执行依序消费前20步，每条适配后再次双端检查。'},
        'must_repreview_if_changed': ['红块或左侧黑框位置', '桌面障碍/工装/人员', '相机位置或身份',
            '机械臂当前姿态超出启动容差', '模型/归一化/提示词/执行参数/硬件身份', '云端服务或网络状态明显变化'],
        'physical_approval_pending': not bool(approval),
        'onsite_conditions': '保持现有供电；执行前无其他CAN控制流量，运行中监督发现其他控制源立即结束；控制静默不证明物理隔离或主臂断电；从臂空载且稳固承托；手退出工作区；现场可立即断电；目标、黑框及全臂路径与审阅画面一致。',
        'configuration_does_not_authorize_physical_grasp': not bool(approval),
    }
    # Hard limits are validated without opening a backend. Pending review is
    # intentionally not passed to validate_review(), whose approval is absent.
    sys.path[:0] = [str(DEPLOYMENT), str(DEPLOYMENT.parent / 'official_adapter')]
    from launch_grasp import validate_limits, reject_custom_conditioning
    validate_limits(config['guard_limits'])
    reject_custom_conditioning(plan)
    reject_custom_conditioning(config)
    if approval:
        from preflight_j_hold import validate_review
        validate_review(review)
    plan['review_artifact_sha256'] = artifact_review_hashes(trial)
    plan['review_content_sha256'] = review_content_id(plan)
    if approval and plan['review_content_sha256'] != args.approve_review_id:
        raise RuntimeError('批准准备过程中非批准内容已变化；拒绝生成批准后的计划。请刷新后重新审阅。')
    return plan


def prepare_transaction(args):
    args.duration = validate_duration(args.duration)
    approval = '' if args.approval_note is None else args.approval_note.strip()
    if args.approval_note is not None and (len(approval) < 20 or not args.onsite_conditions_confirmed):
        raise ValueError('--approval-note requires a complete explicit approval string (at least 20 characters) and --onsite-conditions-confirmed')
    plan_path = HERE / 'grasp_plan.json'
    args.preview = latest_preview(HERE) if args.preview is None else args.preview.resolve()
    if args.preview != latest_preview(HERE):
        raise ValueError('不能回选旧预览；请使用最新预览生成待审方案。')
    before = read(plan_path) if plan_path.is_file() else None
    if approval:
        if args.refresh or before is None or args.approve_review_id != verify_review_content(before):
            raise ValueError('批准必须绑定当前审阅内容ID，且不能同时刷新计划。')
        if args.duration != validate_duration(before.get('duration_s')):
            raise ValueError('批准时的--duration必须与已展示计划完全一致，不能借批准修改时长。')
        if before.get('execution_ready') is not True:
            raise ValueError('最新预览未通过，不能记录执行批准。')
    elif before is not None and not args.refresh:
        raise RuntimeError('已有计划。请用 refresh_plan.sh 显式归档并刷新；不会覆盖已有批准。')
    preview = read(args.preview)
    if (preview.get('purpose') != 'pi05_expert_newcloud_act_bridge_read_only_preview' or preview.get('robot_commands_sent') != 0):
        raise ValueError('A zero-command passive preview report is required')
    requests = preview.get('requests', [])
    with tempfile.TemporaryDirectory(prefix='.prepare-', dir=HERE) as temporary:
        stage = Path(temporary)
        trial = stage / 'trial'
        trial.mkdir()
        if not requests or 'snapshot' not in requests[-1].get('source_metadata', {}):
            plan = {'schema_version': 'pi05-expert-newcloud-act-bridge-diagnostic/v1',
                'execution_ready': False, 'physical_approval_pending': True,
                'configuration_does_not_authorize_physical_grasp': True,
                'blocking_reasons': ['最新只读预览未完成或没有可用实时反馈：' + str(preview.get('error', preview.get('status')))],
                'preview_evidence': {'report': str(args.preview), 'sha256': digest(args.preview)}}
        else:
            plan = build_candidate(args, trial, HERE / 'trial', approval, preview, requests)
        save(stage / 'grasp_plan.json', plan)
        if approval:
            save(stage / 'approval_record.json', {'review_content_sha256': plan['review_content_sha256'],
                'plan_file_sha256': digest(stage / 'grasp_plan.json'), 'approval_note': approval,
                'approved_utc': datetime.now(timezone.utc).isoformat(), 'onsite_conditions_confirmed': True})
        # No active plan/config/log is touched until all candidate validations
        # (including stable approval identity) have succeeded above.
        archived = publish_candidate(stage, approval=bool(approval))
    print(json.dumps({'prepared': True, 'network_or_hardware_connected': False,
        'plan': str(plan_path), 'plan_sha256': digest(plan_path), 'duration_s': args.duration,
        'review_content_sha256': plan.get('review_content_sha256'), 'execution_ready': plan['execution_ready'],
        'blocking_reasons': plan['blocking_reasons'], 'archived_previous_plan': archived,
        'physical_approval_pending': not bool(approval), 'preview_requests': len(requests),
        'preview_rejected': plan.get('preview_evidence', {}).get('rejected_count'),
        'check_command': str(HERE / 'trial.sh') + ' --check-only'}, ensure_ascii=False, indent=2))
    return 0


def main():
    args = arguments()
    try:
        return prepare_transaction(args)
    except BaseException as error:
        # Keep an immutable diagnostic, preserving the old active files. A new
        # preview/pending marker already invalidates the old execution approval.
        stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S_%fZ')
        save(HERE / 'preparation_errors' / (stamp + '.json'), {
            'status': 'preparation_failed', 'error': type(error).__name__ + ': ' + str(error),
            'network_or_hardware_connected': False, 'previous_active_generation_preserved': True,
            'requested_preview': str(args.preview) if args.preview is not None else None})
        raise


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except Exception as error:
        print(f'PI05 trial preparation stopped: {type(error).__name__}: {error}', file=sys.stderr)
        raise SystemExit(1)
