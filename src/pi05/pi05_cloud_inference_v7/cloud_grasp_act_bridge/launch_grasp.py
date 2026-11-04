#!/usr/bin/env python3
"""Review a local complete-expert PI05 plan; run only with matching explicit plan approval.

The default branch reads local files only. The actual control loop remains the
official LeRobot 0.6.1 base/sync loop and the existing guarded Piper executor.
This CPU rollout module calls the new cloud CUDA model process; it does not load a second model.
"""
from __future__ import annotations

import argparse
from contextlib import contextmanager
import hashlib
import importlib.metadata
import json
import logging
import math
import os
from pathlib import Path
import re
import sys

import numpy as np
import torch

from nativequeuepolicy import CloudExpertPolicy, QUEUE_CONTRACT, RUNTIME_ACTION_STEPS, CHECKPOINT_SAVED_ACTION_STEPS

HERE = Path(__file__).resolve().parent
PROJECT = Path('/home/aup/Lerobot/projects/piper')
DEPLOYMENT = PROJECT / 'deployment'
OFFICIAL_ADAPTER = PROJECT / 'official_adapter'
CHECKPOINT = HERE.parent.parent / 'piper_pi05_expert_cloud_backup_v6/pi05_expert_only_v6_r09/best_model'
TASK = 'Pick up the red cube and place it inside the black square on the left side of the table.'
SCHEMA = 'pi05-expert-newcloud-act-bridge-plan/v1'
CANONICAL_KEYS = tuple(f'joint_{i}.deg' for i in range(1, 7)) + ('gripper.m',)
ROLLOUT_KEYS = tuple(f'joint_{i}.pos' for i in range(1, 7)) + ('gripper.pos',)
MODEL_SHA256 = '583ff741239b111053dc969fe5ba69eb70475e888c7ece92b04a78d3319386cf'
MODEL_BYTES = 9354050752
MAX_LIMITS = {
    'bounds': [[-60., 40.], [0., 135.], [-120., 0.], [-45., 45.], [-60., 70.], [-80., 60.], [0., .1]],
    'max_step': [6.]*6 + [.008], 'max_target_speed': [45.]*6 + [.05],
    'max_tracking_error': [8.]*6 + [.015], 'max_observation_age_s': .4,
    'max_command_age_s': .4, 'max_command_gap_s': .75,
}


def sha256_file(path):
    with Path(path).open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def read_json(path):
    value = json.loads(Path(path).read_text(encoding='utf-8'))
    if not isinstance(value, dict):
        raise ValueError(f'Expected a JSON object: {path}')
    return value


def pin_file(plan, field, path, blockers):
    if not isinstance(plan.get(field), str) or not re.fullmatch(r'[0-9a-f]{64}', plan[field]):
        blockers.append(f'plan.{field} must pin an actual SHA256')
    elif sha256_file(path) != plan[field]:
        raise RuntimeError(f'Pinned local file changed: {field}: {path}')


from newconditioning import CONDITIONING_ID, profile_sha256, _profile

ALLOWED_ADAPTER_FIELDS = frozenset((
    'max_relative_target', 'policy_joint_target_margin_deg',
    'policy_gripper_target_margin_m', 'policy_target_rate_fraction'))
FORBIDDEN_CONDITIONING_FIELDS = frozenset((
    'policy_joint_smoothing_alpha', 'policy_release_chunk_extension', 'policy_grasp_chunk_extension',
    'j5_upper_prediction_saturation', 'j5_upper_prediction_tolerance_deg',
    'j5_upper_target_margin_deg', 'raw_tracking_gate'))


def reject_custom_conditioning(value):
    """Only the exact four previously used ACT hardware-adapter fields."""
    if isinstance(value, dict):
        forbidden = FORBIDDEN_CONDITIONING_FIELDS.intersection(value)
        if forbidden:
            raise ValueError('Unreviewed ACT/VLA conditioning fields: ' + ', '.join(sorted(forbidden)))
        present = ALLOWED_ADAPTER_FIELDS.intersection(value)
        if present:
            if present != ALLOWED_ADAPTER_FIELDS:
                raise ValueError('The four ACT hardware-adapter fields must be present together')
            _profile({key:value[key] for key in ALLOWED_ADAPTER_FIELDS})
        if 'command_conditioning' in value:
            _profile(value['command_conditioning'])
            if (value.get('command_conditioning_id') != CONDITIONING_ID
                    or value.get('command_conditioning_sha256') != profile_sha256(value['command_conditioning'])):
                raise ValueError('ACT hardware-adapter profile identity mismatch')
        for nested in value.values():
            reject_custom_conditioning(nested)
    elif isinstance(value, list):
        for nested in value:
            reject_custom_conditioning(nested)


def validate_adapter_binding(plan, deployment):
    expected = read_json(HERE / 'pi05_execution_defaults.json')['command_conditioning']
    _profile(expected)
    if (plan.get('command_conditioning') != expected
            or {key:deployment.get(key) for key in ALLOWED_ADAPTER_FIELDS} != expected):
        raise ValueError('Plan and executor must contain the exact pinned ACT hardware-adapter profile')
    reject_custom_conditioning(plan)
    reject_custom_conditioning(deployment)
    from visual_gripper_gate import validate_config
    visual = read_json(HERE / 'pi05_execution_defaults.json')['vla_visual_gripper_gate']
    validate_config(visual)
    if plan.get('vla_visual_gripper_gate') != visual or deployment.get('vla_visual_gripper_gate') != visual:
        raise ValueError('Plan and deployment must pin the same reviewed VLA visual gripper candidate')
    from closure_sync import validate_config as validate_sync
    sync = read_json(HERE / 'pi05_execution_defaults.json')['vla_closure_sync']
    validate_sync(sync)
    if plan.get('vla_closure_sync') != sync or deployment.get('vla_closure_sync') != sync:
        raise ValueError('Plan and deployment must pin the same reviewed closure phase constraint')


def validate_limits(limits):
    """Pin the task-workspace candidate from the successful ACT physical range."""
    from lerobot_robot_piper_executor.guard import GuardLimits
    GuardLimits(**limits)
    if set(limits) != set(MAX_LIMITS):
        raise ValueError('Guard limits must contain exactly the existing reviewed fields')
    if limits != MAX_LIMITS:
        raise ValueError('The official-queue entry requires the exact task-workspace candidate and unchanged non-bound Guard values')
    for actual, (lo, hi) in zip(limits['bounds'], MAX_LIMITS['bounds'], strict=True):
        if actual[0] < lo or actual[1] > hi:
            raise ValueError('PI05 absolute bounds exceed the reviewed task-workspace candidate')
    for key in ('max_step', 'max_target_speed', 'max_tracking_error'):
        if any(actual > maximum for actual, maximum in zip(limits[key], MAX_LIMITS[key], strict=True)):
            raise ValueError(f'PI05 {key} exceeds the existing trial gate')
    for key in ('max_observation_age_s', 'max_command_age_s', 'max_command_gap_s'):
        if limits[key] > MAX_LIMITS[key]:
            raise ValueError(f'PI05 {key} exceeds the existing trial gate')


class IdentityProcessor:
    """Transport raw state/images and already unnormalized action values."""
    steps = ()

    def __call__(self, value):
        return value

    def reset(self):
        pass


@contextmanager
def remote_processors(context_module, expected_config):
    """Scope processor replacement to this one official rollout construction."""
    original = context_module.make_pre_post_processors

    def make_identity(*, policy_cfg, **_kwargs):
        if policy_cfg is not expected_config:
            raise RuntimeError('Official context requested an unexpected policy config')
        return IdentityProcessor(), IdentityProcessor()

    context_module.make_pre_post_processors = make_identity
    try:
        yield
    finally:
        context_module.make_pre_post_processors = original


def load_proxy_config(checkpoint):
    from lerobot.policies.pi05.configuration_pi05 import PI05Config
    config = PI05Config.from_pretrained(str(checkpoint), local_files_only=True,
        cli_overrides=['--device=cpu', '--push_to_hub=false'])
    config.pretrained_path = checkpoint
    expected = {'observation.state': (7,), 'observation.images.global': (3, 480, 640),
                'observation.images.wrist': (3, 480, 640)}
    if (config.type != 'pi05' or config.n_obs_steps != 1 or config.chunk_size != 50
            or config.num_inference_steps != 10 or config.n_action_steps != CHECKPOINT_SAVED_ACTION_STEPS
            or config.rtc_config is not None or config.use_relative_actions or config.use_peft
            or {k: v.shape for k, v in config.input_features.items()} != expected
            or config.output_features['action'].shape != (7,)):
        raise ValueError('Saved PI05 configuration does not match this robot/local GPU interface')
    # Explicit user runtime override; never edit the saved checkpoint config10.
    config.n_action_steps = RUNTIME_ACTION_STEPS
    # Only alias names change, without rescaling or changing the trained order.
    config.action_feature_names = list(ROLLOUT_KEYS)
    return config


def arguments():
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument('--check-only', action='store_true')
    mode.add_argument('--execute', action='store_true')
    parser.add_argument('--plan', type=Path)
    parser.add_argument('--approve-plan', help='Exact SHA256 of the reviewed, explicitly approved plan file')
    parser.add_argument('--deployment-config', type=Path)
    parser.add_argument('--readiness', type=Path)
    parser.add_argument('--readiness-reviewed', action='store_true')
    startup = parser.add_mutually_exclusive_group()
    startup.add_argument('--startup-command-json', type=Path)
    startup.add_argument('--existing-mode-evidence', type=Path)
    parser.add_argument('--startup-result', type=Path)
    parser.add_argument('--inference-config', type=Path, default=HERE / 'inference_config.json')
    parser.add_argument('--threads', type=int, default=8)
    args = parser.parse_args()
    args.plan_explicit = args.plan is not None
    args.deployment_explicit = args.deployment_config is not None
    args.plan = args.plan or HERE / 'grasp_plan.json'
    args.deployment_config = args.deployment_config or DEPLOYMENT / 'observe_config.json'
    if args.threads < 1 or args.threads > 25:
        parser.error('--threads must be in [1,25]')
    return args


def check_local(args):
    """Validate files and approval references, without network/hardware access."""
    from lerobot_robot_piper_executor.guard import Readiness, GuardLimits
    from rollout_act_piper import local_gate_status, read_startup_command
    plan, cloud = read_json(args.plan), read_json(args.inference_config)
    blockers = []
    from plan_workflow import verify_latest_preview, verify_review_content
    if plan.get('execution_ready') is not True:
        blockers.append('Latest preview does not establish execution readiness; refresh and resolve its reported rejections')
    for verify in (verify_latest_preview, verify_review_content):
        try:
            verify(plan)
        except (ValueError, RuntimeError, KeyError, OSError) as error:
            blockers.append(str(error))
    pin_file(plan, 'execution_defaults_sha256', HERE / 'pi05_execution_defaults.json', blockers)
    pin_file(plan, 'base_io_defaults_sha256', HERE / 'pi05_base_io_config.json', blockers)
    if not args.plan_explicit:
        blockers.append('explicit --plan PATH is required for execution')
    if not args.deployment_explicit:
        blockers.append('explicit --deployment-config PATH is required for execution')
    reject_custom_conditioning(plan)
    if plan.get('schema_version') != SCHEMA or plan.get('task') != TASK:
        raise ValueError('Plan schema/task does not match this PI05 red-cube task')
    from plan_workflow import validate_duration, validate_duration_binding
    duration = validate_duration(plan.get('duration_s'))
    if plan.get('trial_kind') != 'lerobot_native_queue_act_bridge_trial' or plan.get('queue_contract') != QUEUE_CONTRACT:
        raise ValueError('Plan must bind the official PI05 FIFO with explicit runtime20 override and source-slot contract')
    if plan.get('action_indices') != list(range(RUNTIME_ACTION_STEPS)):
        raise ValueError('Plan action indices must be exactly the explicitly requested twenty queue slots')
    for key, expected in [('n_action_steps', RUNTIME_ACTION_STEPS)]:
        if type(plan.get(key)) is not int or plan[key] != expected:
            raise ValueError(f'Plan {key} must be exactly {expected}')
    for key, expected in [('expected_model_sha256', MODEL_SHA256), ('expected_model_bytes', MODEL_BYTES)]:
        if plan.get(key) != expected or cloud.get(key) != expected:
            raise ValueError(f'Model identity mismatch: {key}')
    checkpoint = Path(plan.get('checkpoint', str(CHECKPOINT))).expanduser().resolve()
    if checkpoint != CHECKPOINT.resolve():
        raise ValueError('Plan must name the saved best B complete checkpoint used by this local GPU service')
    pin_file(plan, 'checkpoint_config_sha256', checkpoint / 'config.json', blockers)
    if (checkpoint / 'model.safetensors').stat().st_size != MODEL_BYTES or any(
            (checkpoint / name).exists() for name in ('adapter_model.safetensors', 'adapter_config.json')):
        raise ValueError('Local B must be the independently verified complete policy, without a LoRA adapter')
    pin_file(plan, 'runtime_config_sha256', Path(cloud['runtime_config']), blockers)
    pin_file(plan, 'inference_config_sha256', args.inference_config, blockers)
    deployment = read_json(args.deployment_config)
    duration = validate_duration_binding(plan, deployment)
    reject_custom_conditioning(deployment)
    validate_adapter_binding(plan, deployment)
    readiness, missing, io = local_gate_status(args, deployment, Readiness, GuardLimits)
    blockers.extend(missing)
    if deployment.get('guard_limits'):
        validate_limits(deployment['guard_limits'])
    pin_file(plan, 'deployment_config_sha256', args.deployment_config, blockers)
    if deployment.get('guard_limits') and plan.get('guard_limits') != deployment['guard_limits']:
        blockers.append('plan.guard_limits must match the pinned deployment config')
    force = deployment.get('normal_gripper_force_n')
    if deployment.get('execute') and (force != 1. or isinstance(force, bool)):
        raise ValueError('PI05 normal gripper force is restricted to the existing 1 N trial value')
    if deployment.get('execute') and (not isinstance(deployment.get('tx_ttl_s'), (int, float))
            or isinstance(deployment['tx_ttl_s'], bool) or not 0 < deployment['tx_ttl_s'] <= .12):
        raise ValueError('PI05 tx_ttl_s must not exceed the existing 120 ms delivery deadline')
    if args.readiness is not None:
        pin_file(plan, 'readiness_sha256', args.readiness, blockers)
    approval = plan.get('readiness', {}).get('approved_goal_path_parameters', '')
    if not isinstance(approval, str) or not approval.strip():
        blockers.append('plan.readiness.approved_goal_path_parameters is empty: physical grasp has not been approved')
    elif readiness is None or readiness.approved_goal_path_parameters != approval:
        blockers.append('plan approval text must exactly match the pinned readiness evidence')
    plan_hash = sha256_file(args.plan)
    if not args.approve_plan:
        blockers.append('--approve-plan SHA256 is required for this exact reviewed plan')
    elif args.approve_plan != plan_hash:
        raise ValueError('--approve-plan does not match the actual plan bytes')
    startup_argv = None
    if args.startup_command_json is not None:
        startup_argv = read_startup_command(args.startup_command_json)
        pin_file(plan, 'startup_command_sha256', args.startup_command_json, blockers)
        if args.startup_result is None:
            blockers.append('--startup-result must save the actual preflight result')
    elif args.existing_mode_evidence is not None:
        read_json(args.existing_mode_evidence)
        pin_file(plan, 'existing_mode_evidence_sha256', args.existing_mode_evidence, blockers)
    else:
        blockers.append('explicit startup command/result or existing mode evidence is required')
    config = load_proxy_config(checkpoint)
    return {'plan': plan, 'cloud': cloud, 'plan_sha256': plan_hash, 'checkpoint': checkpoint,
            'duration': float(duration), 'blockers': blockers, 'readiness': readiness,
            'startup_argv': startup_argv, 'policy_config': config, 'io': io}


def validate_compiled_runtime_health(metadata):
    """Pin the independent compiled service without changing model identity."""
    if (metadata.get('model_sha256') != MODEL_SHA256 or metadata.get('peft_present') is not False
            or metadata.get('model_format') != 'full-native-PI05'
            or metadata.get('chunk_size') !=50 or metadata.get('num_inference_steps') !=10
            or metadata.get('compile_model') is not True
            or metadata.get('compile_target') != 'denoise_step'
            or metadata.get('compile_mode') != 'reduce-overhead'
            or metadata.get('float32_matmul_precision') != 'highest'):
        raise RuntimeError('Compiled cloud B health metadata does not match the approved full PI05 expert model')


def main():
    args = arguments()
    for path in (str(OFFICIAL_ADAPTER), str(DEPLOYMENT), '/home/aup/piper SDK/pyAgxArm'):
        if path not in sys.path:
            sys.path.insert(0, path)
    os.environ['HF_HUB_OFFLINE'] = '1'
    os.environ['HF_DATASETS_OFFLINE'] = '1'
    os.environ['HF_HUB_DISABLE_TELEMETRY'] = '1'
    if importlib.metadata.version('lerobot') != '0.6.1':
        raise RuntimeError('This proxy requires the reviewed LeRobot 0.6.1 interfaces')
    torch.set_num_threads(args.threads)
    torch.set_num_interop_threads(1)
    checked = check_local(args)
    report = {'mode': 'execute' if args.execute else 'check-only',
        'plan': str(args.plan.resolve()), 'plan_sha256': checked['plan_sha256'],
        'checkpoint': str(checked['checkpoint']), 'second_model_loaded_in_robot_process': False,
        'policy_proxy': 'CloudExpertPolicy: official PI05 select_action/reset FIFO; official raw FIFO plus existing ACT four-field hardware adaptation',
        'control_loop': 'official LeRobot 0.6.1 base/sync with guarded Piper executor',
        'task': TASK, 'duration_s': checked['duration'], 'n_action_steps': checked['policy_config'].n_action_steps,
        'queue_contract': QUEUE_CONTRACT,
        'source_time': 'original chunk source retained; scheduled slot=source+index/fps, 400ms slot lateness; current feedback checked independently',
        'target_fps': 20., 'target_fps_is_only_scheduling_ceiling': True,
        'actual_20hz_verified': False, 'return_to_initial_position': False,
        'automatic_reset_home': False, 'hardware_constructed_during_check': False,
        'local_model_service_contacted_during_check': False, 'execution_gates_open': not checked['blockers'],
        'execution_gate_blockers': checked['blockers']}
    print(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False), flush=True)
    if not args.execute:
        print('CHECK ONLY: no cloud connection, robot, camera, SSH, CAN, arm or target operation.', flush=True)
        return 0
    if checked['blockers']:
        raise RuntimeError('Execution blocked: ' + '; '.join(checked['blockers']))
    from client import InferenceClient, PassiveSource, warmup_recorded
    from plan_workflow import compare_current_snapshot
    import lerobot.rollout.context as context_module
    from lerobot.rollout import BaseStrategyConfig, RolloutConfig, SyncInferenceConfig
    from lerobot_robot_piper_executor.robot import PiperPolicyExecutorConfig
    from rollout_expert import execute_official
    logging.basicConfig(level=logging.INFO, format='%(asctime)s %(levelname)s %(name)s: %(message)s', force=True)
    inference_client = InferenceClient(port=checked['cloud']['local_port'], timeout_s=.4)
    if args.startup_result is not None:
        inference_client.warmth_report_path = args.startup_result.with_name('connection_warmth_statistics.json')
    with inference_client as client:
        metadata = client.health['metadata']
        validate_compiled_runtime_health(metadata)
        # Existing real-recording prediction is discarded before any live
        # source is opened, robot constructed or lease armed. No FIFO is filled.
        recorded_warmup = warmup_recorded(client)
        logging.getLogger('piper_expert_newcloud_pi05').info(
            'recorded_remote_warmup before live acquisition; discarded, not live evidence: %s',
            json.dumps(recorded_warmup, ensure_ascii=False, allow_nan=False))
        # Refuse changed starting poses before any physical startup operation;
        # the warmup never replaces reviewed poses or live source timestamps.
        source = PassiveSource()
        try:
            source.open()
            compare_current_snapshot(checked['plan'], source.capture()['source_metadata'])
        finally:
            source.close()
        policy = CloudExpertPolicy(checked['policy_config'], client).eval().requires_grad_(False)
        robot = PiperPolicyExecutorConfig(id='piper_pi05_lerobot_queue_executor', deployment_config=args.deployment_config.resolve())
        cfg = RolloutConfig(robot=robot, policy=policy.config, strategy=BaseStrategyConfig(),
            inference=SyncInferenceConfig(), dataset=None, teleop=None, fps=20.,
            duration=checked['duration'], device='cpu', task=TASK,
            return_to_initial_position=False, interpolation_multiplier=1,
            use_torch_compile=False, display_data=False, play_sounds=False)
        with remote_processors(context_module, policy.config):
            execute_official(cfg, policy, checked['readiness'],
                startup_argv=checked['startup_argv'], startup_result=args.startup_result,
                existing_mode_evidence=args.existing_mode_evidence)
    return 0


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except Exception as error:
        print(f'Cloud expert PI05 rollout stopped: {type(error).__name__}: {error}', file=sys.stderr)
        raise SystemExit(1)
