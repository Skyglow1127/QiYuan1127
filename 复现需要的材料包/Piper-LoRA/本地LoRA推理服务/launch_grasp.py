#!/usr/bin/env python3
"""Review a local PI05 plan; run only with matching explicit plan approval.

The default branch reads local files only. The actual control loop remains the
official LeRobot 0.6.1 base/sync loop and the existing guarded Piper executor.
This module delegates model weights to the separate local inference process.
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

HERE = Path(__file__).resolve().parent
PROJECT = Path('/home/aup/Lerobot/projects/piper')
DEPLOYMENT = PROJECT / 'deployment'
OFFICIAL_ADAPTER = PROJECT / 'official_adapter'
CHECKPOINT = Path('/home/aup/Lerobot/models/PI05/piper_lora/checkpoints/020000/pretrained_model')
TASK = 'Pick up the red cube and place it inside the black square on the left side of the table.'
SCHEMA = 'pi05-local-grasp-plan/v1'
CANONICAL_KEYS = tuple(f'joint_{i}.deg' for i in range(1, 7)) + ('gripper.m',)
ROLLOUT_KEYS = tuple(f'joint_{i}.pos' for i in range(1, 7)) + ('gripper.pos',)
BASE_SHA256 = '0eb11ca9587678c1d2ef8cf32807c29f8ce53a2bfdfc1aa4a4c96f16fca59b0f'
ADAPTER_SHA256 = '8e064a68c5991f1b1321e726d901968c516ec728971aa4b9eb1fc6d4213f14a1'
MAX_LIMITS = {
    'bounds': [[-60., 30.], [0., 120.], [-120., 0.], [-45., 45.], [-60., 65.], [-80., 60.], [0., .1]],
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


def reject_relative_cap(value):
    if isinstance(value, dict):
        if 'max_relative_target' in value:
            raise ValueError('max_relative_target is forbidden for this PI05 plan; raw actions must not be clipped')
        for nested in value.values():
            reject_relative_cap(nested)
    elif isinstance(value, list):
        for nested in value:
            reject_relative_cap(nested)


def validate_limits(limits):
    """Reject every bound, rate, step or age wider than the existing trial gate."""
    from lerobot_robot_piper_executor.guard import GuardLimits
    GuardLimits(**limits)
    if set(limits) != set(MAX_LIMITS):
        raise ValueError('Guard limits must contain exactly the existing reviewed fields')
    for actual, (lo, hi) in zip(limits['bounds'], MAX_LIMITS['bounds'], strict=True):
        if actual[0] < lo or actual[1] > hi:
            raise ValueError('PI05 absolute bounds exceed the existing conservative trial range')
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
def service_processors(context_module, expected_config):
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


class LocalPI05Policy(torch.nn.Module):
    """A weight-free proxy; one fresh local call and action[0] per tick."""
    def __init__(self, config, client):
        super().__init__()
        self.config, self.client = config, client
        self.calls = 0
        self.last_result = None

    @property
    def type(self):
        return 'pi05'

    def reset(self):
        # No cached action queue exists. The local server resets each request.
        self.last_result = None

    def select_action(self, observation):
        if observation.get('task') != TASK:
            raise RuntimeError('Unexpected task in the official inference pipeline')
        if self.config.n_action_steps != 1 or self.config.use_relative_actions:
            raise RuntimeError('Local proxy requires n_action_steps=1 and absolute actions')
        result = self.client.infer(observation['observation.state'],
                                   observation['observation.images.global'],
                                   observation['observation.images.wrist'], TASK)
        actions = np.asarray(result['actions'], dtype=np.float32)
        if actions.shape != (50, 7) or not np.isfinite(actions).all():
            raise RuntimeError('Local returned invalid action shape or non-finite values')
        self.calls += 1
        self.last_result = result
        logging.getLogger('piper_local_pi05').info(
            'Fresh local action0 request=%s model_ms=%.1f total_ms=%.1f; source clock remains on the robot observation',
            result.get('request_id'), result['timing_s']['model']*1000,
            result['timing_s']['input_ready_to_validated_actions']*1000)
        return torch.from_numpy(actions[0].copy()).unsqueeze(0)


def load_proxy_config(checkpoint):
    from lerobot.policies.pi05.configuration_pi05 import PI05Config
    config = PI05Config.from_pretrained(str(checkpoint), local_files_only=True,
        cli_overrides=['--device=cpu', '--use_amp=false', '--push_to_hub=false',
                       '--n_action_steps=1', '--num_inference_steps=10', '--compile_model=false'])
    config.pretrained_path = checkpoint
    expected = {'observation.state': (7,), 'observation.images.global': (3, 480, 640),
                'observation.images.wrist': (3, 480, 640)}
    if (config.type != 'pi05' or config.n_obs_steps != 1 or config.chunk_size != 50
            or config.num_inference_steps != 10 or config.use_relative_actions
            or {k: v.shape for k, v in config.input_features.items()} != expected
            or config.output_features['action'].shape != (7,)):
        raise ValueError('Saved PI05 configuration does not match this robot/local interface')
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
    parser.add_argument('--local-config', type=Path, default=HERE / 'local_config.json')
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
    plan, local = read_json(args.plan), read_json(args.local_config)
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
    reject_relative_cap(plan)
    if plan.get('schema_version') != SCHEMA or plan.get('task') != TASK:
        raise ValueError('Plan schema/task does not match this PI05 red-cube task')
    duration = plan.get('duration_s', 5.)
    if isinstance(duration, bool) or not isinstance(duration, (int, float)) or not math.isfinite(duration) or not 0 < duration <= 30:
        raise ValueError('PI05 duration_s must be finite and in (0,30]')
    for key, expected in [('action_index', 0), ('n_action_steps', 1)]:
        if type(plan.get(key)) is not int or plan[key] != expected:
            raise ValueError(f'Plan {key} must be exactly {expected}')
    for key, expected in [('expected_base_sha256', BASE_SHA256), ('expected_adapter_sha256', ADAPTER_SHA256)]:
        if plan.get(key) != expected or local.get(key) != expected:
            raise ValueError(f'Model identity mismatch: {key}')
    checkpoint = Path(plan.get('checkpoint', str(CHECKPOINT))).expanduser().resolve()
    if checkpoint != CHECKPOINT.resolve():
        raise ValueError('Plan must name the saved final PI05 checkpoint used by this local service')
    pin_file(plan, 'checkpoint_config_sha256', checkpoint / 'config.json', blockers)
    if sha256_file(checkpoint / 'adapter_model.safetensors') != ADAPTER_SHA256:
        raise ValueError('Local final LoRA adapter checksum mismatch')
    pin_file(plan, 'local_config_sha256', args.local_config, blockers)
    deployment = read_json(args.deployment_config)
    reject_relative_cap(deployment)
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
    return {'plan': plan, 'local': local, 'plan_sha256': plan_hash, 'checkpoint': checkpoint,
            'duration': float(duration), 'blockers': blockers, 'readiness': readiness,
            'startup_argv': startup_argv, 'policy_config': config, 'io': io}


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
        'checkpoint': str(checked['checkpoint']), 'local_large_model_loaded': False,
        'policy_proxy': 'LocalPI05Policy: original absolute action[0], no clipping/cache',
        'control_loop': 'official LeRobot 0.6.1 base/sync with guarded Piper executor',
        'task': TASK, 'duration_s': checked['duration'], 'n_action_steps': 1,
        'source_time': 'original robot observation acquisition; never replaced by local receive time',
        'target_fps': 20., 'target_fps_is_only_scheduling_ceiling': True,
        'actual_20hz_verified': False, 'return_to_initial_position': False,
        'automatic_reset_home': False, 'hardware_constructed_during_check': False,
        'local_contacted_during_check': False, 'execution_gates_open': not checked['blockers'],
        'execution_gate_blockers': checked['blockers']}
    print(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False), flush=True)
    if not args.execute:
        print('CHECK ONLY: no local connection, robot, camera, SSH, CAN, arm or target operation.', flush=True)
        return 0
    if checked['blockers']:
        raise RuntimeError('Execution blocked: ' + '; '.join(checked['blockers']))
    # This is still unarmed. Refuse stale starting poses before any startup
    # operation; never silently replace a reviewed gripper target or pose.
    from client import PassiveSource
    from plan_workflow import compare_current_snapshot
    source = PassiveSource()
    try:
        source.open()
        compare_current_snapshot(checked['plan'], source.capture()['source_metadata'])
    finally:
        source.close()
    from client import InferenceClient, MODEL_ID
    import lerobot.rollout.context as context_module
    from lerobot.rollout import BaseStrategyConfig, RolloutConfig, SyncInferenceConfig
    from lerobot_robot_piper_executor.robot import PiperPolicyExecutorConfig
    from rollout_act_piper import execute_official
    logging.basicConfig(level=logging.INFO, format='%(asctime)s %(levelname)s %(name)s: %(message)s', force=True)
    with InferenceClient(port=checked['local']['local_port'], timeout_s=.4) as client:
        metadata = client.health['metadata']
        if (metadata.get('model_id') != MODEL_ID or metadata.get('base_sha256') != BASE_SHA256
                or metadata.get('adapter_sha256') != ADAPTER_SHA256
                or metadata.get('chunk_size') != 50 or metadata.get('num_inference_steps') != 10):
            raise RuntimeError('Local health metadata does not match the approved PI05 model')
        # Warm local import/resize/encoding and the persistent local connection
        # before any Robot exists. This synthetic prediction is discarded; it
        # is never a robot observation, source timestamp or executable target.
        offline_warmup = client.infer(np.zeros(7, dtype=np.float32),
            np.zeros((480, 640, 3), dtype=np.uint8),
            np.zeros((480, 640, 3), dtype=np.uint8), TASK)
        logging.getLogger('piper_local_pi05').info(
            'offline_warmup complete before robot construction: request=%s total_ms=%.1f; synthetic actions discarded',
            offline_warmup.get('request_id'),
            offline_warmup['timing_s']['input_ready_to_validated_actions']*1000)
        del offline_warmup
        policy = LocalPI05Policy(checked['policy_config'], client).eval().requires_grad_(False)
        robot = PiperPolicyExecutorConfig(id='piper_local_pi05_executor', deployment_config=args.deployment_config.resolve())
        cfg = RolloutConfig(robot=robot, policy=policy.config, strategy=BaseStrategyConfig(),
            inference=SyncInferenceConfig(), dataset=None, teleop=None, fps=20.,
            duration=checked['duration'], device='cpu', task=TASK,
            return_to_initial_position=False, interpolation_multiplier=1,
            use_torch_compile=False, display_data=False, play_sounds=False)
        with service_processors(context_module, policy.config):
            execute_official(cfg, policy, checked['readiness'],
                startup_argv=checked['startup_argv'], startup_result=args.startup_result,
                existing_mode_evidence=args.existing_mode_evidence)
    return 0


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except Exception as error:
        print(f'Local PI05 rollout stopped: {type(error).__name__}: {error}', file=sys.stderr)
        raise SystemExit(1)
