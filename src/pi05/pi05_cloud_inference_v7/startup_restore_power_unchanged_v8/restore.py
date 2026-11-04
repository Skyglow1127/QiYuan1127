#!/usr/bin/env python3
"""Review-pinned empty-gripper restoration. Default is entirely local/check-only."""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import importlib.util
import json
import math
import os
from pathlib import Path
import shlex
import signal
import subprocess
import sys
import time

HERE = Path(__file__).resolve().parent


def read(path):
    return json.loads(Path(path).read_text(encoding='utf-8'))


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def content_id(plan):
    value = dict(plan)
    value.pop('plan_id', None)
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(',', ':'),
        ensure_ascii=False, allow_nan=False).encode()).hexdigest()


def verify(plan, directory=HERE):
    if plan.get('schema') != 'pi05-empty-gripper-start-restore/v1' or plan.get('plan_id') != content_id(plan):
        raise ValueError('Restoration plan identity changed; explicit fresh review is required')
    if plan.get('approval_pending') is not True:
        raise ValueError('The frozen review plan must retain its pending-approval state')
    if plan['targets_m'] != [.025, .0175, .010] or plan['joint_hold_max_deviation_deg'] != .5:
        raise ValueError('Unexpected restoration sequence or current-joint hold limit')
    if plan['speed_percent'] != 5 or plan['force_n'] != 1 or plan['maximum_actual_gripper_step_m'] != .008:
        raise ValueError('Unexpected restoration control parameters')
    for name, expected in plan['reviewed_files_sha256'].items():
        path = Path(directory) / name
        if path.resolve().parent != Path(directory).resolve():
            raise ValueError('Only exact files within the isolated deployment are allowed')
        if sha(path) != expected:
            raise ValueError('Reviewed file changed: ' + name)
    config = read(Path(directory) / 'restore_config.json')
    if config['expected_identity'] != plan['expected_identity']:
        raise ValueError('Endpoint identity changed')
    expected = plan['expected_start_pose_deg_m']
    if expected != [0., 0., 0., 0., 0., 0., .0329]:
        raise ValueError('This review only covers the recorded all-zero joints and32.9mm opening')
    if config['joint_bounds_deg'] != plan['joint_bounds_deg']:
        raise ValueError('Joint limits changed')
    if config['owner_path'] != '/tmp/piper_policy_executor_can0.lock':
        raise ValueError('Shared executor ownership lock must be retained')
    return config


def authorize(args, plan):
    if (args.approve_plan != plan['plan_id'] or not args.current_setup_approved
            or not args.approval_note or not args.approval_note.strip()):
        raise ValueError('Execution needs this exact plan ID, explicit approval text and current setup approval; power state remains unchanged')


def phase_review(plan, config, phase, approval):
    previous = plan['expected_start_pose_deg_m'][6] if phase == 0 else plan['targets_m'][phase-1]
    return {
        'expected_identity': config['expected_identity'],
        'joint_bounds_deg': config['joint_bounds_deg'],
        'expected_pose_deg_m': [*plan['expected_start_pose_deg_m'][:6], previous],
        'start_joint_tolerance_deg': .5,
        'start_gripper_tolerance_m': .001,
        'max_deviation_deg': .5,
        # Required field of the unchanged helper. Its reuse_enabled branch
        # does not enable/disable motors, including on faults.
        'support_and_disable_on_fault_reviewed': True,
        'approval_note': approval,
        'reuse_enabled': True,
        'takeover_speed_percent': 5,
        'warm_gripper_target_m': plan['targets_m'][phase],
    }


def actual_step_guard(sample, review, anchor=None):
    pose = sample['pose']
    if (len(pose) != 7 or any(type(v) not in (int, float) or not math.isfinite(v) for v in pose)
            or len(sample['enabled']) != 6 or not all(sample['enabled'])):
        raise ValueError('Seven finite live values and all six enabled motors are required')
    if not 0 <= pose[6] <= .1:
        raise ValueError('Actual gripper feedback outside0..100mm')
    if anchor is None and abs(pose[6]-review['expected_pose_deg_m'][6]) > review['start_gripper_tolerance_m']:
        raise ValueError('Actual starting gripper feedback differs from the reviewed phase by more than1mm')
    if any(abs(a-b) > .5 for a, b in zip(pose[:6], review['expected_pose_deg_m'][:6], strict=True)):
        raise ValueError('Joints no longer match the reviewed pose within0.5deg')
    if anchor is not None and any(abs(a-b) > .5 for a, b in zip(pose[:6], anchor[:6], strict=True)):
        raise ValueError('Current-joint hold deviation exceeds0.5deg')
    actual_um = round(pose[6]*1e6)
    target_um = round(review['warm_gripper_target_m']*1e6)
    if abs(target_um-actual_um) > 8000:
        raise ValueError('Phase target differs from actual gripper feedback by more than8mm')
    return {'actual_width_um': actual_um, 'target_width_um': target_um,
            'signed_step_um': target_um-actual_um}


def reuse_one_phase(helper, plan, config, phase, approval):
    """Wrap only the sender guard; reuse the existing hold transaction unchanged."""
    review = phase_review(plan, config, phase, approval)
    OriginalMonitor, OriginalSender = helper.ResetMonitor, helper.LimitedSender
    state = {'monitor': None, 'anchor': None, 'actual_step_checks': []}

    class PhaseMonitor(OriginalMonitor):
        def __init__(self, **kwargs):
            super().__init__(**kwargs)
            state['monitor'] = self

    class PhaseSender(OriginalSender):
        def send(self, label, can_id, data):
            monitor = state['monitor']
            # This is evaluated immediately before every send attempt. The
            # unchanged helper checks freshness, faults, enabled state and mode.
            sample = helper.check_enabled_sample(monitor, review, now=time.monotonic(), anchor=state['anchor'])
            check = actual_step_guard(sample, review, state['anchor'])
            if state['anchor'] is None:
                state['anchor'] = list(sample['pose'])
            if can_id == 0x159:
                if data != round(review['warm_gripper_target_m']*1e6).to_bytes(4, 'big', signed=True)+bytes.fromhex('03e80100'):
                    raise ValueError('Gripper payload differs from this reviewed phase at1N')
                state['actual_step_checks'].append(check)
            return super().send(label, can_id, data)

    helper.ResetMonitor, helper.LimitedSender = PhaseMonitor, PhaseSender
    try:
        result = helper.run_reuse_enabled(review=review,
            helper_path=str(HERE / 'remote_can_source.py'),
            interface=plan['expected_identity']['interface'], owner_path=config['owner_path'])
    finally:
        helper.ResetMonitor, helper.LimitedSender = OriginalMonitor, OriginalSender
    result['phase_number'] = phase+1
    result['reviewed_target_m'] = review['warm_gripper_target_m']
    result['actual_step_checks'] = state['actual_step_checks']
    return result


def run_sequence(plan, config, approval, helper):
    result = {'plan_id': plan['plan_id'], 'success': False, 'phases': [],
              'automatic_retry': False, 'model_or_grasp_actions': False,
              'motor_enable_disable_reset_zero_commands': False,
              'failure_caveat': 'Enable state is left unchanged; stopping sends is not proof of a physical stop.'}
    for phase in range(len(plan['targets_m'])):
        item = reuse_one_phase(helper, plan, config, phase, approval)
        result['phases'].append(item)
        if not item.get('success'):
            result['stopped_at_phase'] = phase+1
            return result
    result['success'] = True
    return result


def claim_once(path, value):
    fd = os.open(path, os.O_CREAT | os.O_EXCL | os.O_WRONLY, 0o600)
    with os.fdopen(fd, 'w', encoding='utf-8') as stream:
        json.dump(value, stream, ensure_ascii=False, indent=2)
        stream.write('\n')


def remote_execute(args, plan, config):
    authorize(args, plan)
    claim_once(HERE / 'execution_attempt.json', {'plan_id': plan['plan_id'],
        'approval_note': args.approval_note, 'current_setup_approved': True, 'leader_power_off_confirmed': False, 'power_changes_requested': False,
        'created_utc': datetime.now(timezone.utc).isoformat()})
    spec = importlib.util.spec_from_file_location('preflight_j_hold', HERE / 'preflight_j_hold.py')
    helper = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(helper)
    def terminate(_signum, _frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, terminate)
    signal.signal(signal.SIGHUP, terminate)
    result = run_sequence(plan, config, args.approval_note, helper)
    (HERE / 'execution_result.json').write_text(json.dumps(result, ensure_ascii=False, indent=2,
        allow_nan=False)+'\n', encoding='utf-8')
    print(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False))
    return 0 if result['success'] else 2


def local_execute(args, plan, config):
    authorize(args, plan)
    claim_once(HERE / 'execution_attempt.json', {'plan_id': plan['plan_id'],
        'approval_note': args.approval_note, 'current_setup_approved': True, 'leader_power_off_confirmed': False, 'power_changes_requested': False,
        'created_utc': datetime.now(timezone.utc).isoformat(), 'may_have_executed': True})
    transport = config['transport']
    options = ['-o', 'BatchMode=yes', '-o', 'StrictHostKeyChecking=yes',
               '-o', 'ConnectTimeout=5', '-o', 'HostName='+transport['hostname']]
    destination = transport['remote_directory']
    # mkdir without -p rejects any prior generation or uncertain attempt.
    subprocess.run(['ssh', '-T', *options, transport['ssh_host'],
        shlex.join(['mkdir', destination])], check=True, timeout=10)
    paths = [HERE / name for name in plan['reviewed_files_sha256']]
    subprocess.run(['scp', '-q', *options, *map(str, paths), str(HERE / 'restore_plan.json'),
        transport['ssh_host']+':'+destination+'/'], check=True, timeout=30)
    command = ['python3', '-S', destination+'/restore.py', '--remote-execute',
        '--approve-plan', plan['plan_id'], '--current-setup-approved',
        '--approval-note', args.approval_note]
    with (HERE / 'execution_stdout.json').open('x', encoding='utf-8') as output:
        completed = subprocess.run(['ssh', '-T', *options, transport['ssh_host'], shlex.join(command)],
            stdout=output, stderr=subprocess.STDOUT, timeout=25)
    print((HERE / 'execution_stdout.json').read_text(encoding='utf-8'))
    return completed.returncode


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument('--check-only', action='store_true')
    mode.add_argument('--execute', action='store_true')
    mode.add_argument('--remote-execute', action='store_true', help=argparse.SUPPRESS)
    parser.add_argument('--approve-plan')
    parser.add_argument('--current-setup-approved', action='store_true')
    parser.add_argument('--approval-note')
    args = parser.parse_args(argv)
    plan = read(HERE / 'restore_plan.json')
    config = verify(plan)
    if not (args.execute or args.remote_execute):
        print(json.dumps({'check_only': True, 'network_connections': 0, 'opens_can': False,
            'robot_commands_sent': 0, 'plan_id': plan['plan_id'], 'approval_pending': True,
            'targets_mm': [v*1000 for v in plan['targets_m']],
            'joint_targets': 'Hold freshly measured six-joint feedback only',
            'does_not_fix_or_validate_pi05_joint_predictions': True}, ensure_ascii=False, indent=2))
        return 0
    return remote_execute(args, plan, config) if args.remote_execute else local_execute(args, plan, config)


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except (Exception, KeyboardInterrupt) as error:
        print('Restoration stopped: '+(str(error) or type(error).__name__), file=sys.stderr)
        raise SystemExit(2)
