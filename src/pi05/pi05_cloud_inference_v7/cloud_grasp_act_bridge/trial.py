#!/usr/bin/env python3
"""Check or stage/run one explicitly approved PI05 trial; default is local check."""
from __future__ import annotations

import argparse
from datetime import datetime
import json
from pathlib import Path
import shlex
import subprocess
import sys
from zoneinfo import ZoneInfo

HERE = Path(__file__).resolve().parent


def arguments():
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument('--check-only', action='store_true')
    mode.add_argument('--execute', action='store_true')
    parser.add_argument('--approve-plan', help='Exact SHA256 of the reviewed and explicitly approved plan')
    parser.add_argument('--onsite-conditions-confirmed', action='store_true',
        help='Unchanged power, no other CAN control traffic before execution and continued source supervision, supported empty follower, hands clear, immediate power cutoff, unchanged approved scene/path')
    return parser.parse_args()


def main():
    args = arguments()
    import launch_grasp as launch
    for path in (str(launch.DEPLOYMENT), str(launch.OFFICIAL_ADAPTER), '/home/aup/piper SDK/pyAgxArm'):
        if path not in sys.path:
            sys.path.insert(0, path)
    plan_path, trial = HERE / 'grasp_plan.json', HERE / 'trial'
    plan = launch.read_json(plan_path)
    from nativequeuepolicy import QUEUE_CONTRACT
    if (plan.get('trial_kind') != 'lerobot_native_queue_act_bridge_trial'
            or plan.get('queue_contract') != QUEUE_CONTRACT):
        raise ValueError('This entry requires the separately reviewed official PI05 queue contract')
    if plan.get('trial_directory') != str(trial) or plan.get('remote_directory') != '/home/ubuntu/kr260-camera/piper_pi05_expert_act_bridge_v8_executor':
        raise ValueError('Plan must pin this independent local and receiver trial directory')
    run_dir = trial / 'runs' / datetime.now(ZoneInfo('Asia/Singapore')).strftime('%Y%m%d_%H%M%S_%f')
    launch_args = argparse.Namespace(plan=plan_path, plan_explicit=True,
        deployment_config=trial / 'execute_config.json', deployment_explicit=True,
        approve_plan=args.approve_plan, readiness=trial / 'readiness.json', readiness_reviewed=True,
        startup_command_json=trial / 'startup_command.json', startup_result=run_dir / 'startup_result.json',
        existing_mode_evidence=None, inference_config=HERE / 'inference_config.json')
    checked = launch.check_local(launch_args)
    deployment = launch.read_json(trial / 'execute_config.json')
    launch.validate_adapter_binding(plan, deployment)
    extra_pins = {'io_config_sha256': trial / 'io_config.json',
        'j_hold_review_sha256': trial / 'j_hold_review.json',
        'receiver_policy_sha256': trial / 'pi05_trial_policy.json'}
    for field, path in extra_pins.items():
        if plan.get(field) != launch.sha256_file(path):
            raise ValueError(f'Trial configuration SHA256 mismatch: {field}')
    io = launch.read_json(trial / 'io_config.json')
    review = launch.read_json(trial / 'j_hold_review.json')
    policy = launch.read_json(trial / 'pi05_trial_policy.json')
    if policy.get('readiness') != plan.get('readiness') or policy.get('limits') != plan.get('guard_limits'):
        raise ValueError('Receiver policy does not match the pinned plan/readiness/limits')
    if io['transport']['expected_identity'] != plan['preview_evidence']['receiver_identity'] or review['expected_identity'] != io['transport']['expected_identity']:
        raise ValueError('Receiver physical identity does not match the preview and trial review')
    if (review.get('reuse_enabled') is not True or 'initial_target_deg' in review
            or 'gripper_hold_force_n' in review or review.get('warm_gripper_target_m') != review['expected_pose_deg_m'][6]
            or not 0 <= review['warm_gripper_target_m'] <= .1):
        raise ValueError('PI05 trial permits only enabled current-pose/current-width takeover')
    expected_names = {'remote_bridge.py', 'guard.py', 'reset_piper_once.py', 'preflight_j_hold.py',
                      'j_hold_review.json', 'pi05_trial_policy.json'}
    staging = plan.get('staging_files', {})
    if set(staging) != expected_names:
        raise ValueError('Plan must pin exactly the independent receiver helper and trial files')
    staged_paths = []
    for name, entry in staging.items():
        path = Path(entry['local_path'])
        if path.name != name or entry.get('sha256') != launch.sha256_file(path):
            raise ValueError(f'Staging source changed: {name}')
        staged_paths.append(path)
    if args.execute and not args.onsite_conditions_confirmed:
        checked['blockers'].append('--onsite-conditions-confirmed must explicitly affirm the displayed site conditions')
    if args.execute:
        if not plan['readiness'].get('approved_goal_path_parameters', '').strip():
            checked['blockers'].append('physical grasp approval remains empty')
        if checked['blockers']:
            raise RuntimeError('No upload or robot connection performed; execution blocked: ' + '; '.join(checked['blockers']))
        from plan_workflow import verify_latest_preview, verify_review_content, compare_current_snapshot
        verify_latest_preview(plan)
        verify_review_content(plan)
        from client import PassiveSource
        source = PassiveSource()
        try:
            source.open()
            compare_current_snapshot(plan, source.capture()['source_metadata'])
        finally:
            source.close()
        from preflight_j_hold import validate_review
        validate_review(review)
        if review.get('approval_note') != plan['readiness']['approved_goal_path_parameters']:
            raise ValueError('Startup approval must exactly match the explicitly approved plan')
    command = [sys.executable, '-u', '-B', str(HERE / 'launch_grasp.py'),
        '--execute' if args.execute else '--check-only', '--plan', str(plan_path),
        '--deployment-config', str(trial / 'execute_config.json'),
        '--readiness', str(trial / 'readiness.json'), '--readiness-reviewed',
        '--startup-command-json', str(trial / 'startup_command.json'),
        '--startup-result', str(run_dir / 'startup_result.json')]
    if args.approve_plan:
        command += ['--approve-plan', args.approve_plan]
    if not args.execute:
        print('LOCAL CHECK: --readiness-reviewed is a parser review flag only; blank physical approval remains blocked.', flush=True)
        return subprocess.run(command, check=False).returncode
    # Every local approval/config gate has passed. Uploading is reachable only
    # on the operator's explicit --execute/--approve-plan/onsite invocation.
    remote = plan['remote_directory']
    transport = io['transport']
    common = ['-o', 'BatchMode=yes', '-o', 'StrictHostKeyChecking=yes',
              '-o', 'ConnectTimeout=5', '-o', 'HostName=' + transport['ssh_hostname']]
    ssh = ['ssh', '-T', *common, transport['ssh_host']]
    subprocess.run(ssh + ['mkdir -p ' + shlex.quote(remote)], check=True, timeout=15)
    subprocess.run(['scp', '-q', *common, *[str(path) for path in staged_paths],
        transport['ssh_host'] + ':' + remote + '/'], check=True, timeout=30)
    # Verify the receiver staged the exact reviewed bytes before invoking the
    # launcher. This verification does not open CAN or enable anything.
    remote_hashes = shlex.join(['sha256sum', '--', *[remote + '/' + path.name for path in staged_paths]])
    received = subprocess.run(ssh + [remote_hashes], capture_output=True, text=True,
                              check=True, timeout=15).stdout.splitlines()
    actual = {}
    for line in received:
        checksum, remote_path = line.split(maxsplit=1)
        actual[Path(remote_path.strip()).name] = checksum
    if actual != {name: entry['sha256'] for name, entry in staging.items()}:
        raise RuntimeError('Receiver staging checksum mismatch; no rollout started')
    run_dir.mkdir(parents=True, exist_ok=False)
    (run_dir / 'invocation.json').write_text(json.dumps({
        'plan_sha256': checked['plan_sha256'], 'onsite_conditions_confirmed': True,
        'launcher_argv': command, 'staged_files': actual}, ensure_ascii=False, indent=2) + '\n')
    print('Approved trial logs: ' + str(run_dir), flush=True)
    from control_logging import AsyncTerminalMirror, persist_and_mirror, write_statistics, stop_failed_log_child
    with (run_dir / 'rollout.log').open('w', encoding='utf-8') as log:
        process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        mirror = AsyncTerminalMirror()
        try:
            persist_and_mirror(process.stdout, log, mirror)
            return process.wait()
        except KeyboardInterrupt:
            # The child receives terminal SIGINT too; allow existing cleanup.
            try:
                return process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.terminate()
                try:
                    return process.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    process.kill()
                    return process.wait(timeout=2)
        except BaseException as error:
            # Never leave an indefinite child running after raw-log failure.
            # No Popen context manager may add an unbounded implicit wait.
            stop_failed_log_child(process, error)
            raise
        finally:
            if process.stdout is not None:
                process.stdout.close()
            write_statistics(run_dir / 'terminal_mirror_statistics.json', mirror.stop())


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except Exception as error:
        print(f'PI05 trial stopped: {type(error).__name__}: {error}', file=sys.stderr)
        raise SystemExit(1)
