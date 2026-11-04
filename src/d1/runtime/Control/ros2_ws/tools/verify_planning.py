#!/usr/bin/env python3
"""Starts and cleans up isolated planning processes; never starts hardware."""
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

ws = Path(__file__).resolve().parents[1]
logs = ws.parent / 'logs'
logs.mkdir(exist_ok=True)
env = dict(os.environ, ROS_DOMAIN_ID='77')
processes, streams = [], []


def run(args, log, expected=0, timeout=120):
    with (logs / log).open('w') as stream:
        p = subprocess.Popen(args, env=env, cwd=ws, stdout=stream, stderr=subprocess.STDOUT, start_new_session=True)
        processes.append(p)
        try:
            result = p.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            os.killpg(p.pid, signal.SIGINT)
            raise RuntimeError(f'{log}: process timeout')
    if result != expected:
        raise RuntimeError(f'{log}: expected exit {expected}, got {result}')


try:
    # Do not join or kill another user's simulation graph.
    listing = subprocess.run(['ros2', 'node', 'list', '--no-daemon'], env=env,
                             capture_output=True, text=True, timeout=10)
    if listing.stdout.strip():
        raise RuntimeError('ROS domain 77 is in use; choose a separate session after existing simulation exits')
    for command, name in [(['ros2', 'launch', 'd1_pick_place', 'planning.launch.py'], 'moveit_planning.log'),
                          (['ros2', 'run', 'd1_pick_place', 'sim_state'], 'sim_state.log')]:
        stream = (logs / name).open('w')
        streams.append(stream)
        processes.append(subprocess.Popen(command, env=env, cwd=ws, stdout=stream,
                                          stderr=subprocess.STDOUT, start_new_session=True))
    # The task also uses bounded wait_for_server/service calls.
    time.sleep(2)
    cfg = ws / 'src/d1_pick_place/config'
    common = ['ros2', 'run', 'd1_pick_place', 'task', '--config', str(cfg/'simulation.yaml'),
              '--synthetic', str(cfg/'synthetic_scene.json')]
    run(common + ['--simulation-unlocked', '--output', str(logs/'moveit_synthetic_plan.json')],
        'moveit_synthetic_test.log')
    forbidden = logs / 'protected_must_not_produce_plan.json'
    # Never remove a pre-existing file; use a unique path instead.
    forbidden = forbidden.with_name(f'protected_must_not_produce_plan_{os.getpid()}.json')
    run(common + ['--output', str(forbidden)], 'moveit_protected_test.log', expected=2)
    if forbidden.exists():
        raise RuntimeError('Protected-joint case unexpectedly produced a plan')
    run([sys.executable, str(ws/'tools/check_scene.py')], 'moveit_collision_test.log')
    plan = json.loads((logs/'moveit_synthetic_plan.json').read_text())
    report = {'real_moveit_planner': True, 'synthetic_scene': True, 'hardware_executed': False,
              'planned_stages': [x['name'] for x in plan['stages']],
              'protected_case': 'rejected', 'collision_case': 'rejected',
              'global_scene_modified_by_plan_only': False}
    (logs/'ros2_validation.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report, indent=2))
finally:
    for p in reversed(processes):
        if p.poll() is None:
            os.killpg(p.pid, signal.SIGINT)
    for p in reversed(processes):
        try:
            p.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(p.pid, signal.SIGTERM)
            try:
                p.wait(timeout=3)
            except subprocess.TimeoutExpired:
                os.killpg(p.pid, signal.SIGKILL)
                p.wait()
    for stream in streams: stream.close()
