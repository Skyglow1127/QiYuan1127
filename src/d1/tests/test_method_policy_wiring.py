"""Method preparation remains read-only and the chosen clock is approval-bound."""
import importlib.util
import json
from pathlib import Path
import sys

import numpy as np
import pytest

CONTROL = Path(__file__).resolve().parents[1] / 'runtime/Control'
sys.path.insert(0, str(CONTROL / 'tools'))


def load_module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


arm = load_module('method_policy_arm', CONTROL / 'ros2_ws/tools/run_arm_stage.py')
cycle = load_module('method_policy_cycle', CONTROL / 'tools/run_grasp_cycle.py')
observe = load_module('method_policy_observe', CONTROL / 'tools/observe_grasp.py')


def manifest(tmp_path, policy='ack_overlap_v1'):
    source = tmp_path / 'plan.json'
    source.write_text(json.dumps(dict(scheduling_policy=policy, progress_policy='feedback_paced')))
    return dict(spec=dict(fixed_ready=True, scheduling_policy=policy), stages=[
        dict(name='prepare_gripper'),
        dict(name='fixed_pregrasp', plan_file=str(source), scheduling_policy=policy)])


def test_clock_policy_must_match_source_stage_and_method(tmp_path):
    saved = manifest(tmp_path)
    cycle.validate_scheduling_policies(saved)
    saved['stages'][-1]['scheduling_policy'] = 'ack_then_period_v1'
    with pytest.raises(ValueError, match='differs'):
        cycle.validate_scheduling_policies(saved)
    saved = manifest(tmp_path)
    Path(saved['stages'][-1]['plan_file']).write_text(json.dumps(dict(
        scheduling_policy='ack_then_period_v1', progress_policy='feedback_paced')))
    with pytest.raises(ValueError, match='differs'):
        cycle.validate_scheduling_policies(saved)


def test_new_clock_cannot_leak_into_grasp_or_gripper(tmp_path):
    saved = manifest(tmp_path)
    saved['spec']['fixed_ready'] = False
    with pytest.raises(ValueError, match='restricted'):
        cycle.validate_scheduling_policies(saved)
    saved = manifest(tmp_path)
    saved['stages'][0]['scheduling_policy'] = 'ack_overlap_v1'
    with pytest.raises(ValueError, match='gripper'):
        cycle.validate_scheduling_policies(saved)
    for stage in ('pregrasp', 'approach', 'lift', 'close'):
        with pytest.raises(ValueError, match='fixed-pregrasp'):
            arm.source_scheduling_policy(dict(scheduling_policy='ack_overlap_v1',
                                             progress_policy='feedback_paced'), stage)


def test_existing_plan_keeps_original_clock():
    assert arm.source_scheduling_policy({}, 'pregrasp') == 'ack_then_period_v1'
    with pytest.raises(ValueError, match='Unsupported'):
        arm.source_scheduling_policy(dict(scheduling_policy='unknown'), 'fixed_pregrasp')


def snapshot():
    return dict(fault=None, q=np.array([3., -90.1, 87.9, -5.3, 12.3, 7.1, 41.]),
                q_time=10., status=dict(power_status=0, enable_status=0, error_status=0),
                status_time=dict(power_status=10., enable_status=10., error_status=10.))


def test_unpowered_read_only_observation_preserves_real_flags():
    saved = snapshot()
    with pytest.raises(RuntimeError, match='power_status'):
        observe.observation_feedback(saved, 10.1, allow_position_start=True)
    result = observe.observation_feedback(saved, 10.1, allow_position_start=True, allow_unpowered=True)
    assert result is saved['q']
    assert saved['status'] == dict(power_status=0, enable_status=0, error_status=0)


@pytest.mark.parametrize('failure', ['stale_angle', 'stale_status', 'fault', 'limit', 'device_error'])
def test_unpowered_planning_does_not_accept_unhealthy_feedback(failure):
    saved = snapshot()
    if failure == 'stale_angle':
        saved['q_time'] = 1.
    elif failure == 'stale_status':
        saved['status_time']['power_status'] = 1.
    elif failure == 'fault':
        saved['fault'] = 'link failure'
    elif failure == 'limit':
        saved['q'][1] = -91.
    else:
        saved['status']['error_status'] = 1
    with pytest.raises(RuntimeError):
        observe.observation_feedback(saved, 10.1, allow_position_start=True, allow_unpowered=True)


def test_arm_only_observation_does_not_contact_or_start_cameras(tmp_path, monkeypatch):
    class Readonly:
        commands = 0
        closed = False
        def __init__(self, *args): pass
        def snapshot(self): return snapshot()
        def now(self): return 10.1
        def close(self): Readonly.closed = True
    output = tmp_path / 'observation'
    monkeypatch.setattr(observe, 'CliTransport', Readonly)
    monkeypatch.setattr(observe.time, 'sleep', lambda _: None)
    monkeypatch.setattr(observe.urllib.request, 'urlopen',
                        lambda *a, **k: pytest.fail('Arm-only preparation must not access a camera'))
    monkeypatch.setattr(sys, 'argv', ['observe_grasp.py', '--allow-position-start',
        '--allow-unpowered-observation', '--arm-only-observation', '--output', str(output)])
    observe.main()
    report = json.loads((output / 'wrist_observation.json').read_text())
    assert report['hardware_commands_sent'] == 0 and report['status']['power_status'] == 0
    assert report['source_kind'] == 'arm_feedback_only' and report['image'] is None
    assert report['camera_observation_performed'] is False and Readonly.closed
