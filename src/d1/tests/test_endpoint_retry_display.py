"""The reviewed retry contract and recorded stop cause must remain explicit."""
import copy
import json
from pathlib import Path

import pytest

from app.arm_diagnostics import execution_error_fields, format_arm_execution_error
from app.backend import ProcessFailure, _executor_reason
from app.gui import endpoint_retry_summary, pregrasp_summary


POLICY = dict(mode='retry_approved_target_v1', retry_interval_s=1., max_seek_s=30.,
              stall_timeout_s=5., min_progress_deg=.2)


def review():
    return dict(operation='fixed_pregrasp', endpoint_policy=POLICY.copy(),
                parameters=dict(open_target=36.), stages=[dict(name='prepare_gripper'),
                    dict(name='fixed_pregrasp', gripper_target=36.,
                         sdk_plan=dict(sdk=[[0.] * 6, [0., -31.518, 45.833, 0., -14.328, 0.]]))])


def test_fixed_review_lists_retry_limits_and_keeps_j6_units_explicit():
    text = pregrasp_summary(review())
    for part in ('J0–J5 / °', 'J6 / SDK：36.00', '每 1 秒', '仍超 1°', '原批准目标',
                 '5 秒内偏差未改善 0.2°', '总计 30 秒', 'J6 和已到位轴不重发'):
        assert part in text


@pytest.mark.parametrize('policy', [None, {}, {'mode': 'some_future_policy'},
                                   {**POLICY, 'retry_interval_s': False},
                                   {**POLICY, 'max_seek_s': float('inf')},
                                   {**POLICY, 'min_progress_deg': -.2}])
def test_legacy_or_invalid_review_does_not_claim_retry_enabled(policy):
    prepared = review()
    prepared['endpoint_policy'] = policy
    assert endpoint_retry_summary(prepared) == ''
    assert '终点收敛' not in pregrasp_summary(prepared)
    assert 'J6 / SDK：36.00' in pregrasp_summary(prepared)


def test_full_grasp_does_not_inherit_fixed_pose_retry_description():
    prepared = review()
    prepared['operation'] = 'full_cycle'
    assert endpoint_retry_summary(prepared) == ''


def failure(code='Endpoint convergence stalled'):
    error = TimeoutError(code)
    error.execution_diagnostics = dict(joint=4, reference_sdk=[0., -31.518, 45.833, 0., -14.328, 0., 36.],
        actual_sdk=[-.2, -31.5, 46.2, .2, -13.2, -.1, 36.1], max_error_deg=1.128)
    error.feedback_diagnostics = dict(final_valid_feedback_age_s=.09, feedback_rejected_count=0)
    error.endpoint_diagnostics = dict(endpoint_retry_policy=POLICY.copy(), endpoint_seek_elapsed_s=5.01,
        endpoint_retry_rounds=4, endpoint_retry_commands=4, endpoint_no_progress_s=5.01,
        endpoint_best_error_deg=1.128, endpoint_initial_error_deg=1.128)
    return error


@pytest.mark.parametrize('code,reason', [('Endpoint convergence stalled', '重发后偏差仍未有效改善'),
    ('Endpoint convergence time budget exceeded', '超过批准的时间上限')])
def test_child_saves_specific_chinese_reason_and_independent_diagnostics(tmp_path, code, reason):
    error = failure(code)
    fields = execution_error_fields(error)
    assert fields['error_code'] == code
    for part in (reason, 'J4 批准终点 -14.328°', '实际反馈 -13.200°', '偏差 1.128°',
                 '允许 ≤1°', '已重发 4 轮、4 条关节指令', '耗时 5.01 秒', '未确认到位'):
        assert part in fields['error']
    assert fields['execution_diagnostics'] == error.execution_diagnostics
    assert fields['feedback_failure_diagnostics'] == error.feedback_diagnostics
    assert fields['endpoint_diagnostics'] == error.endpoint_diagnostics
    # Persisted child output must work for a running parent that only knows
    # how to consume a localized `error`, without importing the new formatter.
    output = tmp_path / 'execution'
    result = output / '01_fixed_pregrasp/executed/result.json'
    result.parent.mkdir(parents=True)
    result.write_text(json.dumps(dict(stage='fixed_pregrasp', completed=False, **fields)))
    assert _executor_reason(fields, result) == fields['error']
    parent_error = ProcessFailure(['run_grasp_cycle.py', '--output', output], 1, tmp_path / 'execution.log')
    assert fields['error'] in str(parent_error)
    assert code not in str(parent_error)
    assert all(cause not in fields['error'] for cause in ('静摩擦', '死区', '力矩不足', '通信断开'))


@pytest.mark.parametrize('diagnostic', [None, [], {}, {'reference_sdk': [0.] * 7,
                                                     'actual_sdk': [float('nan')] * 7}])
def test_incomplete_diagnostics_do_not_invent_an_axis_or_retry_count(diagnostic):
    text = format_arm_execution_error('Endpoint convergence stalled', diagnostic)
    assert '重发后偏差仍未有效改善' in text
    assert 'J4' not in text and '已重发' not in text


def test_raw_unknown_error_preserves_endpoint_record_without_guessing_a_reason():
    error = failure('New unrelated error')
    fields = execution_error_fields(error)
    assert fields['error'] == str(error)
    assert fields['endpoint_diagnostics'] == error.endpoint_diagnostics
    preserved = copy.deepcopy(fields)
    json.dumps(fields)
    assert fields == preserved
