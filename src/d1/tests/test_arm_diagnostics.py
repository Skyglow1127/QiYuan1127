"""Saved arm failures must remain understandable across GUI process versions."""
import json
from pathlib import Path
import subprocess
import sys

import pytest

APP = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(APP))
from app.arm_diagnostics import execution_error_fields, format_arm_execution_error
from app.backend import ProcessFailure, _executor_reason


def diagnostic():
    return dict(actual_sdk=[.6, -84.5, 72.6, 4.4, 25.9, -9., 36.2],
                reference_sdk=[.54, -84.157, 72.087, 4.321, 24.492, -8.551, 36.],
                wait_s=2.004, measured_change_deg=.8)


def test_saved_failure_keeps_actual_reference_and_does_not_invent_axis_motion():
    detail = diagnostic()
    reason = format_arm_execution_error('Fixed-pose feedback did not catch up', detail)
    for text in ('J4', '当前阶段参考 24.492°', '实际反馈 25.900°', '偏差 1.408°', '已停止'):
        assert text in reason
    reason = format_arm_execution_error('Accepted commands but no measured joint progress', detail)
    assert '六轴最大反馈变化 0.800°' in reason
    assert 'J4 反馈变化' not in reason


@pytest.mark.parametrize('error', ['Fixed-pose feedback did not catch up',
                                 'Accepted commands but no measured joint progress',
                                 'Fixed-pose feedback wait budget exceeded'])
def test_waiting_explains_unsent_next_candidate_and_advancement_limit(error):
    detail = diagnostic()
    detail.update(next_reference_sdk=[.5, -83.8, 71.8, 4.3, 22.9, -8.5, 36.],
                  next_reference_max_error_deg=3., next_reference_joint=4,
                  progression_limit_deg=2.5)
    reason = format_arm_execution_error(error, detail)
    for text in ('J4 下一已批准路径点目标 22.900°', '实际反馈 25.900°',
                 '偏差 3.000°', '中间路径点推进门槛 ≤2.5°'):
        assert text in reason
    assert '当前阶段参考 22.900°' not in reason
    assert '偏差 1.408°' not in reason
    # A tracking failure always compares feedback with the reference already sent.
    tracking = format_arm_execution_error('Tracking error >3 degrees', detail)
    assert '当前阶段参考 24.492°' in tracking and '下一已批准' not in tracking


@pytest.mark.parametrize('detail', [None, [], {}, {'actual_sdk': [float('nan')] * 7},
                                  {'actual_sdk': [False] * 7, 'reference_sdk': [0.] * 7}])
def test_incomplete_or_malformed_legacy_diagnostic_still_explains_failure(detail):
    assert '等待实际反馈跟进超时' in format_arm_execution_error(
        'Fixed-pose feedback did not catch up', detail)


@pytest.mark.parametrize('error', ['Camera disconnected', '执行器已停止，需检查反馈', ''])
def test_unknown_and_already_chinese_errors_are_preserved(error):
    assert format_arm_execution_error(error, diagnostic()) == error


@pytest.mark.parametrize('raw_age,pending,rejected,cause', [
    (2.01, False, 0, '原始关节反馈已停更'),
    (.1, True, 3, '原始反馈仍在到达，但连续未通过速度筛选'),
    (.1, False, 3, ''),
    (.1, True, 0, ''),
    (None, True, 3, ''),
])
def test_feedback_hard_timeout_only_explains_cause_proven_by_diagnostics(raw_age, pending, rejected, cause):
    error = RuntimeError('Angle feedback recovery timeout')
    error.feedback_diagnostics = dict(final_raw_feedback_age_s=raw_age,
        final_valid_feedback_age_s=2.01, feedback_pending_at_end=pending,
        feedback_rejected_count=rejected, feedback_gap_timeout_s=2.)
    fields = execution_error_fields(error)
    reason = fields['error']
    assert fields['error_code'] == str(error)
    assert fields['feedback_failure_diagnostics'] == error.feedback_diagnostics
    assert 'execution_diagnostics' not in fields
    assert '距最后有效关节反馈 2.01 秒，超过 2 秒仍未恢复' in reason
    assert '已停止后续目标' in reason
    if cause:
        assert cause in reason
    else:
        assert '原始关节反馈已停更' not in reason
        assert '速度筛选' not in reason
    assert ('查看筛选诊断' in reason) == ('速度筛选' in cause)


def test_feedback_timeout_preserves_independent_execution_diagnostics():
    error = RuntimeError('Angle feedback recovery timeout')
    error.execution_diagnostics = diagnostic()
    error.feedback_diagnostics = dict(final_raw_feedback_age_s=.05, final_valid_feedback_age_s=2.1,
        feedback_pending_at_end=True, rejected_count=4, gap_timeout_s=2.)
    fields = execution_error_fields(error)
    assert fields['execution_diagnostics'] == diagnostic()
    assert '原始反馈仍在到达' in fields['error']
    assert 'actual_sdk' not in fields['feedback_failure_diagnostics']
    assert 'final_raw_feedback_age_s' not in fields['execution_diagnostics']


@pytest.mark.parametrize('feedback', [None, [], {},
    {'feedback_gap_timeout_s': False, 'final_raw_feedback_age_s': float('nan')},
    {'feedback_gap_timeout_s': -1, 'final_valid_feedback_age_s': float('inf')}])
def test_feedback_timeout_without_valid_metadata_does_not_invent_transport_cause(feedback):
    reason = format_arm_execution_error('Angle feedback recovery timeout', feedback_diagnostic=feedback)
    assert '超过 2 秒且未恢复' in reason and '已停止后续目标' in reason
    assert '停更' not in reason and '速度筛选' not in reason


def test_late_frame_after_recovery_deadline_is_not_reported_as_current_stale_feedback():
    reason = format_arm_execution_error('Angle feedback recovery timeout', feedback_diagnostic={
        'final_raw_feedback_age_s': .01, 'final_valid_feedback_age_s': .01,
        'feedback_gap_timeout_s': 2.})
    assert '未在 2 秒恢复窗口内及时恢复' in reason
    assert '距最后有效关节反馈 0.01 秒，超过' not in reason
    assert '停更' not in reason


def test_unknown_error_keeps_original_even_when_feedback_diagnostics_exist():
    error = RuntimeError('Some new transport error')
    error.feedback_diagnostics = {'final_raw_feedback_age_s': 99.}
    fields = execution_error_fields(error)
    assert fields['error'] == fields['error_code'] == str(error)
    assert fields['feedback_failure_diagnostics'] == error.feedback_diagnostics


def test_child_record_retains_raw_error_code_and_backend_does_not_translate_twice(tmp_path):
    error = TimeoutError('Fixed-pose feedback did not catch up')
    error.execution_diagnostics = diagnostic()
    fields = execution_error_fields(error)
    assert fields['error_code'] == str(error)
    assert '当前阶段参考 24.492°' in fields['error']
    assert fields['execution_diagnostics'] == error.execution_diagnostics
    assert _executor_reason(fields, tmp_path / 'result.json') == fields['error']
    # The parent consumes the child's localized report directly, as an older GUI
    # that lacks the new English-to-Chinese mapping also can.
    output = tmp_path / 'execution'
    path = output / '01_fixed_pregrasp/executed/result.json'
    path.parent.mkdir(parents=True)
    path.write_text(json.dumps(dict(stage='fixed_pregrasp', completed=False, **fields)))
    failure = ProcessFailure(['run_grasp_cycle.py', '--output', output], 1,
                             tmp_path / 'execution.log')
    assert fields['error'] in str(failure)
    assert 'Fixed-pose feedback did not catch up' not in str(failure)


def test_formatter_is_importable_without_ros_numpy_or_backend():
    code = '''import sys
sys.path.insert(0, sys.argv[1])
from app.arm_diagnostics import execution_error_fields
result = execution_error_fields(TimeoutError("Fixed-pose feedback did not catch up"))
assert "超时" in result["error"]
assert result["error_code"] == "Fixed-pose feedback did not catch up"
assert not any(name in sys.modules for name in ("app.backend", "numpy", "rclpy", "serial"))
'''
    completed = subprocess.run([sys.executable, '-I', '-c', code, str(APP)],
                               capture_output=True, text=True, check=False)
    assert completed.returncode == 0, completed.stderr
