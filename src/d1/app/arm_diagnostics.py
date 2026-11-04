"""Pure formatting of recorded arm failures; importing this opens no devices."""
import math


def _number(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def _angles(value):
    return isinstance(value, list) and len(value) == 7 and all(_number(v) for v in value)


def _reference_detail(diagnostic, *, next_point=False):
    actual = diagnostic.get('actual_sdk')
    reference = diagnostic.get('next_reference_sdk' if next_point else 'reference_sdk')
    if not (_angles(actual) and _angles(reference)):
        return ''
    joint = max(range(6), key=lambda i: abs(actual[i] - reference[i]))
    label = ('下一已批准路径点目标' if next_point else
             '最近实际下发目标' if diagnostic.get('tracking_reference_basis') == 'last_published_joint_targets'
             else '当前阶段参考')
    detail = (f'J{joint} {label} {reference[joint]:.3f}°，实际反馈 {actual[joint]:.3f}°，'
              f'偏差 {abs(actual[joint] - reference[joint]):.3f}°')
    limit = diagnostic.get('progression_limit_deg')
    if next_point and _number(limit) and limit > 0:
        detail += f'；中间路径点推进门槛 ≤{limit:g}°'
    return detail


def _feedback_timeout_detail(diagnostic):
    diagnostic = diagnostic if isinstance(diagnostic, dict) else {}
    raw_age = diagnostic.get('final_raw_feedback_age_s')
    valid_age = diagnostic.get('final_valid_feedback_age_s')
    pending = diagnostic.get('feedback_pending_at_end', diagnostic.get('feedback_pending'))
    rejected = diagnostic.get('feedback_rejected_count', diagnostic.get('rejected_count'))
    timeout = diagnostic.get('feedback_gap_timeout_s', diagnostic.get('gap_timeout_s', 2.))
    if not _number(timeout) or timeout <= .6:
        timeout = 2.
    filtered = (pending is True and type(rejected) is int and rejected > 0
                and _number(raw_age) and 0 <= raw_age <= .6)
    cause = ('原始反馈仍在到达，但连续未通过速度筛选；' if filtered else
             '原始关节反馈已停更；' if _number(raw_age) and raw_age > .6 else '')
    if _number(valid_age) and valid_age >= timeout:
        timing = f'距最后有效关节反馈 {valid_age:.2f} 秒，超过 {timeout:g} 秒仍未恢复'
    elif not _number(valid_age):
        timing = f'距最后有效关节反馈已超过 {timeout:g} 秒且未恢复'
    else:
        # A frame can arrive after the expired deadline. Do not misstate its age.
        timing = f'有效关节反馈未在 {timeout:g} 秒恢复窗口内及时恢复'
    return (cause + timing + '；已停止后续目标；'
            + ('查看筛选诊断' if filtered else '查看反馈恢复记录'))


def _endpoint_retry_detail(diagnostic, endpoint_diagnostic):
    actual = diagnostic.get('actual_sdk')
    reference = diagnostic.get('reference_sdk')
    parts = []
    if _angles(actual) and _angles(reference):
        joint = max(range(6), key=lambda i: abs(actual[i] - reference[i]))
        parts.append(f'J{joint} 批准终点 {reference[joint]:.3f}°，实际反馈 {actual[joint]:.3f}°，'
                     f'偏差 {abs(actual[joint] - reference[joint]):.3f}°（允许 ≤1°）')
    rounds = endpoint_diagnostic.get('endpoint_retry_rounds')
    commands = endpoint_diagnostic.get('endpoint_retry_commands')
    if type(rounds) is int and rounds >= 0:
        counts = f'已重发 {rounds} 轮'
        if type(commands) is int and commands >= 0:
            counts += f'、{commands} 条关节指令'
        parts.append(counts)
    elapsed = endpoint_diagnostic.get('endpoint_seek_elapsed_s')
    no_progress = endpoint_diagnostic.get('endpoint_no_progress_s')
    if _number(elapsed) and elapsed >= 0:
        parts.append(f'终点收敛耗时 {elapsed:.2f} 秒')
    if _number(no_progress) and no_progress >= 0:
        parts.append(f'连续 {no_progress:.2f} 秒未达到要求的偏差改善')
    return '；'.join(parts)


def format_arm_execution_error(error, diagnostic=None, feedback_diagnostic=None, endpoint_diagnostic=None):
    """Translate known diagnostics, preserving unknown and already localized text.

    Historical reports only have the last sent reference. New pacing reports
    may also have a next reference, which has not been sent: wait failures must
    explain that candidate's advancement test rather than imply it was issued.
    """
    error = str(error or '')
    if error.startswith('Angle feedback recovery timeout'):
        return _feedback_timeout_detail(feedback_diagnostic)
    diagnostic = diagnostic if isinstance(diagnostic, dict) else {}
    endpoint_diagnostic = endpoint_diagnostic if isinstance(endpoint_diagnostic, dict) else {}
    if error.startswith('Required joint feedback recovery timeout:'):
        joints=error.split(':',1)[1].strip()
        return f'{joints} 持续缺少可信的新角度反馈，超过2秒恢复上限；已停止后续指令，其他轴持续反馈不能替代该轴测量'
    if error.startswith('Shared progress stalled'):
        return '整臂共同轨迹进度持续未推进，达到2秒无进展上限；已停止后续指令，需核实下游目标执行与反馈，尚不能据此认定硬件损坏'
    if error.startswith('Shared progress total execution budget exceeded'):
        return '整臂共同轨迹执行超过总时间预算，尚未确认到位；已停止后续指令'
    if error.startswith('Shared group margin lost'):
        detail=_reference_detail(diagnostic,next_point=True)
        return ('同组多轴提交期间，整组跟踪裕量不再满足要求：'+detail+
                '；已停止后续轴指令，先前已提交指令不能撤回，未确认实体停稳')
    if error in ('Endpoint convergence stalled', 'Endpoint convergence time budget exceeded'):
        reason = ('固定姿态终点重发后偏差仍未有效改善' if error == 'Endpoint convergence stalled'
                  else '固定姿态终点收敛超过批准的时间上限')
        detail = _endpoint_retry_detail(diagnostic, endpoint_diagnostic)
        return reason + ('：' + detail if detail else '') + '；已停止后续指令，未确认到位'
    sent_detail = _reference_detail(diagnostic)
    wait_detail = _reference_detail(diagnostic, next_point=True) or sent_detail
    if error.startswith('Accepted commands but no measured joint progress'):
        wait = diagnostic.get('wait_s')
        change = diagnostic.get('measured_change_deg')
        timing = (f'；等待 {wait:.2f} 秒，六轴最大反馈变化 {change:.3f}°'
                  if _number(wait) and _number(change) else '')
        return ('设备已接收指令，但未观察到有效关节运动：'
                + (wait_detail or '实际反馈未跟进当前路径点') + timing
                + '；已停止后续指令，需核查执行器位置控制状态；接收回执不代表到位')
    if error.startswith(('Fixed-pose feedback did not catch up', 'Trajectory feedback did not catch up')):
        context = '固定姿态移动' if error.startswith('Fixed-pose') else '轨迹点'
        return (context + '等待实际反馈跟进超时：'
                + (wait_detail or '实际关节未跟进当前路径点') + '；已停止后续指令')
    if error.startswith(('Fixed-pose feedback wait budget exceeded', 'Trajectory feedback wait budget exceeded')):
        context = '固定姿态' if error.startswith('Fixed-pose') else '轨迹点'
        return (context + '累计反馈等待超过 30 秒：'
                + (wait_detail or '实际运动持续落后于参考') + '；已停止后续指令')
    if error == 'Tracking error >3 degrees':
        return ('运动跟踪偏差超过 3°：' + sent_detail + '；已停止后续指令' if sent_detail
                else '运动跟踪偏差超过 3°，实际关节位置未跟上轨迹参考；已停止后续指令')
    return error


def execution_error_fields(error):
    """Return saveable failure fields for a fresh executor child process."""
    original = str(error) or type(error).__name__
    diagnostic = getattr(error, 'execution_diagnostics', None)
    feedback = getattr(error, 'feedback_diagnostics', None)
    endpoint = getattr(error, 'endpoint_diagnostics', None)
    fields = dict(error=format_arm_execution_error(original, diagnostic, feedback, endpoint), error_code=original)
    if isinstance(diagnostic, dict):
        fields['execution_diagnostics'] = diagnostic
    if isinstance(feedback, dict):
        fields['feedback_failure_diagnostics'] = feedback
    if isinstance(endpoint, dict):
        fields['endpoint_diagnostics'] = endpoint
    adaptive = getattr(error, 'adaptive_reference_steps', None)
    if isinstance(adaptive, list):
        fields['adaptive_reference_steps'] = adaptive
    return fields
