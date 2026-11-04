"""Classify bounded continuation of an ACT-proposed closing action.

This module only inspects physical-unit predictions and actual feedback. It
does not generate targets, establish alignment/contact, or prove grasp success.
Continued actions must retain their original slots and pass the existing guards.
"""
import math


ACTION_KEYS = tuple(f'joint_{i}.deg' for i in range(1, 7)) + ('gripper.m',)

GRASP_CONTINUATION = {
    'extra_steps': 20,
    'trigger_close_m': .035,
    'prior_open_m': .055,
    'min_actual_open_m': .035,
    'max_opening_drift_m': .003,
    'grasp_joint_bounds_deg': [
        [-27., -14.], [96., 113.], [-71., -46.],
        [-4., 13.], [35., 67.], [-34., -12.]],
}


def _number(value, label):
    if type(value) not in (int, float) or not math.isfinite(value):
        raise ValueError(label + ' must be a finite number')
    return value


def _configuration(config):
    label = 'policy_grasp_chunk_extension'
    if not isinstance(config, dict) or set(config) != set(GRASP_CONTINUATION):
        raise ValueError(label + ' requires the documented fields')
    if type(config['extra_steps']) is not int or not 1 <= config['extra_steps'] <= 20:
        raise ValueError(label + ' extra_steps must be an integer in [1, 20]')
    close, prior, actual, drift = (
        _number(config[key], label + ' ' + key)
        for key in ('trigger_close_m', 'prior_open_m',
                    'min_actual_open_m', 'max_opening_drift_m'))
    if not 0 <= close <= actual < prior:
        raise ValueError(label + ' requires ordered nonnegative gripper thresholds')
    if not 0 < drift <= .003:
        raise ValueError(label + ' max_opening_drift_m must be in (0, 0.003]')
    window = config['grasp_joint_bounds_deg']
    if not isinstance(window, (list, tuple)) or len(window) != 6:
        raise ValueError(label + ' requires six grasp joint intervals')
    for interval in window:
        if not isinstance(interval, (list, tuple)) or len(interval) != 2:
            raise ValueError(label + ' requires six grasp joint intervals')
        low, high = (_number(value, label + ' joint bound') for value in interval)
        if low >= high:
            raise ValueError(label + ' joint intervals must be increasing')
    return config


def validate_grasp_continuation(config, *, bounds):
    """Validate the opt-in classifier against unchanged reviewed working bounds."""
    _configuration(config)
    label = 'policy_grasp_chunk_extension'
    if not isinstance(bounds, (list, tuple)) or len(bounds) != 7:
        raise ValueError(label + ' requires seven reviewed bound intervals')
    checked_bounds = []
    for interval in bounds:
        if not isinstance(interval, (list, tuple)) or len(interval) != 2:
            raise ValueError(label + ' requires seven reviewed bound intervals')
        low, high = (_number(value, label + ' reviewed bound') for value in interval)
        if low >= high:
            raise ValueError(label + ' reviewed bounds must be increasing')
        checked_bounds.append((low, high))
    grip_low, grip_high = checked_bounds[6]
    if any(not grip_low <= config[key] <= grip_high
           for key in ('trigger_close_m', 'prior_open_m', 'min_actual_open_m')):
        raise ValueError(label + ' gripper thresholds exceed the working range')
    for interval, (low, high) in zip(
            config['grasp_joint_bounds_deg'], checked_bounds[:6], strict=True):
        if not low <= interval[0] < interval[1] <= high:
            raise ValueError(label + ' grasp window exceeds the working range')
    return config


def grasp_is_pending(config, *, raw_action, state, prior_open_seen):
    """Classify an acknowledged closing proposal, never physical grasp success."""
    _configuration(config)
    if prior_open_seen is not True or raw_action is None:
        return False
    try:
        requested = raw_action['gripper.pos']
        values = [state[key] for key in ACTION_KEYS] + [requested]
        for value in values:
            _number(value, 'grasp classifier input')
    except (KeyError, TypeError, ValueError):
        return False
    actual = state['gripper.m']
    return (requested <= config['trigger_close_m']
        and actual >= config['min_actual_open_m']
        and actual > requested + .003
        and all(low <= state[key] <= high
            for key, (low, high) in zip(
                ACTION_KEYS[:6], config['grasp_joint_bounds_deg'], strict=True)))


def closing_prefix_length(config, *, actions, last_requested_m, actual_width_m):
    """Count only the contiguous original physical-unit closing prefix.

    Opening drift is measured against the fixed last acknowledged raw proposal,
    rather than accumulated between candidates. All supplied rows are validated
    first so malformed or nonfinite cached predictions cannot be hidden after a
    short valid prefix. No target is synthesized or transformed.
    """
    _configuration(config)
    last_requested = _number(last_requested_m, 'last requested gripper width')
    actual = _number(actual_width_m, 'actual gripper width')
    if isinstance(actions, (str, bytes, dict)):
        raise ValueError('Physical continuation actions must be seven-value rows')
    try:
        rows = list(actions)
    except TypeError as error:
        raise ValueError('Physical continuation actions must be seven-value rows') from error
    checked_rows = []
    for row in rows:
        if isinstance(row, (str, bytes, dict)):
            raise ValueError('Physical continuation actions must be seven-value rows')
        try:
            row = tuple(row)
        except TypeError as error:
            raise ValueError('Physical continuation actions must be seven-value rows') from error
        if len(row) != 7:
            raise ValueError('Physical continuation actions must be seven-value rows')
        for value in row:
            _number(value, 'physical continuation action')
        checked_rows.append(row)
    count = 0
    for row in checked_rows[:config['extra_steps']]:
        requested = row[6]
        if (requested > config['trigger_close_m']
                or requested > last_requested + config['max_opening_drift_m']
                or requested >= actual - .003):
            break
        count += 1
    return count
