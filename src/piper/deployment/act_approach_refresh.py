"""Pure once-per-approach visual refresh classifier; no targets or clocks.

The joint window classifies a recorded task phase. It does not establish visual
alignment or contact. A refresh discards cached actions and must run inference
on the existing current observation batch, never relabel an old prediction.
"""
from collections.abc import Mapping
from dataclasses import dataclass
import math

from lerobot_robot_piper_executor.approach_gripper import APPROACH_GRIPPER
from lerobot_robot_piper_executor.guard import ACTION_KEYS, GuardError, vector


APPROACH_REFRESH = {
    'max_visual_age_s': .8,
    'min_cached_steps': 10,
    'min_open_m': .035,
    'approach_joint_bounds_deg': [list(pair) for pair in
        APPROACH_GRIPPER['grasp_joint_bounds_deg']],
    'rearm_joint2_max_deg': 90.,
}
_REARM_OPEN_M = .055


@dataclass(frozen=True)
class ApproachRefreshState:
    used: bool = False


def _number(value, label):
    if type(value) not in (int, float):
        raise GuardError(label + ' must be a finite JSON number')
    try:
        value = float(value)
    except (ValueError, OverflowError) as error:
        raise GuardError(label + ' must be a finite JSON number') from error
    if not math.isfinite(value):
        raise GuardError(label + ' must be a finite JSON number')
    return value


def _pairs(value, *, count, label):
    if not isinstance(value, (list, tuple)) or len(value) != count:
        raise GuardError(label + f' requires {count} bound pairs')
    result = []
    for pair in value:
        if not isinstance(pair, (list, tuple)) or len(pair) != 2:
            raise GuardError(label + ' requires increasing bound pairs')
        low, high = (_number(item, label) for item in pair)
        if low >= high:
            raise GuardError(label + ' requires increasing bound pairs')
        result.append((low, high))
    return tuple(result)


def _config(config):
    if not isinstance(config, Mapping) or set(config) != set(APPROACH_REFRESH):
        raise GuardError('policy_approach_refresh requires the documented fields')
    result = {key: _number(config[key], key) for key in
        ('max_visual_age_s', 'min_open_m', 'rearm_joint2_max_deg')}
    steps = config['min_cached_steps']
    if type(steps) is not int or not 1 <= steps < 100:
        raise GuardError('min_cached_steps must be an integer in [1, 99]')
    result['min_cached_steps'] = steps
    result['approach_joint_bounds_deg'] = _pairs(
        config['approach_joint_bounds_deg'], count=6,
        label='approach_joint_bounds_deg')
    if not 0 < result['max_visual_age_s'] <= 5.:
        raise GuardError('max_visual_age_s must be in (0, 5]')
    if not 0 <= result['min_open_m'] < _REARM_OPEN_M:
        raise GuardError('min_open_m must be nonnegative and below the 55 mm rearm opening')
    if result['rearm_joint2_max_deg'] >= result['approach_joint_bounds_deg'][1][0]:
        raise GuardError('Rearm joint 2 must be strictly below the approach window')
    return result


def validate_approach_refresh(config, *, bounds):
    """Return a copy validated against the unchanged seven reviewed bounds."""
    result = _config(config)
    reviewed = _pairs(bounds, count=7, label='bounds')
    for (low, high), (reviewed_low, reviewed_high) in zip(
            result['approach_joint_bounds_deg'], reviewed[:6], strict=True):
        if not reviewed_low <= low < high <= reviewed_high:
            raise GuardError('Approach refresh window exceeds reviewed bounds')
    if not reviewed[6][0] <= result['min_open_m'] < _REARM_OPEN_M <= reviewed[6][1]:
        raise GuardError('Approach refresh openings exceed reviewed bounds')
    if not reviewed[1][0] <= result['rearm_joint2_max_deg'] <= reviewed[1][1]:
        raise GuardError('Approach refresh rearm joint 2 exceeds reviewed bounds')
    return result


def _scalars(values):
    # Robot observations also contain the two RGB arrays. Read only the seven
    # physical scalar channels, rather than trying to validate image values.
    if isinstance(values, Mapping):
        try:
            values = tuple(values[key] for key in ACTION_KEYS)
        except KeyError as error:
            raise GuardError('Approach refresh requires all seven feedback scalars') from error
    return vector(values)


def approach_refresh_decision(config, *, observed_state, previous_target,
                              closing_started, history, visual_age_s,
                              cached_steps, queue_nonempty):
    """Return (proposed history, refresh) without resetting any robot phase.

    A successful close consumes the attempt even if no refresh occurred. Rearm
    requires an explicit opened retry below the approach window. Thus repeated
    chunks, closure, carrying, placing and return do not repeatedly clear queues.
    The caller must validate the original pending slot and the new source before
    discarding a queue; this classifier never modifies or timestamps an action.
    """
    settings = _config(config)
    if not isinstance(history, ApproachRefreshState) or type(history.used) is not bool:
        raise GuardError('Expected boolean ApproachRefreshState')
    if type(closing_started) is not bool or type(queue_nonempty) is not bool:
        raise GuardError('Approach refresh phase and queue flags must be boolean')
    if type(cached_steps) is not int or cached_steps < 0:
        raise GuardError('Approach refresh cached steps must be nonnegative integer')
    age = _number(visual_age_s, 'visual age')
    if age < 0:
        raise GuardError('Approach refresh visual age must be nonnegative')
    observed, previous = _scalars(observed_state), _scalars(previous_target)
    in_window = all(low <= actual <= high for actual, (low, high) in zip(
        observed[:6], settings['approach_joint_bounds_deg'], strict=True))
    used = history.used
    if closing_started:
        used = True
    elif (used and not in_window
            and observed[1] <= settings['rearm_joint2_max_deg']
            and previous[6] >= _REARM_OPEN_M):
        used = False
    refresh = bool(not used and not closing_started and queue_nonempty
        and cached_steps >= settings['min_cached_steps']
        and age > settings['max_visual_age_s']
        and observed[6] >= settings['min_open_m'] and in_window)
    return ApproachRefreshState(used or refresh), refresh
