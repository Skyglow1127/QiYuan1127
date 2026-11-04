"""Optional bounded coordination of an ACT opening and its following motion.

This classifies a slowly moving recorded placement phase, not table contact,
visual alignment or a successful release. It retains only the last acknowledged
six-joint target for a short, bounded interval. Every gripper proposal stays the
current ACT proposal. Existing projection/cap/rate/source/receiver checks must
still process the result. The caller commits proposed history only after ACK.
"""

from collections.abc import Mapping
from dataclasses import dataclass
import math

from .guard import ACTION_KEYS, GuardError


RELEASE_SETTLE = {
    'trigger_open_m': .055,
    'min_opening_delta_m': .006,
    'min_actual_open_m': .035,
    'stable_window_s': .15,
    'stable_span_deg': .75,
    'max_feedback_age_s': .1,
    'max_sample_gap_s': .1,
    'release_gain_m': .005,
    'max_hold_s': .35,
    'max_joint_departure_deg': 5.,
    'rearm_close_m': .035,
    'placement_joint_bounds_deg': [
        [25., 38.], [118., 133.], [-105., -75.],
        [-15., 15.], [55., 70.], [-75., -40.]],
}


@dataclass(frozen=True)
class ReleaseSettleState:
    armed: bool = False
    active: bool = False
    started_at: float | None = None
    start_open_m: float | None = None
    anchor: tuple[float, ...] | None = None
    samples: tuple[tuple[float, tuple[float, ...]], ...] = ()
    last_sample_at: float | None = None
    last_now: float | None = None


def _number(value, name, *, json_number=False):
    if (isinstance(value, (bool, str, bytes))
            or (json_number and type(value) not in (int, float))):
        raise GuardError(name + ' must be a finite number')
    try:
        result = float(value)
    except (ValueError, TypeError, OverflowError) as error:
        raise GuardError(name + ' must be a finite number') from error
    if not math.isfinite(result):
        raise GuardError(name + ' must be a finite number')
    return result


def _pairs(value, *, count, name):
    if not isinstance(value, (list, tuple)) or len(value) != count:
        raise GuardError(name + f' requires {count} increasing pairs')
    output = []
    for pair in value:
        if not isinstance(pair, (list, tuple)) or len(pair) != 2:
            raise GuardError(name + ' requires increasing pairs')
        low, high = (_number(v, name, json_number=True) for v in pair)
        if low >= high:
            raise GuardError(name + ' requires increasing pairs')
        output.append((low, high))
    return tuple(output)


def _config(config):
    if not isinstance(config, Mapping) or set(config) != set(RELEASE_SETTLE):
        raise GuardError('policy_release_settle requires the documented fields')
    checked = {key: _number(config[key], key, json_number=True)
        for key in RELEASE_SETTLE if key != 'placement_joint_bounds_deg'}
    checked['placement_joint_bounds_deg'] = _pairs(
        config['placement_joint_bounds_deg'], count=6,
        name='release settle placement window')
    ranges = {
        'stable_window_s': (0., 1.), 'stable_span_deg': (0., 2.),
        'max_feedback_age_s': (0., .1), 'max_sample_gap_s': (0., .1),
        'max_hold_s': (0., .5), 'max_joint_departure_deg': (0., 5.),
        'min_opening_delta_m': (0., .025), 'release_gain_m': (0., .01),
    }
    for name, (low, high) in ranges.items():
        if not low < checked[name] <= high:
            raise GuardError(f'{name} must be in ({low}, {high}]')
    if not (0 <= checked['rearm_close_m'] <= checked['min_actual_open_m']
            < checked['trigger_open_m'] <= .1):
        raise GuardError('Release settle gripper thresholds must be ordered within 0..0.1m')
    if checked['release_gain_m'] > checked['min_opening_delta_m']:
        raise GuardError('Relative release gain must not exceed the opening intent gap')
    return checked


def validate_release_settle(config, *, bounds):
    """Validate this optional proposal transform inside reviewed hard bounds."""
    checked = _config(config)
    reviewed = _pairs(bounds, count=7, name='reviewed bounds')
    for (low, high), (hard_low, hard_high) in zip(
            checked['placement_joint_bounds_deg'], reviewed[:6], strict=True):
        if not hard_low <= low < high <= hard_high:
            raise GuardError('Release settle placement window exceeds reviewed bounds')
    if not (reviewed[6][0] <= checked['rearm_close_m']
            <= checked['min_actual_open_m'] < checked['trigger_open_m'] <= reviewed[6][1]):
        raise GuardError('Release settle gripper thresholds exceed reviewed bounds')
    return checked


def _scalars(values, *, count=7):
    if isinstance(values, Mapping):
        try:
            values = tuple(values[key] for key in ACTION_KEYS[:count])
        except KeyError as error:
            raise GuardError('Release settle requires physical scalar channels') from error
    if isinstance(values, (str, bytes)):
        raise GuardError('Release settle requires finite physical scalars')
    try:
        result = tuple(_number(v, 'release settle scalar') for v in values)
    except TypeError as error:
        raise GuardError('Release settle requires finite physical scalars') from error
    if len(result) != count:
        raise GuardError(f'Release settle requires {count} finite scalars')
    return result


def _history(history):
    if not isinstance(history, ReleaseSettleState):
        raise GuardError('Expected ReleaseSettleState')
    if type(history.armed) is not bool or type(history.active) is not bool:
        raise GuardError('Release settle flags must be boolean')
    if history.active:
        if history.armed:
            raise GuardError('Active release settle must have consumed its arm')
        _number(history.started_at, 'release settle start time')
        _number(history.start_open_m, 'release settle starting width')
        _scalars(history.anchor, count=6)
    elif any(value is not None for value in (
            history.started_at, history.start_open_m, history.anchor)):
        raise GuardError('Inactive release settle cannot retain an active anchor')
    if history.last_now is not None:
        _number(history.last_now, 'release settle previous clock')
    if history.last_sample_at is not None:
        _number(history.last_sample_at, 'release settle previous sample time')
    if not isinstance(history.samples, tuple):
        raise GuardError('Release settle history samples must be immutable pairs')
    previous_time = None
    for sample in history.samples:
        if not isinstance(sample, tuple) or len(sample) != 2:
            raise GuardError('Release settle history samples must be immutable pairs')
        time_at, values = sample
        time_at = _number(time_at, 'release settle sample time')
        _scalars(values, count=6)
        if previous_time is not None and time_at <= previous_time:
            raise GuardError('Release settle history samples must increase')
        previous_time = time_at
    if history.samples and previous_time != history.last_sample_at:
        raise GuardError('Release settle samples must end at the last observed sample')
    return history


def shape_release_settle(action, *, observed_state, previous_target, sampled_at,
                         now, history, config):
    """Return (seven proposals, proposed ACK-only history, decision string).

    Stability uses genuine increasing feedback sample times, never invocation
    times. Missing coverage, sample gaps, repeated/future/stale samples cannot
    manufacture 150 ms of evidence. Untrustworthy samples cancel active holds
    without creating a new stop. The unchanged downstream guards still decide
    whether the original proposal and feedback are actually sendable.
    """
    settings = _config(config)
    action, actual, previous = (_scalars(value) for value in (
        action, observed_state, previous_target))
    now = _number(now, 'release settle clock')
    sampled_at = _number(sampled_at, 'release settle feedback time')
    history = _history(history)
    last_sample_at = history.last_sample_at
    last_now = now if history.last_now is None else max(now, history.last_now)

    def idle(armed, samples, decision):
        return action, ReleaseSettleState(armed=armed, samples=samples,
            last_sample_at=last_sample_at, last_now=last_now), decision

    trustworthy = (0 <= now - sampled_at <= settings['max_feedback_age_s']
        and (history.last_now is None or now >= history.last_now)
        and (last_sample_at is None or sampled_at > last_sample_at))
    if not trustworthy:
        # Do not retain a pre-gap sample that a later callback might combine
        # with one fresh endpoint to pretend the whole interval was covered.
        return idle(False if history.active else history.armed, (),
            'cancelled_feedback' if history.active else 'feedback_not_new_or_fresh')
    last_sample_at = sampled_at
    samples = history.samples
    if samples and sampled_at-samples[-1][0] > settings['max_sample_gap_s']:
        samples = ()
        if history.active:
            return idle(False, ((sampled_at, actual[:6]),), 'cancelled_sample_gap')
    samples = (*samples, (sampled_at, actual[:6]))
    cutoff = sampled_at - settings['stable_window_s']
    preceding = [i for i, (time_at, _) in enumerate(samples) if time_at <= cutoff]
    if preceding:
        samples = samples[preceding[-1]:]

    in_window = all(low <= value <= high for value, (low, high) in zip(
        actual[:6], settings['placement_joint_bounds_deg'], strict=True))
    if history.active:
        if not in_window:
            return idle(False, samples, 'cancelled_outside_window')
        if action[6] < settings['trigger_open_m'] or action[6] <= actual[6]:
            return idle(False, samples, 'cancelled_opening_withdrawn')
        if any(abs(value-anchor) > settings['max_joint_departure_deg']
                for value, anchor in zip(action[:6], history.anchor, strict=True)):
            return idle(False, samples, 'cancelled_joint_departure')
        if actual[6] >= history.start_open_m + settings['release_gain_m']:
            return idle(False, samples, 'released_relative')
        if now >= history.started_at + settings['max_hold_s']:
            return idle(False, samples, 'released_timeout')
        proposed = ReleaseSettleState(active=True,
            started_at=history.started_at, start_open_m=history.start_open_m,
            anchor=history.anchor, samples=samples,
            last_sample_at=sampled_at, last_now=last_now)
        return (*history.anchor, action[6]), proposed, 'holding'

    armed = history.armed or (actual[0] < 0. and action[6] <= settings['rearm_close_m'])
    opening = (action[6] >= settings['trigger_open_m']
        and action[6] > actual[6]+settings['min_opening_delta_m']
        and actual[6] >= settings['min_actual_open_m'])
    if not (armed and in_window and opening):
        return idle(armed, samples, 'armed' if armed else 'idle')
    coverage = samples[-1][0]-samples[0][0]
    stable = coverage >= settings['stable_window_s'] and all(
        max(values[axis] for _, values in samples)
            - min(values[axis] for _, values in samples) <= settings['stable_span_deg']
        for axis in range(6))
    if not stable:
        return idle(True, samples, 'waiting_for_stability')
    anchor = previous[:6]
    if any(abs(value-old) > settings['max_joint_departure_deg']
            for value, old in zip(action[:6], anchor, strict=True)):
        return idle(False, samples, 'cancelled_joint_departure')
    proposed = ReleaseSettleState(active=True, started_at=now,
        start_open_m=actual[6], anchor=anchor, samples=samples,
        last_sample_at=sampled_at, last_now=last_now)
    return (*anchor, action[6]), proposed, 'started'
