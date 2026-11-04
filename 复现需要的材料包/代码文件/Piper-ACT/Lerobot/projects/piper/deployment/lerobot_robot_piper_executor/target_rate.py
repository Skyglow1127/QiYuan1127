"""Quantized policy-target rate shaping; no hardware or waiting operations.

The caller must reject the raw policy prediction against absolute bounds before
calling this function. The shaped target still requires ActionGuard.prepare:
step, tracking, bounds, observation age and command age remain hard checks.
"""

from collections.abc import Mapping
import math

from .guard import ACTION_KEYS, GuardError, GuardLimits, vector


_QUANTA = (.001,) * 6 + (1e-6,)
_SCALES = (1000,) * 6 + (1_000_000,)
_INT32_MIN, _INT32_MAX = -(2**31), 2**31 - 1


def _number(value, name):
    try:
        if isinstance(value, bool):
            raise ValueError
        result = float(value)
    except (TypeError, ValueError, OverflowError) as error:
        raise GuardError(f"Invalid {name}") from error
    if not math.isfinite(result):
        raise GuardError(f"Invalid {name}")
    return result


def _caps(relative_cap):
    if isinstance(relative_cap, Mapping):
        result = vector(relative_cap)
    elif isinstance(relative_cap, (int, float)) and not isinstance(relative_cap, bool):
        result = (_number(relative_cap, "relative target cap"),) * 7
    else:
        try:
            result = vector(relative_cap)
        except TypeError as error:
            raise GuardError("Invalid relative target cap") from error
    if any(value <= 0 for value in result):
        raise GuardError("Seven positive finite relative target caps required")
    return result


def _rate_inputs(*, state, previous_target, elapsed_s, relative_cap, speed_fraction):
    state, previous = vector(state), vector(previous_target)
    caps = _caps(relative_cap)
    elapsed = _number(elapsed_s, "target-rate elapsed time")
    fraction = _number(speed_fraction, "target-rate speed fraction")
    if elapsed < 0:
        raise GuardError("Target-rate elapsed time must be nonnegative")
    if not 0 < fraction <= .95:
        raise GuardError("Target-rate speed fraction must be in (0, 0.95]")
    return state, previous, caps, elapsed, fraction


def _validated_target_bounds(target_bounds, limits):
    if target_bounds is None:
        return None
    if isinstance(target_bounds, (str, bytes, Mapping)):
        raise GuardError("Target bounds require seven finite lower/upper pairs")
    try:
        pairs = tuple(tuple(pair) for pair in target_bounds)
    except TypeError as error:
        raise GuardError("Target bounds require seven finite lower/upper pairs") from error
    if len(pairs) != 7 or any(len(pair) != 2 for pair in pairs):
        raise GuardError("Target bounds require seven finite lower/upper pairs")
    result = []
    for key, pair, (hard_low, hard_high) in zip(ACTION_KEYS, pairs, limits.bounds, strict=True):
        if any(isinstance(value, (str, bytes, bool)) for value in pair):
            raise GuardError(f"{key}: target bounds must be finite numeric values")
        low, high = (_number(value, "target bound") for value in pair)
        if not hard_low <= low <= high <= hard_high:
            raise GuardError(f"{key}: target bounds must be inside unchanged guard bounds")
        result.append((low, high))
    return tuple(result)


def _grid_target(desired, low, high, *, quantum, scale, key, admissible):
    # Widen only the integer search, never the admissible float interval. This
    # also retains a grid-aligned previous target when the rate budget is zero:
    # multiplying a decoded float by its scale can lie just below its integer.
    low = max(low, _INT32_MIN * quantum)
    high = min(high, _INT32_MAX * quantum)
    if low > high:
        raise GuardError(f"{key}: feedback cap and target-rate intervals do not intersect")
    first = max(_INT32_MIN, math.floor(low * scale) - 1)
    last = min(_INT32_MAX, math.ceil(high * scale) + 1)
    while first <= last and (first * quantum < low or not admissible(first * quantum)):
        first += 1
    while first <= last and (last * quantum > high or not admissible(last * quantum)):
        last -= 1
    if first > last:
        raise GuardError(f"{key}: no CAN-representable target in the allowed interval")
    desired = min(max(desired, low), high)
    integer = min(max(round(desired * scale), first), last)
    # Match remote_bridge._decode_transaction exactly, including multiplication
    # rather than division. Host and receiver then commit the same float target.
    return integer * quantum


def shape_target_rate(desired, *, state, previous_target, elapsed_s,
                      limits: GuardLimits, relative_cap, speed_fraction=.9,
                      target_bounds=None):
    """Project onto feedback/rate intervals and return seven CAN-grid values.

    elapsed_s is measured since the previous successful host guard.commit, not
    command creation. No timestamp is changed and no additional wait is made.
    A fractional speed budget and one CAN quantum of inward margin keep the
    original host/receiver speed checks strict after floating-point encoding.
    Optional target bounds are intersected before CAN-grid selection, so neither
    the feedback cap nor quantization can push a target outside them. Empty
    intersections fail instead of relaxing any interval. None preserves the
    legacy feedback/rate-only shaping behavior.
    """
    desired = vector(desired)
    state, previous, caps, elapsed, fraction = _rate_inputs(
        state=state, previous_target=previous_target, elapsed_s=elapsed_s,
        relative_cap=relative_cap, speed_fraction=speed_fraction)
    bounds = _validated_target_bounds(target_bounds, limits)
    shaped = []
    for index, (key, target, current, old, cap, speed, quantum, scale) in enumerate(zip(
            ACTION_KEYS, desired, state, previous, caps,
            limits.max_target_speed, _QUANTA, _SCALES, strict=True)):
        allowance = speed * elapsed * fraction
        if not math.isfinite(allowance):
            raise GuardError(f"{key}: non-finite target-rate budget")
        budget = max(0., allowance - quantum)
        low = max(current - cap, old - budget)
        high = min(current + cap, old + budget)
        bound = None if bounds is None else bounds[index]
        if bound is not None:
            low, high = max(low, bound[0]), min(high, bound[1])
        if not math.isfinite(low) or not math.isfinite(high):
            raise GuardError(f"{key}: non-finite target-rate interval")
        shaped.append(_grid_target(target, low, high,
            quantum=quantum, scale=scale, key=key,
            admissible=lambda value: abs(value - current) <= cap
                and abs(value - old) <= budget
                and (bound is None or bound[0] <= value <= bound[1])))
    return tuple(shaped)


def target_rate_wait_s(*, state, previous_target, elapsed_s, limits: GuardLimits,
                       relative_cap, speed_fraction=.9, target_bounds=None):
    """Calculate extra time for a CAN-grid intersection, without waiting.

    The caller must bound any wait and retain the original observation/ACT slot
    deadlines. This function neither changes timestamps nor relaxes shaping or
    guard limits. With optional target bounds, the fixed feedback-cap/target
    intersection must already contain a CAN point; time can only expand the
    rate interval around the previous target. None preserves legacy behavior.
    """
    state, previous, caps, elapsed, fraction = _rate_inputs(
        state=state, previous_target=previous_target, elapsed_s=elapsed_s,
        relative_cap=relative_cap, speed_fraction=speed_fraction)
    bounds = _validated_target_bounds(target_bounds, limits)
    required_elapsed = elapsed
    for index, (key, current, old, cap, speed, quantum, scale) in enumerate(zip(
            ACTION_KEYS, state, previous, caps, limits.max_target_speed,
            _QUANTA, _SCALES, strict=True)):
        allowance = speed * elapsed * fraction
        if not math.isfinite(allowance):
            raise GuardError(f"{key}: non-finite target-rate budget")
        feedback_low, feedback_high = current - cap, current + cap
        if not math.isfinite(feedback_low) or not math.isfinite(feedback_high):
            raise GuardError(f"{key}: non-finite target-rate interval")
        budget = max(0., allowance - quantum)
        gap = abs(old - current) - cap
        bound = None if bounds is None else bounds[index]
        if bound is not None:
            feedback_low = max(feedback_low, bound[0])
            feedback_high = min(feedback_high, bound[1])
            if feedback_low > feedback_high:
                raise GuardError(f"{key}: feedback cap and target bounds do not intersect")
            gap = max(feedback_low - old, old - feedback_high)
        try:
            _grid_target(old, max(feedback_low, old - budget),
                min(feedback_high, old + budget), quantum=quantum, scale=scale,
                key=key, admissible=lambda value: abs(value - current) <= cap
                    and abs(value - old) <= budget
                    and (bound is None or bound[0] <= value <= bound[1]))
            if gap < 0:
                continue
        except GuardError:
            pass
        # Distinguish temporary rate/cap conflict from a feedback interval
        # that has no CAN representation at any future time.
        nearest = _grid_target(old, feedback_low, feedback_high,
            quantum=quantum, scale=scale, key=key,
            admissible=lambda value: abs(value - current) <= cap
                and (bound is None or bound[0] <= value <= bound[1]))
        gap = max(0., gap)
        # One quantum compensates the rate shaper's inward margin; the second
        # allows an interior grid point. Also cover float endpoint subtraction
        # without widening either admissible interval.
        numerical_margin = 8 * math.ulp(max(abs(old), abs(current), cap, quantum))
        needed = max(gap + 2 * quantum, abs(nearest - old) + quantum) + numerical_margin
        required = needed / speed / fraction
        if not math.isfinite(required):
            raise GuardError(f"{key}: non-finite target-rate wait")
        required_elapsed = max(required_elapsed, required)
    return max(0., required_elapsed - elapsed)
