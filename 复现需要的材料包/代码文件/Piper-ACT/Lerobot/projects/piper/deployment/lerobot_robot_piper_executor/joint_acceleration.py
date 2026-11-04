"""Optional joint-only acceleration shaping inside the original safe intervals.

This is a proposal shaper, not a new hardware acceleration guarantee. Original
feedback/rate/bounds checks take priority when the acceleration window has no
CAN-representable intersection. The caller must log the returned relaxed axes,
clear velocity history after such a relaxation, and retain both original guards.
No clocks, timestamps, transport operations or mutable history live here.
"""

from collections.abc import Mapping
import math

from .guard import ACTION_KEYS, GuardError, GuardLimits
from .target_rate import (
    _QUANTA, _SCALES, _grid_target, _number, _rate_inputs,
    _validated_target_bounds, shape_target_rate,
)


def _numeric(value, name):
    if isinstance(value, (str, bytes, bool)):
        raise GuardError(f"Invalid {name}")
    return _number(value, name)


def validate_joint_acceleration(value):
    """Return a finite joint acceleration in (0, 1000] degrees/second²."""
    result = _numeric(value, "joint acceleration")
    if not 0. < result <= 1000.:
        raise GuardError("Joint acceleration must be finite and in (0, 1000]")
    return result


def _velocity(values):
    if isinstance(values, (str, bytes, Mapping)):
        raise GuardError("Expected six finite previous joint velocities")
    try:
        result = tuple(_numeric(v, "previous joint velocity") for v in values)
    except TypeError as error:
        raise GuardError("Expected six finite previous joint velocities") from error
    if len(result) != 6:
        raise GuardError("Expected six finite previous joint velocities")
    return result


def limit_joint_acceleration(safe_target, *, state, previous_target,
                             previous_velocity, previous_interval_s, elapsed_s,
                             rate_elapsed_s, limits: GuardLimits, relative_cap,
                             speed_fraction, target_bounds=None,
                             max_acceleration_deg_s2=160.):
    """Return (seven quantized targets, axes with a relaxed soft constraint).

    elapsed_s and previous_interval_s describe successive successful shaping
    instants. The original target-rate budget instead uses rate_elapsed_s,
    measured since the last successful guard.commit. These intervals must not
    be substituted for one another. Velocity history is advanced by the caller
    only after successful transport and guard.commit, never by this function.

    The acceleration window limits the change between interval-average target
    velocities over the distance between their midpoints. It does not change
    the gripper. A missing velocity or previous interval initializes history by
    returning the original hard projection. Empty soft intersections, including
    sub-quantum intervals, relax only the affected joint; original hard failures
    propagate. Final commands still require the unchanged host/receiver guards.
    """
    acceleration = validate_joint_acceleration(max_acceleration_deg_s2)
    elapsed = _numeric(elapsed_s, "joint acceleration elapsed time")
    if elapsed < 0.:
        raise GuardError("Joint acceleration elapsed time must be nonnegative")
    velocity = None if previous_velocity is None else _velocity(previous_velocity)
    previous_interval = None
    if previous_interval_s is not None:
        previous_interval = _numeric(previous_interval_s, "previous joint acceleration interval")
        if previous_interval <= 0.:
            raise GuardError("Previous joint acceleration interval must be positive")

    # Validate and retain the old projection before considering a soft window.
    # In particular an empty original cap/rate interval may never be repaired by
    # relaxing acceleration, nor may the gripper change through this extension.
    base = shape_target_rate(safe_target, state=state,
        previous_target=previous_target, elapsed_s=rate_elapsed_s, limits=limits,
        relative_cap=relative_cap, speed_fraction=speed_fraction,
        target_bounds=target_bounds)
    if velocity is None or previous_interval is None:
        return base, ()
    if elapsed <= 0.:
        raise GuardError("Joint acceleration elapsed time must be positive with velocity history")

    current, previous, caps, rate_elapsed, fraction = _rate_inputs(
        state=state, previous_target=previous_target, elapsed_s=rate_elapsed_s,
        relative_cap=relative_cap, speed_fraction=speed_fraction)
    bounds = _validated_target_bounds(target_bounds, limits)
    velocity_budget = acceleration * (elapsed / 2. + previous_interval / 2.)
    if not math.isfinite(velocity_budget):
        raise GuardError("Non-finite joint acceleration budget")
    result = list(base)
    relaxed = []
    for index in range(6):
        key = ACTION_KEYS[index]
        old, actual, cap = previous[index], current[index], caps[index]
        budget = max(0., limits.max_target_speed[index] * rate_elapsed * fraction
            - _QUANTA[index])
        low = max(actual - cap, old - budget)
        high = min(actual + cap, old + budget)
        bound = None if bounds is None else bounds[index]
        if bound is not None:
            low, high = max(low, bound[0]), min(high, bound[1])
        acceleration_low = old + elapsed * (velocity[index] - velocity_budget)
        acceleration_high = old + elapsed * (velocity[index] + velocity_budget)
        if not math.isfinite(acceleration_low) or not math.isfinite(acceleration_high):
            raise GuardError(f"{key}: non-finite joint acceleration interval")
        low, high = max(low, acceleration_low), min(high, acceleration_high)
        try:
            result[index] = _grid_target(base[index], low, high,
                quantum=_QUANTA[index], scale=_SCALES[index], key=key,
                admissible=lambda value: abs(value - actual) <= cap
                    and abs(value - old) <= budget
                    and (bound is None or bound[0] <= value <= bound[1])
                    and acceleration_low <= value <= acceleration_high
                    and abs((value - old) / elapsed - velocity[index]) <= velocity_budget)
        except GuardError:
            # Base has already passed the unchanged projection. Do not invent
            # a failure, widen a hard interval or affect another axis here.
            relaxed.append(key)
    return tuple(result), tuple(relaxed)
