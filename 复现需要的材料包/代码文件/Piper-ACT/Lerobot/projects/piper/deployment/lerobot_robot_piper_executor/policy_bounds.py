"""Explicit ACT proposal transforms; controller/feedback bounds stay strict.

Six-joint projection, its optional gripper margin and legacy J5 modes are explicit.
All still require the complete normal guarded send.
"""

import math

from .guard import ACTION_KEYS, GuardError, GuardLimits, vector


ACT_J5_TRAINING_UPPER_DEG = 70.


def joint_policy_target_bounds(limits: GuardLimits, margin_deg, gripper_margin_m=None):
    """Inward proposal ranges; absent gripper margin preserves strict input."""
    if type(margin_deg) not in (int, float) or not 0 <= margin_deg <= 5:
        raise GuardError("policy_joint_target_margin_deg must be a finite JSON number in [0, 5]")
    gripper_bounds = limits.bounds[6]
    if gripper_margin_m is not None:
        if type(gripper_margin_m) not in (int, float) or not 0 <= gripper_margin_m <= .005:
            raise GuardError("policy_gripper_target_margin_m must be a finite JSON number in [0, 0.005]")
        low, high = gripper_bounds
        gripper_bounds = (low + gripper_margin_m, high - gripper_margin_m)
    bounds = tuple((low + margin_deg, high - margin_deg)
                   for low, high in limits.bounds[:6]) + (gripper_bounds,)
    if any(low > high for low, high in bounds):
        raise GuardError("Policy target margin leaves no feasible target interval")
    return bounds


def project_joint_predictions(action, *, limits: GuardLimits, margin_deg, gripper_margin_m=None):
    """Bound finite ACT proposals; never transform actual feedback.

    The gripper retains strict input checks unless its inward margin is explicit.
    Negative learned closed-width proposals then map to a positive target, in
    metres, without changing calibration, feedback bounds or closing force.
    Final feedback/rate/CAN-grid shaping must also intersect these bounds.
    """
    bounds = joint_policy_target_bounds(limits, margin_deg, gripper_margin_m)
    target = list(vector(action))
    corrections = []
    for index, (key, value, (low, high)) in enumerate(zip(ACTION_KEYS, target, bounds, strict=True)):
        if index == 6 and gripper_margin_m is None:
            if not low <= value <= high:
                raise GuardError(f"{key}={value} outside [{low}, {high}]; no prediction projection permitted")
            continue
        projected = min(max(value, low), high)
        if projected != value:
            target[index] = projected
            corrections.append({"key": key, "original": value, "projected": projected})
    return tuple(target), tuple(corrections)


def j5_upper_target_bound(limits: GuardLimits, upper_margin_deg=0.0):
    """Return the inward policy ceiling without changing physical bounds.

    The margin is an explicit JSON number in [0, 5] degrees. It is not an
    allowance for observed feedback to exceed the unchanged hard limits.
    """
    if (type(upper_margin_deg) not in (int, float)
            or not 0 <= upper_margin_deg <= 5
            or not math.isfinite(upper_margin_deg)):
        raise GuardError("J5 upper target margin must be a finite JSON number in [0, 5] deg")
    low, high = limits.bounds[4]
    upper = float(min(high, ACT_J5_TRAINING_UPPER_DEG) - upper_margin_deg)
    if upper < low:
        raise GuardError("J5 upper target margin puts the policy ceiling below the guard lower bound")
    return upper


def saturate_j5_upper_prediction(action, *, limits: GuardLimits, upper_margin_deg=0.0):
    """Saturate the reviewed ACT output at its observed J5 training ceiling.

    This is an explicit policy-output transformation for the fixed checkpoint,
    whose training actions have a 70-degree upper plateau. An optional margin
    moves only this policy ceiling inward. It never transforms
    feedback or permits another axis, the gripper, J5's lower bound, or a
    nonfinite value to exceed the unchanged guard bounds. The normal relative
    cap, rate shaping, ActionGuard and receiver checks are still required.
    """
    upper = j5_upper_target_bound(limits, upper_margin_deg)
    target = list(vector(action))
    corrections = []
    for index, (key, value, (low, high)) in enumerate(zip(
            ACTION_KEYS, target, limits.bounds, strict=True)):
        if index == 4 and value > upper:
            target[index] = upper
            corrections.append({"key": key, "original": value, "projected": upper})
        elif not low <= value <= high:
            raise GuardError(f"{key}={value} outside [{low}, {high}]; prediction saturation not permitted")
    return tuple(target), tuple(corrections)


def project_j5_upper_prediction(action, *, limits: GuardLimits, tolerance_deg):
    """Return (bounded target, corrections), allowing only J5 upper overshoot.

    tolerance_deg is a JSON numeric value in [0, 1] degrees; zero disables the
    correction. Every other bound, including J5's lower bound, is a hard error.
    Corrections describe predictions, never actual observed feedback.
    """
    if (isinstance(tolerance_deg, bool) or not isinstance(tolerance_deg, (int, float))
            or not math.isfinite(tolerance_deg) or not 0 <= tolerance_deg <= 1):
        raise GuardError("J5 upper prediction tolerance must be a finite number in [0, 1] deg")
    target = list(vector(action))
    corrections = []
    for index, (key, value, (low, high)) in enumerate(zip(
            ACTION_KEYS, target, limits.bounds, strict=True)):
        if low <= value <= high:
            continue
        if index == 4 and high < value <= high + tolerance_deg:
            target[index] = float(high)
            corrections.append({"key": key, "original": value, "projected": float(high)})
            continue
        raise GuardError(f"{key}={value} outside [{low}, {high}]; prediction projection not permitted")
    return tuple(target), tuple(corrections)
