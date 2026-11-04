"""Optional proposal shaping for the reviewed PiPER approach phase.

This is a joint-window heuristic, not visual alignment or grasp confirmation.
It never changes a joint, generates an opening target, or bypasses the normal
feedback, rate, source-age and receiver guards. The caller must only retain the
returned phase after the corresponding send has been acknowledged/committed.
"""

from dataclasses import dataclass
import math
from collections.abc import Mapping

from .guard import GuardError, vector


# Joint window established from the recorded grasp approaches; it classifies a
# phase only and does not establish visual alignment or contact with the block.
APPROACH_GRIPPER = {
    "opening_intent_m": 0.035,
    "min_close_delta_m": 0.001,
    "retry_open_m": 0.055,
    "retry_joint1_max_deg": -10.0,
    "grasp_joint_bounds_deg": [
        [-27.0, -14.0], [96.0, 113.0], [-71.0, -46.0],
        [-4.0, 13.0], [35.0, 67.0], [-34.0, -12.0],
    ],
}


@dataclass(frozen=True)
class ApproachGripperState:
    opening_seen: bool = False
    closing_started: bool = False


_KEYS = frozenset(("opening_intent_m", "min_close_delta_m", "retry_open_m",
                   "retry_joint1_max_deg", "grasp_joint_bounds_deg"))


def _number(value, name):
    if type(value) not in (int, float):
        raise GuardError(f"{name} must be a finite JSON number")
    try:
        result = float(value)
    except (ValueError, OverflowError) as error:
        raise GuardError(f"{name} must be a finite JSON number") from error
    if not math.isfinite(result):
        raise GuardError(f"{name} must be a finite JSON number")
    return result


def _pairs(value, *, count, name):
    if not isinstance(value, (list, tuple)) or len(value) != count:
        raise GuardError(f"{name} must contain exactly {count} bound pairs")
    result = []
    for index, pair in enumerate(value):
        if not isinstance(pair, (list, tuple)) or len(pair) != 2:
            raise GuardError(f"{name}[{index}] must be a bound pair")
        low, high = (_number(x, f"{name}[{index}]") for x in pair)
        if low >= high:
            raise GuardError(f"{name}[{index}] must have strictly ordered bounds")
        result.append((low, high))
    return tuple(result)


def _config_values(config):
    if not isinstance(config, Mapping) or set(config) != _KEYS:
        raise GuardError("policy_approach_gripper must contain exactly: "
                         + ", ".join(sorted(_KEYS)))
    result = {key: _number(config[key], key) for key in _KEYS
              if key != "grasp_joint_bounds_deg"}
    result["grasp_joint_bounds_deg"] = _pairs(
        config["grasp_joint_bounds_deg"], count=6, name="grasp_joint_bounds_deg")
    if not 0 <= result["opening_intent_m"] < result["retry_open_m"]:
        raise GuardError("Opening intent and retry widths must be nonnegative and ordered")
    if not 0 < result["min_close_delta_m"] < result["retry_open_m"]:
        raise GuardError("min_close_delta_m must be positive and below retry_open_m")
    return result


def validate_approach_gripper(config, *, bounds):
    """Return a validated copy, with all windows inside reviewed guard bounds."""
    result = _config_values(config)
    reviewed = _pairs(bounds, count=7, name="bounds")
    for index, ((low, high), (guard_low, guard_high)) in enumerate(zip(
            result["grasp_joint_bounds_deg"], reviewed[:6], strict=True)):
        if not guard_low <= low < high <= guard_high:
            raise GuardError(f"grasp_joint_bounds_deg[{index}] exceeds reviewed bounds")
    low, high = reviewed[6]
    if not low <= result["opening_intent_m"] < result["retry_open_m"] <= high:
        raise GuardError("Approach gripper widths exceed reviewed bounds")
    if result["min_close_delta_m"] >= high - low:
        raise GuardError("min_close_delta_m must be smaller than the reviewed width range")
    if not reviewed[0][0] <= result["retry_joint1_max_deg"] <= reviewed[0][1]:
        raise GuardError("retry_joint1_max_deg exceeds reviewed joint 1 bounds")
    return result


def shape_approach_gripper(action, *, observed_state, previous_target, phase, config):
    """Return (candidate, proposed phase, held) without mutating any input.

    Before an opening has been observed/intended, startup closed-width targets
    pass through. Later, an early closing proposal outside the reviewed joint
    window holds only the last acknowledged gripper target. Once closing has
    started inside the window, carrying/placing/returning pass through until a
    new approach opening satisfies the explicit retry reset conditions.
    """
    target, observed, previous = (vector(x) for x in
                                   (action, observed_state, previous_target))
    if (not isinstance(phase, ApproachGripperState)
            or type(phase.opening_seen) is not bool
            or type(phase.closing_started) is not bool):
        raise GuardError("Expected a boolean ApproachGripperState")
    settings = _config_values(config)
    opening = settings["opening_intent_m"]
    delta = settings["min_close_delta_m"]
    in_window = all(low <= actual <= high for actual, (low, high) in zip(
        observed[:6], settings["grasp_joint_bounds_deg"], strict=True))
    opening_seen = (phase.opening_seen or observed[6] >= opening
                    or (target[6] >= opening and target[6] > observed[6] + delta))
    closing_started = phase.closing_started
    if (target[6] >= settings["retry_open_m"]
            and observed[0] <= settings["retry_joint1_max_deg"]
            and not in_window):
        closing_started = False
    wants_close = target[6] < previous[6] - delta
    if not closing_started and opening_seen and in_window and wants_close:
        closing_started = True
    # The 1 mm intent threshold must not act as a per-command allowance here:
    # smaller decreases could otherwise accumulate into a full early closure.
    # Retain the last acknowledged width on every actual decrease outside the
    # approach window; the threshold only classifies opening/closing intent.
    held = bool(not closing_started and opening_seen and not in_window
                and target[6] < previous[6])
    if held:
        target = target[:6] + (previous[6],)
    return target, ApproachGripperState(opening_seen, closing_started), held
