"""Pure VLA gripper candidate gate; 2-D red geometry is not contact proof.

The caller owns image freshness, frame identity, existing physical Guards and
transactional state: commit the returned GateState only after a successful send.
No robot, camera, network, clock, file or thread is owned by this module.
"""
from __future__ import annotations

from dataclasses import dataclass
import math

import cv2
import numpy as np


CONFIG_KEYS = frozenset({
    "center_x_range", "center_y_min", "bottom_y_min", "area_fraction_range",
    "second_largest_ratio_max", "opening_seen_m", "closing_delta_min_m",
    "near_frames", "max_joint_error_deg", "grasp_joint_bounds_deg",
    "reset_opening_m", "reset_j1_max_deg",
})


def _number(value, name):
    if isinstance(value, (bool, np.bool_)) or not isinstance(value, (int, float, np.integer, np.floating)) \
            or not math.isfinite(float(value)):
        raise ValueError(f"{name} must be a finite number")
    return float(value)


def _range(value, name, *, minimum=None, maximum=None):
    if not isinstance(value, (tuple, list)) or len(value) != 2:
        raise ValueError(f"{name} must contain two ordered numbers")
    lower, upper = (_number(x, name) for x in value)
    if not lower < upper or minimum is not None and lower < minimum \
            or maximum is not None and upper > maximum:
        raise ValueError(f"{name} is outside its allowed ordered range")
    return (lower, upper)


def validate_config(config):
    """Return an independent normalized config, rejecting unknown/missing keys."""
    if not isinstance(config, dict) or set(config) != CONFIG_KEYS:
        raise ValueError("Visual gate config must contain exactly the documented keys")
    result = {
        "center_x_range": _range(config["center_x_range"], "center_x_range", minimum=0, maximum=1),
        "area_fraction_range": _range(config["area_fraction_range"], "area_fraction_range", minimum=0, maximum=1),
    }
    for key in ("center_y_min", "bottom_y_min", "second_largest_ratio_max"):
        value = _number(config[key], key)
        if not 0 < value <= 1 or key == "second_largest_ratio_max" and value >= 1:
            raise ValueError(f"{key} must be in its normalized image range")
        result[key] = value
    for key in ("opening_seen_m", "reset_opening_m", "closing_delta_min_m"):
        value = _number(config[key], key)
        if not 0 < value <= .1:
            raise ValueError(f"{key} must be positive and at most 0.1 metre")
        result[key] = value
    if not result["closing_delta_min_m"] < result["opening_seen_m"] < result["reset_opening_m"]:
        raise ValueError("Expected close delta < opening threshold < reset opening threshold")
    if type(config["near_frames"]) is not int or config["near_frames"] != 2:
        raise ValueError("This candidate requires exactly two distinct near frames")
    result["near_frames"] = 2
    result["max_joint_error_deg"] = _number(config["max_joint_error_deg"], "max_joint_error_deg")
    if not 0 < result["max_joint_error_deg"] <= 3:
        raise ValueError("Joint agreement threshold must be positive and no greater than 3 degrees")
    result["reset_j1_max_deg"] = _number(config["reset_j1_max_deg"], "reset_j1_max_deg")
    if not -180 <= result["reset_j1_max_deg"] < 0:
        raise ValueError("Reset J1 threshold must be between -180 and zero degrees")
    bounds = config["grasp_joint_bounds_deg"]
    if not isinstance(bounds, (list, tuple)) or len(bounds) != 6:
        raise ValueError("Expected six grasp joint ranges")
    result["grasp_joint_bounds_deg"] = tuple(
        _range(value, f"grasp_joint_bounds_deg[{i}]", minimum=-180, maximum=180)
        for i, value in enumerate(bounds)
    )
    return result


def _invalid_geometry(reason):
    return {"valid": False, "red_detected": False, "near": False,
            "ambiguous": False, "reason": reason, "bbox_xywh": None,
            "centroid_xy": None, "area_px": 0, "area_fraction": 0.0,
            "second_to_largest_area_ratio": None,
            "candidate_only": True, "contact_or_alignment_proven": False}


def red_geometry(rgb, config):
    """Measure the largest red component in uint8 RGB [480,640,3]."""
    cfg = validate_config(config)
    if not isinstance(rgb, np.ndarray) or rgb.dtype != np.uint8 or rgb.shape != (480, 640, 3):
        return _invalid_geometry("Expected uint8 RGB image with shape [480,640,3]")
    hsv = cv2.cvtColor(np.ascontiguousarray(rgb), cv2.COLOR_RGB2HSV)
    low = cv2.inRange(hsv, (0, 90, 30), (12, 255, 255))
    high = cv2.inRange(hsv, (170, 90, 30), (179, 255, 255))
    mask = cv2.bitwise_or(low, high)
    kernel = np.ones((3, 3), dtype=np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    count, _, stats, centers = cv2.connectedComponentsWithStats(mask, connectivity=8)
    if count <= 1:
        result = _invalid_geometry("No red component after morphology")
        result["valid"] = True
        return result
    ordered = sorted(range(1, count), key=lambda i: int(stats[i, cv2.CC_STAT_AREA]), reverse=True)
    index = ordered[0]
    x, y, width, height, area = (int(v) for v in stats[index])
    cx, cy = (float(v) for v in centers[index])
    area_fraction = area / (640 * 480)
    second_area = int(stats[ordered[1], cv2.CC_STAT_AREA]) if len(ordered) > 1 else 0
    ratio = second_area / area
    ambiguous = ratio > cfg["second_largest_ratio_max"]
    checks = {
        "center_x": cfg["center_x_range"][0] <= cx / 640 <= cfg["center_x_range"][1],
        "center_y": cy / 480 >= cfg["center_y_min"],
        "bottom_y": (y + height) / 480 >= cfg["bottom_y_min"],
        "area": cfg["area_fraction_range"][0] <= area_fraction <= cfg["area_fraction_range"][1],
        "unambiguous": not ambiguous,
    }
    return {
        "valid": True, "red_detected": True, "near": all(checks.values()),
        "ambiguous": ambiguous, "reason": "Candidate image conditions satisfied" if all(checks.values())
        else "Candidate image conditions failed: " + ",".join(k for k, v in checks.items() if not v),
        "bbox_xywh": (x, y, width, height), "centroid_xy": (cx, cy),
        "area_px": area, "area_fraction": area_fraction,
        "center_x_fraction": cx / 640, "center_y_fraction": cy / 480,
        "bottom_y_fraction": (y + height) / 480,
        "second_area_px": second_area, "second_to_largest_area_ratio": ratio,
        "red_components": count - 1, "checks": checks,
        "candidate_only": True, "contact_or_alignment_proven": False,
    }


@dataclass(frozen=True)
class GateState:
    opening_seen: bool = False
    closing_started: bool = False
    near_streak: int = 0
    last_frame_id: object = None


def _frame_id_valid(value):
    if value is None:
        return True
    if type(value) is int:
        return value >= 0
    if type(value) is str:
        return bool(value)
    return type(value) is tuple and bool(value) and all(
        v is not None and _frame_id_valid(v) for v in value
    )


def _vector(value, name):
    try:
        raw = np.asarray(value)
    except (TypeError, ValueError) as error:
        raise ValueError(f"{name} must contain seven finite numbers") from error
    if raw.shape != (7,) or raw.dtype.kind not in "iuf" or not np.isfinite(raw).all():
        raise ValueError(f"{name} must contain seven finite numbers")
    return tuple(float(v) for v in raw)


def _geometry_near(geometry, cfg):
    if not isinstance(geometry, dict) or geometry.get("valid") is not True \
            or geometry.get("red_detected") is not True or geometry.get("near") is not True \
            or geometry.get("ambiguous") is not False:
        return False
    try:
        cx, cy, bottom, area, ratio = (
            _number(geometry[key], key) for key in (
                "center_x_fraction", "center_y_fraction", "bottom_y_fraction",
                "area_fraction", "second_to_largest_area_ratio")
        )
    except (KeyError, ValueError):
        return False
    return (cfg["center_x_range"][0] <= cx <= cfg["center_x_range"][1]
            and cfg["center_y_min"] <= cy <= 1
            and cfg["bottom_y_min"] <= bottom <= 1
            and cfg["area_fraction_range"][0] <= area <= cfg["area_fraction_range"][1]
            and 0 <= ratio <= cfg["second_largest_ratio_max"])


def shape_gripper(action7, observed7, previous_target7, phase, geometry, frame_id, config):
    """Return candidate, uncommitted next state, and diagnostic; keep six joints."""
    cfg = validate_config(config)
    action, observed, previous = (_vector(v, name) for v, name in (
        (action7, "action7"), (observed7, "observed7"), (previous_target7, "previous_target7")))
    if not isinstance(phase, GateState) or type(phase.opening_seen) is not bool \
            or type(phase.closing_started) is not bool or type(phase.near_streak) is not int \
            or not 0 <= phase.near_streak <= cfg["near_frames"] \
            or phase.closing_started and not phase.opening_seen \
            or not _frame_id_valid(phase.last_frame_id) or not _frame_id_valid(frame_id):
        raise ValueError("Expected a valid frozen GateState and immutable frame identity")
    opening_intent = (action[6] >= cfg["opening_seen_m"]
                      and action[6] - observed[6] > cfg["closing_delta_min_m"])
    opening_seen = (phase.opening_seen or observed[6] >= cfg["opening_seen_m"]
                    or previous[6] >= cfg["opening_seen_m"] or opening_intent)
    in_grasp = all(lo <= v <= hi for v, (lo, hi) in zip(observed[:6], cfg["grasp_joint_bounds_deg"]))
    errors = tuple(abs(a - o) for a, o in zip(action[:6], observed[:6]))
    aligned = max(errors) <= cfg["max_joint_error_deg"]
    visual_near = frame_id is not None and _geometry_near(geometry, cfg)
    unique_frame = frame_id is not None and frame_id != phase.last_frame_id
    streak, last_frame = phase.near_streak, phase.last_frame_id
    closing_started = phase.closing_started
    reset = action[6] >= cfg["reset_opening_m"] and observed[0] <= cfg["reset_j1_max_deg"] and not in_grasp
    if reset:
        closing_started, streak = False, 0
        if frame_id is not None:
            last_frame = frame_id
    elif opening_seen and not closing_started:
        if unique_frame:
            streak = min(streak + 1, cfg["near_frames"]) if visual_near else 0
            last_frame = frame_id
        elif not visual_near:
            # Missing/invalid current geometry never preserves permission.
            streak = 0
    reference = max(previous[6], observed[6])
    requested_closing = action[6] < reference
    closing_delta = reference - action[6]
    may_start = (opening_seen and visual_near and streak >= cfg["near_frames"]
                 and in_grasp and aligned and closing_delta > cfg["closing_delta_min_m"])
    candidate = action
    held = False
    if not opening_seen:
        decision = "initialization_passthrough"
    elif closing_started:
        decision = "latched_passthrough"
    elif requested_closing:
        if may_start and not reset:
            closing_started = True
            decision = "closure_latched"
        else:
            candidate = (*action[:6], reference)
            held = True
            decision = "closure_held"
    else:
        decision = "opening_passthrough"
    pending = GateState(opening_seen, closing_started, streak, last_frame)
    return candidate, pending, {
        "candidate_only": True, "contact_or_alignment_proven": False,
        "decision": decision, "held": held, "reset": reset,
        "requested_closing": requested_closing, "closing_delta_m": closing_delta,
        "reference_gripper_m": reference, "raw_proposal_gripper_m": action[6],
        "candidate_gripper_m": candidate[6], "visual_near": visual_near,
        "unique_frame": unique_frame, "near_streak": streak,
        "actual_in_grasp_window": in_grasp, "raw_actual_joint_errors_deg": errors,
        "joints_within_agreement": aligned, "opening_seen": opening_seen,
        "opening_intent": opening_intent, "closing_started": closing_started,
    }
