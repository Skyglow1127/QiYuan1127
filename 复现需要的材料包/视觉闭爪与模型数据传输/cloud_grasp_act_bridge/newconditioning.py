"""Pure first-command feasibility for the explicitly conditioned PI05 trial.

This module has no transport, sleeps, Robot construction, camera or model calls.
It reuses the existing executor's projection, rate shaper and ActionGuard. The
numerical guard here has synthetic readiness labels: it cannot authorize motion.
"""
from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path
import sys

DEPLOYMENT = Path("/home/aup/Lerobot/projects/piper/deployment")
if str(DEPLOYMENT) not in sys.path:
    sys.path.insert(0, str(DEPLOYMENT))

from lerobot_robot_piper_executor.guard import ACTION_KEYS, ActionGuard, GuardError, GuardLimits, Readiness, vector
from lerobot_robot_piper_executor.policy_bounds import joint_policy_target_bounds, project_joint_predictions
from lerobot_robot_piper_executor.target_rate import shape_target_rate, target_rate_wait_s

CONDITIONING_ID = "pi05-native-queue-act-hardware-bridge/v1"
MAX_FIRST_COMMAND_WAIT_S = .05


def guard_limits(value):
    return value if isinstance(value, GuardLimits) else GuardLimits(**value)


def profile_sha256(profile):
    return hashlib.sha256(json.dumps(profile, sort_keys=True, separators=(",", ":"),
                                     allow_nan=False).encode()).hexdigest()


def raw_tracking_rejections(raw, state, limits):
    """Report raw proposal deviations for diagnostics, never for motion approval.

    Finite vectors are mandatory. The unchanged ActionGuard applies the actual
    8-degree/15-mm tracking budget to the conditioned command at both endpoints.
    """
    raw, state, limits = vector(raw), vector(state), guard_limits(limits)
    return [f"{key}: raw prediction differs from input feedback more than max_tracking_error"
            for key, target, observed, maximum in zip(ACTION_KEYS, raw, state,
                                                       limits.max_tracking_error, strict=True)
            if abs(target-observed) > maximum]


def _profile(profile):
    expected = {
        "policy_joint_target_margin_deg": 2.0,
        "policy_gripper_target_margin_m": .001,
        "max_relative_target": dict(zip(ACTION_KEYS, (2.,)*6+(.003,), strict=True)),
        "policy_target_rate_fraction": .9,
    }
    if profile != expected:
        raise GuardError("Unexpected command-conditioning profile; review a new profile instead of changing this trial silently")
    return profile


def condition_first_command(raw, state, *, limits, profile, sampled_at,
                            validated_at, source_kind, rate_anchor_at=None):
    """Return a hypothetical guarded first command; never emit or wait for it.

    The hypothetical previous target is the observed state. The optional rate
    anchor is input-ready time: real execution arms before its first loop input,
    so this later time grants no more elapsed rate budget than actual arming.
    This does not verify the actual arm target/time or later feedback. Omission
    preserves the earlier zero-budget-at-validation assumption. Computed extra
    wait stays bounded and included in the unchanged original source deadline.
    """
    limits, profile = guard_limits(limits), _profile(profile)
    raw, state = vector(raw), vector(state)
    guard = ActionGuard(limits)
    # Actual observed feedback remains a hard error before any target projection.
    guard._bounded(state)
    raw_rejections = raw_tracking_rejections(raw, state, limits)
    if source_kind != "live" or sampled_at is None:
        raise GuardError("First-command feasibility requires original live capture timestamps")
    rate_anchor = validated_at if rate_anchor_at is None else rate_anchor_at
    if not all(isinstance(t, (int, float)) and not isinstance(t, bool) and math.isfinite(t)
               for t in (sampled_at, rate_anchor, validated_at)) or not sampled_at <= rate_anchor <= validated_at:
        raise GuardError("Invalid/future original observation or hypothetical rate-anchor timing")
    elapsed = validated_at-rate_anchor
    if validated_at-sampled_at > min(limits.max_observation_age_s, limits.max_command_age_s):
        raise GuardError("Original observation/proposal already expired before conditioning")
    targets = joint_policy_target_bounds(limits, profile["policy_joint_target_margin_deg"],
                                         profile["policy_gripper_target_margin_m"])
    projected, projection_corrections = project_joint_predictions(raw, limits=limits,
        margin_deg=profile["policy_joint_target_margin_deg"],
        gripper_margin_m=profile["policy_gripper_target_margin_m"])
    guard._bounded(projected)
    caps = vector(profile["max_relative_target"])
    # Exactly the arithmetic in official ensure_safe_goal_position, without its
    # logging side effect. The normal executor calls that official helper itself.
    capped = tuple(observed+max(min(target-observed, cap), -cap)
                   for target, observed, cap in zip(projected, state, caps, strict=True))
    shape_args = dict(state=state, previous_target=state, limits=limits,
        relative_cap=profile["max_relative_target"],
        speed_fraction=profile["policy_target_rate_fraction"], target_bounds=targets)
    wait = target_rate_wait_s(elapsed_s=elapsed, **shape_args)
    if not math.isfinite(wait) or not 0 <= wait <= MAX_FIRST_COMMAND_WAIT_S:
        raise GuardError("First-command rate/bound intervals require more than 50ms; no preview wait is performed")
    simulated_at = validated_at+wait
    expiry = sampled_at+min(limits.max_observation_age_s, limits.max_command_age_s)
    if simulated_at >= expiry:
        raise GuardError("Original source/proposal would expire during computed first-command wait")
    shaped = shape_target_rate(capped, elapsed_s=elapsed+wait, **shape_args)
    synthetic_readiness = Readiness(**{key: "PURE NUMERICAL PREVIEW ONLY; no physical readiness or transport"
                                      for key in Readiness.__dataclass_fields__})
    guard.arm(state, sampled_at=sampled_at, now=rate_anchor, readiness=synthetic_readiness)
    command = guard.prepare(shaped, observed_state=state, sampled_at=sampled_at,
                            proposal_at=sampled_at, now=simulated_at)
    guard.require_sendable(command, now=simulated_at)
    corrections = []
    for stage, before, after in (("inward_policy_projection", raw, projected),
                                  ("feedback_relative_cap", projected, capped),
                                  ("rate_and_CAN_grid", capped, shaped)):
        corrections.extend({"stage": stage, "key": key, "before": old, "after": new}
                           for key, old, new in zip(ACTION_KEYS, before, after, strict=True)
                           if old != new)
    return {
        "command_conditioning_id": CONDITIONING_ID,
        "command_conditioning_sha256": profile_sha256(profile),
        "raw_tracking_rejections": raw_rejections,
        "raw_tracking_gate": "diagnostic_only",
        "raw_prediction": list(raw), "projected_target": list(projected),
        "relative_capped_target": list(capped), "quantized_target": list(shaped),
        "policy_target_bounds": [list(pair) for pair in targets],
        "projection_corrections": list(projection_corrections), "corrections": corrections,
        "numerical_guard_prepare_pass": True,
        "computed_first_command_wait_s": wait, "actual_wait_performed_s": 0.,
        "hypothetical_command_at_monotonic": simulated_at,
        "original_source_monotonic": sampled_at, "original_expiry_monotonic": expiry,
        "source_age_at_hypothetical_command_s": simulated_at-sampled_at,
        "assumptions": {"first_command_only": True, "anchor_target": list(state),
            "anchor_at_monotonic": rate_anchor, "initial_rate_elapsed_s": elapsed,
            "rate_anchor_source": "validation_default_zero_budget" if rate_anchor_at is None else "input_ready_hypothesis_after_actual_execution_arm",
            "rate_anchor_is_hypothetical_not_verified_arm_time": True,
            "observed_state_assumed_unchanged_during_inference_and_computed_wait": True,
            "fresh_feedback_after_inference_verified": False,
            "real_arming_anchor_verified": False, "continuous_commit_history_verified": False,
            "receiver_or_CAN_transmission_tested_by_live_preview": False},
    }
