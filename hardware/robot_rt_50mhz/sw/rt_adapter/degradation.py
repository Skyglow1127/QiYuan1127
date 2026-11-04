"""Offline recommendation/state machine. No automatic model or hardware changes."""
from dataclasses import dataclass
import math
import numpy as np
from .trajectory import C2Tracker, TrajectoryLimitError


@dataclass(frozen=True)
class DegradationDecision:
    state: str
    reason: str
    recommendations: tuple = ()


class DegradationController:
    def __init__(self, tracker: C2Tracker, *, consume_period_s, source_max_age_s, stop_velocity_tolerance):
        if min(consume_period_s, source_max_age_s, stop_velocity_tolerance) <= 0 or not all(math.isfinite(x) for x in (consume_period_s, source_max_age_s, stop_velocity_tolerance)):
            raise ValueError("explicit positive degradation time/tolerance parameters required")
        self.tracker, self.consume_period_s, self.source_max_age_s = tracker, consume_period_s, source_max_age_s
        self.stop_velocity_tolerance = stop_velocity_tolerance
        self.starve_started = None
        self.state = "RUN"
        self.last_evaluation_at = -float("inf")

    def inference_health(self, inference_s):
        if inference_s < 0 or not math.isfinite(inference_s):
            raise ValueError("finite nonnegative inference time required")
        recommendations = ("review compute/model settings and preprocessing equivalence", "consider a larger explicit consume window within freshness constraints") if inference_s > self.consume_period_s/2 else ()
        return DegradationDecision(self.state, "slow_inference" if recommendations else "within_budget", recommendations)

    def evaluate(self, *, now, observation_at=None, block_available=False, bounds_ok=True, ik_ok=True):
        if not math.isfinite(now) or now < self.last_evaluation_at:
            raise ValueError("finite monotonic current time required")
        if observation_at is not None and (not math.isfinite(observation_at) or observation_at > now):
            raise ValueError("observation timestamp must be finite and not future")
        if block_available and observation_at is None:
            raise ValueError("available block requires its original observation timestamp")
        self.last_evaluation_at = now
        if self.state == "FAULT":
            return DegradationDecision("FAULT", "external_stop_required_latched")
        stale = observation_at is not None and now-observation_at > self.source_max_age_s
        if not bounds_ok or not ik_ok:
            return self._stop_decision(now, "reject_block_bounds_or_ik")
        if stale:
            return self._stop_decision(now, "stale_observation_stop_new_blocks")
        if block_available:
            self.starve_started, self.state = None, "RUN"
            return DegradationDecision(self.state, "fresh_block")
        return self._stop_decision(now, "starve_braking")

    def _stop_decision(self, now, reason):
        if self.starve_started is None: self.starve_started = now
        stopped = np.max(abs(self.tracker.v)) <= self.stop_velocity_tolerance and np.max(abs(self.tracker.a)) <= self.stop_velocity_tolerance
        elapsed = now-self.starve_started
        if elapsed > 2*self.consume_period_s and not stopped:
            self.state = "FAULT"
            return DegradationDecision(self.state, "braking_deadline_unmet_external_stop_required")
        self.state = "HOLD" if stopped else "RAMP"
        return DegradationDecision(self.state, reason)

    def brake_tick(self, dt):
        # Bounded braking is needed before HOLD; never set nonzero v/a instantly to zero.
        try:
            return self.tracker.advance(self.tracker.q, dt, braking=True)
        except TrajectoryLimitError:
            self.state = "FAULT"
            raise
