"""Finite-action longitudinal planning using public observations exclusively."""
from __future__ import annotations

from time import perf_counter

import numpy as np

from .prediction import AnalyticPredictor, longitudinal_motion
from .types import ControlDecision, EgoState, TrackSample


class SafetyController:
    """Short-horizon planner with a separate analytic braking guard.

    The guard assumes a lead vehicle may have braked at 8 m/s² since its latest
    observation, and ego can brake at 6 m/s² after one 0.1-second action period.
    Cut-in tracks predicted to occupy ego's lane are conservatively guarded as
    potential leads. This finite-grid, ideal-state model is NOT a safety proof.
    """

    CANDIDATES = (-6.0, -4.0, -2.0, 0.0, 1.0, 2.0)
    CONTROL_DT = 0.1
    HORIZON = 2.0
    EGO_BRAKING = 6.0
    LEAD_BRAKING = 8.0

    def __init__(self, predictor=None, margin_mode: str = "fixed",
                 target_speed: float = 25.0, fixed_margin: float = 2.0,
                 max_age: float = 0.8, guard: bool = True, smooth: bool = False, recovery_step: float = .4):
        if margin_mode not in {"fixed", "adaptive"}:
            raise ValueError("margin_mode must be 'fixed' or 'adaptive'")
        if not np.all(np.isfinite([target_speed, fixed_margin, max_age])) or min(
                target_speed, fixed_margin, max_age) < 0:
            raise ValueError("Controller parameters must be finite and nonnegative")
        self.predictor = predictor if predictor is not None else AnalyticPredictor()
        self.margin_mode = margin_mode
        self.target_speed = target_speed
        self.fixed_margin = fixed_margin
        self.max_age = max_age
        self.guard = guard
        if not np.isfinite(recovery_step) or recovery_step < .4:
            raise ValueError("recovery_step must be finite and >= .4")
        self.recovery_step = recovery_step
        self.smooth = smooth
        if smooth:
            self.CANDIDATES = tuple(float(x) for x in np.round(np.arange(-6., 2.001, .25), 8))
        self.comfort_step = .4  # preferred |delta a| per 0.1s; safety overrides
        self.last_diagnostics = {}
        self._offsets = np.linspace(0.0, self.HORIZON, 21)

    def _margin(self, horizons: np.ndarray, short_history: bool) -> np.ndarray:
        if self.margin_mode == "fixed":
            margin = np.tile([self.fixed_margin, 0.35], (len(horizons), 1))
        else:
            margin = np.asarray(self.predictor.uncertainty(horizons), dtype=float)
            if margin.shape != (len(horizons), 2) or np.any(~np.isfinite(margin)) or np.any(margin <= 0):
                raise ValueError("Predictor uncertainty must be positive finite (T, 2)")
            margin = np.maximum(margin, [0.5, 0.15])
        if short_history:
            margin = margin + np.array([1.0, 0.15])
        return margin

    def _braking_guard(self, ego: EgoState, last: TrackSample,
                       age: float, acceleration: float) -> bool:
        """Check a candidate prefix followed by braking against a braking lead.

        Lead motion starts at its old measurement, not the predictor's estimated
        current state. This accounts for unobserved braking during data latency.
        """
        dt = self.CONTROL_DT
        xp, vp = longitudinal_motion(ego.x, ego.speed, acceleration, np.array([dt]))
        end = dt + float(vp[0]) / self.EGO_BRAKING + 0.04
        times = np.arange(0.0, end + 0.02, 0.02)
        prefix = np.minimum(times, dt)
        ego_x, _ = longitudinal_motion(ego.x, ego.speed, acceleration, prefix)
        later = times > dt
        ego_x[later], _ = longitudinal_motion(float(xp[0]), float(vp[0]),
                                              -self.EGO_BRAKING, times[later] - dt)
        lead_x, _ = longitudinal_motion(last.state.x, max(0.0, last.state.vx),
                                        -self.LEAD_BRAKING, age + times)
        required = (ego.length + last.state.length) / 2 + 1.0
        return bool(np.all(lead_x - ego_x >= required - 1e-9))

    def decide(self, ego: EgoState,
               tracks: dict[int, tuple[TrackSample, ...]]) -> ControlDecision:
        self.last_diagnostics = {"candidates": 0, "prediction_rejected": 0, "guard_rejected": 0, "guard_only_rejected": 0, "comfort_override": 0}
        if not np.all(np.isfinite([ego.time, ego.x, ego.y, ego.speed])) or ego.speed < 0:
            raise ValueError("Invalid ego state")
        histories = [history for history in tracks.values() if history]
        if not histories:
            # The interface has no timestamp for a valid empty sensor frame, so
            # an empty dictionary cannot be distinguished from absent sensing.
            return ControlDecision(-6.0, "no_observation", True, None, 0.0,
                                   degraded=True)
        latest = max(history[-1].captured_at for history in histories)
        ages = [ego.time - history[-1].captured_at for history in histories]
        if min(ages) < -1e-8:
            raise ValueError("Future observations must not enter the controller")
        max_age = max(ages)
        if max_age > self.max_age + 1e-9:
            return ControlDecision(-6.0, "stale_observation", True, latest,
                                   max_age, degraded=True)

        started = perf_counter()
        predictions = []
        degraded = False
        any_short_history = False
        for history, age in zip(histories, ages):
            short_history = len(history) < 2
            any_short_history |= short_history
            degraded |= short_history
            # Predictor and uncertainty must see the same call. Learned and
            # calibrated wrappers own their short-history fallback and select
            # the corresponding analytic calibration in uncertainty().
            prediction = np.asarray(self.predictor.predict(history, ego.time + self._offsets), dtype=float)
            degraded |= bool(getattr(self.predictor, "last_degraded", False))
            if prediction.shape != (len(self._offsets), 4) or np.any(~np.isfinite(prediction)):
                raise ValueError("Predictor must return finite (T, 4) world states")
            margin = self._margin(self._offsets + max(0.0, age), short_history)
            if self.margin_mode == "adaptive":
                degraded |= bool(getattr(self.predictor, "last_uncertainty_degraded", False))
            last = history[-1]
            lateral_overlap = np.abs(prediction[:, 1] - ego.y) < (
                (ego.width + last.state.width) / 2 + margin[:, 1])
            potential_lead = prediction[0, 0] >= ego.x and bool(np.any(lateral_overlap))
            predictions.append((prediction, margin, last, age, lateral_overlap, potential_lead))
        prediction_ms = (perf_counter() - started) * 1000

        admissible = []
        for acceleration in self.CANDIDATES:
            ego_x, ego_v = longitudinal_motion(ego.x, ego.speed, acceleration, self._offsets)
            prediction_rejected = False
            guard_rejected = False
            for prediction, margin, last, age, lateral_overlap, potential_lead in predictions:
                longitudinal_overlap = np.abs(prediction[:, 0] - ego_x) < (
                    (ego.length + last.state.length) / 2 + margin[:, 0])
                prediction_rejected |= bool(np.any(longitudinal_overlap & lateral_overlap))
                if self.guard and potential_lead:
                    guard_rejected |= not self._braking_guard(ego, last, max(0.0, age), acceleration)
            d = self.last_diagnostics
            d['candidates'] += 1
            d['prediction_rejected'] += int(prediction_rejected)
            d['guard_rejected'] += int(guard_rejected)
            d['guard_only_rejected'] += int(guard_rejected and not prediction_rejected)
            safe = not prediction_rejected and not guard_rejected
            if safe:
                speed_error = float(np.mean((ego_v - self.target_speed) ** 2))
                comfort = 0.10 * acceleration ** 2 + 0.10 * (acceleration - ego.last_acceleration) ** 2
                admissible.append((speed_error + comfort, acceleration))
        if not admissible:
            return ControlDecision(-6.0, "no_safe_candidate", True, latest,
                                   max_age, 0, prediction_ms, degraded)
        if self.smooth:
            upper_step = self.recovery_step if ego.last_acceleration < -.5 else self.comfort_step
            comfortable = [item for item in admissible if -self.comfort_step-1e-9 <= item[1] - ego.last_acceleration <= upper_step+1e-9]
            if comfortable:
                admissible = comfortable
            else:
                self.last_diagnostics['comfort_override'] = 1
        acceleration = min(admissible)[1]
        reason = "short_history_fallback" if any_short_history else "predictor_fallback" if degraded else "planned"
        return ControlDecision(acceleration, reason, acceleration <= -6.0,
                               latest, max_age, len(admissible), prediction_ms, degraded)
