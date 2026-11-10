"""Observation-only motion predictors; initial uncertainty is NOT calibrated."""
from __future__ import annotations

import numpy as np

from .types import TrackSample


def longitudinal_motion(position: float, speed: float, acceleration: float,
                        times: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Integrate constant acceleration exactly, stopping at zero without reversing.

    ``times`` are nonnegative offsets, not absolute simulator timestamps.
    """
    t = np.asarray(times, dtype=float)
    if np.any(~np.isfinite(t)) or np.any(t < 0):
        raise ValueError("Motion offsets must be finite and nonnegative")
    if not np.all(np.isfinite([position, speed, acceleration])) or speed < 0:
        raise ValueError("Invalid forward longitudinal state")
    moving = np.minimum(t, speed / -acceleration) if acceleration < 0 else t
    x = position + speed * moving + 0.5 * acceleration * moving ** 2
    v = np.maximum(0.0, speed + acceleration * moving)
    return x, v


class AnalyticPredictor:
    """CV/limited-CA predictor using only timestamped observations.

    A single sample falls back to CV. CA uses the last two velocity observations;
    acceleration bounds are explicit engineering assumptions, not inferred truth.
    """

    uncertainty_kind = "engineering_heuristic_not_calibrated"

    def __init__(self, kind: str = "ca"):
        if kind not in {"cv", "ca"}:
            raise ValueError("kind must be 'cv' or 'ca'")
        self.kind = kind

    def predict(self, samples: tuple[TrackSample, ...],
                query_times: np.ndarray) -> np.ndarray:
        if not samples:
            raise ValueError("At least one observation is required")
        q = np.asarray(query_times, dtype=float)
        if q.ndim != 1 or np.any(~np.isfinite(q)):
            raise ValueError("query_times must be a finite 1-D array")
        last = samples[-1]
        if any(s.state.track_id != last.state.track_id for s in samples):
            raise ValueError("A history must belong to one track")
        timestamps = np.array([s.captured_at for s in samples])
        if np.any(~np.isfinite(timestamps)) or np.any(np.diff(timestamps) <= 0):
            raise ValueError("History timestamps must be strictly increasing")
        h = q - last.captured_at
        if np.any(h < -1e-9):
            raise ValueError("Cannot predict before the latest observation")
        h = np.maximum(h, 0)
        s = last.state
        if not np.all(np.isfinite([s.x, s.y, s.vx, s.vy])):
            raise ValueError("Non-finite track state")
        ax = ay = 0.0
        if self.kind == "ca" and len(samples) >= 2:
            previous = samples[-2]
            dt = last.captured_at - previous.captured_at
            ax = float(np.clip((s.vx - previous.state.vx) / dt, -8.0, 3.0))
            ay = float(np.clip((s.vy - previous.state.vy) / dt, -2.0, 2.0))
        x, vx = longitudinal_motion(s.x, max(0.0, s.vx), ax, h)
        # Stop a decelerating lateral maneuver at zero rather than extrapolating
        # it into an unobserved reversal back out of the lane.
        if abs(s.vy) < 1e-9:
            ay = 0.0
        lateral_time = np.minimum(h, -s.vy / ay) if s.vy * ay < 0 else h
        y = s.y + s.vy * lateral_time + 0.5 * ay * lateral_time ** 2
        vy = s.vy + ay * lateral_time
        return np.column_stack((x, y, vx, vy))

    def uncertainty(self, horizons: np.ndarray) -> np.ndarray:
        """Positive x/y half-widths in metres; an uncalibrated MVP heuristic.

        The caller supplies observation age PLUS future planning offset. These
        bounds have no probability interpretation or safety guarantee.
        """
        h = np.asarray(horizons, dtype=float)
        if h.ndim != 1 or np.any(~np.isfinite(h)) or np.any(h < 0):
            raise ValueError("horizons must be finite nonnegative offsets")
        return np.column_stack((0.5 + 0.3 * h + 0.25 * h ** 2,
                                0.15 + 0.08 * h + 0.04 * h ** 2))
