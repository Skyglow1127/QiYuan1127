"""Causal optional blending. Default mode is no ensembling."""
import math
import numpy as np


class Ensembler:
    def __init__(self, mode="none", *, alpha=None):
        if mode not in ("none", "first_order"):
            raise ValueError("mode must be none or first_order")
        if mode == "first_order" and (alpha is None or not math.isfinite(alpha) or not 0 < alpha <= 1):
            raise ValueError("first_order requires explicit 0<alpha<=1")
        self.mode, self.alpha, self.previous = mode, alpha, None
        self.previous_time = -float("inf")

    def apply(self, target, *, scheduled_at, now):
        target = np.asarray(target, dtype=float)
        if not math.isfinite(scheduled_at) or not math.isfinite(now) or not np.all(np.isfinite(target)) or scheduled_at > now+1e-9 or now < self.previous_time:
            raise ValueError("blend needs finite present/past input and monotonic evaluation")
        if self.previous is not None and self.previous.shape != target.shape:
            raise ValueError("blend dimensions differ")
        result = target.copy() if self.mode == "none" or self.previous is None else self.alpha*target+(1-self.alpha)*self.previous
        self.previous, self.previous_time = result.copy(), now
        return result
