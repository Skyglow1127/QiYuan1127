"""Choose a smaller point on the current approved reference interval."""
import math

import numpy as np


def _six_finite(value, name):
    result = np.asarray(value, dtype=float)
    if result.shape != (6,) or not np.all(np.isfinite(result)):
        raise ValueError(f'{name} must contain six finite joint positions')
    return result


def admissible_reference(previous, target, actual, margin_deg=2.5,
                         minimum_step_deg=.1, rounding_margin_deg=.001):
    """Return ``(fraction, point)`` on previous -> target, or no usable step.

    Every joint uses the same fraction, so this only subdivides the approved
    line segment. The actual feedback defines the admission box; it is never
    used as the origin of a new path. Partial references retain a small margin
    for the transport's three-decimal wire representation.
    """
    previous = _six_finite(previous, 'previous')
    target = _six_finite(target, 'target')
    actual = _six_finite(actual, 'actual')
    values = (margin_deg, minimum_step_deg, rounding_margin_deg)
    if (any(isinstance(value, (bool, np.bool_)) or not math.isfinite(value)
            for value in values)
            or margin_deg <= 0 or minimum_step_deg <= 0
            or not 0 <= rounding_margin_deg < margin_deg):
        raise ValueError('Reference margins and minimum step must be finite and valid')

    if np.max(np.abs(target - actual)) <= margin_deg + 1e-5:
        return 1., target.copy()

    delta = target - previous
    inner_margin = margin_deg - rounding_margin_deg
    lower, upper = 0., 1.
    for start, change, measured in zip(previous, delta, actual):
        if change == 0:
            if abs(start - measured) > inner_margin:
                return None
            continue
        first = (measured - inner_margin - start) / change
        second = (measured + inner_margin - start) / change
        lower = max(lower, min(first, second))
        upper = min(upper, max(first, second))
        if upper < lower:
            return None

    fraction = upper
    if not 0 < fraction < 1:
        return None
    candidate = previous + fraction * delta
    if np.max(np.abs(candidate - previous)) < minimum_step_deg:
        return None
    return float(fraction), candidate
