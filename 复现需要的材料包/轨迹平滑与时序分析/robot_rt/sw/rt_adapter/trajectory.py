"""Model-independent, offline physical-unit trajectory references.

Every bound is supplied by the caller. No bound or output rate here is a device
capability claim. Fixed-time shaping never extends an observation's lifetime.
"""
from dataclasses import dataclass
import math
import numpy as np


class TrajectoryLimitError(ValueError):
    pass


@dataclass(frozen=True)
class MotionBounds:
    lower: np.ndarray
    upper: np.ndarray
    velocity: np.ndarray
    acceleration: np.ndarray
    jerk: np.ndarray
    dq_max: np.ndarray

    def __post_init__(self):
        arrays = []
        for name in self.__dataclass_fields__:
            value = np.asarray(getattr(self, name), dtype=float).copy()
            if value.ndim != 1 or not len(value) or not np.all(np.isfinite(value)):
                raise ValueError(f"{name}: finite nonempty vector required")
            value.setflags(write=False)
            object.__setattr__(self, name, value)
            arrays.append(value)
        if len({len(value) for value in arrays}) != 1:
            raise ValueError("bound dimensions differ")
        if np.any(self.lower >= self.upper):
            raise ValueError("lower must be below upper")
        for name in ("velocity", "acceleration", "jerk", "dq_max"):
            if np.any(getattr(self, name) <= 0):
                raise ValueError(f"{name} must be positive")

    @classmethod
    def from_dict(cls, value):
        return cls(**{key: value[key] for key in cls.__dataclass_fields__})

    def position(self, q):
        q = np.asarray(q, dtype=float)
        if q.shape != self.lower.shape or not np.all(np.isfinite(q)):
            raise TrajectoryLimitError("target dimension/nonfinite value")
        if np.any(q < self.lower) or np.any(q > self.upper):
            raise TrajectoryLimitError("target outside supplied physical-unit bounds")
        return q


def exact_tick_times(start: float, duration_s: float, rate_hz: float):
    """Each tick from its integer index; no accumulated floating-point phase."""
    if not all(math.isfinite(x) for x in (start, duration_s, rate_hz)) or duration_s <= 0 or rate_hz <= 0:
        raise ValueError("finite start, positive duration and explicit rate required")
    return start + np.arange(int(math.ceil(duration_s*rate_hz)), dtype=np.int64)/rate_hz


def real_roots(coefficients, duration):
    roots = np.polynomial.polynomial.polyroots(np.trim_zeros(coefficients, "b")) if np.any(coefficients) else []
    return [float(root.real) for root in roots if abs(root.imag) < 1e-8 and 0 < root.real < duration]


class QuinticSegment:
    """C2 polynomial bridge with analytical extrema, fixed absolute timing."""
    def __init__(self, t0, t1, q0, v0, a0, q1, v1, a1):
        self.t0, self.t1 = float(t0), float(t1)
        self.duration = self.t1-self.t0
        if not math.isfinite(self.duration) or self.duration <= 0:
            raise ValueError("quintic needs increasing finite endpoints")
        states = [np.asarray(item, dtype=float) for item in (q0, v0, a0, q1, v1, a1)]
        if any(item.ndim != 1 or item.shape != states[0].shape or not np.all(np.isfinite(item)) for item in states):
            raise ValueError("quintic states need matching finite vectors")
        q0, v0, a0, q1, v1, a1 = states
        T = self.duration
        self.coefficients = np.zeros((6, len(q0)))
        self.coefficients[:3] = q0, v0, a0/2
        matrix = np.array([[T**3, T**4, T**5], [3*T**2, 4*T**3, 5*T**4], [6*T, 12*T**2, 20*T**3]])
        residual = np.vstack((q1-q0-v0*T-a0*T*T/2, v1-v0-a0*T, a1-a0))
        self.coefficients[3:] = np.linalg.solve(matrix, residual)

    def evaluate(self, time, derivative=0):
        elapsed = np.asarray(time, dtype=float)-self.t0
        if not np.all(np.isfinite(elapsed)) or np.any(elapsed < -1e-10) or np.any(elapsed > self.duration+1e-10):
            raise ValueError("quintic evaluation outside its original time interval")
        coeff = np.polynomial.polynomial.polyder(self.coefficients, m=derivative, axis=0)
        return np.polynomial.polynomial.polyval(np.clip(elapsed, 0, self.duration), coeff).T

    def extrema(self, derivative=0):
        coeff = np.polynomial.polynomial.polyder(self.coefficients, m=derivative, axis=0)
        low, high = [], []
        for axis in range(coeff.shape[1]):
            critical = [0, self.duration, *real_roots(np.polynomial.polynomial.polyder(coeff[:, axis]), self.duration)]
            values = np.polynomial.polynomial.polyval(critical, coeff[:, axis])
            low.append(min(values)); high.append(max(values))
        return np.array(low), np.array(high)

    def validate(self, bounds):
        for order, name in ((0, "position"), (1, "velocity"), (2, "acceleration"), (3, "jerk")):
            low, high = self.extrema(order)
            lower, upper = (bounds.lower, bounds.upper) if not order else (-getattr(bounds, name), getattr(bounds, name))
            if np.any(low < lower-1e-9) or np.any(high > upper+1e-9):
                raise TrajectoryLimitError(f"quintic exceeds supplied {name} bound; original time is not stretched")
        return True


class C2Tracker:
    """Bounded piecewise constant-jerk reference, continuous q/v/a at tick edges.

    The target is re-read at the current absolute time. Tracking lag is reported,
    not hidden by replaying a stale prefix. If no feasible jerk interval exists,
    fault rather than claiming an instantaneous safe hold.
    """
    def __init__(self, q, bounds: MotionBounds, response_time_s: float, *, velocity=None, acceleration=None):
        self.bounds = bounds
        self.q = bounds.position(q).copy()
        self.v = np.zeros_like(self.q) if velocity is None else np.asarray(velocity, dtype=float).copy()
        self.a = np.zeros_like(self.q) if acceleration is None else np.asarray(acceleration, dtype=float).copy()
        if response_time_s <= 0 or not math.isfinite(response_time_s):
            raise ValueError("explicit positive response_time_s required")
        self.omega = 4/response_time_s
        if self.v.shape != self.q.shape or self.a.shape != self.q.shape or not np.all(np.isfinite(self.v)) or not np.all(np.isfinite(self.a)):
            raise ValueError("invalid initial derivatives")
        if np.any(abs(self.v) > bounds.velocity) or np.any(abs(self.a) > bounds.acceleration):
            raise TrajectoryLimitError("initial velocity/acceleration outside supplied bounds")

    def _interval(self, dt):
        b, q, v, a = self.bounds, self.q, self.v, self.a
        # t->0 constraints cannot be represented by finite endpoint bounds:
        # already on a boundary with outward lower-order derivatives is not viable.
        for axis in range(len(q)):
            at_high = q[axis] >= b.upper[axis]
            at_low = q[axis] <= b.lower[axis]
            if (at_high and (v[axis] > 0 or v[axis] == 0 and a[axis] > 0) or
                at_low and (v[axis] < 0 or v[axis] == 0 and a[axis] < 0) or
                v[axis] >= b.velocity[axis] and a[axis] > 0 or
                v[axis] <= -b.velocity[axis] and a[axis] < 0):
                raise TrajectoryLimitError("initial boundary state has outward derivative; no finite jerk can remain within bounds")
        low = np.maximum(-b.jerk, (-b.acceleration-a)/dt)
        high = np.minimum(b.jerk, (b.acceleration-a)/dt)
        # Continuous-time constraints reduce to scalar affine bounds on jerk.
        for axis in range(len(q)):
            for boundary, is_lower in ((-b.velocity[axis], True), (b.velocity[axis], False)):
                points = [dt]
                if abs(a[axis]) > 1e-15:
                    critical = 2*(boundary-v[axis])/a[axis]
                    if 0 < critical < dt:
                        points.append(critical)
                limits = [2*(boundary-v[axis]-a[axis]*t)/t**2 for t in points]
                if is_lower: low[axis] = max(low[axis], max(limits))
                else: high[axis] = min(high[axis], min(limits))
            for boundary, is_lower in ((max(b.lower[axis], q[axis]-b.dq_max[axis]), True), (min(b.upper[axis], q[axis]+b.dq_max[axis]), False)):
                points = [dt, *real_roots(np.array([-3*(boundary-q[axis]), 2*v[axis], .5*a[axis]]), dt)]
                limits = [6*(boundary-q[axis]-v[axis]*t-.5*a[axis]*t*t)/t**3 for t in points]
                if is_lower: low[axis] = max(low[axis], max(limits))
                else: high[axis] = min(high[axis], min(limits))
        if np.any(low > high+1e-9):
            raise TrajectoryLimitError("no bounded continuous-time reference exists from current state; external stop required")
        return low, high

    def advance(self, target, dt, *, braking=False):
        if not math.isfinite(dt) or dt <= 0:
            raise ValueError("positive finite tick duration required")
        target = self.bounds.position(target)
        low, high = self._interval(dt)
        desired_a = -2*self.omega*self.v if braking else self.omega**2*(target-self.q)-2*self.omega*self.v
        desired_a = np.clip(desired_a, -self.bounds.acceleration, self.bounds.acceleration)
        jerk = np.clip((desired_a-self.a)/dt, low, high)
        q0, v0, a0 = self.q.copy(), self.v.copy(), self.a.copy()
        self.q = q0+v0*dt+.5*a0*dt**2+jerk*dt**3/6
        self.v = v0+a0*dt+.5*jerk*dt**2
        self.a = a0+jerk*dt
        return {"q": self.q.copy(), "v": self.v.copy(), "a": self.a.copy(), "j": jerk.copy(),
                "target": target.copy(), "tracking_error": self.q-target}
