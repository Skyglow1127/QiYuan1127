"""Hardware-independent validation of absolute PiPER policy targets.

This is a software gate, not a collision checker or an emergency stop. No
physical limits are guessed from the demonstrations. A robot-side owner lease,
expiry watchdog and verified stop procedure are required before hardware use.
"""

from dataclasses import dataclass, fields
import fcntl
import math
from pathlib import Path
from typing import Mapping, Sequence

ACTION_KEYS = tuple(f"joint_{i}.deg" for i in range(1, 7)) + ("gripper.m",)


class GuardError(RuntimeError):
    pass


def vector(values: Mapping[str, float] | Sequence[float]) -> tuple[float, ...]:
    if isinstance(values, Mapping):
        if set(values) != set(ACTION_KEYS):
            raise GuardError("Expected exactly six joint_*.deg fields and gripper.m")
        values = tuple(values[k] for k in ACTION_KEYS)
    try:
        result = tuple(float(v) for v in values)
    except (TypeError, ValueError, OverflowError) as error:
        raise GuardError("Invalid action/state values") from error
    if len(result) != 7 or not all(math.isfinite(v) for v in result):
        raise GuardError("Expected seven finite action/state values")
    return result


@dataclass(frozen=True)
class GuardLimits:
    # Six degree bounds followed by signed width bounds in metres. These must
    # come from the reviewed hardware/fixture, never the training min/max alone.
    bounds: tuple[tuple[float, float], ...]
    max_step: tuple[float, ...]
    max_target_speed: tuple[float, ...]  # deg/s for joints, m/s for width
    max_tracking_error: tuple[float, ...]
    max_observation_age_s: float
    max_command_age_s: float
    max_command_gap_s: float

    def __post_init__(self):
        if len(self.bounds) != 7:
            raise ValueError("Seven independently reviewed bounds required")
        for lo, hi in self.bounds:
            if not math.isfinite(lo) or not math.isfinite(hi) or lo >= hi:
                raise ValueError("Invalid bounds")
        for values in (self.max_step, self.max_target_speed, self.max_tracking_error):
            if len(values) != 7 or not all(math.isfinite(x) and x > 0 for x in values):
                raise ValueError("Seven finite positive limits required")
        for value in (self.max_observation_age_s, self.max_command_age_s, self.max_command_gap_s):
            if not math.isfinite(value) or value <= 0:
                raise ValueError("Finite positive time limits required")


@dataclass(frozen=True)
class Readiness:
    # Evidence references are deliberate review gates, not automatic proof.
    # Empty defaults prevent an accidental arm with an unfinished review.
    leader_isolation: str = ""
    firmware_and_j_mode: str = ""
    hardware_and_fixture_limits: str = ""
    signed_gripper_mapping: str = ""
    verified_stop_strategy: str = ""
    robot_side_lease_and_watchdog: str = ""
    approved_goal_path_parameters: str = ""

    def require_complete(self):
        missing = [f.name for f in fields(self) if not getattr(self, f.name).strip()]
        if missing:
            raise GuardError("Cannot arm; review incomplete: " + ", ".join(missing))


@dataclass(frozen=True)
class CheckedCommand:
    sequence: int
    target: tuple[float, ...]
    created_at: float
    expires_at: float


class ActionGuard:
    def __init__(self, limits: GuardLimits):
        self.limits = limits
        self.armed = False
        self.last_target = None
        self.last_command_at = None
        self.last_clock = None
        self.sequence = 0
        self.pending = None
        self.reason = "default disarmed"

    def disarm(self, reason: str):
        # This ONLY closes the software gate. A robot may retain its last
        # absolute target; disarming Python is not a verified physical stop.
        self.armed, self.pending, self.reason = False, None, reason

    def _fail(self, reason):
        self.disarm(reason)
        raise GuardError(reason)

    def _clock(self, now):
        if not math.isfinite(now) or (self.last_clock is not None and now < self.last_clock):
            self._fail("Non-finite or backwards monotonic clock")
        self.last_clock = now

    def _fresh(self, sampled_at, now, max_age):
        if not math.isfinite(sampled_at) or not 0 <= now - sampled_at <= max_age:
            self._fail("Stale, future or invalid timestamp")

    def _bounded(self, values):
        try:
            result = vector(values)
        except GuardError as error:
            self._fail(str(error))
        for key, value, (lo, hi) in zip(ACTION_KEYS, result, self.limits.bounds, strict=True):
            if not lo <= value <= hi:
                self._fail(f"{key}={value} outside [{lo}, {hi}]; no clipping performed")
        return result

    def arm(self, current_state, *, sampled_at: float, now: float, readiness: Readiness):
        self.disarm("rearming requires a fresh current-state anchor")
        readiness.require_complete()
        self._clock(now)
        self._fresh(sampled_at, now, self.limits.max_observation_age_s)
        self.last_target = self._bounded(current_state)
        self.last_command_at = now
        self.armed = True
        self.reason = "armed from current feedback; no motion emitted"

    def prepare(self, action, *, observed_state, sampled_at: float,
                proposal_at: float, now: float) -> CheckedCommand:
        if not self.armed:
            raise GuardError("Disarmed: " + self.reason)
        self._clock(now)
        if self.pending is not None:
            self._fail("Uncommitted command; transport outcome is unknown")
        self._fresh(self.last_command_at, now, self.limits.max_command_gap_s)
        self._fresh(sampled_at, now, self.limits.max_observation_age_s)
        self._fresh(proposal_at, now, self.limits.max_command_age_s)
        state, target = self._bounded(observed_state), self._bounded(action)
        elapsed = now - self.last_command_at
        for i, key in enumerate(ACTION_KEYS):
            if abs(target[i] - self.last_target[i]) > self.limits.max_step[i]:
                self._fail(f"{key}: target step too large; no clipping performed")
            if abs(target[i] - self.last_target[i]) > self.limits.max_target_speed[i] * elapsed:
                self._fail(f"{key}: target rate too large; no clipping performed")
            if abs(target[i] - state[i]) > self.limits.max_tracking_error[i]:
                self._fail(f"{key}: target too far from current feedback")
        self.sequence += 1
        self.pending = CheckedCommand(self.sequence, target, now, min(
            sampled_at + self.limits.max_observation_age_s,
            proposal_at + self.limits.max_command_age_s))
        return self.pending

    def require_sendable(self, command: CheckedCommand, *, now: float):
        self._clock(now)
        if not self.armed or command is not self.pending or now > command.expires_at:
            self._fail("Disarmed, foreign or expired command")

    def commit(self, command: CheckedCommand, *, now: float):
        # Call only after the receiving owner accepted the whole target and
        # its deadline. A partial CAN batch must fault rather than commit.
        self.require_sendable(command, now=now)
        self.last_target, self.last_command_at = command.target, now
        self.pending = None


class LocalOwner:
    """Advisory single-owner lock on this computer; NOT a CAN-bus lease."""
    def __init__(self, path: str | Path):
        self.path, self.file = Path(path), None

    def acquire(self):
        if self.file is not None:
            raise GuardError("Owner lock already held")
        handle = self.path.open("a")
        try:
            fcntl.flock(handle, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except OSError:
            handle.close()
            raise GuardError("Another local executor owns the lock") from None
        self.file = handle

    def release(self):
        if self.file is not None:
            fcntl.flock(self.file, fcntl.LOCK_UN)
            self.file.close()
            self.file = None
