"""Pure offline references for a canonical seven-axis demonstration plan.

This is not a physical executor: no clocks are read, no device commands are sent,
and plan blockers are not removed. J0..J5 stay in degrees and J6 stays SDK raw.
The caller supplies elapsed time to ``clock_step``. A late caller must stop on
an exception; this module neither catches up, skips gripper phases, nor changes
the reviewed plan's deadlines.
"""
from __future__ import annotations

from dataclasses import dataclass, replace
import math

from planner import (
    ARM_ACCELERATION_DEG_S2, JOINTS, LIMITS, MAX_INPUT_GAP_S, MIN_SEGMENT_S,
    QUINTIC_MAX_FIRST_DERIVATIVE, QUINTIC_MAX_SECOND_DERIVATIVE, UNITS,
    PlanValidationError, canonical_plan_sha256,
)


MAX_RATE_HZ = 10.0
EPSILON_S = 1e-9


@dataclass(frozen=True)
class Reference:
    plan_time_s: float
    q: tuple[float, ...]
    segment_index: int
    is_waypoint: bool


@dataclass(frozen=True)
class ReferenceSequence:
    plan_sha256: str
    duration_s: float
    rate_hz: float
    points: tuple[Reference, ...]
    joint_order: tuple[str, ...] = tuple(JOINTS)
    joint_units: tuple[str, ...] = tuple(UNITS)
    physical_execution_ready: bool = False


@dataclass(frozen=True)
class ClockState:
    next_index: int = 0
    last_observed_s: float | None = None
    last_emitted_s: float | None = None


@dataclass(frozen=True)
class ClockStep:
    state: ClockState
    reference: Reference | None
    done: bool


def _finite(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def _fail(code, message):
    raise PlanValidationError(code, message)


def _vector(q):
    if not isinstance(q, list) or len(q) != 7 or not all(_finite(v) for v in q):
        _fail("INVALID_Q", "A reference requires seven finite SDK positions")
    if any(not low <= value <= high for value, (low, high) in zip(q, LIMITS)):
        _fail("JOINT_LIMIT", "Position exceeds the planner limits; clipping is forbidden")


def validate_plan(plan: dict, *, expected_sha256: str | None = None) -> None:
    """Validate in-memory content; hash integrity is not approval or source authentication."""
    if not isinstance(plan, dict) or plan.get("schema") != "d1-offline-seven-axis-demo-plan-v1":
        _fail("PLAN_SCHEMA", "Expected the canonical offline seven-axis plan")
    try:
        digest = canonical_plan_sha256(plan)
    except (ValueError, TypeError, OverflowError) as exc:
        raise PlanValidationError("INVALID_PLAN_JSON", "Plan is not finite canonical JSON") from exc
    if plan.get("plan_sha256") != digest or (expected_sha256 is not None and digest != expected_sha256):
        _fail("PLAN_HASH", "Canonical plan hash does not match the reviewed hash")
    if plan.get("joint_order") != JOINTS or plan.get("joint_units") != UNITS:
        _fail("UNITS_MISMATCH", "Expected J0..J5 degrees and J6 uncalibrated SDK raw")
    bounds = plan.get("limits")
    if (not isinstance(bounds, list) or len(bounds) != 7
            or any(not isinstance(pair, list) or len(pair) != 2 or not all(_finite(v) for v in pair)
                   for pair in bounds) or bounds != LIMITS):
        _fail("LIMITS_MISMATCH", "Plan limits must equal the planner's fixed seven-axis limits")
    interpolation = plan.get("interpolation", {})
    if (not isinstance(interpolation, dict)
            or interpolation.get("type") != "per_segment_synchronized_quintic"
            or interpolation.get("all_seven_axes_share_u") is not True):
        _fail("INTERPOLATION_MISMATCH", "All seven axes must share the canonical quintic time")
    gripper = plan.get("gripper_timing", {})
    if (not isinstance(gripper, dict) or gripper.get("position_unit") != UNITS[6]
            or gripper.get("velocity_unit") != "sdk_raw/s"
            or gripper.get("acceleration_unit") != "sdk_raw/s^2"):
        _fail("UNITS_MISMATCH", "Gripper timing must retain SDK-raw units")
    speed, acceleration, duration = (plan.get(key) for key in ("speed_deg_s", "acceleration_deg_s2", "duration"))
    if not _finite(speed) or speed <= 0 or acceleration != ARM_ACCELERATION_DEG_S2 or not _finite(acceleration):
        _fail("TIMING_PARAMETERS", "Positive arm speed and exactly 1 degree/s² are required")
    if not _finite(duration) or duration <= 0:
        _fail("PLAN_DURATION", "Plan duration must be finite and positive")
    points, segments = plan.get("original_points"), plan.get("segments")
    if (not isinstance(points, list) or len(points) < 2 or not isinstance(segments, list)
            or len(segments) != len(points)-1 or plan.get("point_count") != len(points)
            or plan.get("segment_count") != len(segments)):
        _fail("PLAN_STRUCTURE", "Every original waypoint interval must remain in the plan")
    for point in points:
        if not isinstance(point, dict):
            _fail("PLAN_STRUCTURE", "Invalid original waypoint")
        _vector(point.get("q"))
        if not all(_finite(point.get(key)) for key in ("elapsed_s", "received_monotonic_s")):
            _fail("NONFINITE_TIME", "Original waypoint timestamps must be finite")
    if plan.get("start_q") != points[0]["q"] or plan.get("end_q") != points[-1]["q"]:
        _fail("ENDPOINT_MISMATCH", "Plan endpoints must match the original seven-axis positions")
    end = 0.0
    for index, segment in enumerate(segments):
        if not isinstance(segment, dict) or segment.get("index") != index:
            _fail("PLAN_STRUCTURE", "Segment order must be unchanged")
        first, second = points[index:index+2]
        if segment.get("q0") != first["q"] or segment.get("q1") != second["q"]:
            _fail("WAYPOINT_MISMATCH", "A segment changed or omitted an original seven-axis waypoint")
        t0, t1, length = (segment.get(key) for key in ("plan_t0", "plan_t1", "T"))
        if (not all(_finite(v) for v in (t0, t1, length)) or t0 != end or t1 <= t0
                or length < MIN_SEGMENT_S or not math.isclose(t1-t0, length, rel_tol=1e-12, abs_tol=EPSILON_S)):
            _fail("SEGMENT_DURATION", "Segment times must be contiguous and preserve their duration")
        source_dt = second["received_monotonic_s"] - first["received_monotonic_s"]
        elapsed_dt = second["elapsed_s"] - first["elapsed_s"]
        original_dt = segment.get("original_dt")
        if (not _finite(original_dt) or source_dt <= 0 or elapsed_dt <= 0 or source_dt > MAX_INPUT_GAP_S
                or abs(source_dt-elapsed_dt) > .001
                or not math.isclose(original_dt, source_dt, rel_tol=1e-12, abs_tol=EPSILON_S)
                or length+EPSILON_S < source_dt):
            _fail("SOURCE_TIMING", "Original sampling intervals or segment timing are inconsistent")
        arm_delta = max(abs(b-a) for a, b in zip(first["q"][:6], second["q"][:6]))
        if (QUINTIC_MAX_FIRST_DERIVATIVE*arm_delta/length > speed*(1+1e-12)
                or QUINTIC_MAX_SECOND_DERIVATIVE*(arm_delta/length)/length > acceleration*(1+1e-12)):
            _fail("ARM_TIMING_LIMIT", "Quintic arm speed or acceleration exceeds its declared bound")
        end = t1
    source_duration = points[-1]["received_monotonic_s"] - points[0]["received_monotonic_s"]
    if duration != end or not _finite(plan.get("source_duration")) or not math.isclose(
            plan["source_duration"], source_duration, rel_tol=1e-12, abs_tol=EPSILON_S):
        _fail("PLAN_DURATION", "Plan/source duration does not match the complete segment sequence")


def build_reference_sequence(plan: dict, *, rate_hz: float = MAX_RATE_HZ,
                             expected_sha256: str | None = None,
                             max_references: int = 250_000) -> ReferenceSequence:
    """Keep every original waypoint at its exact plan time, with spacing >= 1/rate.

    Each segment is divided into floor(T * rate) equal intervals. This is a
    nonuniform, at-most-10-Hz clock, not resampling to a different duration.
    Too-low rates fail instead of omitting a short gripper open/close phase.
    """
    validate_plan(plan, expected_sha256=expected_sha256)
    if not _finite(rate_hz) or not 0 < rate_hz <= MAX_RATE_HZ:
        _fail("REFERENCE_RATE", "Reference rate must be positive and at most 10 Hz")
    if not isinstance(max_references, int) or isinstance(max_references, bool) or max_references < 2:
        _fail("REFERENCE_BUDGET", "Reference allocation budget must be at least two")
    counts = []
    for segment in plan["segments"]:
        ticks = (segment["plan_t1"]-segment["plan_t0"])*rate_hz
        if not math.isfinite(ticks) or ticks > max_references:
            _fail("REFERENCE_BUDGET", "Segment exceeds the explicit reference allocation budget")
        count = math.floor(ticks + 1e-10)
        if count < 1:
            _fail("WAYPOINT_RATE_CONFLICT", "Requested rate cannot preserve every segment endpoint")
        counts.append(count)
    if 1+sum(counts) > max_references:
        _fail("REFERENCE_BUDGET", "Plan exceeds the explicit reference allocation budget")
    references = [Reference(0.0, tuple(plan["start_q"]), 0, True)]
    for segment, count in zip(plan["segments"], counts):
        for step in range(1, count+1):
            endpoint = step == count
            stamp = segment["plan_t1"] if endpoint else segment["plan_t0"]+(segment["plan_t1"]-segment["plan_t0"])*step/count
            if endpoint:
                q = tuple(segment["q1"])  # Exact recorded endpoint, including J6.
            else:
                u = (stamp-segment["plan_t0"])/segment["T"]
                # Symmetry avoids cancellation close to 1 without clipping values.
                v = u if u <= .5 else 1-u
                blend = v*v*v*(10+v*(-15+6*v))
                if u > .5:
                    blend = 1-blend
                q = tuple(a+(b-a)*blend for a, b in zip(segment["q0"], segment["q1"]))
            if stamp-references[-1].plan_time_s < 1/rate_hz-EPSILON_S:
                _fail("REFERENCE_RATE", "Numerical timing cannot maintain the maximum reference rate")
            _vector(list(q))
            references.append(Reference(stamp, q, segment["index"], endpoint))
    return ReferenceSequence(plan["plan_sha256"], plan["duration"], float(rate_hz), tuple(references))


def clock_step(sequence: ReferenceSequence, state: ClockState, elapsed_s: float, *,
               allowed_lateness_s: float = .02) -> ClockStep:
    """Return zero/one next reference; an overrun raises instead of replaying backlog.

    Times are caller-supplied elapsed monotonic seconds from the original plan
    epoch. Deadlines never move. Actual emissions are also capped at rate_hz.
    The caller must observe the supplied nonuniform deadlines (or poll more
    often); a fixed 100 ms polling loop can miss their 20 ms lateness budget.
    An exception is an offline scheduling failure, not a hardware stop command.
    """
    if not _finite(elapsed_s) or elapsed_s < 0 or not _finite(allowed_lateness_s) or allowed_lateness_s < 0:
        _fail("CLOCK_TIME", "Clock values must be finite and nonnegative")
    if (not isinstance(state.next_index, int) or isinstance(state.next_index, bool)
            or not 0 <= state.next_index <= len(sequence.points)
            or (state.next_index == 0) != (state.last_emitted_s is None)):
        _fail("CLOCK_STATE", "Clock progress and previous emission disagree")
    for stamp in (state.last_observed_s, state.last_emitted_s):
        if stamp is not None and (not _finite(stamp) or stamp < 0 or elapsed_s < stamp):
            _fail("CLOCK_REVERSED", "Elapsed clock must not run backwards")
    observed = replace(state, last_observed_s=elapsed_s)
    if state.next_index == len(sequence.points):
        return ClockStep(observed, None, True)
    reference = sequence.points[state.next_index]
    if elapsed_s < reference.plan_time_s:
        return ClockStep(observed, None, False)
    if elapsed_s-reference.plan_time_s > allowed_lateness_s+EPSILON_S:
        _fail("CLOCK_OVERRUN", "Reference deadline missed: no skipping, catch-up burst, or automatic retiming")
    if state.last_emitted_s is not None and elapsed_s-state.last_emitted_s < 1/sequence.rate_hz-EPSILON_S:
        return ClockStep(observed, None, False)
    advanced = ClockState(state.next_index+1, elapsed_s, elapsed_s)
    return ClockStep(advanced, reference, advanced.next_index == len(sequence.points))
