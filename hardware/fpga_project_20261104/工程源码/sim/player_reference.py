#!/usr/bin/env python3
"""Integer golden model for one fixed-time constant-jerk player segment.

This is a DESIGN-ONLY mathematical reference, with no hardware/transport imports.
q/v/a/j are signed int64 Q32.32 in each axis' physical unit and derivatives.
Each endpoint uses exact rational arithmetic and ONE nearest/ties-even Q32
reduction; output then uses Q32 -> Q16 nearest/ties-even plus inward endpoint
projection. Bernstein envelopes round OUTWARD, never by a float tolerance.

Example (invented numeric values, not robot limits or a selected f_exec):
  python3 sim/player_reference.py --design-example --dt-cycles 727273 --steps 4

Numerical continuity: the next record must start at the exact previous INTEGER
q/v/a. Each such state differs from that segment's unrounded polynomial endpoint
by at most 0.5 Q32 LSB. This is NOT a proof of mathematical C2 continuity of the
quantized wire positions, physical jerk, or equivalence with B's float tracker.
"""
from __future__ import annotations

import argparse
from dataclasses import dataclass
from fractions import Fraction
import json
from typing import Sequence

CLOCK_HZ = 100_000_000
MAX_CLOCK_HZ = (1 << 31)-1  # Matches a positive SystemVerilog integer parameter.
FRAC_BITS = 32
SCALE = 1 << FRAC_BITS
Q16_SCALE = 1 << 16
INT64_MIN, INT64_MAX = -(1 << 63), (1 << 63)-1
UINT32_MAX = (1 << 32)-1


class PlayerFault(ValueError):
    """Reject a segment without changing the immutable current state."""


def checked_clock_hz(clock_hz: int) -> int:
    if not isinstance(clock_hz, int) or isinstance(clock_hz, bool) or not 1 <= clock_hz <= MAX_CLOCK_HZ:
        raise PlayerFault("clock_hz must be a positive signed integer (1..2147483647)")
    return clock_hz


def checked_i64(value: int) -> int:
    if not isinstance(value, int) or isinstance(value, bool) or not INT64_MIN <= value <= INT64_MAX:
        raise PlayerFault("Q32.32 value is not signed int64")
    return value


def fraction(value) -> Fraction:
    if isinstance(value, bool):
        raise ValueError("boolean is not a physical numeric value")
    return value if isinstance(value, Fraction) else Fraction(str(value))


def round_even(value: Fraction) -> int:
    """Exact symmetric nearest/ties-even, including negative halves."""
    value = Fraction(value)
    quotient, remainder = divmod(abs(value.numerator), value.denominator)
    if 2*remainder > value.denominator or 2*remainder == value.denominator and quotient % 2:
        quotient += 1
    return -quotient if value < 0 else quotient


def floor(value: Fraction) -> int:
    return value.numerator // value.denominator


def ceil(value: Fraction) -> int:
    return -((-value.numerator) // value.denominator)


def encode_q32(value) -> int:
    return checked_i64(round_even(fraction(value)*SCALE))


def decode_q32(value: int) -> Fraction:
    return Fraction(checked_i64(value), SCALE)


@dataclass(frozen=True)
class State:
    q: tuple[int, ...]
    v: tuple[int, ...]
    a: tuple[int, ...]

    def __post_init__(self):
        for name in ("q", "v", "a"):
            values = tuple(checked_i64(value) for value in getattr(self, name))
            object.__setattr__(self, name, values)
        if not self.q or len({len(self.q), len(self.v), len(self.a)}) != 1:
            raise PlayerFault("q/v/a need matching nonempty vectors")

    @classmethod
    def from_physical(cls, q, v, a):
        return cls(*(tuple(encode_q32(value) for value in vector) for vector in (q, v, a)))

    def as_dict(self):
        return {name: list(getattr(self, name)) for name in ("q", "v", "a")}


@dataclass(frozen=True)
class Bounds:
    lower: tuple[int, ...]
    upper: tuple[int, ...]
    velocity: tuple[int, ...]
    acceleration: tuple[int, ...]
    jerk: tuple[int, ...]
    dq_max: tuple[int, ...]

    def __post_init__(self):
        lengths = set()
        for name in self.__dataclass_fields__:
            values = tuple(checked_i64(value) for value in getattr(self, name))
            object.__setattr__(self, name, values)
            lengths.add(len(values))
        if len(lengths) != 1 or not self.lower:
            raise PlayerFault("bounds need matching nonempty vectors")
        for axis in range(len(self.lower)):
            if self.lower[axis] >= self.upper[axis]:
                raise PlayerFault("empty/inverted position bounds")
            if any(getattr(self, name)[axis] <= 0 for name in ("velocity", "acceleration", "jerk", "dq_max")):
                raise PlayerFault("derivative and DQ_MAX bounds must be positive")

    @classmethod
    def from_physical(cls, *, lower, upper, velocity, acceleration, jerk, dq_max):
        # Bounds are inward: a quantized threshold must not enlarge a physical limit.
        vectors = [tuple(checked_i64(ceil(fraction(value)*SCALE)) for value in lower)]
        for vector in (upper, velocity, acceleration, jerk, dq_max):
            vectors.append(tuple(checked_i64(floor(fraction(value)*SCALE)) for value in vector))
        return cls(*vectors)


def to_q16(value_q32: int, lower_q32: int, upper_q32: int) -> int:
    """Preserve the physical-unit Q16 contract and its inward representable domain."""
    value_q32, lower_q32, upper_q32 = map(checked_i64, (value_q32, lower_q32, upper_q32))
    if not lower_q32 <= value_q32 <= upper_q32:
        raise PlayerFault("Q16 input outside supplied bounds")
    lower = ceil(Fraction(lower_q32, Q16_SCALE))
    upper = floor(Fraction(upper_q32, Q16_SCALE))
    if lower > upper or lower < -(1 << 31) or upper > (1 << 31)-1:
        raise PlayerFault("empty/overflowing Q16.16 representable domain")
    return min(upper, max(lower, round_even(Fraction(value_q32, Q16_SCALE))))


def encode_can(q16: Sequence[int], bounds: Bounds,
               previous_wire: Sequence[int] | None = None) -> tuple[int, ...]:
    """Encode seven Q16 targets into six millidegrees and gripper micrometres.

    RNE is followed by inward projection into the Q32 physical bounds' integer
    wire domain. Projection may correct at most one wire integer step; actual
    out-of-bounds Q16 input is rejected. The final decoded wire delta must pass
    DQ_MAX, independently of the earlier Q16 delta check.

    previous_wire=None is ONLY for encoding an explicitly reviewed initial seed;
    it performs no adjacency proof. Every normal output must use the last seeded
    or committed wire tuple. This numerical function has no hardware connection.
    """
    q16 = tuple(q16)
    if len(q16) != 7 or len(bounds.lower) != 7:
        raise PlayerFault("CAN encoding requires six degree axes and one metre axis")
    if any(not isinstance(value, int) or isinstance(value, bool) or not -(1 << 31) <= value <= (1 << 31)-1 for value in q16):
        raise PlayerFault("CAN source must be signed int32 Q16.16")
    previous = None if previous_wire is None else tuple(previous_wire)
    if previous is not None and (len(previous) != 7 or any(not isinstance(value, int) or isinstance(value, bool) or not -(1 << 31) <= value <= (1 << 31)-1 for value in previous)):
        raise PlayerFault("previous CAN wire tuple must contain seven signed int32 values")
    wire = []
    for axis, (value, scale) in enumerate(zip(q16, (1000,)*6+(1_000_000,))):
        lower, upper = bounds.lower[axis], bounds.upper[axis]
        if not lower <= value*Q16_SCALE <= upper:
            raise PlayerFault(f"axis {axis}: CAN source Q16 outside physical bounds")
        minimum = ceil(Fraction(lower*scale, SCALE))
        maximum = floor(Fraction(upper*scale, SCALE))
        if minimum > maximum or minimum < -(1 << 31) or maximum > (1 << 31)-1:
            raise PlayerFault(f"axis {axis}: empty/overflowing CAN integer domain")
        nearest = round_even(Fraction(value*scale, Q16_SCALE))
        encoded = min(maximum, max(minimum, nearest))
        if abs(encoded-nearest) > 1:
            raise PlayerFault(f"axis {axis}: CAN projection exceeds one quantization step")
        if previous is not None:
            old = previous[axis]
            if not lower*scale <= old*SCALE <= upper*scale:
                raise PlayerFault(f"axis {axis}: previous CAN seed/output outside physical bounds")
            if abs(encoded-old)*SCALE > bounds.dq_max[axis]*scale:
                raise PlayerFault(f"axis {axis}: decoded CAN adjacent DQ_MAX bound")
        wire.append(encoded)
    return tuple(wire)


def _enclose(coefficients: Sequence[Fraction]):
    # Each endpoint of this enclosure is <1 Q32 LSB from its exact control point.
    return tuple((floor(value), ceil(value)) for value in coefficients)


@dataclass(frozen=True)
class Step:
    next_state: State
    q16: tuple[int, ...]
    exact_endpoint_q32: dict[str, tuple[Fraction, ...]]
    certificate: dict


def advance(current: State, jerk: Sequence[int], dt_cycles: int, bounds: Bounds,
            *, expected_start: State | None = None, clock_hz: int = CLOCK_HZ) -> Step:
    """Integrate then certify the whole segment before exposing a normal output.

    Checking Bernstein control-point hulls is conservative; a valid polynomial
    whose hull crosses a limit is rejected. There is no clamp of internal state,
    guessed brake, expiry extension, or instantaneous replacement with HOLD.
    Sequence/clock/deadline/CRC checks belong to the surrounding player/FSM.
    """
    clock_hz = checked_clock_hz(clock_hz)
    if not isinstance(dt_cycles, int) or isinstance(dt_cycles, bool) or not 1 <= dt_cycles <= UINT32_MAX:
        raise PlayerFault("actual dt_cycles must be a positive uint32")
    if expected_start is not None and expected_start != current:
        raise PlayerFault("segment start q/v/a does not match the committed integer state")
    jerks = tuple(checked_i64(value) for value in jerk)
    axes = len(current.q)
    if len(jerks) != axes or len(bounds.lower) != axes:
        raise PlayerFault("axis dimensions differ")
    h = Fraction(dt_cycles, clock_hz)
    exact = {name: [] for name in ("q", "v", "a")}
    certificates = []
    for axis, (q, v, a, j) in enumerate(zip(current.q, current.v, current.a, jerks)):
        if abs(j) > bounds.jerk[axis]:
            raise PlayerFault(f"axis {axis}: jerk bound")
        q1 = q+v*h+a*h*h/2+j*h*h*h/6
        v1 = v+a*h+j*h*h/2
        a1 = a+j*h
        q_control = tuple(map(Fraction, (q, q+v*h/3, q+2*v*h/3+a*h*h/6, q1)))
        v_control = tuple(map(Fraction, (v, v+a*h/2, v1)))
        a_control = tuple(map(Fraction, (a, a1)))
        enclosures = {"q": _enclose(q_control), "v": _enclose(v_control), "a": _enclose(a_control)}
        lower = max(bounds.lower[axis], q-bounds.dq_max[axis])
        upper = min(bounds.upper[axis], q+bounds.dq_max[axis])
        for name, minimum, maximum in (("q", lower, upper), ("v", -bounds.velocity[axis], bounds.velocity[axis]), ("a", -bounds.acceleration[axis], bounds.acceleration[axis])):
            if any(low < minimum or high > maximum for low, high in enclosures[name]):
                raise PlayerFault(f"axis {axis}: continuous {name} Bernstein envelope / DQ_MAX bound")
        for name, value in (("q", q1), ("v", v1), ("a", a1)):
            exact[name].append(value)
        certificates.append({"axis": axis, "q_enclosure": enclosures["q"], "v_enclosure": enclosures["v"], "a_enclosure": enclosures["a"],
            "q_allowed": [lower, upper], "max_enclosure_rounding_error_lsb32": "strictly less than 1; exact at integer control points"})
    next_state = State(*(tuple(checked_i64(round_even(value)) for value in exact[name]) for name in ("q", "v", "a")))
    wire = tuple(to_q16(value, lower, upper) for value, lower, upper in zip(next_state.q, bounds.lower, bounds.upper))
    previous_wire = tuple(to_q16(value, lower, upper) for value, lower, upper in zip(current.q, bounds.lower, bounds.upper))
    if any(abs(new-old)*Q16_SCALE > maximum for new, old, maximum in zip(wire, previous_wire, bounds.dq_max)):
        raise PlayerFault("quantized Q16 output exceeds supplied adjacent DQ_MAX")
    return Step(next_state, wire, {name: tuple(values) for name, values in exact.items()},
        {"dt_cycles": dt_cycles, "clock_hz": clock_hz, "gate": "conservative outward-rounded exact cubic Bernstein hulls",
         "axes": certificates, "endpoint_rounding_error_lsb32_le": "1/2",
         "wire_adjacent_dq_gate": "abs(next_q16-current_q16)*65536 <= dq_max_q32; wire velocity/jerk are not inferred from internal bounds",
         "continuity": "next integer state must match exactly; unrounded polynomial seam error <= 1/2 Q32 LSB per state component; physical/wire C2 is not claimed"})


def width_requirements(max_dt_cycles=UINT32_MAX, *, clock_hz: int = CLOCK_HZ):
    """Conservative signed raw-numerator widths, BEFORE division, for int64 inputs."""
    clock_hz = checked_clock_hz(clock_hz)
    if not isinstance(max_dt_cycles, int) or isinstance(max_dt_cycles, bool) or not 1 <= max_dt_cycles <= UINT32_MAX:
        raise ValueError("max_dt_cycles must be positive uint32")
    c, n, magnitude = clock_hz, max_dt_cycles, 1 << 63
    values = {"q_numerator": magnitude*(6*c**3+6*n*c*c+3*n*n*c+n**3),
              "v_numerator": magnitude*(2*c*c+2*n*c+n*n), "a_numerator": magnitude*(c+n)}
    return {"clock_hz": c, "max_dt_cycles": n,
        "signed_bits": {name: value.bit_length()+1 for name, value in values.items()},
        "suggested_uniform_signed_intermediate_bits": 192,
        "rule": "full-width products/sums and signed comparisons; no wrap/saturation; range-check before final int64 reduction",
        "timing": f"112 cycles at {c} Hz is a design budget, not a proven timing result; width analysis only, requiring a pipelined/divider architecture and later synthesis/timing evidence"}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--design-example", action="store_true", required=True, help="Acknowledge invented offline demo values, not hardware limits")
    parser.add_argument("--dt-cycles", type=int, required=True, help="Explicit ACTUAL interval in selected-clock cycles; no execution-frequency default")
    parser.add_argument("--clock-hz", type=int, default=CLOCK_HZ, help="Positive signed-integer clock metadata (default: 100000000); not physical f_exec")
    parser.add_argument("--steps", type=int, default=4)
    args = parser.parse_args(argv)
    try:
        if args.steps < 1: raise ValueError("steps must be positive")
        bounds = Bounds.from_physical(lower=["-10"], upper=["10"], velocity=["10"], acceleration=["10"], jerk=["10"], dq_max=["1"])
        state = State.from_physical([0], [0], [0])
        records = []
        for index in range(args.steps):
            result = advance(state, [encode_q32("1")], args.dt_cycles, bounds, expected_start=state, clock_hz=args.clock_hz)
            records.append({"index": index, "start_q32": state.as_dict(), "next_q32": result.next_state.as_dict(), "q16": list(result.q16), "certificate": result.certificate})
            state = result.next_state
        print(json.dumps({"provenance": "design_only", "physical_f_exec_hz": None, "width_requirements": width_requirements(clock_hz=args.clock_hz), "segments": records}, ensure_ascii=False, indent=2))
        return 0
    except (PlayerFault, ValueError, TypeError) as exc:
        parser.exit(2, f"integer player reference: {exc}\n")


if __name__ == "__main__":
    raise SystemExit(main())
