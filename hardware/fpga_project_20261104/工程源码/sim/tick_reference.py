#!/usr/bin/env python3
"""Exact, design-only reference for a fixed-rate rational DDS.

Example:
  python sim/tick_reference.py --f-num 275 --f-den 2 --count 8
  python sim/tick_reference.py --clock-hz 99999001 --f-num 275 --f-den 2 --count 8

After reset phase=0. Each rising clock edge adds f_num; when the sum reaches
clock_hz*f_den, subtract that threshold once and emit a tick. Configuration
is immutable. A new configuration requires a new reference and reset origin.

This module uses integer arithmetic and analytical events, never per-clock
simulation, wall-clock sleeps, RTL, Vivado, or robot I/O. The 256-cycle minimum
interval is a logic budget constraint, not an actuator frequency capability.
"""
from __future__ import annotations

import argparse
from dataclasses import dataclass
from fractions import Fraction
import json
from typing import Iterator


CLOCK_HZ = 100_000_000
MIN_INTERVAL_CYCLES = 256
UINT64_MAX = (1 << 64) - 1


def integer(value: int, name: str, *, minimum: int) -> int:
    if type(value) is not int or value < minimum:
        raise ValueError(f"{name} must be an integer >= {minimum}; bool/float are not accepted")
    return value


def ceil_div(numerator: int, denominator: int) -> int:
    return (numerator + denominator - 1) // denominator


@dataclass(frozen=True, slots=True)
class DDSConfig:
    f_num: int
    f_den: int
    clock_hz: int = CLOCK_HZ

    def __post_init__(self):
        integer(self.f_num, "f_num", minimum=1)
        integer(self.f_den, "f_den", minimum=1)
        integer(self.clock_hz, "clock_hz", minimum=1)
        if self.clock_hz > (1 << 31) - 1:
            raise ValueError("clock_hz must fit a positive signed 32-bit RTL integer")
        if self.threshold > UINT64_MAX:
            raise ValueError("clock_hz * f_den must fit an unsigned 64-bit threshold")
        if self.f_num > self.threshold:
            raise ValueError("requested rational frequency exceeds the configured clock")
        # Fractional DDS intervals are floor(T/N) or ceil(T/N). Checking only
        # ceil(T/N) would incorrectly admit configurations with 255-cycle gaps.
        if self.threshold // self.f_num < MIN_INTERVAL_CYCLES:
            raise ValueError("shortest tick interval is below the 256-cycle logic budget")

    @property
    def threshold(self) -> int:
        return self.clock_hz * self.f_den

    @property
    def frequency_hz(self) -> Fraction:
        return Fraction(self.f_num, self.f_den)

    @property
    def interval_bounds(self) -> tuple[int, int]:
        return (self.threshold // self.f_num, ceil_div(self.threshold, self.f_num))

    def tick_cycle(self, tick_index: int) -> int:
        """Absolute clock edge of the one-based n-th tick from reset."""
        integer(tick_index, "tick_index", minimum=1)
        return ceil_div(tick_index * self.threshold, self.f_num)

    def event(self, tick_index: int) -> "TickEvent":
        cycle = self.tick_cycle(tick_index)
        residual = cycle * self.f_num - tick_index * self.threshold
        # ceil(x)-x is in [0,1), hence 0<=residual<f_num for every n.
        return TickEvent(tick_index, cycle, residual, self.f_num)


@dataclass(frozen=True, slots=True)
class TickEvent:
    tick_index: int
    cycle: int
    phase_after: int
    f_num: int

    @property
    def timing_error_cycles(self) -> Fraction:
        """Actual edge minus ideal edge; always nonnegative and strictly <1."""
        return Fraction(self.phase_after, self.f_num)

    def as_dict(self) -> dict:
        error = self.timing_error_cycles
        return {"tick_index": self.tick_index, "cycle": self.cycle,
                "phase_after": self.phase_after,
                "timing_error_cycles": {"numerator": error.numerator, "denominator": error.denominator}}


class TickReference:
    """Resettable state machine with O(1) jumps and O(number of events) output.

    Python integer cycle/tick counters are intentionally unbounded for reference
    proofs. Only the hardware DDS threshold and phase domain are constrained to
    64 bits here. A hardware adder needs a 65-bit intermediate sum or equivalent
    overflow-safe comparison/subtraction; silently wrapping a 64-bit sum differs.
    """
    __slots__ = ("_config", "cycle", "tick_count", "phase")

    def __init__(self, f_num: int, f_den: int, clock_hz: int = CLOCK_HZ):
        self._config = DDSConfig(f_num, f_den, clock_hz)
        self.reset()

    @property
    def config(self) -> DDSConfig:
        return self._config

    def reset(self) -> None:
        self.cycle = self.tick_count = self.phase = 0

    def advance_cycles(self, cycles: int) -> int:
        """Advance through clocks analytically; return the number of ticks emitted."""
        integer(cycles, "cycles", minimum=0)
        emitted, self.phase = divmod(self.phase + cycles * self.config.f_num, self.config.threshold)
        self.cycle += cycles
        self.tick_count += emitted
        return emitted

    def next_event(self) -> TickEvent:
        event = self.config.event(self.tick_count + 1)
        self.cycle, self.tick_count, self.phase = event.cycle, event.tick_index, event.phase_after
        return event

    def events(self, count: int) -> Iterator[TickEvent]:
        integer(count, "count", minimum=0)
        for _ in range(count):
            yield self.next_event()


def describe(config: DDSConfig, *, count: int, first_tick: int) -> dict:
    integer(count, "count", minimum=1)
    integer(first_tick, "first_tick", minimum=1)
    low, high = config.interval_bounds
    return {"design_only": True, "physical_frequency_validated": False, "f_exec_measured_hz": None,
            "clock_hz": config.clock_hz, "f_num": config.f_num, "f_den": config.f_den,
            "threshold": config.threshold, "phase_after_reset": 0,
            "first_tick_cycle_after_reset": config.tick_cycle(1),
            "logic_minimum_interval_cycles": MIN_INTERVAL_CYCLES,
            "interval_bounds_cycles": [low, high],
            "event_formula": "tick_cycle(n) = ceil(n * threshold / f_num), n >= 1",
            "proof": "0 <= tick_cycle(n)*f_num - n*threshold < f_num, so edge error is [0,1) clocks for every n",
            "counter_scope": "unbounded Python reference counters; threshold and phase constrained to uint64",
            "events": [config.event(index).as_dict() for index in range(first_tick, first_tick + count)],
            "limitations": ["Logical timing reference only; no RTL, Vivado, board or actuator was exercised",
                            "The explicit input rate is a design parameter, not a measured physical execution rate"]}


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--f-num", type=int, required=True, help="positive numerator of the explicit rational rate in Hz")
    parser.add_argument("--f-den", type=int, required=True, help="positive denominator; clock_hz*f_den must fit uint64")
    parser.add_argument("--clock-hz", type=int, default=CLOCK_HZ, help="explicit nominal design clock; default 100000000, PS profile 99999001")
    parser.add_argument("--count", type=int, default=10, help="number of analytical tick events to print")
    parser.add_argument("--first-tick", type=int, default=1, help="one-based first event; supports long-horizon checks without clock stepping")
    args = parser.parse_args(argv)
    try:
        report = describe(DDSConfig(args.f_num, args.f_den, args.clock_hz), count=args.count, first_tick=args.first_tick)
    except ValueError as exc:
        parser.error(str(exc))
    print(json.dumps(report, indent=2, allow_nan=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
