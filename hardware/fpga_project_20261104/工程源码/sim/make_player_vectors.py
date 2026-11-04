#!/usr/bin/env python3
"""Generate exact Q32.32 player_math RTL vectors, using the integer golden RNE.

No model/device/toolchain dependency. Example:
  python3 sim/make_player_vectors.py --output /tmp/player_vectors.mem
  xsim math_sim -runall -testplusarg VECTORS=/tmp/player_vectors.mem
Cases include negative ties, every terminal-state overflow, full uint32 dt,
different values in all seven axes, and seeded deterministic integer stress.
"""
import argparse
from fractions import Fraction
import hashlib
import json
from pathlib import Path
import random

from player_reference import CLOCK_HZ, INT64_MAX, INT64_MIN, SCALE, UINT32_MAX, checked_clock_hz, round_even


def packed(values):
    return sum((value & ((1 << 64)-1)) << (64*axis) for axis, value in enumerate(values))


def expected(dt, q, v, a, j, *, clock_hz=CLOCK_HZ):
    clock_hz = checked_clock_hz(clock_hz)
    if dt == 0:
        return [0]*7, [0]*7, [0]*7, 1
    h = Fraction(dt, clock_hz)
    outputs = [[], [], []]
    for qi, vi, ai, ji in zip(q, v, a, j):
        exact = (qi+vi*h+ai*h*h/2+ji*h**3/6, vi+ai*h+ji*h*h/2, ai+ji*h)
        for output, value in zip(outputs, exact):
            output.append(round_even(value))
    bad = any(not INT64_MIN <= value <= INT64_MAX for output in outputs for value in output)
    return ([0]*7, [0]*7, [0]*7, 1) if bad else (*outputs, 0)


def cases(seed=20261102, random_cases=40, *, clock_hz=CLOCK_HZ):
    clock_hz = checked_clock_hz(clock_hz)
    values = []
    def add(name, dt, q=None, v=None, a=None, j=None):
        values.append((name, dt, q or [0]*7, v or [0]*7, a or [0]*7, j or [0]*7))
    add("zero", 1)
    # Real-board failure: a stationary nonzero gripper was synthesized to zero.
    add("board_stationary_gripper_50mm", 727266, q=[0]*6+[214748365])
    add("invalid_zero_dt", 0, q=[SCALE]*7)
    add("different_axes_signed", 727273, q=[(axis-3)*SCALE for axis in range(7)],
        v=[(-1)**axis*(axis+1)*SCALE for axis in range(7)], a=[(3-axis)*SCALE for axis in range(7)], j=[(axis-2)*SCALE for axis in range(7)])
    if clock_hz % 2 == 0:
        add("half_even_components", clock_hz//2, j=[1, 3, -1, -3, 4, 24, -24])
        add("half_odd_components", clock_hz//2, q=[1, -1, 0, 0, 0, 0, 0],
            v=[0, 0, 1, -1, 0, 0, 0], a=[0, 0, 0, 0, 1, -1, 0], j=[24, -24, 4, -4, 1, -1, 0])
    else:
        # Odd C cannot represent a half-second interval exactly. At one second,
        # the acceleration/2 and jerk/2 terms still exercise signed RNE ties.
        add("half_even_components", clock_hz, a=[1, 3, -1, -3, 0, 0, 0], j=[0, 0, 0, 0, 1, 3, -3])
        add("half_odd_components", clock_hz, q=[1, -1, 0, 0, 0, 0, 0],
            v=[0, 0, 1, -1, 0, 0, 0], a=[1, -1, 3, -3, 0, 0, 0], j=[0, 0, 0, 0, 3, -3, 1])
    add("int64_min_hold", UINT32_MAX, q=[INT64_MIN]*7)
    add("int64_max_hold", UINT32_MAX, q=[INT64_MAX]*7)
    add("max_dt_small_jerk", UINT32_MAX, q=[-SCALE]*7, v=[SCALE]*7, a=[-SCALE]*7, j=[SCALE]*7)
    add("max_dt_positive_overflow", UINT32_MAX, j=[INT64_MAX]*7)
    add("max_dt_negative_overflow", UINT32_MAX, j=[INT64_MIN]*7)
    add("q_positive_overflow", clock_hz, q=[INT64_MAX]*7, v=[1]*7)
    add("q_negative_overflow", clock_hz, q=[INT64_MIN]*7, v=[-1]*7)
    add("v_positive_overflow", clock_hz, v=[INT64_MAX]*7, a=[1]*7)
    add("a_positive_overflow", clock_hz, a=[INT64_MAX]*7, j=[1]*7)
    add("a_negative_overflow", clock_hz, a=[INT64_MIN]*7, j=[-1]*7)
    add("one_cycle_extreme_derivatives", 1, v=[INT64_MAX, INT64_MIN, 0, 1, -1, SCALE, -SCALE], a=[0]*7, j=[0]*7)
    rng = random.Random(seed)
    for index in range(random_cases):
        dt = rng.choice((1, 2, 99, 727272, 727273, 1_000_000, 50_000_000, clock_hz, UINT32_MAX))
        states = []
        for _ in range(4):
            states.append([rng.choice((INT64_MIN, INT64_MAX)) if rng.random() < .1 else rng.randint(-(1 << 42), 1 << 42) for _ in range(7)])
        add(f"seeded_{index}", dt, *states)
    return values


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--output", required=True, type=Path, help="New .mem path; companion .json stores names, input/expected hashes and provenance")
    parser.add_argument("--seed", type=int, default=20261102)
    parser.add_argument("--random-cases", type=int, default=40)
    parser.add_argument("--clock-hz", type=int, default=CLOCK_HZ, help="Positive signed-integer clock metadata (default: 100000000)")
    args = parser.parse_args(argv)
    manifest = args.output.with_suffix(".json")
    if args.output.exists() or manifest.exists(): parser.error("output/manifest exists; preserve evidence with a new path")
    if args.random_cases < 0: parser.error("random-cases must be nonnegative")
    try:
        generated = cases(args.seed, args.random_cases, clock_hz=args.clock_hz)
    except ValueError as exc:
        parser.error(str(exc))
    lines, descriptions = [str(len(generated))], []
    for name, dt, q, v, a, j in generated:
        qo, vo, ao, bad = expected(dt, q, v, a, j, clock_hz=args.clock_hz)
        fields = [f"{dt:08x}", *(f"{packed(vector):0112x}" for vector in (q, v, a, j, qo, vo, ao)), str(bad)]
        lines.append(" ".join(fields))
        descriptions.append({"name": name, "dt_cycles": dt, "expected_overflow": bad})
    content = "\n".join(lines)+"\n"
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(content, encoding="ascii")
    manifest.write_text(json.dumps({"provenance": "design_only_integer_arithmetic", "physical_f_exec_hz": None,
        "clock_hz": args.clock_hz, "seed": args.seed, "vector_count": len(generated), "format": "dt_hex q0 v0 a0 j q1 v1 a1 (448bithex axis0low) overflow_decimal",
        "vectors_sha256": hashlib.sha256(content.encode("ascii")).hexdigest(), "cases": descriptions}, indent=2)+"\n")
    print(json.dumps({"vectors": str(args.output.resolve()), "manifest": str(manifest.resolve()), "count": len(generated)}))
    return 0


if __name__ == "__main__": raise SystemExit(main())
