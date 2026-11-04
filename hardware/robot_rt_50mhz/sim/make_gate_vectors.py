#!/usr/bin/env python3
"""Generate reproducible DESIGN-ONLY limit_gate vectors with the integer oracle.

No hardware, SDK, network or scientific-library imports. Existing output files
and companion manifests are rejected. The TSV-like whitespace format is read
by tb_limit_gate's +VECTORS= argument; each field is an unsigned hex bit pattern.

Example:
  python3 sim/make_gate_vectors.py --output /tmp/new_gate_vectors.txt --seed 2601102 --count 700
"""
from __future__ import annotations

import argparse
from fractions import Fraction
import hashlib
import json
from pathlib import Path
import random
import sys

from player_reference import (
    CLOCK_HZ, Bounds, PlayerFault, SCALE, State, advance, checked_clock_hz, encode_can, round_even, to_q16,
)

ROOT = Path(__file__).resolve().parents[1]


def pack(values, bits):
    return sum((value & ((1 << bits)-1)) << (bits*axis) for axis, value in enumerate(values))


def endpoints(q, v, a, jerk, cycles, *, clock_hz=CLOCK_HZ):
    """Independently form all endpoint fractions, including rejected segments."""
    h = Fraction(cycles, checked_clock_hz(clock_hz))
    return tuple(tuple(round_even(value) for value in vector) for vector in (
        [q[i]+v[i]*h+a[i]*h*h/2+jerk[i]*h*h*h/6 for i in range(7)],
        [v[i]+a[i]*h+jerk[i]*h*h/2 for i in range(7)],
        [a[i]+jerk[i]*h for i in range(7)],
    ))


def generate(seed, count, *, clock_hz=CLOCK_HZ):
    clock_hz = checked_clock_hz(clock_hz)
    rng = random.Random(seed)
    # Invented oracle-test limits, never a deployed robot configuration.
    bounds = Bounds.from_physical(
        lower=['-10']*6+['0'], upper=['10']*6+['0.1'],
        velocity=['20']*6+['0.2'], acceleration=['50']*6+['1'],
        jerk=['100']*6+['5'], dq_max=['2']*6+['0.02'],
    )
    rows = []
    counts = dict(allow=0, reject=0, clamp=0)
    for case in range(count):
        q = tuple([rng.randrange(-8*SCALE, 8*SCALE) for _ in range(6)]
                  + [rng.randrange(SCALE//100, 9*SCALE//100)])
        v = tuple([rng.randrange(-5*SCALE, 5*SCALE) for _ in range(6)]
                  + [rng.randrange(-SCALE//50, SCALE//50)])
        a = tuple([rng.randrange(-5*SCALE, 5*SCALE) for _ in range(6)]
                  + [rng.randrange(-SCALE//5, SCALE//5)])
        jerk = tuple([rng.randrange(-50*SCALE, 50*SCALE) for _ in range(6)]
                     + [rng.randrange(-SCALE, SCALE)])
        cycles = rng.choice([1, 256, 10000, 1000000, 727273, 10000000, 30000000, 0xffffffff])
        if case % 9 == 0:
            v = a = jerk = (0,)*7
        if case % 29 == 0:
            # Exact physical upper Q32 endpoint is not representable in Q16.
            # This exercises intentional inward quantization, independently of
            # tb_limit_gate's directed RNE and empty-domain cases.
            q = (0,)*6+(bounds.upper[6],)
            v = a = jerk = (0,)*7
        if case % 17 == 0:
            cycles = 0
        qe, ve, ae = endpoints(q, v, a, jerk, cycles, clock_hz=clock_hz)
        if any(not -(1 << 63) <= value < (1 << 63) for vector in (qe, ve, ae) for value in vector):
            raise ValueError('Generator produced an endpoint requiring upstream math overflow handling')
        previous_valid = case % 5 != 0
        seed_q16 = tuple(to_q16(value, lo, hi) for value, lo, hi in zip(q, bounds.lower, bounds.upper))
        previous = encode_can(seed_q16, bounds)
        if case % 7 == 0:
            previous = tuple(value+rng.choice([-1, 1])*rng.randrange(1, 100000) for value in previous)
        allowed = clamped = False
        expected_q = expected_wire = (0,)*7
        try:
            step = advance(State(q, v, a), jerk, cycles, bounds, clock_hz=clock_hz)
            wire = encode_can(step.q16, bounds, previous if previous_valid else None)
            allowed = True
            expected_q, expected_wire = step.q16, wire
            clamped = any(code != round_even(Fraction(raw, 65536)) for code, raw in zip(expected_q, qe)) or any(
                value != round_even(Fraction(code*scale, 65536))
                for value, code, scale in zip(wire, expected_q, (1000,)*6+(1000000,))
            )
        except PlayerFault:
            pass
        counts['allow' if allowed else 'reject'] += 1
        counts['clamp'] += int(clamped)
        buses = [q, v, a, jerk, qe, ve, ae, bounds.lower, bounds.upper,
                 bounds.velocity, bounds.acceleration, bounds.jerk, bounds.dq_max]
        fields = [f'{pack(bus, 64):0112x}' for bus in buses]
        fields += [f'{cycles:08x}', f'{pack(previous, 32):056x}', str(int(previous_valid)),
                   str(int(allowed)), str(int(clamped)), f'{pack(expected_q, 32):056x}',
                   f'{pack(expected_wire, 32):056x}']
        rows.append(' '.join(fields))
    return '\n'.join(rows)+'\n', counts


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--output', type=Path, required=True, help='New vector file; parent directory must exist')
    parser.add_argument('--seed', type=int, default=2601102, help='Deterministic random seed (default: 2601102)')
    parser.add_argument('--count', type=int, default=700, help='Positive number of oracle rows (default: 700)')
    parser.add_argument('--clock-hz', type=int, default=CLOCK_HZ, help='Positive signed-integer clock metadata (default: 100000000)')
    args = parser.parse_args(argv)
    if args.count < 1:
        parser.error('--count must be positive')
    output = args.output.resolve()
    companion = output.with_name(output.name+'.json')
    if output.exists() or companion.exists():
        parser.error('Output and companion manifest must both be new files')
    try:
        text, counts = generate(args.seed, args.count, clock_hz=args.clock_hz)
        sources = ['sim/player_reference.py', 'sim/make_gate_vectors.py', 'rtl/limit_gate.sv', 'sim/tb_limit_gate.sv']
        hashes = {name: hashlib.sha256((ROOT/name).read_bytes()).hexdigest() for name in sources}
        payload = text.encode('ascii')
        manifest = dict(
            schema='robot_rt.limit_gate_vectors.v1', provenance='design_only',
            seed=args.seed, count=args.count, clock_hz=args.clock_hz, **counts,
            python=sys.version.split()[0], vectors_sha256=hashlib.sha256(payload).hexdigest(),
            source_sha256=hashes, axis_units=['deg']*6+['m'],
            oracle='player_reference.advance + encode_can, exact Fraction arithmetic',
            upstream_overflow_vectors=False,
        )
        with output.open('xb') as handle:
            handle.write(payload)
        with companion.open('x') as handle:
            json.dump(manifest, handle, indent=2)
            handle.write('\n')
    except (OSError, ValueError) as exc:
        parser.exit(2, f'gate vectors: {exc}\n')
    print(json.dumps(dict(output=str(output), manifest=str(companion), **manifest), indent=2))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
