#!/usr/bin/env python3
"""Generate 35 reproducible synthetic cache vectors with an integer-only oracle.

No hardware, SDK, network, RTL import or scientific-library dependency. Existing
vector files and companion <output>.json manifests are never overwritten.
Read with tb_limits_product_cache +VECTORS=... and the same CLOCK_HZ parameter.

Examples:
  python3 sim/make_cache_vectors.py --clock-hz 99999001 --output /tmp/new_cache.mem
  python3 sim/make_cache_vectors.py --clock-hz 0 --output /tmp/new_reject_clock.mem
Nonpositive signed32 clocks deliberately generate rejection-only fixtures.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import random
import sys

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_SEED = 20261102


def pack(values, width):
    return sum((value & ((1 << width)-1)) << (index*width) for index, value in enumerate(values))


def raw_valid(groups, clock_hz):
    """Independent exact ceil/floor domain checks; no HDL/reference imports."""
    lo, hi, dq, vmax, amax = groups
    return clock_hz > 0 and all(
        lower < upper and d > 0 and v > 0 and a > 0 and
        -((-lower)//65536) <= upper//65536 and
        -((-lower)//65536) >= -(1 << 31) and upper//65536 <= (1 << 31)-1
        for lower, upper, d, v, a in zip(lo, hi, dq, vmax, amax))


def cases(seed):
    rng = random.Random(seed)
    result = []
    # Invented test limits only, never a deployed configuration or motion goal.
    baseline = [[-58 << 32, 2 << 32, -118 << 32, -43 << 32, -58 << 32, -78 << 32, 0],
                [38 << 32, 133 << 32, -2 << 32, 43 << 32, 68 << 32, 58 << 32, (1 << 32)//10],
                [1 << 31]*6+[(1 << 32)//1000], [100 << 32]*6+[1 << 32], [160 << 32]*6+[2 << 32]]
    result.append(baseline)
    for _ in range(24):
        lo = [rng.randrange(-2000*65536, 1000*65536)*65536 for _ in range(7)]
        hi = [value+rng.randrange(1, 10000*65536)*65536 for value in lo]
        positive = [[rng.choice((1, (1 << 63)-1, rng.randrange(1, 1 << 63))) for _ in range(7)] for _ in range(3)]
        result.append([lo, hi, *positive])
    result.append([[-(1 << 47)]*7, [((1 << 31)-1) << 16]*7, [(1 << 63)-1]*7,
                   [(1 << 63)-1]*7, [(1 << 63)-1]*7])
    for group, axis, value in ((0, 0, baseline[1][0]), (0, 6, baseline[1][6]+1),
                               (2, 1, 0), (2, 2, -1), (3, 3, 0), (4, 4, 0),
                               (0, 5, -(1 << 63)), (1, 6, (1 << 63)-1)):
        invalid = [list(values) for values in baseline]
        invalid[group][axis] = value
        result.append(invalid)
    empty = [list(values) for values in baseline]
    empty[0][0], empty[1][0] = 65537, 65538
    result.append(empty)
    return result


def generate(seed, clock_hz):
    if type(clock_hz) is not int or not -(1 << 31) <= clock_hz < (1 << 31):
        raise ValueError("clock-hz must fit a signed32 RTL integer")
    rows, inputs, answers = [], [], []
    counts = {"allow": 0, "reject": 0}
    for index, groups in enumerate(cases(seed)):
        if any(len(values) != 7 or any(not -(1 << 63) <= value < (1 << 63) for value in values) for values in groups):
            raise ValueError("internal fixture exceeds seven signed64 axes")
        good = raw_valid(groups, clock_hz)
        counts["allow" if good else "reject"] += 1
        factors = (6*clock_hz**3, 6*clock_hz**3, 6*clock_hz**3, 2*clock_hz**2, clock_hz)
        outputs = [[value*factor for value in values] if good else [0]*7 for values, factor in zip(groups, factors)]
        if any(not -(1 << 161) <= value < (1 << 161) for values in outputs for value in values):
            raise ValueError("integer oracle product exceeds exact signed162 multiplier domain")
        epoch = f"{index+1:08x}"
        raw = " ".join([epoch]+[f"{pack(values, 64):0112x}" for values in groups])
        answer = " ".join([str(int(good))]+[f"{pack(values, 192):0336x}" for values in outputs])
        rows.append(" ".join([str(int(good)), raw]+[f"{pack(values, 192):0336x}" for values in outputs]))
        inputs.append(raw)
        answers.append(answer)
    return "\n".join(rows)+"\n", counts, inputs, answers


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--output", type=Path, required=True, help="New ASCII vector file; companion <output>.json must also be new")
    parser.add_argument("--clock-hz", type=int, default=100000000,
                        help="Signed32 compiled CLOCK_HZ (default: 100000000); zero/negative values exercise rejection")
    parser.add_argument("--seed", type=int, default=DEFAULT_SEED, help="Deterministic seed (default: 20261102)")
    args = parser.parse_args(argv)
    output = args.output.resolve()
    companion = output.with_name(output.name+".json")
    if output.exists() or companion.exists():
        parser.error("output and companion manifest must both be new files")
    try:
        content, counts, inputs, answers = generate(args.seed, args.clock_hz)
        sources = ("sim/make_cache_vectors.py", "sim/tb_limits_product_cache.sv", "rtl/limits_product_cache.sv")
        info = {"schema": "robot_rt.limits_product_cache_vectors.v1", "provenance": "synthetic_integer_goldens",
                "hardware_access": False, "physical_f_exec_hz": None, "physical_frequency_verified": False,
                "seed": args.seed, "count": len(inputs), "clock_hz": args.clock_hz, **counts,
                "python_version": sys.version.split()[0], "expected_done_edge_after_accept": 38,
                "format": "one row per job: valid_decimal epoch32_hex lower448_hex upper448_hex dq448_hex vmax448_hex amax448_hex expected_lower1344_hex expected_upper1344_hex expected_dq1344_hex expected_vmax1344_hex expected_amax1344_hex; axis0 low bits",
                "oracle": "independent Python signed integers: lo/hi/dq*6*C**3, vmax*2*C**2, amax*C; exact ceil/floor Q16 domain; rejected outputs zero",
                "tb_protocol_checks": ["E0 snapshot", "E38 atomic publish", "busy start/raw disturbances ignored",
                                       "busy epoch excursion rejects even after restoring epoch", "unknown raw rejection",
                                       "idle epoch change invalidates without resurrection", "reset abort without delayed done"],
                "vectors_sha256": hashlib.sha256(content.encode("ascii")).hexdigest(),
                "input_rows_sha256": hashlib.sha256(("\n".join(inputs)+"\n").encode("ascii")).hexdigest(),
                "expected_rows_sha256": hashlib.sha256(("\n".join(answers)+"\n").encode("ascii")).hexdigest(),
                "source_sha256": {name: hashlib.sha256((ROOT/name).read_bytes()).hexdigest() for name in sources},
                "working_directory": str(Path.cwd()),
                "invocation": [sys.executable, str(Path(__file__).resolve()), *(sys.argv[1:] if argv is None else argv)]}
        output.parent.mkdir(parents=True, exist_ok=True)
        with output.open("x", encoding="ascii") as handle:
            handle.write(content)
        with companion.open("x", encoding="utf-8") as handle:
            json.dump(info, handle, indent=2, allow_nan=False)
            handle.write("\n")
    except (OSError, ValueError) as exc:
        parser.exit(2, f"cache vectors: {exc}\n")
    print(json.dumps({"vectors": str(output), "manifest": str(companion), "count": len(inputs),
                      "clock_hz": args.clock_hz, **counts, "vectors_sha256": info["vectors_sha256"]}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
