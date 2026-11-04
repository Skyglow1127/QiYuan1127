#!/usr/bin/env python3
"""Independent unsigned-integer oracle vectors for ceil_div128_64 RTL.

Example:
  python3 sim/make_ceil_vectors.py --output /tmp/ceil_vectors.mem
  xsim ceil_sim -runall -testplusarg VECTORS=/tmp/ceil_vectors.mem

Python divmod is the oracle; no restoring-divider model, NumPy or hardware is
imported. Defaults produce 2306 vectors and a companion seed/hash manifest.
"""
import argparse
import hashlib
import json
from pathlib import Path
import random

U64 = (1 << 64)-1
U128 = (1 << 128)-1


def expected(dividend, divisor, origin):
    if any(type(value) is not int for value in (dividend, divisor, origin)) or not (0 <= dividend <= U128 and 0 <= divisor <= U64 and 0 <= origin <= U64):
        raise ValueError("unsigned128 dividend and unsigned64 divisor/origin required")
    if divisor == 0:
        return 0, 0, 1, 0
    quotient, remainder = divmod(dividend, divisor)
    ceiling = quotient+int(remainder != 0)
    absolute = origin+ceiling
    overflow = absolute > U64
    assert 0 <= ceiling <= U128
    return ceiling, 0 if overflow else absolute, 0, int(overflow)


def cases(seed=20261102, random_cases=256):
    values = []
    directed = ((0, 0, 0), (U128, 0, U64), (0, U64, U64), (U128, 1, 0),
                (U128, U64, 0), (U128, U64-1, U64), (U64, 1, 0), (U64, 1, 1),
                (U64-1, 1, 1), (U64*2, 2, 0), (U64*2, 2, 1),
                ((U64-1)*2+1, 2, 0), ((U64-1)*2+1, 2, 1))
    values.extend((f"directed_{index}", *operands) for index, operands in enumerate(directed))
    for dividend in range(64):
        for divisor in range(1, 32):
            values.append((f"small_{dividend}_{divisor}", dividend, divisor, 0))
    for clock in (100_000_000, 99_999_001):
        for numerator, denominator in ((20, 1), (275, 2), (clock, 256), (2*clock, 513)):
            threshold = clock*denominator
            for index in (0, 1, 2, 64, 10**12, U64):
                values.append((f"dds_{clock}_{numerator}_{denominator}_{index}", index*threshold, numerator, 17))
        threshold = clock*(U64//clock)
        numerator = (threshold >> 8)-1
        for index in (1, U64):
            values.append((f"max_threshold_{clock}_{index}", index*threshold, numerator, U64))
    generator = random.Random(seed)
    for index in range(random_cases):
        values.append((f"seeded_{index}", generator.getrandbits(128), generator.randrange(1, 1 << 64), generator.getrandbits(64)))
    values.append(("reset_restart_numeric", 7, 3, 9))
    return values


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--output", required=True, type=Path, help="New ASCII .mem file; companion .json records seed, hashes and format")
    parser.add_argument("--seed", type=int, default=20261102)
    parser.add_argument("--random-cases", type=int, default=256)
    args = parser.parse_args(argv)
    manifest = args.output.with_suffix(".json")
    if args.output.exists() or manifest.exists():
        parser.error("output/manifest exists; choose a new evidence path")
    if args.random_cases < 0:
        parser.error("random-cases must be nonnegative")
    generated = cases(args.seed, args.random_cases)
    lines, inputs, answers, descriptions = [str(len(generated))], [], [], []
    for name, dividend, divisor, origin in generated:
        ceiling, absolute, invalid, overflow = expected(dividend, divisor, origin)
        raw = f"{dividend:032x} {divisor:016x} {origin:016x}"
        answer = f"{ceiling:032x} {absolute:016x} {invalid} {overflow}"
        inputs.append(raw);answers.append(answer);lines.append(raw+" "+answer)
        descriptions.append({"name": name, "invalid": invalid, "absolute_overflow": overflow})
    content = "\n".join(lines)+"\n"
    info = {"schema": "robot_rt.ceil_vectors.v1", "provenance": "independent_python_integer_oracle",
            "hardware_access": False, "physical_f_exec_hz": None, "seed": args.seed,
            "vector_count": len(generated), "done_latency_cycles": 130,
            "format": "first line decimal count; then dividend128_hex divisor64_hex origin64_hex ceil128_hex absolute64_hex invalid_decimal absolute_overflow_decimal",
            "oracle": "Python divmod; ceil=floor+(remainder!=0); absolute overflow iff origin+ceil>UINT64_MAX",
            "vectors_sha256": hashlib.sha256(content.encode("ascii")).hexdigest(),
            "input_rows_sha256": hashlib.sha256(("\n".join(inputs)+"\n").encode("ascii")).hexdigest(),
            "expected_rows_sha256": hashlib.sha256(("\n".join(answers)+"\n").encode("ascii")).hexdigest(),
            "generator_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(), "cases": descriptions}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("x", encoding="ascii") as handle:
        handle.write(content)
    with manifest.open("x") as handle:
        json.dump(info, handle, indent=2);handle.write("\n")
    print(json.dumps({"vectors": str(args.output.resolve()), "manifest": str(manifest.resolve()),
                      "count": len(generated), "seed": args.seed, "vectors_sha256": info["vectors_sha256"]}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
