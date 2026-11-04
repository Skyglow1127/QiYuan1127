#!/usr/bin/env python3
"""B5: compare official select_action with the new ACT queue, using recordings only.

Example:
  /home/aup/Lerobot/.venv/bin/python measure/check_action_alignment.py --output-dir measure/alignment_run

No robot/camera/CAN connection. Saved normalization is applied exactly once;
intentional safety clipping/filtering is excluded from the float32 equality test.
"""
from __future__ import annotations
import argparse
import inspect
from pathlib import Path
import sys

from act_runtime import (DEFAULT_INPUTS, DEFAULT_PROFILE, ROOT, environment, load_runtime,
                         require_idle_host, sha256, write_json)


def check(runtime, chunks=4, tolerance=1e-5):
    import numpy as np
    from rt_adapter.act import ACTQueueAdapter
    torch, policy = runtime["torch"], runtime["policy"]
    builder, post = runtime["builder"], runtime["postprocessor"]
    nsteps = runtime["cfg"].n_action_steps
    adapter = ACTQueueAdapter(policy, builder, post, n_action_steps=nsteps, enforce_bounds=False)
    adapter.reset()
    official, adapted, quantization_errors, encoded_actions = [], [], [], []
    outside_bounds = 0
    with torch.inference_mode():
        for step in range(chunks * nsteps):
            record_index = (step // nsteps + step % nsteps) % len(runtime["records"])
            record = runtime["records"][record_index]
            # Independent official processors vs new adapter entry point.
            raw = builder.raw_observation(record["images"], record["state"])
            normalized = policy.select_action(runtime["official_pre"](raw))
            reference = runtime["official_post"](normalized).detach().cpu()
            candidate = adapter.select_action(record["images"], record["state"]).detach().cpu()
            official.append(reference.numpy())
            adapted.append(candidate.numpy())
            bounds = torch.as_tensor(post.bounds, dtype=candidate.dtype)
            if bool(((candidate < bounds[:, 0]) | (candidate > bounds[:, 1])).any()):
                outside_bounds += 1
            else:
                encoded = post.to_q16(candidate)
                restored = encoded.to(torch.float64) / 65536
                quantization_errors.append(float((restored - candidate.to(torch.float64)).abs().max()))
                encoded_actions.append(encoded.numpy())
    reference, candidate = np.concatenate(official), np.concatenate(adapted)
    difference = np.abs(reference.astype(np.float64) - candidate.astype(np.float64))
    if not quantization_errors:
        raise ValueError("No policy action inside explicit bounds available for independent Q16.16 validation")
    result = {"passed": bool(np.isfinite(difference).all() and difference.max() <= tolerance and max(quantization_errors) <= 1/65536),
              "official_path": "saved preprocessor -> ACTPolicy.select_action -> saved postprocessor once",
              "adapter_path": "ACTQueueAdapter(enforce_bounds=False) -> independent predict_action_chunk queue -> saved postprocessor once",
              "steps": chunks * nsteps, "chunks": chunks, "n_action_steps": nsteps,
              "float32_tolerance": tolerance, "maximum_absolute_difference": float(difference.max()),
              "per_axis_maximum_difference": difference.max(axis=0).tolist(),
              "q16_one_lsb": 1/65536, "q16_maximum_roundtrip_error": max(quantization_errors),
              "q16_valid_domain_actions": len(quantization_errors), "actions_outside_deployment_bounds": outside_bounds,
              "intentional_limit_shaping_tested_as_numerical_equality": False,
              "physical_output_units": ["deg"] * 6 + ["m"],
              "hardware_motion": False, "limitation": "Numerical consistency does not establish task success, safety or physical smoothness"}
    return result, {"official_physical": reference, "adapter_physical": candidate,
                    "absolute_difference": difference, "q16_encoded_valid_actions": np.concatenate(encoded_actions)}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--profile", type=Path, default=DEFAULT_PROFILE)
    parser.add_argument("--inputs", type=Path, default=DEFAULT_INPUTS)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--chunks", type=int, default=4)
    parser.add_argument("--threads", type=int, default=8)
    parser.add_argument("--device", default="cpu")
    args = parser.parse_args(argv)
    try:
        if args.chunks < 2:
            raise ValueError("At least two chunks required to verify queue refill")
        inventory = require_idle_host()
        if args.output_dir.exists():
            raise ValueError("Choose a new output directory; results are never overwritten")
        runtime = load_runtime(args.profile, args.inputs, args.device, args.threads)
        result, arrays = check(runtime, args.chunks)
        import numpy as np
        from rt_adapter import act
        result.update(environment=environment(runtime), hashes=runtime["hashes"],
                      source_hashes={str(Path(__file__).resolve()): sha256(__file__),
                                     str(Path(__file__).with_name("act_runtime.py").resolve()): sha256(Path(__file__).with_name("act_runtime.py")),
                                     str(Path(act.__file__).resolve()): sha256(act.__file__),
                                     str(Path(inspect.getfile(runtime["policy"].__class__)).resolve()): sha256(inspect.getfile(runtime["policy"].__class__))},
                      process_inventory=inventory, invocation=[sys.executable, *sys.argv])
        args.output_dir.mkdir(parents=True)
        np.savez_compressed(args.output_dir / "action_comparison.npz", **arrays)
        write_json(args.output_dir / "verification.json", result)
        print({"passed": result["passed"], "steps": result["steps"],
               "maximum_absolute_difference": result["maximum_absolute_difference"],
               "q16_maximum_roundtrip_error": result["q16_maximum_roundtrip_error"]})
        return 0 if result["passed"] else 2
    except (ValueError, OSError, RuntimeError, ImportError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
