#!/usr/bin/env python3
"""Prepare fixed PI05 inputs, or benchmark a resident full expert policy.

Offline only. No camera, CAN, robot adapter, training or network imports.
The runtime owns GPU synchronization and resets before every real action chunk.
"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import io
import json
import math
from pathlib import Path
import statistics
import sys
import time
import traceback

PROTOCOL = "pi05-expert-inference/v1"
TASK = "Pick up the red cube and place it inside the black square on the left side of the table."
BEST_SHA256 = "583ff741239b111053dc969fe5ba69eb70475e888c7ece92b04a78d3319386cf"
IMAGE_KEYS = ("observation.images.global", "observation.images.wrist")
INPUT_KEYS = {"protocol_version", "task", "observation.state", "noise", *IMAGE_KEYS}
TIMING_KEYS = ("preprocess", "model", "postprocess")
UNITS = ["degree"] * 6 + ["metre"]
REPRESENTATION = "absolute joint targets and gripper width"


def now():
    return datetime.now(timezone.utc).isoformat()


def sha_bytes(value):
    return hashlib.sha256(value).hexdigest()


def write_report(path, data):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(data, indent=2, ensure_ascii=False, allow_nan=False) + "\n")
    temporary.replace(path)


def output_paths(output):
    output = Path(output)
    if output.suffix.lower() == ".json":
        report, actions = output, output.with_name(output.stem + "_actions.json")
    else:
        report, actions = output / "report.json", output / "actions.json"
    if report.exists() or actions.exists():
        raise FileExistsError(f"Use a new output location to preserve evidence: {output}")
    report.parent.mkdir(parents=True, exist_ok=True)
    return report, actions


def load_input(path):
    import numpy as np

    started = time.perf_counter()
    payload = Path(path).read_bytes()
    if len(payload) > 2 * 1024**2:
        raise ValueError("Expected a compact canonical input smaller than 2 MiB")
    with np.load(io.BytesIO(payload), allow_pickle=False) as archive:
        if set(archive.files) != INPUT_KEYS:
            raise ValueError(f"Canonical input fields differ: {sorted(archive.files)}")
        for key, expected in (("protocol_version", PROTOCOL), ("task", TASK)):
            value = archive[key]
            if value.shape != () or value.dtype.kind != "U" or str(value.item()) != expected:
                raise ValueError(f"Unexpected canonical {key}")
        state = archive["observation.state"].copy()
        noise = archive["noise"].copy()
        if state.dtype != np.float32 or state.shape != (7,) or not np.isfinite(state).all():
            raise ValueError("Expected finite float32 state[7] in degrees/metres")
        if noise.dtype != np.float32 or noise.shape != (1, 50, 32) or not np.isfinite(noise).all():
            raise ValueError("Expected fixed finite float32 noise[1,50,32]")
        sample = {"task": TASK, "observation.state": state}
        for key in IMAGE_KEYS:
            value = archive[key].copy()
            if value.dtype != np.uint8 or value.shape != (3, 224, 224):
                raise ValueError(f"Expected canonical RGB uint8[3,224,224]: {key}")
            sample[key] = value
    return sample, noise, payload, {
        "input": str(Path(path).resolve()), "input_sha256": sha_bytes(payload),
        "input_bytes": len(payload), "noise_sha256": sha_bytes(noise.tobytes()),
        "disk_read_decode_validate_seconds": time.perf_counter() - started,
        "source_capture_wallclock": None,
        "source_note": "Recorded input; input-ready/request timestamps are not camera capture timestamps.",
        "image_encoding": "Official CPU resize/pad to 224 followed by rounded uint8; saved once before timed runs.",
    }


def validate_actions(actions):
    import numpy as np

    value = np.asarray(actions)
    if value.shape != (50, 7) or value.dtype.kind not in "fi" or not np.isfinite(value).all():
        raise ValueError("Expected finite action chunk[50,7]")
    return value.astype(np.float64, copy=False).tolist()


def validate_timing(timing):
    if not isinstance(timing, dict):
        raise ValueError("Missing timing dictionary")
    result = {}
    for key in TIMING_KEYS:
        value = timing.get(key)
        if not isinstance(value, (int, float)) or not math.isfinite(value) or value < 0:
            raise ValueError(f"Invalid {key} timing: {value}")
        result[key] = float(value)
    return result


def percentile(values, fraction):
    values = sorted(values)
    index = (len(values) - 1) * fraction
    lower = math.floor(index)
    upper = math.ceil(index)
    return values[lower] + (values[upper] - values[lower]) * (index - lower)


def describe(values):
    if not values:
        return {"count": 0}
    return {"count": len(values), "median": statistics.median(values),
            "p95": percentile(values, .95), "min": min(values), "max": max(values)}


def summarize(rows, fields):
    selected = [row for row in rows if row["phase"] == "measured"]
    return {key: describe([row[key] for row in selected]) for key in fields}


def prepare(args):
    import numpy as np
    import torch
    from lerobot.policies.common.vla_utils import resize_with_pad_torch

    destination = Path(args.output)
    report_path = destination.with_suffix(destination.suffix + ".json")
    if destination.exists() or report_path.exists():
        raise FileExistsError(f"Use a new canonical input path: {destination}")
    torch.set_num_threads(args.threads)
    source = Path(args.input).read_bytes()
    converted, image_reports = {}, {}
    with np.load(io.BytesIO(source), allow_pickle=False) as archive:
        state = archive["observation.state"]
        if state.shape == (1, 7):
            state = state[0]
        if state.shape != (7,) or state.dtype.kind != "f" or not np.isfinite(state).all():
            raise ValueError("Expected recorded finite floating state[7]")
        task = archive["task"]
        if task.shape != () or task.dtype.kind != "U" or str(task.item()) != TASK:
            raise ValueError("Recorded task differs from the trained task")
        converted.update(protocol_version=np.array(PROTOCOL), task=np.array(TASK),
                         **{"observation.state": np.asarray(state, dtype=np.float32).copy()})
        for key in IMAGE_KEYS:
            original = archive[key]
            if original.shape != (3, 480, 640):
                raise ValueError(f"Expected recorded RGB CHW[3,480,640]: {key}")
            if original.dtype == np.uint8:
                image = torch.from_numpy(original.copy()).float() / 255
            elif original.dtype.kind == "f" and np.isfinite(original).all() \
                    and float(original.min()) >= 0 and float(original.max()) <= 1:
                image = torch.from_numpy(original.astype(np.float32, copy=True))
            else:
                raise ValueError(f"Expected uint8 RGB or finite RGB float in [0,1]: {key}")
            resized = resize_with_pad_torch(image.unsqueeze(0).permute(0, 2, 3, 1), 224, 224)
            resized = resized[0].permute(2, 0, 1).contiguous()
            encoded = torch.round(resized * 255).clamp(0, 255).to(torch.uint8)
            error = (encoded.float() / 255 - resized).abs()
            converted[key] = encoded.numpy()
            image_reports[key] = {"original_shape": list(original.shape),
                "original_dtype": str(original.dtype), "wire_shape": [3, 224, 224],
                "wire_dtype": "uint8", "quantization_max_abs_error_0_1": error.max().item(),
                "quantization_mean_abs_error_0_1": error.mean().item()}
    converted["noise"] = np.random.default_rng(args.seed).standard_normal((1, 50, 32), dtype=np.float32)
    buffer = io.BytesIO()
    np.savez_compressed(buffer, **converted)
    payload = buffer.getvalue()
    destination.parent.mkdir(parents=True, exist_ok=True)
    with destination.open("xb") as stream:
        stream.write(payload)
    _, _, _, checked = load_input(destination)
    report = {"created_utc": now(), "mode": "prepare", "success": True, "robot_io": False,
              "protocol_version": PROTOCOL, "recorded_source": str(Path(args.input).resolve()),
              "recorded_source_sha256": sha_bytes(source), "numpy_version": np.__version__,
              "torch_version": torch.__version__, "noise_rng": "numpy.random.default_rng.standard_normal",
              "noise_seed": args.seed, "image_conversion": image_reports, **checked,
              "note": "Canonical resizing/quantization changes full-resolution float input; this does not establish policy quality."}
    write_report(report_path, report)
    print(json.dumps({"canonical_input": str(destination), "report": str(report_path),
                      "input_sha256": checked["input_sha256"], "noise_sha256": checked["noise_sha256"]}))


def benchmark(args):
    report_path, actions_path = output_paths(args.output)
    report = {"created_utc": now(), "mode": "local", "success": False, "robot_io": False,
              "protocol_version": PROTOCOL, "batch_size": 1, "chunk_size": 50,
              "flow_steps": 10, "warmup_count": args.warmup, "measured_count": args.repeats,
              "rows": [], "actions_file": str(actions_path)}
    try:
        sample, noise, _, provenance = load_input(args.input)
        report.update(provenance)
        settings = json.loads(Path(args.config).read_text())
        report["config"] = str(Path(args.config).resolve())
        report["config_sha256"] = sha_bytes(Path(args.config).read_bytes())
        from runtime import Runtime

        started = time.perf_counter()
        runtime = Runtime(settings)
        report["runtime_initialization_seconds"] = time.perf_counter() - started
        report["runtime_metadata"] = runtime.metadata
        for index in range(args.warmup + args.repeats):
            phase = "warmup" if index < args.warmup else "measured"
            ready = time.perf_counter()
            current = {key: value.copy() if hasattr(value, "copy") else value for key, value in sample.items()}
            actions, timing = runtime.predict(current, noise=noise.copy())
            actions = validate_actions(actions)
            timing = validate_timing(timing)
            elapsed = time.perf_counter() - ready
            row = {"index": index, "phase": phase, "first_prediction": index == 0,
                   "timing_s": timing, "model_seconds": timing["model"],
                   "pipeline_seconds": sum(timing.values()),
                   "input_ready_to_validated_actions_seconds": elapsed,
                   "actions": actions}
            report["rows"].append(row)
            if index == 0:
                report["first_prediction"] = row
            write_report(report_path, report)
            print(json.dumps({key: value for key, value in row.items() if key != "actions"}), flush=True)
        report["summary_seconds"] = summarize(report["rows"],
            ("model_seconds", "pipeline_seconds", "input_ready_to_validated_actions_seconds"))
        write_report(actions_path, {"model_metadata": runtime.metadata,
            "input_sha256": provenance["input_sha256"], "noise_sha256": provenance["noise_sha256"],
            "units": UNITS, "representation": REPRESENTATION,
            "actions": report["rows"][-1]["actions"], "robot_io": False})
        report.update(success=True,
            timing_scope="Prepared canonical arrays in memory -> sample/noise copies -> runtime preprocess/model/postprocess -> action validation. GPU synchronization is performed by Runtime; input disk read, prepare, model load, camera/feedback/control excluded.",
            summary_note="Measured rows exclude requested warmup rows. With warmup=0 the summary includes the first cold prediction; p95 is descriptive for this sample count.")
    except BaseException as error:
        report["error"] = {"type": type(error).__name__, "message": str(error),
                           "traceback": traceback.format_exc()}
        raise
    finally:
        write_report(report_path, report)
        print("Report:", report_path, flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command")
    preparation = commands.add_parser("prepare", help="Prepare one fixed canonical NPZ; no model loading")
    preparation.add_argument("--input", type=Path, required=True)
    preparation.add_argument("--output", type=Path, required=True)
    preparation.add_argument("--seed", type=int, default=0)
    preparation.add_argument("--threads", type=int, default=8)
    local = commands.add_parser("local", help="Run a full expert policy locally, without robot I/O")
    local.add_argument("--config", type=Path, required=True)
    local.add_argument("--input", type=Path, required=True)
    local.add_argument("--output", type=Path, required=True,
                       help="New directory, or new JSON report path")
    local.add_argument("--warmup", type=int, default=3)
    local.add_argument("--repeats", type=int, default=10)
    # The common single-device CLI may omit the literal local subcommand.
    argv = sys.argv[1:]
    if argv and argv[0].startswith("--") and argv[0] not in ("--help", "-h"):
        argv = ["local", *argv]
    args = parser.parse_args(argv)
    if args.command == "prepare":
        if args.seed < 0 or not 1 <= args.threads <= 32:
            parser.error("Use nonnegative seed and 1..32 threads")
        prepare(args)
    elif args.command == "local":
        if not 0 <= args.warmup <= 100 or not 1 <= args.repeats <= 1000:
            parser.error("Use warmup 0..100 and repeats 1..1000")
        benchmark(args)
    else:
        parser.print_help()
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
