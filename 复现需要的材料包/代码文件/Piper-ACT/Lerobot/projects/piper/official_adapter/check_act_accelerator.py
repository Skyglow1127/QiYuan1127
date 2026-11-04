#!/usr/bin/env python3
"""One bounded ACT ROCm check on one recorded frame; never uses robot hardware."""

from __future__ import annotations

import argparse
from datetime import datetime
import importlib.metadata
import json
import os
from pathlib import Path
import statistics
import subprocess
import sys
import time
import traceback
from zoneinfo import ZoneInfo


DEFAULT_CHECKPOINT = Path(
    "/home/aup/Lerobot/models/ACT/rad_place_v5_r05/020000/pretrained_model"
)
DEFAULT_DATASET = Path("/home/aup/Lerobot/datasets/rad_place_merged_v5")
DEFAULT_REPORT = Path("/home/aup/Lerobot/outputs/preview_act_v5/act_accelerator_check.json")


def arguments():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--checkpoint", type=Path, default=DEFAULT_CHECKPOINT)
    parser.add_argument("--dataset-root", type=Path, default=DEFAULT_DATASET)
    parser.add_argument("--cpu-reference", type=Path, default=DEFAULT_REPORT.parent / "report.json")
    parser.add_argument("--report", type=Path, default=DEFAULT_REPORT)
    parser.add_argument("--timeout-seconds", type=int, default=180)
    parser.add_argument("--worker", action="store_true", help=argparse.SUPPRESS)
    return parser.parse_args()


def write_report(path, report):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False) + "\n",
                    encoding="utf-8")


def versions(names):
    result = {}
    for name in names:
        try:
            result[name] = importlib.metadata.version(name)
        except importlib.metadata.PackageNotFoundError:
            result[name] = None
    return result


def worker(args):
    report = {
        "created_at": datetime.now(ZoneInfo("Asia/Singapore")).isoformat(),
        "status": "running", "stage": "checking_gpu", "passed": False,
        "checkpoint": str(args.checkpoint.resolve()),
        "dataset_root": str(args.dataset_root.resolve()),
        "hardware_connected": False, "actions_sent": False,
        "training_performed": False, "packages_installed": False,
        "configuration": {"device": "cuda", "dtype": "float32", "use_amp": False,
                          "compile": False, "threads": 8, "warmup_runs": 1,
                          "measured_runs": 3, "retries": 0},
        "versions": versions(["torch", "torchvision", "lerobot", "safetensors",
                              "numpy", "datasets", "pyarrow", "torchcodec", "av"]),
    }
    write_report(args.report, report)
    try:
        os.environ["HF_HUB_OFFLINE"] = "1"
        os.environ["HF_DATASETS_OFFLINE"] = "1"
        os.environ["HF_HUB_DISABLE_TELEMETRY"] = "1"
        os.environ["HF_DATASETS_CACHE"] = str(args.report.parent / "act_accelerator_cache")
        os.environ["OMP_NUM_THREADS"] = "8"
        os.environ["MKL_NUM_THREADS"] = "8"
        os.environ["OPENBLAS_NUM_THREADS"] = "1"
        import torch

        torch.set_num_threads(8)
        torch.set_num_interop_threads(1)
        report["gpu_available"] = torch.cuda.is_available()
        report["hip_version"] = torch.version.hip
        if not report["gpu_available"]:
            raise RuntimeError("GPU is unavailable; stopping without alternative configurations.")
        properties = torch.cuda.get_device_properties(0)
        report["gpu_name"] = properties.name
        report["gpu_arch"] = getattr(properties, "gcnArchName", None)
        report["gpu_total_memory_bytes"] = properties.total_memory
        report["stage"] = "checking_official_dataset_dependencies"
        write_report(args.report, report)

        required = ("torchvision", "lerobot", "safetensors", "numpy", "datasets", "pyarrow")
        missing = [name for name in required if report["versions"][name] is None]
        if not (report["versions"]["torchcodec"] or report["versions"]["av"]):
            missing.append("a_video_decoder: torchcodec_or_av")
        if missing:
            report.update({
                "status": "unsupported_missing_dependencies", "missing_dependencies": missing,
                "stopped_before_model_loading": True, "gpu_inference_performed": False,
                "reason": "Existing ROCm environment lacks official dataset/inference dependencies. "
                          "No installation, path injection, alternative configuration or retry was attempted.",
                "fallback": "Use the already verified CPU ACT deployment.",
            })
            write_report(args.report, report)
            print(json.dumps({key: report[key] for key in
                              ("status", "gpu_name", "missing_dependencies", "fallback")},
                             ensure_ascii=False), flush=True)
            return 2

        import numpy as np
        from safetensors.torch import load_model
        from lerobot.configs import PreTrainedConfig
        from lerobot.datasets import LeRobotDataset
        from lerobot.policies import make_policy, make_pre_post_processors
        from preview_act_offline import ensure_checkpoint_files

        checkpoint = args.checkpoint.resolve()
        ensure_checkpoint_files(checkpoint)
        cpu_report = json.loads(args.cpu_reference.read_text(encoding="utf-8"))
        if (Path(cpu_report["checkpoint"]).resolve() != checkpoint
                or Path(cpu_report["dataset_root"]).resolve() != args.dataset_root.resolve()):
            raise ValueError("CPU reference does not match this checkpoint and dataset.")
        reference = cpu_report["samples"][0]
        cpu_chunk = np.asarray(reference["predicted_chunk"], dtype=np.float64)
        if not np.isfinite(cpu_chunk).all():
            raise ValueError("Existing same-frame CPU prediction is not finite.")
        backend = "torchcodec" if report["versions"]["torchcodec"] else "pyav"
        dataset = LeRobotDataset(
            repo_id=f"local/{args.dataset_root.name}", root=args.dataset_root,
            episodes=[int(reference["episode_index"])], video_backend=backend, return_uint8=False,
        )
        indices = dataset.hf_dataset.data.column("index").to_pylist()
        position = indices.index(int(reference["dataset_index"]))
        sample = dataset[position]
        if not np.allclose(sample["observation.state"].numpy(), reference["raw_state"]):
            raise ValueError("Recorded frame state differs from the same-frame CPU reference.")
        cfg = PreTrainedConfig.from_pretrained(
            checkpoint, local_files_only=True,
            cli_overrides=["--device=cuda", "--use_amp=false", "--pretrained_backbone_weights=null"],
        )
        if cfg.type != "act" or cfg.device != "cuda" or cfg.use_peft:
            raise ValueError("Expected regular ACT on the requested GPU, without CPU fallback.")
        cfg.pretrained_path = None
        report["stage"] = "strict_model_loading"
        write_report(args.report, report)
        policy = make_policy(cfg=cfg, ds_meta=dataset.meta)
        missing_keys, unexpected_keys = load_model(
            policy, str(checkpoint / "model.safetensors"), strict=True, device="cuda"
        )
        if missing_keys or unexpected_keys:
            raise RuntimeError("Strict ACT checkpoint load returned unmatched keys.")
        cfg.pretrained_path = checkpoint
        policy.eval().requires_grad_(False)
        pre, post = make_pre_post_processors(
            policy_cfg=cfg, pretrained_path=str(checkpoint),
            preprocessor_overrides={
                "device_processor": {"device": "cuda", "float_dtype": "float32"},
                "normalizer_processor": {"device": "cuda"},
            },
            postprocessor_overrides={
                "device_processor": {"device": "cuda", "float_dtype": "float32"},
                "unnormalizer_processor": {"device": "cuda"},
            },
        )

        def one_prediction():
            observation = {key: sample[key].clone() for key in cfg.input_features if key in sample}
            torch.cuda.synchronize()
            started = time.perf_counter()
            batch = pre(observation)
            torch.cuda.synchronize()
            model_started = time.perf_counter()
            with torch.inference_mode():
                chunk = policy.predict_action_chunk(batch)
                torch.cuda.synchronize()
                model_seconds = time.perf_counter() - model_started
                chunk = post(chunk)
                torch.cuda.synchronize()
            pipeline_seconds = time.perf_counter() - started
            if not torch.isfinite(chunk).all().item():
                raise RuntimeError("ACT accelerator output contains nonfinite values.")
            return chunk.squeeze(0).detach().cpu().numpy(), model_seconds, pipeline_seconds

        report.update({"stage": "warmup", "strict_weights_load": True,
                       "sample": {key: reference[key] for key in
                                  ("episode_index", "frame_index", "dataset_index", "timestamp_s")},
                       "video_backend": backend})
        write_report(args.report, report)
        started = time.perf_counter()
        one_prediction()
        report["warmup_seconds"] = time.perf_counter() - started
        results = []
        for run in range(3):
            report["stage"] = f"measured_run_{run + 1}"
            write_report(args.report, report)
            predicted, model_seconds, pipeline_seconds = one_prediction()
            if predicted.shape != cpu_chunk.shape:
                raise ValueError("GPU and CPU predicted action chunk shapes differ.")
            absolute_difference = np.abs(predicted.astype(np.float64) - cpu_chunk)
            results.append({
                "model_latency_ms": model_seconds * 1000,
                "pipeline_latency_ms": pipeline_seconds * 1000,
                "finite": True, "predicted_action": predicted[0].tolist(),
                "difference_from_cpu_first_action": absolute_difference[0].tolist(),
                "chunk_mae_from_cpu_per_dimension": absolute_difference.mean(axis=0).tolist(),
                "chunk_max_absolute_difference_per_dimension": absolute_difference.max(axis=0).tolist(),
            })
        model_times = [item["model_latency_ms"] for item in results]
        pipeline_times = [item["pipeline_latency_ms"] for item in results]
        cpu_ms = reference["model_latency_ms"]
        report.update({
            "status": "passed", "stage": "complete", "passed": True,
            "gpu_inference_performed": True, "runs": results,
            "units": ["deg"] * 6 + ["m"], "all_predictions_finite": True,
            "cpu_reference_model_latency_ms": cpu_ms,
            "cpu_reference_source": str(args.cpu_reference.resolve()),
            "cpu_reference_torch_version": cpu_report["versions"]["torch"],
            "model_latency_ms_mean": statistics.mean(model_times),
            "model_latency_ms_median": statistics.median(model_times),
            "pipeline_latency_ms_mean": statistics.mean(pipeline_times),
            "model_speedup_vs_existing_cpu_frame": cpu_ms / statistics.mean(model_times),
            "all_model_runs_below_50ms": all(value < 50 for value in model_times),
            "all_pipeline_runs_below_50ms": all(value < 50 for value in pipeline_times),
            "scope": "One recorded frame, one FP32 configuration; no robot execution or real-time claim.",
        })
        write_report(args.report, report)
        print(json.dumps({key: report[key] for key in
                          ("status", "gpu_name", "model_latency_ms_mean", "pipeline_latency_ms_mean")},
                         ensure_ascii=False), flush=True)
        return 0
    except Exception as exc:
        report.update({"status": "failed", "passed": False,
                       "error_type": type(exc).__name__, "error": str(exc),
                       "traceback": traceback.format_exc(),
                       "fallback": "Use the already verified CPU ACT deployment."})
        write_report(args.report, report)
        print(f"ACT accelerator check stopped: {type(exc).__name__}: {exc}", flush=True)
        return 1


def main():
    args = arguments()
    if args.worker:
        return worker(args)
    command = [sys.executable, str(Path(__file__).resolve()), *sys.argv[1:], "--worker"]
    try:
        result = subprocess.run(command, timeout=args.timeout_seconds, check=False)
        return_code = result.returncode
        if return_code >= 0:
            return return_code
        error = f"ACT check process stopped by signal {-return_code}."
    except subprocess.TimeoutExpired:
        error = f"ACT check exceeded its {args.timeout_seconds}s bound; no retry."
    report = json.loads(args.report.read_text(encoding="utf-8")) if args.report.exists() else {}
    report.update({"status": "failed", "passed": False, "error": error,
                   "fallback": "Use the already verified CPU ACT deployment."})
    write_report(args.report, report)
    print(error, flush=True)
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
