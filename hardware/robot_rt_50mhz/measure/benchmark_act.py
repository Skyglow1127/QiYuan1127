#!/usr/bin/env python3
"""B6: sustained local ACT inference on saved RGB/state inputs, without robot I/O.

Example:
  /home/aup/Lerobot/.venv/bin/python measure/benchmark_act.py --duration 600 --output-dir measure/bench_cpu_run

The timer starts after loading and warmup. Forward and recorded-observation to
physical-action latency are separate. The latter includes RAM input conversion,
saved normalization, forward and saved unnormalization, but excludes disk loading,
camera exposure/network/robot transport. This is inference saturation, not a live
20Hz controller. Existing physical rollout processes block this benchmark.
"""
from __future__ import annotations
import argparse
import csv
import inspect
import json
import math
from pathlib import Path
import sys
import time

from act_runtime import (DEFAULT_INPUTS, DEFAULT_PROFILE, ROOT, environment, load_runtime,
                         require_idle_host, sha256, sync, telemetry, write_json)


def percentiles(values):
    import numpy as np
    return {"count": len(values), "p50_ms": float(np.quantile(values, .5)),
            "p99_ms": float(np.quantile(values, .99)), "min_ms": float(min(values)), "max_ms": float(max(values))}


def benchmark(runtime, output, duration=600, warmup=3, progress_interval=30):
    torch, policy, builder, post = runtime["torch"], runtime["policy"], runtime["builder"], runtime["postprocessor"]
    records = runtime["records"]
    with torch.inference_mode():
        for index in range(warmup):
            record = records[index % len(records)]
            chunk = policy.predict_action_chunk(builder.build(record["images"], record["state"]))
            post.normalized_to_physical(chunk[:, :1])
            sync(runtime)
    timings, telemetry_rows, window_rows = [], [], []
    checksum = 0.0
    start = time.monotonic()
    progress_at = start
    completed = False
    abort_reason = None
    with (output / "timings.csv").open("x", newline="", encoding="utf-8") as stream, \
            (output / "telemetry.jsonl").open("x", encoding="utf-8") as sensors:
        writer = csv.writer(stream)
        writer.writerow(["iteration", "elapsed_s", "recording_index", "forward_ms", "recorded_observation_to_physical_ms"])
        try:
            with torch.inference_mode():
                index = 0
                while time.monotonic() - start < duration:
                    record = records[index % len(records)]
                    sync(runtime)
                    observation_at = time.perf_counter()
                    batch = builder.build(record["images"], record["state"])
                    sync(runtime)
                    forward_at = time.perf_counter()
                    chunk = policy.predict_action_chunk(batch)
                    sync(runtime)
                    forward_done = time.perf_counter()
                    physical = post.normalized_to_physical(chunk[:, :1])
                    sync(runtime)
                    action_done = time.perf_counter()
                    checksum += float(physical[0, 0, 0])
                    row = dict(iteration=index, elapsed_s=time.monotonic() - start,
                               recording_index=index % len(records), forward_ms=(forward_done-forward_at)*1000,
                               recorded_observation_to_physical_ms=(action_done-observation_at)*1000)
                    timings.append(row)
                    window_rows.append(row)
                    writer.writerow(list(row.values()))
                    now = time.monotonic()
                    if now >= progress_at:
                        # Check again during the long run; never interrupt a user rollout.
                        require_idle_host()
                        observation = telemetry(runtime)
                        observation["elapsed_s"] = now - start
                        telemetry_rows.append(observation)
                        sensors.write(json.dumps(observation, allow_nan=False) + "\n")
                        sensors.flush()
                        stream.flush()
                        print(json.dumps({"event": "progress", "elapsed_s": now-start, "iterations": index+1,
                                          "forward": percentiles([item["forward_ms"] for item in window_rows]),
                                          "rss_bytes": observation["rss_bytes"]}), flush=True)
                        window_rows = []
                        progress_at = now + progress_interval
                    index += 1
            completed = time.monotonic() - start >= duration
        except (KeyboardInterrupt, RuntimeError, ValueError) as exc:
            abort_reason = f"{type(exc).__name__}: {exc}"
    actual_duration = time.monotonic() - start
    if not timings:
        raise RuntimeError("No inference sample completed")
    first = [row for row in timings if row["elapsed_s"] <= min(60, duration/4)]
    last = [row for row in timings if row["elapsed_s"] >= actual_duration - min(60, duration/4)]
    first = first or timings[:1]
    last = last or timings[-1:]
    windows = []
    for minute in range(math.ceil(actual_duration / 60)):
        rows = [row for row in timings if minute*60 <= row["elapsed_s"] < (minute+1)*60]
        if rows:
            windows.append({"start_s": minute*60, "end_s": min((minute+1)*60, actual_duration),
                            "forward": percentiles([row["forward_ms"] for row in rows]),
                            "observation_to_physical": percentiles([row["recorded_observation_to_physical_ms"] for row in rows])})
    first_p50, last_p50 = percentiles([row["forward_ms"] for row in first]), percentiles([row["forward_ms"] for row in last])
    return {"complete": completed and abort_reason is None, "abort_reason": abort_reason,
            "target_duration_s": duration, "actual_measured_duration_s": actual_duration,
            "ten_minute_steady_run_completed": completed and abort_reason is None and actual_duration >= 600,
            "forward": percentiles([row["forward_ms"] for row in timings]),
            "recorded_observation_to_physical": percentiles([row["recorded_observation_to_physical_ms"] for row in timings]),
            "first_window": first_p50, "last_window": last_p50,
            "last_first_p50_ratio": last_p50["p50_ms"]/first_p50["p50_ms"], "minute_windows": windows,
            "telemetry": telemetry_rows, "physical_action_checksum": checksum,
            "rss_peak_observed_bytes": max((row["rss_bytes"] for row in telemetry_rows if row["rss_bytes"] is not None), default=None),
            "gpu_allocated_peak_observed_bytes": max((row["gpu_allocated_bytes"] for row in telemetry_rows if row["gpu_allocated_bytes"] is not None), default=None),
            "latency_scope": "recorded RGB/state in RAM -> conversion -> saved normalization -> forward -> saved physical unnormalization",
            "load_pattern": "back-to-back chunk inference saturation; queue30 semantics separately verified by check_action_alignment.py",
            "limitations": ["CPU frequency sysfs snapshots are not a throttling event log",
                            "thermal_zone names identify sensors; acpitz/WiFi values do not identify CPU core temperature",
                            "An unchanged latency trend does not prove the absence of thermal or power throttling",
                            "Recorded-input latency excludes live camera acquisition, communication and mechanical response",
                            "CPU runtime does not benchmark Radeon 890M GPU or NPU"]}


def report_markdown(result, output):
    env = result["environment"]
    forward, total = result["forward"], result["recorded_observation_to_physical"]
    return ("# ACT recorded-input AI PC benchmark\n\n"
            f"Measured {result['actual_measured_duration_s']:.3f}s; target {result['target_duration_s']:g}s; complete={result['complete']}. "
            f"Actual ten-minute run completed={result['ten_minute_steady_run_completed']}.\n\n"
            "| Item | Measured |\n|---|---|\n"
            f"| CPU | {env['cpu']} |\n| Runtime | Torch {env['torch']}, device {env['device']}, {env['threads']} threads, FP32 |\n"
            f"| Forward P50 / P99 | {forward['p50_ms']:.3f} / {forward['p99_ms']:.3f} ms |\n"
            f"| Recorded observation to physical action P50 / P99 | {total['p50_ms']:.3f} / {total['p99_ms']:.3f} ms |\n"
            f"| Final/initial window forward P50 | {result['last_first_p50_ratio']:.4f} |\n"
            f"| Model chunk / runtime consumed steps | {env['chunk_size']} / {env['n_action_steps']} |\n\n"
            "The source is four recorded episode41 RGB/state inputs. No live camera, CAN or robot connection was opened. "
            "This run repeatedly computes chunks at saturation; numerical queue30 equivalence is checked separately. "
            "Physical smoothness and task success were not tested.\n\n"
            "Raw samples: timings.csv; periodic memory/frequency/temperature: telemetry.jsonl; source hashes and environment: results.json.\n\n"
            + "\n".join("- " + item for item in result["limitations"]) + "\n\n"
            "Reproduce with a new output directory:\n\n```json\n" + json.dumps(result["invocation"], ensure_ascii=False) + "\n```\n")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--profile", type=Path, default=DEFAULT_PROFILE)
    parser.add_argument("--inputs", type=Path, default=DEFAULT_INPUTS)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--duration", type=float, default=600)
    parser.add_argument("--warmup", type=int, default=3)
    parser.add_argument("--threads", type=int, default=8)
    parser.add_argument("--device", default="cpu")
    args = parser.parse_args(argv)
    try:
        if not math.isfinite(args.duration) or args.duration <= 0 or args.warmup < 1:
            raise ValueError("positive finite duration and >=1 warmup required")
        inventory = require_idle_host()
        if args.output_dir.exists():
            raise ValueError("Choose a new output directory; no results overwritten")
        runtime = load_runtime(args.profile, args.inputs, args.device, args.threads)
        args.output_dir.mkdir(parents=True)
        result = benchmark(runtime, args.output_dir, args.duration, args.warmup)
        from rt_adapter import act
        result.update(environment=environment(runtime), hashes=runtime["hashes"], process_inventory=inventory,
                      source_hashes={str(Path(__file__).resolve()): sha256(__file__),
                                     str(Path(act.__file__).resolve()): sha256(act.__file__),
                                     str(Path(inspect.getfile(runtime["policy"].__class__)).resolve()): sha256(inspect.getfile(runtime["policy"].__class__))},
                      invocation=[sys.executable, *sys.argv])
        write_json(args.output_dir / "results.json", result)
        (args.output_dir / "act_bench.md").write_text(report_markdown(result, args.output_dir), encoding="utf-8")
        print(json.dumps({"event": "complete", "complete": result["complete"],
                          "actual_duration_s": result["actual_measured_duration_s"], "forward": result["forward"]}), flush=True)
        return 0 if result["complete"] else 2
    except (ValueError, OSError, RuntimeError, ImportError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
