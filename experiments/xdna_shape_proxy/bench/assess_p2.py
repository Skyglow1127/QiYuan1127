#!/usr/bin/env python3
"""Assess P2 using an existing successful full proxy run; never execute inference.

This is evidence reuse, not an independent A/B sample set. ONNX is loaded with
load_external_data=False, so multi-GB random tensors are neither read nor copied.
"""
from __future__ import annotations

import argparse
import copy
import datetime as dt
import hashlib
import importlib
import json
import math
from pathlib import Path
import statistics
import sys
import traceback

ROOT = Path(__file__).resolve().parents[1]
STAGES = ("vision", "backbone", "expert")
NPU_EP = "VitisAIExecutionProvider"
FORBIDDEN = {"Einsum", "TopK", "NonZero"}


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def load_json(path: str | Path) -> dict | list:
    return json.loads(Path(path).read_text())


def write_json(path: Path, value: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(value, indent=2, ensure_ascii=False, allow_nan=False) + "\n")
    temporary.replace(path)


def positive(value: object) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value) and value > 0


def close(a: object, b: float) -> bool:
    return isinstance(a, (int, float)) and not isinstance(a, bool) and math.isfinite(a) and math.isclose(a, b, rel_tol=1e-8, abs_tol=1e-6)


def kind(label: str) -> str | None:
    value = label.upper()
    if "CPU" in value:
        return "CPU"
    if any(token in value for token in ("NPU", "DPU", "VAIML", "AIE", "VITIS_EP")):
        return "NPU"
    return None


def p95(values: list[float]) -> float:
    ordered = sorted(values)
    index = 0.95 * (len(values) - 1)
    lo, hi = math.floor(index), math.ceil(index)
    return ordered[lo] + (ordered[hi] - ordered[lo]) * (index - lo)


def static_graph_check(path: Path, onnx: object) -> dict:
    model = onnx.load_model(str(path), load_external_data=False)
    failures, dynamic, operators = [], [], {}
    graph_count = 0

    def visit(graph: object, prefix: str) -> None:
        nonlocal graph_count
        graph_count += 1
        for item in list(graph.input) + list(graph.output) + list(graph.value_info):
            if not item.type.HasField("tensor_type"):
                failures.append(f"{prefix}:{item.name}:non_tensor_type")
                continue
            tensor = item.type.tensor_type
            if not tensor.HasField("shape"):
                dynamic.append({"tensor": item.name, "graph": prefix, "reason": "missing shape"})
            else:
                for index, dim in enumerate(tensor.shape.dim):
                    if not dim.HasField("dim_value") or dim.dim_value <= 0:
                        dynamic.append({"tensor": item.name, "graph": prefix, "axis": index,
                                        "dim_param": dim.dim_param, "dim_value": dim.dim_value})
        for tensor in graph.initializer:
            if any(d <= 0 for d in tensor.dims):
                failures.append(f"{prefix}:{tensor.name}:nonpositive_initializer_dimension")
        for node in graph.node:
            qualified = node.op_type if not node.domain else node.domain + "::" + node.op_type
            operators[qualified] = operators.get(qualified, 0) + 1
            for attr in node.attribute:
                if attr.HasField("g"):
                    visit(attr.g, prefix + "/" + node.name + "/" + attr.name)
                for index, subgraph in enumerate(attr.graphs):
                    visit(subgraph, prefix + "/" + node.name + "/" + attr.name + str(index))

    visit(model.graph, "main")
    forbidden = {op: count for op, count in operators.items() if op.split("::")[-1] in FORBIDDEN}
    return {"path": str(path), "onnx_sha256": sha256(path), "load_external_data": False,
            "graph_count": graph_count, "all_recorded_tensor_shapes_static": not dynamic and not failures,
            "dynamic_axes": dynamic, "other_shape_failures": failures,
            "forbidden_operators": forbidden, "node_types": operators,
            "inputs": [{"name": item.name, "onnx_elem_type": item.type.tensor_type.elem_type,
                        "shape": [dim.dim_value for dim in item.type.tensor_type.shape.dim]} for item in model.graph.input],
            "passed": not dynamic and not failures and not forbidden,
            "scope": "Graph I/O, declared value_info, initializer dimensions, and nested graphs; external weight bytes are not loaded"}


def raw_check(source: dict) -> dict:
    errors = []
    for series in ("warmup_raw", "samples_raw"):
        rows = source.get(series, [])
        if not isinstance(rows, list) or len(rows) < 20:
            errors.append(f"{series}: at least 20 complete rows required")
            continue
        for index, row in enumerate(rows):
            if not isinstance(row, dict):
                errors.append(f"{series}[{index}]: not an object")
                continue
            for field in ("vision_ms", "backbone_ms", "expert_ms", "total_ms"):
                if not positive(row.get(field)):
                    errors.append(f"{series}[{index}].{field}: positive finite latency required")
            steps = row.get("expert_steps_ms", [])
            if len(steps) != 10 or not all(positive(value) for value in steps):
                errors.append(f"{series}[{index}]: 10 complete expert steps required")
            elif not close(row.get("expert_ms"), sum(steps)):
                errors.append(f"{series}[{index}]: expert sum differs from raw steps")
            for field in ("host_python_update_ms", "vision_to_backbone_host_bridge_ms"):
                value = row.get(field)
                if not isinstance(value, (int, float)) or isinstance(value, bool) or not math.isfinite(value) or value < 0:
                    errors.append(f"{series}[{index}].{field}: nonnegative finite value required")
    return {"warmup_n": len(source.get("warmup_raw", [])), "sample_n": len(source.get("samples_raw", [])),
            "errors": errors, "passed": not errors}


def formula_check(source: dict) -> dict:
    errors, recomputed = [], {}
    rows = source.get("samples_raw", [])
    manifest = source["manifest"]["contents"]
    stages = manifest["stages"]
    stage_flops = {name: stages[name]["flops_per_call"] for name in STAGES}
    total_flops = stage_flops["vision"] + stage_flops["backbone"] + 10 * stage_flops["expert"]
    if not all(positive(value) for value in stage_flops.values()):
        raise ValueError("Manifest FLOPs must be positive")
    if "flops_per_block" in manifest and not close(manifest["flops_per_block"], total_flops):
        errors.append("Manifest flops_per_block differs from sum of stage calls")
    series = {
        "total": ([row["total_ms"] for row in rows], total_flops),
        "vision": ([row["vision_ms"] for row in rows], stage_flops["vision"]),
        "backbone": ([row["backbone_ms"] for row in rows], stage_flops["backbone"]),
        "expert_10_steps": ([row["expert_ms"] for row in rows], 10 * stage_flops["expert"]),
    }
    for name, (values, flops) in series.items():
        expected = {"n": len(values), "p50_ms": statistics.median(values), "p95_ms": p95(values),
                    "algorithmic_flops": flops}
        expected["effective_tflops"] = flops / (expected["p50_ms"] / 1000) / 1e12
        expected["effective_tflops_at_p95_latency"] = flops / (expected["p95_ms"] / 1000) / 1e12
        measured = source.get("performance", {}).get(name, {})
        for key, value in expected.items():
            if not close(measured.get(key), value):
                errors.append(f"performance.{name}.{key}: mismatch")
        recomputed[name] = expected
    for index in range(10):
        values = [row["expert_steps_ms"][index] for row in rows]
        measured = source["performance"]["expert_per_step"][index]
        expected = {"n": len(values), "p50_ms": statistics.median(values), "p95_ms": p95(values),
                    "effective_tflops": stage_flops["expert"] / (statistics.median(values) / 1000) / 1e12}
        if any(not close(measured.get(key), value) for key, value in expected.items()):
            errors.append(f"performance.expert_per_step[{index}]: mismatch")
    return {"passed": not errors, "errors": errors, "recomputed": recomputed,
            "formula": "effective TFLOPS = manifest algorithmic GEMM FLOPs / (p50_ms / 1000) / 1e12; no peak-TOPS denominator"}


def npu_evidence(session: dict) -> dict:
    audit = session.get("ep_audit", {})
    errors, references = [], []
    report = Path(audit["amd_report_path"])
    raw_report = load_json(report)
    references.append({"kind": "amd_assignment_report", "path": str(report), "sha256": sha256(report)})
    counts = {"CPU": 0, "NPU": 0}
    unknown = []
    for item in raw_report.get("deviceStat", []):
        label, count = str(item.get("name", "")), item.get("nodeNum")
        if label.lower() == "all":
            continue
        assignment = kind(label)
        if assignment and isinstance(count, int) and not isinstance(count, bool) and count >= 0:
            counts[assignment] += count
        else:
            unknown.append(item)
    cpu_records = [item for item in raw_report.get("nodeStat", []) if kind(str(item.get("device", ""))) == "CPU"]
    assignment_ok = counts["NPU"] > 0 and counts["CPU"] == 0 and not cpu_records and not unknown
    if not assignment_ok:
        errors.append("AMD assignment report is not fully NPU, or contains unknown device statistics")

    profile = Path(audit["ort"]["path"])
    events = load_json(profile)
    kernel_events = [event for event in events if event.get("cat") == "Node" and event.get("args", {}).get("provider")]
    actual_npu = [event for event in kernel_events if event["args"]["provider"] == NPU_EP]
    actual_other = [event for event in kernel_events if event["args"]["provider"] != NPU_EP]
    runtime_ok = bool(actual_npu) and not actual_other
    references.append({"kind": "actual_ort_runtime_profile", "path": str(profile), "sha256": sha256(profile)})
    if not runtime_ok:
        errors.append("ORT actual Node execution has CPU/other providers, or lacks VitisAI events")

    compiler = audit.get("compiler", {})
    runners = []
    for item in compiler.get("contexts", []):
        path = Path(item["path"])
        value = load_json(path)
        digest = sha256(path)
        references.append({"kind": "compiled_context", "path": str(path), "sha256": digest})
        if item.get("sha256") != digest:
            errors.append("Compiled context differs from source measurement snapshot hash")
        for group in value.get("metaDef", []):
            if str(group.get("device", "")).upper() == "VAIML":
                param = group.get("vaimlParam", {})
                runners.append({"name": group.get("id"), "runner_type": param.get("runnerType"),
                                "expected_summary": str(Path(param.get("vaimlModelPath", str(path.parent / str(group.get("id"))))) / "fail_safe_summary.json")})
    hardware_ok = bool(runners) and all(item["runner_type"] == "hw" for item in runners)
    if not hardware_ok:
        errors.append("Every VAIML subgraph must have a compiled context runnerType=hw")

    summaries = []
    for item in compiler.get("offload_summaries", []):
        path = Path(item.get("snapshot_path", item["path"]))
        if not path.is_file():
            path = Path(item["path"])
        value = load_json(path)
        digest = sha256(path)
        references.append({"kind": "compiler_offload_summary", "path": str(path), "sha256": digest})
        if digest != item.get("sha256"):
            errors.append("Compiler summary differs from source measurement hash")
        original = item.get("original_path")
        if original is None and (path.parent / "source.json").is_file():
            original = load_json(path.parent / "source.json").get("original_path")
        summaries.append({"original_path": original or str(path), "captured_utc": item.get("captured_utc", ""),
                          "path": str(path), "offload_map": value.get("offload_map")})
    coverage = []
    for runner in runners:
        matches = [item for item in summaries if Path(item["original_path"]).resolve() == Path(runner["expected_summary"]).resolve()]
        final = max(matches, key=lambda item: item["captured_utc"]) if matches else None
        mapping = final.get("offload_map") if final else None
        explicit_zero = isinstance(mapping, dict) and any(kind(str(label)) == "CPU" for label in mapping)
        if explicit_zero:
            explicit_zero = (all(isinstance(count, (int, float)) and not isinstance(count, bool) and count == 0
                                 for label, count in mapping.items() if kind(str(label)) == "CPU")
                             and any(kind(str(label)) == "NPU" and positive(count) for label, count in mapping.items()))
        coverage.append({"subgraph": runner["name"], "matched_final_summary": final["path"] if final else None,
                         "explicit_cpu_zero": explicit_zero})
    internal_cpu_ok = bool(coverage) and all(item["explicit_cpu_zero"] for item in coverage)
    if not internal_cpu_ok:
        errors.append("A final CPU:0 offload_map is required for every VAIML subgraph; frontend summaries do not suffice")
    if not audit.get("npu_only_proven"):
        errors.append("Source measurement did not mark this session NPU-only proven")
    return {"passed": not errors, "errors": errors,
            "four_evidence_checks": {"amd_node_assignment_zero_cpu": assignment_ok,
                                     "context_hardware_runner": hardware_ok,
                                     "every_final_subgraph_explicit_cpu_zero": internal_cpu_ok,
                                     "actual_ort_vitis_execution_zero_other": runtime_ok},
            "amd_device_counts": counts, "actual_ort_npu_event_count": len(actual_npu),
            "final_summary_coverage": coverage, "references": references}


def cache_check(session: dict) -> dict:
    initial, cached = session.get("initial_creation", {}), session.get("cached_creation", {})
    errors, evidence = [], {}
    elapsed = cached.get("session_create_ms")
    if not positive(elapsed) or elapsed >= 10000:
        errors.append("Cached creation single observation must be below 10000 ms")
    for key in ("cache_key", "cache_dir", "model_sha256"):
        if not initial.get(key) or initial.get(key) != cached.get(key):
            errors.append(f"Initial/cached identity mismatch: {key}")
    if cached.get("cache_nonempty_before_create") is not True:
        errors.append("Cache was not known to be populated before cached session creation")
    reports = []
    for metadata in (initial, cached):
        path = Path(metadata.get("amd_ep_report_snapshot", ""))
        if not path.is_file():
            errors.append("Missing immutable AMD report snapshot for initial or cached creation")
        else:
            reports.append({"path": str(path), "sha256": sha256(path)})
    if len(reports) == 2 and reports[0]["sha256"] != reports[1]["sha256"]:
        errors.append("AMD assignment report changed between initial and cached session construction")
    contexts = session.get("ep_audit", {}).get("compiler", {}).get("contexts", [])
    associated_contexts = []
    for item in contexts:
        path = Path(item["path"])
        value = load_json(path)
        config = value.get("config", {})
        matches = (config.get("cacheKey") == cached.get("cache_key")
                   and Path(config.get("cacheDir", "")).resolve() == Path(cached.get("cache_dir", "")).resolve())
        associated_contexts.append({"path": str(path), "sha256": sha256(path), "same_cache_identity": matches})
    if not associated_contexts or not all(item["same_cache_identity"] for item in associated_contexts):
        errors.append("Compiled context does not establish reuse of the reported cache identity")
    evidence.update({"reports": reports, "contexts": associated_contexts,
                     "cache_nonempty_before_create": cached.get("cache_nonempty_before_create")})
    return {"passed": not errors, "errors": errors, "single_cached_creation_observation_ms": elapsed,
            "n": 1, "p50_ms": None, "p95_ms": None, "evidence": evidence,
            "statistical_limitation": "Session creation is one observation, without 20 warmups/20 samples. It is not presented as p50/p95; full inference samples remain independently validated.",
            "cache_hit_evidence_scope": "Same model/cache identity, populated cache, reused assignment report, compiled context association, and short reconstruction; no separate vendor cache-hit telemetry was collected."}


def assess(source_path: Path, output_path: Path) -> int:
    result = {"schema_version": 1, "stage": "P2", "optimization": "static_cached_verification",
              "status": "incomplete", "assessed_utc": dt.datetime.now(dt.timezone.utc).isoformat(),
              "environment": {}, "performance": None, "stop_optimization": False,
              "ab_comparison": {"applicable": False, "gain": None,
                                "reason": "P1 graphs already satisfy static construction. This assessment changes no graph and collects no new inference samples; no speedup claim."}}
    try:
        source = load_json(source_path)
        result["source_result"] = {"path": str(source_path.resolve()), "sha256": sha256(source_path)}
        result["environment"] = copy.deepcopy(source.get("environment", {}))
        result["measurement_reuse"] = {"reused_from": str(source_path.resolve()), "new_npu_calls": 0,
                                        "new_performance_samples": 0, "independent_ab_measurement": False,
                                        "measurement_started_utc": source.get("started_utc"),
                                        "measurement_ended_utc": source.get("ended_utc"),
                                        "environment_timestamp_preserved_from_measurement": True}
        source_ok = (source.get("status") == "ok" and source.get("mode") == "proxy"
                     and source.get("performance_scope") == "full_block" and source.get("npu_only_proven") is True)
        result["checks"] = {"successful_complete_full_block_source": source_ok}
        if not source_ok:
            result["incomplete_reason"] = "Input must be a completed successful full-block pure-NPU proxy run; running/failed/standalone results are not assessed"
            write_json(output_path, result)
            return 2
        result["raw_validation"] = raw_check(source)
        result["checks"]["at_least_20_warmup_20_complete_samples"] = result["raw_validation"]["passed"]
        if not result["raw_validation"]["passed"]:
            result["incomplete_reason"] = "Raw samples are incomplete or invalid"
            write_json(output_path, result)
            return 2
        result["formula_validation"] = formula_check(source)
        result["checks"]["p50_p95_and_effective_tflops_recomputed"] = result["formula_validation"]["passed"]
        onnx = importlib.import_module("onnx")
        result["assessor_dependencies"] = {"python": sys.version, "onnx_version": onnx.__version__}
        result["graph_checks"], result["npu_checks"], result["cache_checks"] = {}, {}, {}
        for name in STAGES:
            session = source["sessions"][name]
            model = Path(session["cached_creation"]["model"])
            graph = static_graph_check(model, onnx)
            if graph["onnx_sha256"] != session["cached_creation"]["model_sha256"]:
                graph["passed"] = False
                graph["model_hash_mismatch"] = True
            result["graph_checks"][name] = graph
            result["npu_checks"][name] = npu_evidence(session)
            result["cache_checks"][name] = cache_check(session)
        result["checks"].update({
            "all_stages_static_and_no_Einsum_TopK_NonZero": all(item["passed"] for item in result["graph_checks"].values()),
            "all_stages_four_npu_evidence_checks": all(item["passed"] for item in result["npu_checks"].values()),
            "all_cached_creation_observations_under_10s_and_reuse_evidence": all(item["passed"] for item in result["cache_checks"].values()),
        })
        integrity = result["formula_validation"]["passed"] and result["checks"]["all_stages_four_npu_evidence_checks"]
        if integrity:
            result["performance"] = copy.deepcopy(source["performance"])
            result["performance_scope"] = "full_block"
            result["npu_only_proven"] = True
        complete = all(result["checks"].values())
        result["status"] = "complete" if complete else "incomplete"
        if complete:
            effective = result["performance"]["total"]["effective_tflops"]
            result["stop_optimization"] = effective < 6.0
            result["stop_rule"] = {"threshold_effective_tflops": 6.0, "measured_effective_tflops": effective,
                                    "eligible": True, "triggered": effective < 6.0}
            if effective < 6.0:
                result["conclusion"] = "NPU 不适合该负载，建议 iGPU + PyTorch"
                result["next_action"] = "Stop P3-P7 optimization and issue the report, as explicitly required by the user's P2 stop rule"
            else:
                result["next_action"] = "Continue to P3; this assessment itself establishes no A/B gain"
        else:
            result["stop_rule"] = {"threshold_effective_tflops": 6.0, "eligible": False, "triggered": False,
                                    "reason": "P2 is incomplete; do not invoke the low-throughput stop rule"}
        write_json(output_path, result)
        print(json.dumps({"output": str(output_path.resolve()), "status": result["status"],
                          "stop_optimization": result["stop_optimization"], "checks": result["checks"]}, ensure_ascii=False))
        return 0 if complete else 2
    except Exception as exc:
        result["status"] = "incomplete"
        result["stop_optimization"] = False
        result["error"] = {"type": type(exc).__name__, "message": str(exc), "traceback": traceback.format_exc()}
        write_json(output_path, result)
        print(f"P2 assessment incomplete: {exc}; evidence={output_path}", file=sys.stderr)
        return 2


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, required=True, help="Successful completed full-block P1 results JSON")
    parser.add_argument("--output", type=Path, default=ROOT / "results" / "P2_static_cached.json")
    args = parser.parse_args()
    return assess(args.input.resolve(), args.output.resolve())


if __name__ == "__main__":
    raise SystemExit(main())
