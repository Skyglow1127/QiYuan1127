#!/usr/bin/env python3
"""Local-only, fail-closed Ryzen AI benchmark for pi0.5 shape proxies.

No model downloads, cloud calls, CPU substitution, or power-plan changes occur here.
Model generation is separate; see bench/proxy/manifest.json for exact operations.
An AMD EP placement report is required in addition to the ORT profile because
VitisAIExecutionProvider can itself execute work on VITIS_EP_CPU.
"""
from __future__ import annotations

import argparse
import datetime as dt
import gc
import hashlib
import importlib
import importlib.metadata
import json
import math
import multiprocessing
import os
from pathlib import Path
import platform
import re
import shlex
import shutil
import statistics
import subprocess
import sys
import time
import traceback
from typing import Any

ROOT = Path(__file__).resolve().parents[1]
NPU_EP = "VitisAIExecutionProvider"
STAGES = ("vision", "backbone", "expert")


class BenchmarkError(RuntimeError):
    pass


class _CompilerEvidenceCollector:
    """File collector used only by a separate process, never the ORT GIL."""

    def __init__(self, cache_key_dir: Path):
        self.root = cache_key_dir
        self.archive = cache_key_dir / "compiler_evidence"
        self.captured: set[str] = set()
        self.errors: set[str] = set()

    def _scan(self) -> None:
        try:
            candidates = self.root.glob("**/fail_safe_summary.json")
            for source in candidates:
                if self.archive in source.parents:
                    continue
                try:
                    contents = source.read_bytes()
                    parsed = json.loads(contents)
                    digest = hashlib.sha256(contents).hexdigest()
                    identity = hashlib.sha256((str(source) + ":" + digest).encode()).hexdigest()[:24]
                    if identity in self.captured:
                        continue
                    target = self.archive / identity / "fail_safe_summary.json"
                    target.parent.mkdir(parents=True, exist_ok=True)
                    target.write_bytes(contents)
                    write_json(target.parent / "source.json", {"original_path": str(source), "sha256": digest,
                                                               "captured_utc": utc_now(), "parsed_json": parsed,
                                                               "capture_method": "independent spawned process; ORT may hold Python GIL"})
                    self.captured.add(identity)
                except (FileNotFoundError, json.JSONDecodeError):
                    # Compiler files may be in flight; retry on the next scan.
                    continue
                except OSError as exc:
                    self.errors.add(str(exc))
        except OSError as exc:
            self.errors.add(str(exc))


def _compiler_watcher_process(root: str, stop_event: Any, result_queue: Any) -> None:
    collector = _CompilerEvidenceCollector(Path(root))
    while not stop_event.is_set():
        collector._scan()
        stop_event.wait(0.15)
    collector._scan()
    result_queue.put({"captured_summaries": len(collector.captured), "errors": sorted(collector.errors)})


class CompilerEvidenceWatcher:
    """Capture summaries in a process: ORT's constructor may hold the GIL."""

    def __init__(self, cache_key_dir: Path):
        self.root = cache_key_dir
        self.archive = cache_key_dir / "compiler_evidence"
        context = multiprocessing.get_context("spawn")
        self.stop_event = context.Event()
        self.result_queue = context.Queue()
        self.process = context.Process(target=_compiler_watcher_process,
                                       args=(str(cache_key_dir), self.stop_event, self.result_queue),
                                       name="local-compiler-evidence", daemon=True)

    def start(self) -> None:
        self.process.start()

    def stop(self) -> dict:
        self.stop_event.set()
        self.process.join(timeout=5)
        stopped = not self.process.is_alive()
        metadata = {"captured_summaries": None, "errors": []}
        if stopped and self.process.exitcode == 0:
            try:
                metadata.update(self.result_queue.get(timeout=1))
            except Exception as exc:
                metadata["errors"].append(str(exc))
        elif not stopped:
            self.process.terminate()
            self.process.join(timeout=2)
            metadata["errors"].append("Compiler evidence subprocess did not exit in time")
        metadata.update({"scope": str(self.root), "archive": str(self.archive), "poll_interval_seconds": 0.15,
                "process_stopped": stopped, "process_exitcode": self.process.exitcode,
                "note": "Copies only compiler-produced fail_safe_summary.json inside this exact cache key; does not alter compiler outputs."}
        )
        self.result_queue.close()
        return metadata


def utc_now() -> str:
    return dt.datetime.now(dt.timezone.utc).isoformat()


def write_json(path: Path, value: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(value, indent=2, ensure_ascii=False, allow_nan=False) + "\n")
    temporary.replace(path)


def file_sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(8 * 1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def read_text(path: str) -> str | None:
    try:
        return Path(path).read_text().strip()
    except (OSError, UnicodeError):
        return None


def command_evidence(command: list[str], timeout: int = 10) -> dict:
    try:
        proc = subprocess.run(command, text=True, capture_output=True, timeout=timeout, check=False)
        return {"command": command, "returncode": proc.returncode,
                "stdout": proc.stdout.strip(), "stderr": proc.stderr.strip()}
    except (OSError, subprocess.TimeoutExpired) as exc:
        return {"command": command, "error": str(exc)}


def environment_fingerprint(path: Path | None) -> dict:
    supplied = json.loads(path.read_text()) if path else {}
    if not isinstance(supplied, dict):
        raise BenchmarkError("--fingerprint must contain a JSON object")
    supplied = supplied.get("environment", supplied)
    if not isinstance(supplied, dict):
        raise BenchmarkError("--fingerprint environment must be a JSON object")
    fingerprint = {
        "os": platform.platform(), "cpu": platform.processor() or "unknown",
        "python_version": platform.python_version(), "python_executable": sys.executable,
        "driver": "unknown", "rocm_version": "unknown", "ryzen_ai_version": "unknown",
        "ort_version": "unavailable", "ep_version": "unknown",
        "power_mode": "unknown", "npu_frequency": "unknown",
        "ac_power": "unknown", "timestamp_utc": utc_now(),
    }
    if platform.system() == "Linux":
        fingerprint["driver"] = read_text("/sys/module/amdxdna/version") or "unknown"
        cpuinfo = read_text("/proc/cpuinfo") or ""
        match = re.search(r"^model name\s*:\s*(.+)$", cpuinfo, re.MULTILINE)
        if match:
            fingerprint["cpu"] = match.group(1)
        if shutil.which("powerprofilesctl"):
            evidence = command_evidence(["powerprofilesctl", "get"])
            fingerprint["power_mode_probe"] = evidence
            if evidence.get("returncode") == 0:
                fingerprint["power_mode"] = evidence["stdout"]
    fingerprint.update(supplied)
    packages = {}
    for distribution in ("onnxruntime-vitisai", "voe", "flexml", "ryzenai-onnx-utils", "vitis-aie-essentials"):
        try:
            packages[distribution] = importlib.metadata.version(distribution)
        except importlib.metadata.PackageNotFoundError:
            packages[distribution] = None
    fingerprint["runtime_distribution_versions"] = packages
    if packages["onnxruntime-vitisai"]:
        fingerprint["ort_version"] = packages["onnxruntime-vitisai"]
    if packages["voe"]:
        fingerprint["ep_version"] = packages["voe"]
        fingerprint["ep_version_scope"] = "VOE distribution version; VitisAI EP implementation packaged with onnxruntime-vitisai"
    if packages["ryzenai-onnx-utils"] or packages["flexml"]:
        fingerprint["ryzen_ai_version"] = packages["ryzenai-onnx-utils"] or packages["flexml"]
    fingerprint["timestamp_utc"] = utc_now()
    fingerprint["benchmark_python_executable"] = sys.executable
    if path:
        fingerprint["supplied_fingerprint"] = {"path": str(path.resolve()), "sha256": file_sha256(path)}
    return fingerprint


def percentile(values: list[float], q: float) -> float:
    ordered = sorted(values)
    index = (len(ordered) - 1) * q
    lo = math.floor(index)
    hi = math.ceil(index)
    return ordered[lo] + (ordered[hi] - ordered[lo]) * (index - lo)


def summarize_ms(values: list[float], flops: int | float | None = None) -> dict:
    if not values:
        return {"n": 0, "p50_ms": None, "p95_ms": None, "effective_tflops": None}
    p50 = statistics.median(values)
    result = {"n": len(values), "p50_ms": p50, "p95_ms": percentile(values, 0.95),
              "min_ms": min(values), "max_ms": max(values),
              "effective_tflops": flops / (p50 / 1000) / 1e12 if flops is not None and p50 > 0 else None}
    if flops is not None:
        result["algorithmic_flops"] = flops
        result["effective_tflops_at_p95_latency"] = flops / (result["p95_ms"] / 1000) / 1e12
    return result


def load_manifest(model_dir: Path) -> tuple[dict, dict]:
    manifest_path = model_dir / "manifest.json"
    manifest = json.loads(manifest_path.read_text())
    stages = manifest.get("stages", manifest.get("models"))
    if isinstance(stages, list):
        stages = {item.get("name", item.get("stage")): item for item in stages}
    if not isinstance(stages, dict) or any(stage not in stages for stage in STAGES):
        raise BenchmarkError("manifest needs stages.{vision,backbone,expert}")
    normalized = {}
    for name in STAGES:
        spec = dict(stages[name])
        spec["path"] = (model_dir / spec.get("file", spec.get("filename", f"{name}_proxy.onnx"))).resolve()
        if not spec["path"].is_file():
            raise BenchmarkError(f"Missing local proxy: {spec['path']}")
        flops = spec.get("flops_per_call", spec.get("flops"))
        if not isinstance(flops, (int, float)) or flops <= 0:
            raise BenchmarkError(f"{name}: manifest must specify positive exact flops_per_call")
        spec["flops_per_call"] = flops
        normalized[name] = spec
    steps = manifest.get("expert_steps", manifest.get("denoising_steps", 10))
    if not isinstance(steps, int) or steps != 10:
        raise BenchmarkError("pi0.5 proxy requires expert_steps=10")
    return manifest, normalized


def _label_kind(value: Any) -> str | None:
    text = str(value).upper()
    if "CPU" in text:
        return "CPU"
    if any(token in text for token in ("NPU", "DPU", "VITIS_EP", "VITISAIEXECUTIONPROVIDER", "VAIP", "VAIML", "AIE")):
        return "NPU"
    return None


def parse_amd_report(value: Any) -> dict:
    """Parse known placement fields, preserving raw evidence separately.

    deviceStat summaries take precedence over lists because lists may contain
    fused/original nodes at different levels. Unknown schemas stay unknown.
    """
    assignments: list[dict] = []
    stat_entries: list[dict] = []
    count_keys = ("nodeNum", "node_num", "nodeCount", "node_count", "count", "nodes", "numNodes", "node_size")
    label_keys = ("device", "deviceName", "device_name", "name", "type", "provider", "deviceType")

    def count_of(item: Any) -> int | None:
        if isinstance(item, int) and not isinstance(item, bool) and item >= 0:
            return item
        if isinstance(item, dict):
            for key in count_keys:
                number = item.get(key)
                if isinstance(number, int) and not isinstance(number, bool) and number >= 0:
                    return number
                if isinstance(number, list):
                    return len(number)
        if isinstance(item, list):
            return len(item)
        return None

    def parse_stats(stats: Any, location: str) -> None:
        if isinstance(stats, list):
            for index, entry in enumerate(stats):
                if isinstance(entry, dict):
                    label = next((entry[k] for k in label_keys if k in entry), None)
                    count = count_of(entry)
                    if label is not None and count is not None:
                        stat_entries.append({"device": str(label), "kind": _label_kind(label), "count": count,
                                             "source": f"{location}[{index}]"})
                    else:
                        parse_stats(entry, f"{location}[{index}]")
        elif isinstance(stats, dict):
            for label, entry in stats.items():
                kind, count = _label_kind(label), count_of(entry)
                if kind and count is not None:
                    stat_entries.append({"device": str(label), "kind": kind, "count": count,
                                         "source": f"{location}.{label}"})
                elif isinstance(entry, (dict, list)):
                    parse_stats(entry, f"{location}.{label}")

    def visit(item: Any, location: str = "$") -> None:
        if isinstance(item, dict):
            for key in ("deviceStat", "device_stat", "deviceStats"):
                if key in item:
                    parse_stats(item[key], location + "." + key)
            # Only explicit device/provider attributes establish assignment.
            device = next((item[key] for key in ("device", "deviceName", "device_name", "deviceType", "provider", "execution_provider")
                           if key in item and isinstance(item[key], str)), None)
            node_name = next((item[key] for key in ("node_name", "nodeName", "name", "id") if key in item), None)
            if node_name is None and device is not None and isinstance(item.get("output"), list):
                node_name = ",".join(str(name) for name in item["output"])
            if device is not None and node_name is not None and _label_kind(device):
                assignments.append({"node": str(node_name), "device": device, "kind": _label_kind(device),
                                    "op_type": item.get("opType", item.get("op_type")), "source": location})
            for key, child in item.items():
                if key not in ("deviceStat", "device_stat", "deviceStats"):
                    visit(child, location + "." + key)
        elif isinstance(item, list):
            for index, child in enumerate(item):
                visit(child, f"{location}[{index}]")

    visit(value)
    # Do not sum repeated whole-model summaries. Multiple copies must agree.
    stats_by_device: dict[str, list[int]] = {}
    for entry in stat_entries:
        if entry["kind"]:
            stats_by_device.setdefault(entry["device"], []).append(entry["count"])
    consistent = all(len(set(counts)) == 1 for counts in stats_by_device.values())
    counts = {"CPU": 0, "NPU": 0}
    if stat_entries and consistent:
        for label, numbers in stats_by_device.items():
            counts[_label_kind(label)] += numbers[0]
    else:
        unique = {(entry["node"], entry["device"]): entry for entry in assignments}
        for entry in unique.values():
            counts[entry["kind"]] += 1
    return {"device_stats": stat_entries, "node_assignments": assignments,
            "cpu_node_count": counts["CPU"], "npu_node_count": counts["NPU"],
            "schema_recognized": bool(stats_by_device) and consistent,
            "statistics_consistent": consistent,
            "note": "CPU includes VITIS_EP_CPU; unknown deviceStat schemas do not establish NPU-only execution."}


def parse_ort_profile(path: Path) -> dict:
    events = json.loads(path.read_text())
    nodes: dict[tuple[str, str], dict] = {}
    for event in events:
        args = event.get("args", {})
        provider = args.get("provider")
        if event.get("cat") == "Node" and provider:
            name = event.get("name", "unnamed")
            nodes[(name, provider)] = {"node": name, "provider": provider,
                                      "op_name": args.get("op_name"),
                                      "kind": "NPU" if provider == NPU_EP else "CPU" if "CPU" in provider else "OTHER"}
    assignments = list(nodes.values())
    return {"path": str(path), "node_assignments": assignments,
            "schema_recognized": bool(assignments),
            "cpu_node_count": sum(n["kind"] == "CPU" for n in assignments),
            "npu_node_count": sum(n["kind"] == "NPU" for n in assignments),
            "other_node_count": sum(n["kind"] == "OTHER" for n in assignments)}


def placement_node_counts(amd: dict, ort: dict) -> dict:
    """Keep compiler/logical and executed-runtime graph denominators separate.

    Fusion commonly turns many AMD logical nodes into one ORT EP node, so the
    two node counts must never be added or compared numerically. Public counts
    use actual executed ORT Node assignments when available. Both sources still
    independently constrain the fail-closed NPU-only decision.
    """
    amd_cpu = amd.get("cpu_node_count", 0)
    amd_npu = amd.get("npu_node_count", 0)
    ort_cpu = ort.get("cpu_node_count", 0)
    ort_npu = ort.get("npu_node_count", 0)
    ort_other = ort.get("other_node_count", 0)
    amd_total = amd_cpu + amd_npu
    ort_total = ort_cpu + ort_npu + ort_other
    amd_known = bool(amd.get("schema_recognized")) and amd_total > 0
    ort_known = not ort.get("error") and ort_total > 0 and ort.get("schema_recognized", True)
    logical = {"recognized": amd_known, "cpu_node_count": amd_cpu,
               "npu_node_count": amd_npu, "node_count": amd_total,
               "cpu_node_fraction": amd_cpu / amd_total if amd_known else None,
               "count_basis": "AMD deviceStat logical/compiler graph nodes"}
    runtime = {"recognized": bool(ort_known), "cpu_node_count": ort_cpu,
               "npu_node_count": ort_npu, "other_node_count": ort_other,
               "node_count": ort_total,
               "cpu_node_fraction": ort_cpu / ort_total if ort_known else None,
               "count_basis": "ORT executed Node assignments, unique (node name, provider) in untimed validation"}
    both_known = amd_known and ort_known
    cpu_disagrees = both_known and bool(amd_cpu) != bool(ort_cpu)
    npu_disagrees = both_known and bool(amd_npu) != bool(ort_npu)
    disagreement = bool(cpu_disagrees or npu_disagrees)
    if ort_known:
        cpu, total, fraction, basis = ort_cpu, ort_total, runtime["cpu_node_fraction"], runtime["count_basis"]
    elif amd_known:
        cpu, total, fraction = amd_cpu, amd_total, logical["cpu_node_fraction"]
        basis = "AMD logical/compiler graph only; ORT executed-runtime assignment unverified"
    else:
        cpu, total, fraction, basis = None, None, None, "unavailable: no recognized graph assignment counts"
    return {"cpu_node_count": cpu, "node_count": total, "cpu_node_fraction": fraction,
            "count_basis": basis, "amd_logical_node_counts": logical,
            "ort_runtime_node_counts": runtime,
            "placement_report_disagreement": disagreement,
            "placement_report_comparison": {
                "both_sources_recognized": bool(both_known),
                "cpu_presence_disagrees": bool(cpu_disagrees),
                "npu_presence_disagrees": bool(npu_disagrees),
                "note": "Different graph levels; counts are never summed. Fusion count differences alone are expected; CPU/NPU presence disagreement prevents an NPU-only claim."}}


def compiler_cache_evidence(cache_root: Path | None, extra_root: Path | None = None) -> dict:
    evidence: dict = {"contexts": [], "offload_summaries": [], "hardware_runner_proven": False,
                      "compiler_cpu_offload_zero_proven": False}
    if not cache_root:
        return evidence
    for path in cache_root.glob("**/context.json"):
        value = json.loads(path.read_text())
        groups = value.get("metaDef", [])
        runners = [{"subgraph": group.get("id"), "device": group.get("device"),
                    "runner_type": group.get("vaimlParam", {}).get("runnerType"),
                    "expected_final_summary": str(Path(group.get("vaimlParam", {}).get("vaimlModelPath", str(path.parent / str(group.get("id"))))) / "fail_safe_summary.json")}
                   for group in groups if str(group.get("device", "")).upper() == "VAIML"]
        if runners:
            evidence["contexts"].append({"path": str(path), "sha256": file_sha256(path), "vaiml_runners": runners})
    roots = [cache_root] + ([extra_root] if extra_root else [])
    for root in roots:
        for path in root.glob("**/fail_safe_summary.json"):
            value = json.loads(path.read_text())
            maps = []
            def walk(item: Any) -> None:
                if isinstance(item, dict):
                    for key, child in item.items():
                        if key == "offload_map" and isinstance(child, dict):
                            maps.append(child)
                        else:
                            walk(child)
                elif isinstance(item, list):
                    for child in item:
                        walk(child)
            walk(value)
            if maps:
                source_metadata = json.loads((path.parent / "source.json").read_text()) if (path.parent / "source.json").is_file() else {}
                evidence["offload_summaries"].append({"path": str(path), "sha256": file_sha256(path), "offload_maps": maps,
                                                       "original_path": source_metadata.get("original_path", str(path)),
                                                       "captured_utc": source_metadata.get("captured_utc", dt.datetime.fromtimestamp(path.stat().st_mtime, dt.timezone.utc).isoformat())})
    runners = [runner for item in evidence["contexts"] for runner in item["vaiml_runners"]]
    evidence["hardware_runner_proven"] = bool(runners) and all(item["runner_type"] == "hw" for item in runners)
    coverage = []
    for runner in runners:
        matches = [item for item in evidence["offload_summaries"]
                   if Path(item["original_path"]).resolve() == Path(runner["expected_final_summary"]).resolve()]
        # A partition's final summary, rather than a front-end support summary,
        # is necessary for every actual VAIML hardware runner.
        latest = max(matches, key=lambda item: item["captured_utc"]) if matches else None
        zero_cpu = bool(latest) and all(
            any(_label_kind(label) == "NPU" and isinstance(count, (int, float)) and count > 0 for label, count in mapping.items())
            and any(_label_kind(label) == "CPU" for label in mapping)
            and all(count == 0 for label, count in mapping.items() if _label_kind(label) == "CPU")
            for mapping in latest["offload_maps"])
        coverage.append({"subgraph": runner["subgraph"], "expected_final_summary": runner["expected_final_summary"],
                         "matched_summary": latest["path"] if latest else None,
                         "explicit_cpu_zero": zero_cpu})
    evidence["final_subgraph_summary_coverage"] = coverage
    evidence["compiler_cpu_offload_zero_proven"] = bool(coverage) and all(item["explicit_cpu_zero"] for item in coverage)
    return evidence


def audit_session(session: Any, report_path: Path, profile_path: Path | None,
                  cache_root: Path | None = None, compiler_root: Path | None = None,
                  requested_options: dict | None = None, option_files: list | None = None) -> dict:
    amd = parse_amd_report(json.loads(report_path.read_text())) if report_path.is_file() else {
        "schema_recognized": False, "cpu_node_count": 0, "npu_node_count": 0, "error": "AMD report absent"}
    ort = parse_ort_profile(profile_path) if profile_path and profile_path.is_file() else {
        "cpu_node_count": 0, "npu_node_count": 0, "other_node_count": 0, "error": "ORT profile absent"}
    placement = placement_node_counts(amd, ort)
    compiler = compiler_cache_evidence(cache_root, compiler_root)
    if profile_path:
        for item in compiler["offload_summaries"]:
            source = Path(item["path"])
            # Equal summaries for different subgraphs retain different source
            # records, so coverage survives export outside the original cache.
            snapshot_id = hashlib.sha256((item["original_path"] + ":" + item["sha256"]).encode()).hexdigest()[:24]
            destination = profile_path.parent / "compiler_summaries" / snapshot_id / "fail_safe_summary.json"
            destination.parent.mkdir(parents=True, exist_ok=True)
            if source.resolve() != destination.resolve():
                shutil.copy2(source, destination)
            item["snapshot_path"] = str(destination)
            if (source.parent / "source.json").is_file():
                shutil.copy2(source.parent / "source.json", destination.parent / "source.json")
    uses_vaiml = (bool(compiler["contexts"])
                  or any(str(item["device"]).upper() == "VAIML" for item in amd.get("device_stats", []))
                  or str((requested_options or {}).get("target", "")).upper() == "VAIML"
                  or any("VAIML" in json.dumps(item.get("json_contents", {})).upper() for item in (option_files or [])))
    proven = (amd.get("schema_recognized", False) and amd.get("npu_node_count", 0) > 0
              and amd.get("cpu_node_count", 0) == 0
              and placement["ort_runtime_node_counts"]["recognized"]
              and ort.get("npu_node_count", 0) > 0 and ort.get("cpu_node_count", 0) == 0
              and ort.get("other_node_count", 0) == 0
              and not placement["placement_report_disagreement"])
    if uses_vaiml:
        proven = proven and compiler["hardware_runner_proven"] and compiler["compiler_cpu_offload_zero_proven"]
    maps = read_text("/proc/self/maps") if platform.system() == "Linux" else None
    runtime_libraries = sorted(set(line.split()[-1] for line in (maps or "").splitlines()
                                   if any(token in line.lower() for token in ("xrt", "xdna", "vaip", "flexml", "onnxruntime", "voe"))
                                   and line.split()[-1].startswith("/")))
    library_maps_path = None
    if maps and profile_path:
        library_maps_path = profile_path.with_suffix(".loaded_npu_libraries.txt")
        library_maps_path.write_text("\n".join(line for line in maps.splitlines()
                                              if any(token in line.lower() for token in ("xrt", "xdna", "vaip", "flexml", "onnxruntime", "voe"))) + "\n")
    return {"providers": session.get_providers(), "provider_options": session.get_provider_options(),
            "amd_report_path": str(report_path), "amd": amd, "ort": ort,
            **placement,
            "compiler": compiler, "vaiml_requires_hardware_runner_and_zero_internal_cpu": uses_vaiml,
            "loaded_runtime_libraries": runtime_libraries,
            "loaded_runtime_maps_evidence": str(library_maps_path) if library_maps_path else None,
            "npu_only_proven": proven}


def input_metadata(spec: dict) -> dict:
    inputs = spec.get("inputs", [])
    if isinstance(inputs, dict):
        return {name: value if isinstance(value, dict) else {"shape": value} for name, value in inputs.items()}
    return {item["name"]: item for item in inputs}


def make_feed(session: Any, spec: dict, np: Any, rng: Any, seq: int = 968) -> dict:
    types = {"tensor(float)": "float32", "tensor(float16)": "float16", "tensor(bfloat16)": None,
             "tensor(int8)": "int8", "tensor(uint8)": "uint8", "tensor(int64)": "int64",
             "tensor(int32)": "int32", "tensor(bool)": "bool"}
    overrides = input_metadata(spec)
    result = {}
    for item in session.get_inputs():
        override = overrides.get(item.name, {})
        shape = item.shape
        if any(not isinstance(d, int) or d <= 0 for d in shape):
            raise BenchmarkError(f"Dynamic/nonpositive input shape forbidden: {item.name}={shape}")
        if override.get("shape") is not None and list(override["shape"]) != list(shape):
            raise BenchmarkError(f"Manifest/session shape mismatch for {item.name}")
        dtype = types.get(item.type)
        if dtype is None:
            raise BenchmarkError(f"Unsupported host input dtype {item.type}; supply an explicit supported model")
        if "value" in override:
            array = np.array(override["value"], dtype=dtype)
            array = np.broadcast_to(array, shape).copy()
        elif "fill" in override:
            array = np.full(shape, override["fill"], dtype=dtype)
        elif "position" in item.name:
            array = np.broadcast_to(np.arange(seq, seq + shape[-1], dtype=dtype), shape).copy()
        elif "bias" in item.name or "mask" in item.name or item.name.startswith("kv_"):
            array = np.zeros(shape, dtype=dtype)
        elif "step_size" in item.name:
            array = np.full(shape, 0.1, dtype=dtype)
        elif "timestep" in item.name:
            array = np.ones(shape, dtype=dtype)
        elif dtype.startswith("int") or dtype.startswith("uint") or dtype == "bool":
            array = np.zeros(shape, dtype=dtype)
        else:
            array = rng.standard_normal(shape).astype(dtype) * 0.02
        result[item.name] = np.ascontiguousarray(array)
    return result


def provider_option_file_evidence(options: dict) -> list[dict]:
    evidence = []
    for key, value in options.items():
        if isinstance(value, str) and Path(value).is_file():
            item = {"option": key, "path": str(Path(value).resolve()), "sha256": file_sha256(Path(value))}
            if Path(value).suffix == ".json":
                item["json_contents"] = json.loads(Path(value).read_text())
            evidence.append(item)
    return evidence


def effective_stage_ep_options(args: Any, spec: dict) -> tuple[dict, list[dict]]:
    overrides = spec.get("ep_options", {})
    if not isinstance(overrides, dict):
        raise BenchmarkError("manifest stage ep_options must be a JSON object")
    effective = dict(args.ep_options)
    effective.update(overrides)
    if any(key in effective for key in ("cache_dir", "cache_key", "enable_cache_file_io_in_mem")):
        raise BenchmarkError("Cache options are owned by the benchmark; use --cache-dir")
    return effective, provider_option_file_evidence(effective)


def open_session(ort: Any, spec: dict, name: str, args: Any, evidence_dir: Path,
                 allow_cpu: bool, profile: bool) -> tuple[Any, dict, Path]:
    model_path = spec["path"]
    effective_options, effective_option_files = effective_stage_ep_options(args, spec)
    digest = file_sha256(model_path)
    # External tensors affect cache identity and must be recorded.
    external = []
    external_paths = set(model_path.parent.glob(model_path.name + ".*"))
    declared_external = spec.get("external_data", spec.get("external_data_file", []))
    if isinstance(declared_external, str):
        declared_external = [declared_external]
    for entry in declared_external:
        location = entry.get("file", entry.get("location")) if isinstance(entry, dict) else entry
        if location:
            external_paths.add((model_path.parent / location).resolve())
    for file in sorted(external_paths):
        if file.is_file():
            external.append({"file": str(file), "sha256": file_sha256(file)})
    identity = hashlib.sha256(json.dumps({"onnx": digest, "external": external, "ort": ort.__version__,
                                         "fingerprint": args.fingerprint_hash, "options": effective_options,
                                         "provider_option_files": effective_option_files}, sort_keys=True).encode()).hexdigest()[:20]
    cache_dir = args.cache_dir.resolve() / name / identity
    existed = cache_dir.exists() and any(cache_dir.iterdir())
    cache_dir.mkdir(parents=True, exist_ok=True)
    cache_key = f"pi05_{name}_{identity}"
    # AMD often writes this only while compiling. A fixed cache-associated path
    # survives subsequent cache hits; deleting it would discard placement proof.
    report = cache_dir / cache_key / "vitisai_ep_report.json"
    report.parent.mkdir(parents=True, exist_ok=True)
    os.environ["XLNX_ONNX_EP_REPORT_FILE"] = str(report)
    opts = ort.SessionOptions()
    opts.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
    if not allow_cpu:
        opts.add_session_config_entry("session.disable_cpu_ep_fallback", "1")
    opts.log_severity_level = 2
    opts.enable_profiling = profile
    if profile:
        opts.profile_file_prefix = str(evidence_dir / (name + "_ort_profile"))
    provider_options = {"cache_dir": str(cache_dir), "cache_key": cache_key,
                        "enable_cache_file_io_in_mem": "0"}
    provider_options.update(effective_options)
    providers = [(NPU_EP, provider_options)]
    if allow_cpu:
        providers.append("CPUExecutionProvider")
    start = time.perf_counter_ns()
    watcher = CompilerEvidenceWatcher(report.parent)
    watcher.start()
    try:
        session = ort.InferenceSession(str(model_path), sess_options=opts, providers=providers,
                                       enable_fallback=False)
    finally:
        watcher_metadata = watcher.stop()
    elapsed = (time.perf_counter_ns() - start) / 1e6
    session.disable_fallback()
    if NPU_EP not in session.get_providers():
        raise BenchmarkError(f"{name}: VitisAI EP missing from created session: {session.get_providers()}")
    report_snapshot = evidence_dir / f"{name}_{'cached' if profile else 'initial'}_amd_ep_report.json"
    if report.is_file():
        shutil.copy2(report, report_snapshot)
    metadata = {"model": str(model_path), "model_sha256": digest, "external_data": external,
                "session_create_ms": elapsed, "cache_dir": str(cache_dir),
                "cache_key": provider_options["cache_key"], "cache_nonempty_before_create": existed,
                "provider_options_requested": provider_options,
                "provider_option_files": effective_option_files,
                "manifest_stage_ep_options": spec.get("ep_options", {}),
                "host_inputs": [{"name": item.name, "shape": item.shape, "type": item.type} for item in session.get_inputs()],
                "host_outputs": [{"name": item.name, "shape": item.shape, "type": item.type} for item in session.get_outputs()],
                "amd_ep_report_cache_path": str(report), "amd_ep_report_snapshot": str(report_snapshot),
                "compiler_evidence_watcher": watcher_metadata,
                "disable_cpu_ep_fallback": not allow_cpu,
                "note": "Creation times are single observations, not warmed p50/p95 measurements."}
    return session, metadata, report


def run_proxy(args: Any, result: dict) -> None:
    ort = importlib.import_module("onnxruntime")
    np = importlib.import_module("numpy")
    result["environment"]["ort_version"] = ort.__version__
    result["environment"]["available_providers"] = ort.get_available_providers()
    result["environment"]["ort_build_info"] = ort.get_build_info() if hasattr(ort, "get_build_info") else "unknown"
    if NPU_EP not in ort.get_available_providers():
        raise BenchmarkError(f"Required {NPU_EP} unavailable; no CPU run attempted")
    if args.model:
        if args.mode != "ep-report":
            raise BenchmarkError("--model is reserved for single-file --mode ep-report audits")
        evidence_dir = args.output.parent / (args.output.stem + "_evidence")
        evidence_dir.mkdir(parents=True, exist_ok=True)
        session, metadata, report = open_session(ort, {"path": args.model.resolve()}, "single_model",
                                               args, evidence_dir, True, True)
        result["sessions"] = {"single_model": {"creation": metadata}}
        feed = make_feed(session, {}, np, np.random.default_rng(args.seed))
        session.run(None, feed)
        profile = Path(session.end_profiling())
        audit = audit_session(session, report, profile, Path(metadata["cache_dir"]), args.compiler_evidence_dir,
                              metadata["provider_options_requested"], metadata["provider_option_files"])
        result["sessions"]["single_model"]["ep_audit"] = audit
        result["npu_only_proven"] = audit["npu_only_proven"]
        result["status"] = "ok" if audit["npu_only_proven"] else "fallback_or_unverified"
        result["performance"] = None
        result["measurement_scope"] = "Placement audit only; one untimed validation, zero warmup/samples; not a performance benchmark"
        result["warning"] = None if audit["npu_only_proven"] else "CPU or unknown assignment; not a pure NPU result"
        for node in audit["amd"].get("node_assignments", []) + audit["ort"].get("node_assignments", []):
            print(json.dumps({"assignment": node}, ensure_ascii=False))
        return
    manifest, specs = load_manifest(args.model_dir)
    result["manifest"] = {"path": str(args.model_dir / "manifest.json"),
                          "sha256": file_sha256(args.model_dir / "manifest.json"), "contents": manifest}
    evidence_dir = args.output.parent / (args.output.stem + "_evidence")
    evidence_dir.mkdir(parents=True, exist_ok=True)
    result["evidence_directory"] = str(evidence_dir)
    result["sessions"] = {}
    sessions, feeds, reports = {}, {}, {}
    rng = np.random.default_rng(args.seed)
    selected_stages = (args.only_stage,) if args.only_stage else STAGES
    for name in selected_stages:
        result["sessions"][name] = {}
        if args.mode == "proxy":
            first, first_metadata, _ = open_session(ort, specs[name], name, args, evidence_dir, False, False)
            result["sessions"][name]["initial_creation"] = first_metadata
            write_json(args.output, result)
            del first
            gc.collect()
        session, metadata, report = open_session(ort, specs[name], name, args, evidence_dir,
                                               args.mode == "ep-report", True)
        result["sessions"][name]["cached_creation" if args.mode == "proxy" else "creation"] = metadata
        write_json(args.output, result)
        sessions[name], reports[name] = session, report
        feeds[name] = make_feed(session, specs[name], np, rng)
        # Compile and audit a separate untimed validation call. Profiling ends
        # before warmup, removing profile-event overhead from measured samples.
        session.run(None, feeds[name])
        profile = Path(session.end_profiling())
        audit = audit_session(session, report, profile, Path(metadata["cache_dir"]), args.compiler_evidence_dir,
                              metadata["provider_options_requested"], metadata["provider_option_files"])
        result["sessions"][name]["ep_audit"] = audit
        for node in audit["amd"].get("node_assignments", []) + audit["ort"].get("node_assignments", []):
            print(json.dumps({"stage": name, "assignment": node}, ensure_ascii=False))
        write_json(args.output, result)
        if args.mode == "proxy" and not audit["npu_only_proven"]:
            raise BenchmarkError(f"{name}: pure NPU execution not proven (CPU={audit['cpu_node_count']}); inspect AMD and ORT reports")
    all_npu = all(result["sessions"][name]["ep_audit"]["npu_only_proven"] for name in selected_stages)
    result["npu_only_proven"] = all_npu
    if args.mode == "ep-report":
        result["status"] = "ok" if all_npu else "fallback_or_unverified"
        result["performance"] = None
        result["warning"] = None if all_npu else "Placement includes CPU or is unverified; never report this as pure NPU performance."
        return

    if args.only_stage:
        name = args.only_stage
        session, feed = sessions[name], feeds[name]
        result["measurement_scope"] = f"standalone {name} stage call, not a complete pi0.5 block; expert is one denoising step"
        result["performance_scope"] = "stage:" + name
        result["warmup_raw"], result["samples_raw"] = [], []
        for index in range(args.warmup + args.samples):
            start = time.perf_counter_ns()
            session.run(None, feed)
            elapsed = (time.perf_counter_ns() - start) / 1e6
            result["warmup_raw" if index < args.warmup else "samples_raw"].append({"stage_ms": elapsed})
            if index % 5 == 4:
                write_json(args.output, result)
        result["performance"] = {"stage": summarize_ms([row["stage_ms"] for row in result["samples_raw"]], specs[name]["flops_per_call"])}
        result["status"] = "ok"
        return

    # Preserve batch=3 vision and unsliced sequence=968 backbone. The random
    # embedding-stage inputs are independent by design; this is a shape proxy.
    initial_x = feeds["expert"].get("x_t")
    if initial_x is None:
        raise BenchmarkError("Expert proxy must expose x_t")
    initial_x = initial_x.copy()
    expert_outputs = [item.name for item in sessions["expert"].get_outputs()]
    if "next_x" not in expert_outputs:
        raise BenchmarkError("Expert proxy must output next_x")
    next_x_index = expert_outputs.index("next_x")
    backbone_outputs = [item.name for item in sessions["backbone"].get_outputs()]
    kv_names = [name for name in feeds["expert"] if name.startswith("kv_")]
    if len(kv_names) != 36 or any(name not in backbone_outputs for name in kv_names):
        raise BenchmarkError("Expert must consume the 36 same-named KV tensors returned by backbone")

    def one_block() -> dict:
        start = time.perf_counter_ns()
        vision_values = sessions["vision"].run(None, feeds["vision"])
        after_vision = time.perf_counter_ns()
        bridge_start = time.perf_counter_ns()
        vision_names = [item.name for item in sessions["vision"].get_outputs()]
        if "vision_hidden" in vision_names and "tokens" in feeds["backbone"]:
            vision_hidden = vision_values[vision_names.index("vision_hidden")]
            image_tokens = vision_hidden.reshape(1, -1, vision_hidden.shape[-1])
            if image_tokens.shape[2] != feeds["backbone"]["tokens"].shape[2]:
                raise BenchmarkError("Vision projection/backbone embedding dimensions differ")
            feeds["backbone"]["tokens"][:, :image_tokens.shape[1], :] = image_tokens
        bridge_ns = time.perf_counter_ns() - bridge_start
        outputs = sessions["backbone"].run(None, feeds["backbone"])
        after_backbone = time.perf_counter_ns()
        kv = dict(zip(backbone_outputs, outputs))
        feeds["expert"].update({name: kv[name] for name in kv_names})
        feeds["expert"]["x_t"] = initial_x.copy()
        steps = []
        host_update_ns = time.perf_counter_ns() - after_backbone
        for index in range(10):
            before_update = time.perf_counter_ns()
            if "timestep" in feeds["expert"]:
                feeds["expert"]["timestep"].fill(1 - index / 10)
            host_update_ns += time.perf_counter_ns() - before_update
            before_expert = time.perf_counter_ns()
            output = sessions["expert"].run(None, feeds["expert"])
            after_expert = time.perf_counter_ns()
            steps.append((after_expert - before_expert) / 1e6)
            before_update = time.perf_counter_ns()
            feeds["expert"]["x_t"] = output[next_x_index]
            host_update_ns += time.perf_counter_ns() - before_update
        end = time.perf_counter_ns()
        return {"vision_ms": (after_vision - start) / 1e6,
                "backbone_ms": (after_backbone - after_vision - bridge_ns) / 1e6,
                "vision_to_backbone_host_bridge_ms": bridge_ns / 1e6,
                "expert_steps_ms": steps, "expert_ms": sum(steps),
                "host_python_update_ms": host_update_ns / 1e6,
                "total_ms": (end - start) / 1e6}

    result["timing_scope"] = {
        "clock": "time.perf_counter_ns", "boundary": "synchronous ORT session.run; includes EP transfers and synchronization",
        "host_python_update_ms": "Python feed/timestep/x_t bookkeeping only; NOT a measurement of EP memcpy or sync",
        "vision_backbone_link": "If vision_hidden is exposed, its projected batch=3 output overwrites the first 768 backbone tokens; otherwise independent synthetic embeddings",
        "expert_loop": "10 serial host calls, backbone KV reused as host arrays; no device residency claim",
        "profiling": "one untimed validation per session; profiling disabled before benchmark warmup",
    }
    result["warmup_raw"] = []
    result["samples_raw"] = []
    for index in range(args.warmup):
        result["warmup_raw"].append(one_block())
        if index % 5 == 4:
            write_json(args.output, result)
    for index in range(args.samples):
        result["samples_raw"].append(one_block())
        write_json(args.output, result)
    raw = result["samples_raw"]
    total_flops = specs["vision"]["flops_per_call"] + specs["backbone"]["flops_per_call"] + 10 * specs["expert"]["flops_per_call"]
    result["performance"] = {
        "total": summarize_ms([row["total_ms"] for row in raw], total_flops),
        "vision": summarize_ms([row["vision_ms"] for row in raw], specs["vision"]["flops_per_call"]),
        "backbone": summarize_ms([row["backbone_ms"] for row in raw], specs["backbone"]["flops_per_call"]),
        "expert_10_steps": summarize_ms([row["expert_ms"] for row in raw], 10 * specs["expert"]["flops_per_call"]),
        "expert_per_step": [summarize_ms([row["expert_steps_ms"][i] for row in raw], specs["expert"]["flops_per_call"]) for i in range(10)],
        "host_python_update": summarize_ms([row["host_python_update_ms"] for row in raw]),
        "vision_to_backbone_host_bridge": summarize_ms([row["vision_to_backbone_host_bridge_ms"] for row in raw]),
    }
    result["performance_scope"] = "full_block"
    result["acceptance"] = {
        "warmup_at_least_20": len(result["warmup_raw"]) >= 20,
        "samples_at_least_20": len(raw) >= 20,
        "fallback_nodes_zero_proven": all_npu,
        "cached_session_creation_under_10_s": all(result["sessions"][name]["cached_creation"]["session_create_ms"] < 10000 for name in STAGES),
        "target_effective_tflops_at_least_8": result["performance"]["total"]["effective_tflops"] >= 8,
    }
    result["status"] = "ok"


def parse_flm_json(stdout: str, tokens: int) -> dict:
    try:
        value = json.loads(stdout)
    except json.JSONDecodeError:
        candidates = [line for line in stdout.splitlines() if line.strip().startswith("{")]
        if not candidates:
            raise BenchmarkError("FLM command must emit a JSON measurement object")
        value = json.loads(candidates[-1])
    device = str(value.get("device", value.get("backend", ""))).lower()
    if not any(name in device for name in ("npu", "xdna", "fastflowlm", "flm")) or "cpu" in device:
        raise BenchmarkError("FLM JSON must identify the actual NPU backend/device; CPU or unknown is rejected")
    count = value.get("prefill_tokens", value.get("prompt_eval_count"))
    seconds = value.get("prefill_seconds")
    if seconds is None and "prompt_eval_duration" in value:
        seconds = value["prompt_eval_duration"] / 1e9
    if count is None or int(count) != tokens or not isinstance(seconds, (int, float)) or seconds <= 0:
        raise BenchmarkError("FLM JSON needs exact prefill_tokens and prefill_seconds (or prompt_eval_count/duration in ns)")
    return {"prefill_tokens": int(count), "prefill_seconds": seconds, "prefill_tok_s": count / seconds,
            "source_json": value}


def run_flm(args: Any, result: dict) -> None:
    if not args.flm_command_template or "{tokens}" not in args.flm_command_template:
        raise BenchmarkError("FLM retest disabled: supply --flm-command-template with {tokens}, using an already installed local model; no downloads are performed")
    known = {1024: 766.4, 2048: 892.9, 4096: 927.7}
    result["known_baseline_source"] = "User-supplied FastFlowLM 1.0.10 Llama 3.2 3B Q4_1; accepted evidence, independent of this run"
    result["flm_results"] = {}
    for tokens, baseline in known.items():
        entry = {"warmup_raw": [], "samples_raw": [], "command": shlex.split(args.flm_command_template.format(tokens=tokens))}
        result["flm_results"][str(tokens)] = entry
        for index in range(args.warmup + args.samples):
            proc = subprocess.run(entry["command"], capture_output=True, text=True, timeout=args.command_timeout, check=False)
            if proc.returncode:
                entry["failed_command"] = {"returncode": proc.returncode, "stdout": proc.stdout, "stderr": proc.stderr}
                raise BenchmarkError(f"FLM command failed for {tokens} tokens")
            measured = parse_flm_json(proc.stdout, tokens)
            entry["warmup_raw" if index < args.warmup else "samples_raw"].append(measured)
            write_json(args.output, result)
        speeds = [row["prefill_tok_s"] for row in entry["samples_raw"]]
        entry["latency"] = summarize_ms([row["prefill_seconds"] * 1000 for row in entry["samples_raw"]])
        entry["prefill_tok_s_p50"] = statistics.median(speeds)
        entry["prefill_tok_s_p95"] = percentile(speeds, 0.95)
        entry["known_prefill_tok_s"] = baseline
        entry["relative_difference"] = entry["prefill_tok_s_p50"] / baseline - 1
        entry["within_five_percent"] = abs(entry["relative_difference"]) <= 0.05
    result["status"] = "ok"
    result["performance"] = None
    result["note"] = "No FLOPs conversion without exact model-operation manifest; throughput is real subprocess JSON evidence."


def apply_ab(result: dict, args: Any) -> None:
    baseline_path = args.compare_to
    if not baseline_path:
        return
    baseline = json.loads(baseline_path.read_text())
    current = result.get("performance")
    previous = baseline.get("performance")
    if not current or not previous:
        result["ab_comparison"] = {"status": "unavailable", "reason": "Both runs need measured pure-NPU proxy performance"}
        return
    if not baseline.get("npu_only_proven") or not result.get("npu_only_proven"):
        raise BenchmarkError("A/B cannot compare unproven NPU performance")
    if baseline.get("performance_scope") != result.get("performance_scope"):
        raise BenchmarkError("A/B requires identical full-block or standalone-stage scope")
    timing_key = "total" if "total" in current else "stage"
    if current[timing_key]["algorithmic_flops"] != previous[timing_key]["algorithmic_flops"]:
        raise BenchmarkError("A/B requires equal algorithmic FLOPs")
    if baseline.get("status") != "ok" or min(previous[timing_key].get("n", 0), current[timing_key].get("n", 0)) < 20:
        raise BenchmarkError("A/B requires successful baselines with at least 20 measured samples")
    env_old, env_new = baseline.get("environment", {}), result.get("environment", {})
    volatile = {"timestamp", "timestamp_utc", "captured_at", "supplied_fingerprint"}
    environment_changes = {key: {"baseline": env_old.get(key), "candidate": env_new.get(key)}
                           for key in set(env_old) | set(env_new)
                           if key not in volatile and env_old.get(key) != env_new.get(key)}
    old, new = previous[timing_key]["p50_ms"], current[timing_key]["p50_ms"]
    gain = old / new - 1
    result["ab_comparison"] = {"baseline": str(baseline_path.resolve()), "baseline_sha256": file_sha256(baseline_path),
                               "baseline_p50_ms": old, "candidate_p50_ms": new,
                               "throughput_gain_fraction": gain, "latency_reduction_fraction": 1 - new / old,
                               "adopt": gain >= args.minimum_gain,
                               "minimum_gain_fraction": args.minimum_gain,
                               "decision": "采纳候选" if gain >= args.minimum_gain else "无效：回滚候选",
                               "single_changed_variable": args.changed_variable,
                               "environment_changes_to_review": environment_changes,
                               "note": "Decision only: this benchmark never edits or rolls back model files automatically."}


def parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--mode", required=True, choices=("proxy", "ep-report", "flm"))
    p.add_argument("--model-dir", type=Path, default=ROOT / "bench" / "proxy")
    p.add_argument("--model", type=Path, help="Single static ONNX file for --mode ep-report only; no benchmark timing")
    p.add_argument("--only-stage", choices=STAGES,
                   help="Audit or benchmark one standalone stage; explicitly NOT complete block performance")
    p.add_argument("--fingerprint", type=Path, help="P0 environment JSON; unknown fields remain explicit")
    p.add_argument("--output", type=Path, help="Default: results/<stage>_<optimization>.json")
    p.add_argument("--stage", default="P1")
    p.add_argument("--optimization", default="baseline")
    p.add_argument("--warmup", type=int, default=20)
    p.add_argument("--samples", type=int, default=20)
    p.add_argument("--seed", type=int, default=20261030)
    p.add_argument("--cache-dir", type=Path, default=ROOT.parent / "work" / "npu-cache")
    p.add_argument("--ep-options", type=Path, help="JSON object of explicit VitisAI provider options")
    p.add_argument("--compiler-evidence-dir", type=Path,
                   help="This run's compiler evidence root, containing fail_safe_summary.json with offload_map (required for VAIML internal CPU audit)")
    p.add_argument("--flm-command-template", help="Local executable argv template with {tokens}; emits measured JSON")
    p.add_argument("--command-timeout", type=float, default=600)
    p.add_argument("--compare-to", type=Path, help="Baseline results JSON; evaluates gain and adoption")
    p.add_argument("--changed-variable", help="Exactly one changed variable, required for --compare-to")
    p.add_argument("--minimum-gain", type=float, default=0.03, help="A/B acceptance fraction; default 0.03")
    return p


def main(argv: list[str] | None = None) -> int:
    args = parser().parse_args(argv)
    args.output = (args.output or ROOT / "results" / f"{args.stage}_{args.optimization}.json").resolve()
    result = {"schema_version": 1, "mode": args.mode, "stage": args.stage,
              "optimization": args.optimization, "status": "running", "started_utc": utc_now(),
              "command": [sys.executable, *sys.argv], "warmup_requested": args.warmup,
              "samples_requested": args.samples, "seed": args.seed, "environment": {},
              "performance": None, "npu_only_proven": False}
    try:
        result["environment"] = environment_fingerprint(args.fingerprint)
        args.fingerprint_hash = hashlib.sha256(json.dumps(result["environment"], sort_keys=True).encode()).hexdigest()[:16]
        # Time-varying timestamps must not invalidate a cache; the source P0
        # fingerprint plus runtime versions/options determine reusable identity.
        stable_fingerprint = {key: value for key, value in result["environment"].items() if key not in ("timestamp_utc", "timestamp", "captured_at")}
        args.fingerprint_hash = hashlib.sha256(json.dumps(stable_fingerprint, sort_keys=True).encode()).hexdigest()[:16]
        if args.warmup < 20 or args.samples < 20:
            raise BenchmarkError("warmup >= 20 and samples >= 20 are mandatory")
        if args.compare_to and not args.changed_variable:
            raise BenchmarkError("--compare-to requires --changed-variable describing exactly one changed variable")
        if args.minimum_gain < 0.03:
            raise BenchmarkError("Single-variable gain below 3% is ineffective; --minimum-gain cannot be lower than .03")
        args.ep_options = json.loads(args.ep_options.read_text()) if args.ep_options else {}
        if not isinstance(args.ep_options, dict):
            raise BenchmarkError("--ep-options must contain an object")
        if any(key in args.ep_options for key in ("cache_dir", "cache_key", "enable_cache_file_io_in_mem")):
            raise BenchmarkError("Cache options are owned by the benchmark; use --cache-dir")
        args.ep_option_files = provider_option_file_evidence(args.ep_options)
        write_json(args.output, result)
        if args.mode == "flm":
            run_flm(args, result)
        else:
            run_proxy(args, result)
        apply_ab(result, args)
        result["ended_utc"] = utc_now()
        write_json(args.output, result)
        print(json.dumps({"result": str(args.output), "status": result["status"],
                          "npu_only_proven": result["npu_only_proven"], "performance": result["performance"]}, ensure_ascii=False))
        return 0 if result["status"] == "ok" else 2
    except Exception as exc:
        result["status"] = "error"
        result["error"] = {"type": type(exc).__name__, "message": str(exc), "traceback": traceback.format_exc()}
        result["performance"] = None
        result["ended_utc"] = utc_now()
        write_json(args.output, result)
        print(f"ERROR: {exc}\nEvidence: {args.output}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
