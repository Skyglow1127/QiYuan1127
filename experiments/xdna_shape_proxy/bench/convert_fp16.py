#!/usr/bin/env python3
"""Stream the FP32 shape-proxy weights into an independent FP16 derivative.

No inference, NPU compilation, downloads or calibration occur. Learned values
are rounded from the preserved FP32 tensors, never resampled. ONNX operators,
layers, shapes and algorithmic GEMM FLOPs are preserved. Float32 graph-boundary
and intermediate types become float16; a Cast-to-float32 becomes Cast-to-float16
to keep its consumers type-consistent. LayerNormalization stash_type is left
untouched so its specified accumulation type remains float32.

Examples:
  python convert_fp16.py --smoke --stage vision
  python convert_fp16.py --stage all
"""
from __future__ import annotations

import argparse
from collections import Counter
import copy
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import sys
import time


STAGES = ("vision", "backbone", "expert")


def now():
    return datetime.now(timezone.utc).isoformat()


def sha256_file(path):
    h = hashlib.sha256()
    with path.open("rb") as f:
        for block in iter(lambda: f.read(4 * 1024 * 1024), b""):
            h.update(block)
    return h.hexdigest()


def write_json(path, value):
    tmp = path.with_suffix(path.suffix + ".building")
    tmp.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + "\n")
    tmp.replace(path)


class Converter:
    def __init__(self, source, output, chunk_elements):
        import numpy as np
        import onnx
        self.np, self.onnx = np, onnx
        self.source, self.output = source, output
        self.chunk_elements = chunk_elements
        self.output_data_name = output.name + ".data"
        self.output_data = output.parent / self.output_data_name
        self.output_tmp = self.output_data.with_suffix(".data.building")
        self.stream = self.output_tmp.open("wb")
        self.digest = hashlib.sha256()
        self.counts = Counter()
        self.converted_values = 0
        self.finite_before = True
        self.finite_after = True
        self.underflows_to_zero = 0
        self.max_abs_rounding_error = 0.0
        self.random_weight_bytes = 0

    def pad(self):
        padding = (-self.stream.tell()) % 64
        if padding:
            buf = bytes(padding)
            self.stream.write(buf)
            self.digest.update(buf)

    def record_conversion(self, src, dst):
        np = self.np
        if not np.isfinite(src).all():
            self.finite_before = False
        if not np.isfinite(dst).all():
            self.finite_after = False
        self.underflows_to_zero += int(np.count_nonzero((src != 0) & (dst == 0)))
        if src.size:
            err = np.max(np.abs(src - dst.astype(np.float32)))
            self.max_abs_rounding_error = max(self.max_abs_rounding_error, float(err))
        self.converted_values += int(src.size)

    def external_tensor(self, tensor):
        onnx, np = self.onnx, self.np
        info = {x.key: x.value for x in tensor.external_data}
        if "location" not in info:
            raise ValueError(f"External tensor lacks location: {tensor.name}")
        location = Path(info["location"])
        if location.is_absolute():
            raise ValueError(f"Expected portable relative tensor path: {location}")
        source_data = (self.source.parent / location).resolve()
        count = math.prod(tensor.dims)
        offset = int(info.get("offset", "0"))
        source_float = tensor.data_type == onnx.TensorProto.FLOAT
        itemsize = 4 if source_float else np.dtype(onnx.helper.tensor_dtype_to_np_dtype(tensor.data_type)).itemsize
        length = int(info.get("length", str(count * itemsize)))
        if length != count * itemsize:
            raise ValueError(f"Tensor length mismatch: {tensor.name} {length} != {count * itemsize}")
        self.pad()
        new_offset = self.stream.tell()
        with source_data.open("rb") as f:
            f.seek(offset)
            remaining = count
            while remaining:
                n = min(remaining, self.chunk_elements)
                raw = f.read(n * itemsize)
                if len(raw) != n * itemsize:
                    raise EOFError(f"Truncated external data: {tensor.name}")
                if source_float:
                    src = np.frombuffer(raw, dtype="<f4")
                    dst = src.astype("<f2")
                    self.record_conversion(src, dst)
                    raw = dst.tobytes()
                self.stream.write(raw)
                self.digest.update(raw)
                remaining -= n
        new_length = count * (2 if source_float else itemsize)
        result = onnx.TensorProto()
        result.CopyFrom(tensor)
        result.ClearField("external_data")
        if source_float:
            result.data_type = onnx.TensorProto.FLOAT16
            self.counts["external_float32_tensors_to_float16"] += 1
            self.random_weight_bytes += new_length
        else:
            self.counts["external_nonfloat_tensors_preserved"] += 1
        for k, v in (("location", self.output_data_name), ("offset", str(new_offset)), ("length", str(new_length))):
            entry = result.external_data.add()
            entry.key, entry.value = k, v
        return result

    def tensor(self, tensor):
        onnx = self.onnx
        if tensor.data_location == onnx.TensorProto.EXTERNAL:
            return self.external_tensor(tensor)
        if tensor.data_type == onnx.TensorProto.FLOAT:
            source_values = onnx.numpy_helper.to_array(tensor)
            converted = source_values.astype(self.np.float16)
            self.record_conversion(source_values, converted)
            result = onnx.numpy_helper.from_array(converted, tensor.name)
            if tensor.doc_string:
                result.doc_string = tensor.doc_string
            self.counts["inline_float32_tensors_to_float16"] += 1
            return result
        self.counts["inline_nonfloat_tensors_preserved"] += 1
        return tensor

    def graph(self, graph):
        onnx = self.onnx
        for field in ("input", "output", "value_info"):
            for value in getattr(graph, field):
                type_info = value.type.tensor_type
                if type_info.elem_type == onnx.TensorProto.FLOAT:
                    type_info.elem_type = onnx.TensorProto.FLOAT16
                    self.counts[f"{field}_float32_to_float16"] += 1
        for i, tensor in enumerate(graph.initializer):
            graph.initializer[i].CopyFrom(self.tensor(tensor))
        for node in graph.node:
            for attribute in node.attribute:
                if node.op_type == "Cast" and attribute.name == "to" and attribute.i == onnx.TensorProto.FLOAT:
                    attribute.i = onnx.TensorProto.FLOAT16
                    self.counts["cast_targets_float32_to_float16"] += 1
                elif attribute.type == onnx.AttributeProto.TENSOR:
                    attribute.t.CopyFrom(self.tensor(attribute.t))
                elif attribute.type == onnx.AttributeProto.TENSORS:
                    for i, tensor in enumerate(attribute.tensors):
                        attribute.tensors[i].CopyFrom(self.tensor(tensor))
                elif attribute.type == onnx.AttributeProto.GRAPH:
                    self.graph(attribute.g)
                elif attribute.type == onnx.AttributeProto.GRAPHS:
                    for inner in attribute.graphs:
                        self.graph(inner)

    def convert(self):
        onnx = self.onnx
        started = time.perf_counter()
        source_sha = sha256_file(self.source)
        model = onnx.load(str(self.source), load_external_data=False)
        old_ops = Counter(n.op_type for n in model.graph.node)
        old_nodes = [n.name for n in model.graph.node]
        self.graph(model.graph)
        assert old_ops == Counter(n.op_type for n in model.graph.node)
        assert old_nodes == [n.name for n in model.graph.node]
        if not self.finite_before or not self.finite_after:
            raise ValueError("Non-finite source or FP16 output found; refusing to publish derivative")
        self.stream.flush()
        self.stream.close()
        self.output_tmp.replace(self.output_data)
        props = {x.key: x.value for x in model.metadata_props}
        props.update({"precision": "FP16 reference rounded from preserved FP32 proxy",
                      "fp32_source_sha256": source_sha,
                      "flops_convention": "unchanged algorithmic GEMM 2 FLOPs/MAC; no speedup claim"})
        onnx.helper.set_model_props(model, props)
        model.doc_string = "Static FP16 derivative of independent-weight FP32 preembedded-token shape proxy; same nodes/shapes/GEMM FLOPs."
        tmp_model = self.output.with_suffix(".onnx.building")
        onnx.save_model(model, str(tmp_model))
        # External data location is relative to this output directory, so checking
        # the temporary protobuf still resolves the final data file correctly.
        onnx.checker.check_model(str(tmp_model), full_check=True)
        tmp_model.replace(self.output)
        return dict(generated=True, dtype="float16", onnx_bytes=self.output.stat().st_size,
                    onnx_sha256=sha256_file(self.output), external_data_file=self.output_data_name,
                    external_data_bytes=self.output_data.stat().st_size,
                    external_data_sha256=self.digest.hexdigest(),
                    random_weight_bytes=self.random_weight_bytes,
                    nodes=len(model.graph.node), node_types=dict(old_ops),
                    validation="onnx.checker.check_model(path, full_check=True)",
                    conversion={"source_onnx": str(self.source), "source_onnx_sha256": source_sha,
                        "method": "streamed numpy float32 -> float16 rounding; no resampling",
                        "chunk_elements": self.chunk_elements, "counts": dict(self.counts),
                        "float_values_converted": self.converted_values,
                        "all_source_and_output_values_finite": self.finite_before and self.finite_after,
                        "nonzero_float32_values_rounded_to_float16_zero": self.underflows_to_zero,
                        "max_abs_tensor_value_rounding_error": self.max_abs_rounding_error,
                        "rounding_error_scope": "all FLOAT tensors, including large timestep-frequency constants; not an output-accuracy measurement",
                        "operator_names_types_counts_preserved": True,
                        "algorithmic_gemm_flops_preserved": True,
                        "cast_to_float32_retargeted_to_float16": self.counts["cast_targets_float32_to_float16"],
                        "layernorm_stash_type": "preserved FLOAT32 accumulation",
                        "generation_seconds": time.perf_counter() - started,
                        "converted_at_utc": now()})


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    root = Path(__file__).resolve().parent
    p.add_argument("--source-dir", type=Path, default=root / "proxy")
    p.add_argument("--output-dir", type=Path, default=root / "proxy_fp16")
    p.add_argument("--smoke", action="store_true", help="Use source and output smoke_1layer subdirectories")
    p.add_argument("--stage", choices=(*STAGES, "all"), default="all")
    p.add_argument("--chunk-elements", type=int, default=1024 * 1024)
    p.add_argument("--force", action="store_true")
    args = p.parse_args()
    if args.chunk_elements < 1 or args.chunk_elements > 4 * 1024 * 1024:
        p.error("--chunk-elements must be 1..4194304 to bound conversion memory")
    source_dir = args.source_dir.resolve()
    output_dir = args.output_dir.resolve()
    if args.smoke:
        source_dir /= "smoke_1layer"
        output_dir /= "smoke_1layer"
    if source_dir == output_dir:
        p.error("Output must preserve the original source directory")
    output_dir.mkdir(parents=True, exist_ok=True)
    source_manifest = source_dir / "manifest.json"
    original = json.loads(source_manifest.read_text())
    target_manifest = output_dir / "manifest.json"
    source_manifest_sha = sha256_file(source_manifest)
    if target_manifest.exists():
        manifest = json.loads(target_manifest.read_text())
        if manifest.get("derivative_of", {}).get("manifest_sha256") != source_manifest_sha:
            p.error("Source manifest changed; choose a new output directory")
    else:
        manifest = copy.deepcopy(original)
        manifest.update(reference_dtype="float16", runtime_dtype="must_be_measured_from_EP_compiler_report",
            complete=False, derivative_of={"manifest": str(source_manifest), "manifest_sha256": source_manifest_sha,
                "reference_dtype": original.get("reference_dtype"),
                "method": "same independent weights rounded once to FP16; same layers/operators/shapes/GEMM arithmetic",
                "performance_ab_claim": False})
        manifest["scope"]["precision_status"] = "FP16 rounded reference; neither native INT8 nor a measured optimization gain"
        for stage, info in manifest["stages"].items():
            for field in ("inputs", "outputs"):
                for item in info[field]:
                    if item["dtype"] == "float32":
                        item["dtype"] = "float16"
            info["generated"] = False
            for stale in ("generation_seconds", "external_data_bytes", "external_data_sha256", "onnx_bytes", "onnx_sha256", "random_weight_bytes", "validation"):
                info.pop(stale, None)
    selected = STAGES if args.stage == "all" else (args.stage,)
    for stage in selected:
        source_spec = original["stages"][stage]
        source = source_dir / source_spec["file"]
        if not source_spec.get("generated") or not source.is_file():
            p.error(f"Source stage not generated: {source}")
        output = output_dir / source_spec["file"]
        if output.exists() and not args.force:
            p.error(f"Existing derivative: {output}; use --force to replace")
        print(f"Converting {stage} to FP16 with {args.chunk_elements} float32 elements/chunk ...", flush=True)
        converter = Converter(source, output, args.chunk_elements)
        try:
            converted = converter.convert()
        finally:
            if not converter.stream.closed:
                converter.stream.close()
        manifest["stages"][stage].update(converted)
        assert manifest["stages"][stage]["flops_per_call"] == source_spec["flops_per_call"]
        manifest["complete"] = all(s["generated"] for s in manifest["stages"].values())
        manifest["generated_at_utc"] = now()
        write_json(target_manifest, manifest)
        print(json.dumps({"stage": stage, "output": str(output), "manifest": str(target_manifest),
                          "onnx_full_check": "passed", "external_data_bytes": converted["external_data_bytes"],
                          "flops_per_call": source_spec["flops_per_call"]}, ensure_ascii=False), flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
