#!/usr/bin/env python3
"""Canonicalize singleton-head K layout with a static, data-equivalent Reshape.

The source graph changes [B,T,1,H] -> [B,1,T,H] with Transpose(0,2,1,3).
Since the moved head axis has size one, the same contiguous data and output
shape are obtained by Reshape. Removing this Transpose prevents its optimizer
propagation through RoPE onto exported K-cache outputs. This is a graph change,
not a kernel or a measured speedup. Every external weight value is preserved.
Full models use cp --reflink=auto (independent inodes, streaming copy fallback);
the one-layer smoke compacts only its used tensors into a separate data file.
All external graph I/O shapes/dtypes remain unchanged.

  python patch_kv_layout.py --smoke  # first-layer backbone, explicitly smoke only
  python patch_kv_layout.py          # full workload, only backbone changed
"""
from __future__ import annotations

import argparse
from collections import Counter
import copy
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import re
import subprocess


def sha256(path):
    h = hashlib.sha256()
    with path.open("rb") as f:
        for data in iter(lambda: f.read(4 * 1024 * 1024), b""):
            h.update(data)
    return h.hexdigest()


def copy_independent(source, target, force=False):
    if target.exists():
        if os.path.samefile(source, target):
            # A previous candidate may have created a hardlink. Remove only the
            # candidate name before copying, preserving all source bytes.
            target.unlink()
        elif not force:
            raise FileExistsError(target)
        else:
            target.unlink()
    subprocess.run(["/usr/bin/cp", "--reflink=auto", "--", str(source), str(target)], check=True)
    if os.path.samefile(source, target) or target.stat().st_nlink != 1:
        raise RuntimeError("Expected independent single-link output inode")


def compact_external(model, source_dir, out, force=False):
    """Copy only live smoke tensor byte ranges, without dtype/value changes."""
    import onnx
    streams = {}
    digests = {}
    files = {}
    try:
        for tensor in model.graph.initializer:
            if tensor.data_location != onnx.TensorProto.EXTERNAL:
                continue
            info = {x.key: x.value for x in tensor.external_data}
            location = info["location"]
            source = source_dir / location
            target = out / location
            if location not in streams:
                if target.exists():
                    if os.path.samefile(source, target) or force:
                        target.unlink()
                    else:
                        raise FileExistsError(target)
                streams[location] = target.open("wb")
                digests[location] = hashlib.sha256()
                files[location] = {"location": location, "source": str(source), "target": str(target),
                    "storage": "streamed_compaction_of_live_tensor_bytes; independent_inode"}
            output = streams[location]
            digest = digests[location]
            pad = (-output.tell()) % 64
            if pad:
                output.write(bytes(pad)); digest.update(bytes(pad))
            offset = output.tell()
            length = int(info["length"])
            with source.open("rb") as input_stream:
                input_stream.seek(int(info.get("offset", 0)))
                remaining = length
                while remaining:
                    chunk = input_stream.read(min(remaining, 4 * 1024 * 1024))
                    if not chunk:
                        raise EOFError(tensor.name)
                    output.write(chunk); digest.update(chunk)
                    remaining -= len(chunk)
            for entry in tensor.external_data:
                if entry.key == "offset":
                    entry.value = str(offset)
    finally:
        for stream in streams.values():
            stream.close()
    for location, item in files.items():
        target = Path(item["target"])
        item.update(bytes=target.stat().st_size, inode=target.stat().st_ino,
                    sha256=digests[location].hexdigest())
    return list(files.values())


def first_layer(model, onnx):
    """Prune to layer0 plus the original final norm; no learned values change."""
    outputs = list(model.graph.output)
    last_residual = next(n.output[0] for n in model.graph.node if n.name.startswith("node_layer17_mlp_residual_"))
    first_residual = next(n.output[0] for n in model.graph.node if n.name.startswith("node_layer0_mlp_residual_"))
    for n in model.graph.node:
        if n.name.startswith("node_final_norm_"):
            for i, name in enumerate(n.input):
                if name == last_residual:
                    n.input[i] = first_residual
    keep_outputs = [v for v in outputs if v.name in ("hidden", "kv_k_0", "kv_v_0")]
    producer = {o: n for n in model.graph.node for o in n.output}
    needed_tensors = {o.name for o in keep_outputs}
    needed_nodes = set()
    todo = list(needed_tensors)
    while todo:
        n = producer.get(todo.pop())
        if n is not None and n.name not in needed_nodes:
            needed_nodes.add(n.name)
            for i in n.input:
                if i not in needed_tensors:
                    needed_tensors.add(i)
                    todo.append(i)
    retained = [n for n in model.graph.node if n.name in needed_nodes]
    initializers = [t for t in model.graph.initializer if t.name in needed_tensors]
    model.graph.ClearField("node")
    model.graph.node.extend(retained)
    model.graph.ClearField("initializer")
    model.graph.initializer.extend(initializers)
    model.graph.ClearField("output")
    model.graph.output.extend(keep_outputs)
    model.graph.ClearField("value_info")
    return model


def patch(model, onnx, np):
    inferred = onnx.shape_inference.infer_shapes(model, strict_mode=True)
    value_shapes = {}
    for v in list(inferred.graph.input) + list(inferred.graph.output) + list(inferred.graph.value_info):
        value_shapes[v.name] = [d.dim_value for d in v.type.tensor_type.shape.dim]
    replacements = []
    for i, node in enumerate(model.graph.node):
        if not re.match(r"node_layer\d+_attn_k_heads_\d+$", node.name):
            continue
        perm = next((list(a.ints) for a in node.attribute if a.name == "perm"), None)
        shape = value_shapes[node.input[0]]
        if node.op_type != "Transpose" or perm != [0, 2, 1, 3] or len(shape) != 4 or shape[2] != 1:
            raise ValueError(f"Cannot prove singleton-axis equivalence for {node.name}: {shape}, {perm}")
        target_shape = [shape[j] for j in perm]
        shape_name = node.name + "_static_shape"
        constant = onnx.numpy_helper.from_array(np.asarray(target_shape, dtype=np.int64), shape_name)
        model.graph.initializer.append(constant)
        original = str(node)
        new = onnx.helper.make_node("Reshape", [node.input[0], shape_name], list(node.output),
                                    name=node.name.replace("_heads_", "_heads_static_reshape_"))
        replacements.append({"original_node": node.name, "replacement_node": new.name,
                             "input": node.input[0], "outputs": list(node.output),
                             "input_shape": shape, "output_shape": target_shape,
                             "original_perm": perm, "proof": "Only size-one axis moves; linear data order is identical",
                             "original_onnx_node": original})
        model.graph.node[i].CopyFrom(new)
    if not replacements:
        raise ValueError("No source K-head Transposes matched")
    return replacements


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    root = Path(__file__).resolve().parent
    p.add_argument("--source-dir", type=Path, default=root / "proxy_fp16")
    p.add_argument("--output-dir", type=Path, default=root / "proxy_fp16_static_kv")
    p.add_argument("--smoke", action="store_true")
    p.add_argument("--force", action="store_true")
    args = p.parse_args()
    import numpy as np
    import onnx
    source_dir, out = args.source_dir.resolve(), args.output_dir.resolve()
    if args.smoke:
        out /= "smoke_1layer"
    if source_dir == out:
        p.error("Preserve the source graph in a separate directory")
    out.mkdir(parents=True, exist_ok=True)
    source_manifest = source_dir / "manifest.json"
    manifest = json.loads(source_manifest.read_text())
    original_spec = copy.deepcopy(manifest["stages"]["backbone"])
    source_model = source_dir / original_spec["file"]
    model = onnx.load(str(source_model), load_external_data=False)
    if args.smoke:
        model = first_layer(model, onnx)
    reference = copy.deepcopy(model)
    changes = patch(model, onnx, np)
    expected = 1 if args.smoke else 18
    if len(changes) != expected:
        raise ValueError(f"Expected {expected} K-head substitutions, got {len(changes)}")
    # ONNX 1.21 rejects multiply linked external-data files, so every output
    # needs its own inode. Preserve values with reflink/copy or smoke compaction.
    external_files = []
    if args.smoke:
        external_files = compact_external(model, source_dir, out, args.force)
        by_name = {t.name: t for t in model.graph.initializer}
        for tensor in reference.graph.initializer:
            if tensor.data_location == onnx.TensorProto.EXTERNAL:
                tensor.CopyFrom(by_name[tensor.name])
    else:
        for tensor in model.graph.initializer:
            if tensor.data_location != onnx.TensorProto.EXTERNAL:
                continue
            location = next(x.value for x in tensor.external_data if x.key == "location")
            if any(item["location"] == location for item in external_files):
                continue
            src, dest = source_dir / location, out / location
            dest.parent.mkdir(parents=True, exist_ok=True)
            copy_independent(src, dest, args.force)
            external_files.append({"location": location, "source": str(src), "target": str(dest),
                "storage": "cp --reflink=auto; independent_inode; full streaming-copy fallback allowed",
                "source_inode": src.stat().st_ino, "inode": dest.stat().st_ino,
                "bytes": dest.stat().st_size, "sha256": sha256(dest)})
    target = out / original_spec["file"]
    if target.exists() and not args.force:
        raise FileExistsError(target)
    props = {v.key: v.value for v in model.metadata_props}
    props["static_kv_layout_patch"] = "K singleton-head Transpose -> data-equivalent static Reshape"
    props["source_onnx_sha256"] = sha256(source_model)
    if args.smoke:
        props["workload_scope"] = "one-layer compatibility smoke only; not complete pi0.5 block"
    onnx.helper.set_model_props(model, props)
    temp = target.with_suffix(".onnx.building")
    onnx.save_model(model, str(temp))
    onnx.checker.check_model(str(temp), full_check=True)
    temp.replace(target)
    if args.smoke:
        reference_path = out / "backbone_reference.onnx"
        onnx.save_model(reference, str(reference_path))
        onnx.checker.check_model(str(reference_path), full_check=True)
    else:
        # All other stages are byte-identical to the source, including weights.
        for stage in ("vision", "expert"):
            stage_spec = manifest["stages"][stage]
            for filename in (stage_spec["file"], stage_spec["external_data_file"]):
                copy_independent(source_dir / filename, out / filename, args.force)
    spec = manifest["stages"]["backbone"]
    spec.update(nodes=len(model.graph.node), node_types=dict(Counter(n.op_type for n in model.graph.node)),
                onnx_bytes=target.stat().st_size, onnx_sha256=sha256(target),
                validation="onnx.checker.check_model(path, full_check=True)",
                graph_change={"single_variable": "K cache singleton-axis static layout canonicalization",
                    "source_onnx": str(source_model), "source_onnx_sha256": sha256(source_model),
                    "substitutions": changes, "external_data_files": external_files,
                    "io_shapes_and_dtypes_unchanged": True, "gemm_flops_unchanged": True,
                    "performance_gain": None, "status": "candidate_unmeasured"})
    if args.smoke:
        manifest["benchmark_eligible"] = False
        manifest["workload"] = "compatibility_smoke_only"
        manifest["complete"] = False
        spec["outputs"] = [v for v in spec["outputs"] if v["name"] in ("hidden", "kv_k_0", "kv_v_0")]
        spec["shapes"]["layers"] = 1
        spec["random_parameters"] = sum(math.prod(t.dims) for t in model.graph.initializer if t.data_location == onnx.TensorProto.EXTERNAL)
        spec["random_weight_bytes"] = 2 * spec["random_parameters"]
        spec["external_data_bytes"] = external_files[0]["bytes"]
        spec["external_data_sha256"] = external_files[0]["sha256"]
        spec["flop_terms"] = {k: v // 18 for k, v in original_spec["flop_terms"].items()}
        spec["flops_per_call"] = sum(spec["flop_terms"].values())
        spec["gemm_ledger"] = [v for v in spec["gemm_ledger"] if v["name"].startswith("layer0_")]
        spec["graph_change"]["smoke_reference"] = str(out / "backbone_reference.onnx")
        spec["graph_change"]["io_shapes_and_dtypes_unchanged"] = "relative to the same one-layer reference; reduced layers never used as full-block performance"
        for stage in ("vision", "expert"):
            manifest["stages"][stage]["generated"] = False
        manifest["flops_per_block"] = None
    manifest["generated_at_utc"] = datetime.now(timezone.utc).isoformat()
    manifest["layout_candidate_of"] = {"source_manifest": str(source_manifest),
        "source_manifest_sha256": sha256(source_manifest), "status": "unmeasured_candidate",
        "note": "Only K singleton-axis Transpose is replaced; external weight values preserved with independent single-link output inodes"}
    temp_json = out / "manifest.json.building"
    temp_json.write_text(json.dumps(manifest, indent=2, ensure_ascii=False) + "\n")
    temp_json.replace(out / "manifest.json")
    print(json.dumps({"model": str(target), "substitutions": len(changes),
                      "smoke": args.smoke, "onnx_full_check": "passed",
                      "prepared_external_data_bytes": sum(x["bytes"] for x in external_files),
                      "external_storage": [x["storage"] for x in external_files]}, ensure_ascii=False))


if __name__ == "__main__":
    main()
