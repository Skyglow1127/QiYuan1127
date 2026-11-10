#!/usr/bin/env python3
"""Candidate: statically pad internal K/V reads from 968 to 1024 tokens.

Queries, prefill inputs, hidden states and exported KV stay at logical 968.
K/V padding is local to QK and PV consumers. Padded logits receive -65504
before Softmax; valid logits receive zero. Learned weights are unchanged.
Useful algorithmic FLOPs remain the logical unpadded count; extra executed
GEMM work is recorded separately and never added to effective-TFLOPS numerator.

  python pad_attention_kv.py --smoke  # prepare only one-layer compatibility graph
"""
from __future__ import annotations

import argparse
from collections import Counter
from datetime import datetime, timezone
import hashlib
import json
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


def independent_copy(source, target, force):
    if target.exists():
        if not force:
            raise FileExistsError(target)
        target.unlink()
    subprocess.run(["/usr/bin/cp", "--reflink=auto", "--", str(source), str(target)], check=True)
    if os.path.samefile(source, target) or target.stat().st_nlink != 1:
        raise RuntimeError("Output external data must have an independent single-link inode")


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    root = Path(__file__).resolve().parent
    p.add_argument("--source-dir", type=Path, default=root / "proxy_fp16_static_kv")
    p.add_argument("--output-dir", type=Path, default=root / "proxy_fp16_static_kv_pad")
    p.add_argument("--smoke", action="store_true")
    p.add_argument("--padded-kv-tokens", type=int, default=1024)
    p.add_argument("--force", action="store_true")
    args = p.parse_args()
    import numpy as np
    import onnx
    source_dir, out = args.source_dir.resolve(), args.output_dir.resolve()
    if args.smoke:
        source_dir /= "smoke_1layer"
        out /= "smoke_1layer"
    if source_dir == out:
        p.error("Preserve source graph in a separate directory")
    source_manifest = source_dir / "manifest.json"
    manifest = json.loads(source_manifest.read_text())
    spec = manifest["stages"]["backbone"]
    logical = spec["shapes"]["seq"]
    padded = args.padded_kv_tokens
    if logical != 968 or padded != 1024:
        p.error("This measured-shape candidate is specifically logical968 -> internal1024")
    out.mkdir(parents=True, exist_ok=True)
    source = source_dir / spec["file"]
    target = out / spec["file"]
    if target.exists() and not args.force:
        raise FileExistsError(target)
    model = onnx.load(str(source), load_external_data=False)
    source_io = [v.SerializeToString() for v in list(model.graph.input) + list(model.graph.output)]
    source_learned = [v.SerializeToString() for v in model.graph.initializer if v.data_location == onnx.TensorProto.EXTERNAL]
    source_types = Counter(n.op_type for n in model.graph.node)
    pads_name = "internal_kv_padding_static_pads"
    zero_name = "internal_kv_padding_zero"
    bias_name = "internal_kv_padding_logit_bias"
    pads = np.asarray([0, 0, 0, 0, 0, 0, padded - logical, 0], dtype=np.int64)
    bias = np.zeros(padded, dtype=np.float16)
    bias[logical:] = np.float16(-65504)
    model.graph.initializer.extend([
        onnx.numpy_helper.from_array(pads, pads_name),
        onnx.numpy_helper.from_array(np.asarray(0, dtype=np.float16), zero_name),
        onnx.numpy_helper.from_array(bias, bias_name),
    ])
    nodes = []
    changes = []
    affected_shapes = set()
    for node in model.graph.node:
        key = re.match(r"node_layer(\d+)_attn_key_transposed_\d+$", node.name)
        qk = re.match(r"node_layer(\d+)_attn_qk_\d+$", node.name)
        pv = re.match(r"node_layer(\d+)_attn_pv_\d+$", node.name)
        if key:
            original = node.input[0]
            padded_name = f"layer{key[1]}_internal_k_padded"
            nodes.append(onnx.helper.make_node("Pad", [original, pads_name, zero_name], [padded_name],
                                               name=f"node_layer{key[1]}_internal_k_pad", mode="constant"))
            node.input[0] = padded_name
            changes.append({"layer": int(key[1]), "branch": "K_QK_read_only", "unpadded_tensor": original,
                            "padded_tensor": padded_name, "exported_kv_unchanged": True})
            affected_shapes.add(node.output[0])
        if pv:
            original = node.input[1]
            padded_name = f"layer{pv[1]}_internal_v_padded"
            nodes.append(onnx.helper.make_node("Pad", [original, pads_name, zero_name], [padded_name],
                                               name=f"node_layer{pv[1]}_internal_v_pad", mode="constant"))
            node.input[1] = padded_name
            changes.append({"layer": int(pv[1]), "branch": "V_PV_read_only", "unpadded_tensor": original,
                            "padded_tensor": padded_name, "exported_kv_unchanged": True})
        if qk:
            original_output = node.output[0]
            raw_output = original_output + "_internal_unmasked"
            node.output[0] = raw_output
            nodes.append(node)
            nodes.append(onnx.helper.make_node("Add", [raw_output, bias_name], [original_output],
                                               name=f"node_layer{qk[1]}_internal_padding_mask"))
            changes.append({"layer": int(qk[1]), "branch": "Softmax_additive_bias", "output": original_output,
                            "valid_tokens_bias": 0, "padding_tokens_bias": -65504})
            affected_shapes.add(original_output)
            continue
        if re.match(r"node_layer\d+_attn_softmax_\d+$", node.name):
            affected_shapes.add(node.output[0])
        nodes.append(node)
    layers = spec["shapes"]["layers"]
    if len(changes) != 3 * layers:
        raise RuntimeError(f"Expected {3 * layers} changes, got {len(changes)}")
    model.graph.ClearField("node")
    model.graph.node.extend(nodes)
    for v in model.graph.value_info:
        if v.name in affected_shapes:
            # Transposed K and attention scores/probabilities all have the
            # padded key-token dimension last, while public KV remain untouched.
            v.type.tensor_type.shape.dim[-1].dim_value = padded
    if source_io != [v.SerializeToString() for v in list(model.graph.input) + list(model.graph.output)]:
        raise AssertionError("Public IO changed")
    if source_learned != [v.SerializeToString() for v in model.graph.initializer if v.data_location == onnx.TensorProto.EXTERNAL]:
        raise AssertionError("Learned tensor metadata changed")
    external = []
    for tensor in model.graph.initializer:
        if tensor.data_location != onnx.TensorProto.EXTERNAL:
            continue
        location = next(e.value for e in tensor.external_data if e.key == "location")
        if any(x["location"] == location for x in external):
            continue
        src_data, target_data = source_dir / location, out / location
        independent_copy(src_data, target_data, args.force)
        digest = sha256(target_data)
        if digest != spec["external_data_sha256"]:
            raise AssertionError("Copied learned data differs from source manifest")
        external.append({"location": location, "source": str(src_data), "target": str(target_data),
                         "bytes": target_data.stat().st_size, "sha256": digest,
                         "storage": "cp --reflink=auto; independent inode; streaming-copy fallback"})
    props = {v.key: v.value for v in model.metadata_props}
    props.update(internal_attention_kv_padding="968->1024 for K/V reads only; public KV968",
                 useful_flops="logical unpadded GEMM count; padded extra work excluded")
    onnx.helper.set_model_props(model, props)
    temp = target.with_suffix(".onnx.building")
    onnx.save_model(model, str(temp))
    onnx.checker.check_model(str(temp), full_check=True)
    temp.replace(target)
    onnx.checker.check_model(str(target), full_check=True)
    b, h, t, hd = (spec["shapes"][k] for k in ("batch", "heads", "seq", "head_dim"))
    extra = layers * 4 * b * h * t * (padded - logical) * hd
    useful = spec["flops_per_call"]
    spec.update(nodes=len(model.graph.node), node_types=dict(Counter(n.op_type for n in model.graph.node)),
                onnx_bytes=target.stat().st_size, onnx_sha256=sha256(target),
                validation="onnx.checker.check_model(final_path, full_check=True)",
                logical_flops_per_call=useful, executed_gemm_flops_per_call=useful + extra,
                executed_padding_extra_gemm_flops_per_call=extra,
                attention_padding={"single_variable": "internal static K/V token padding with masked logits",
                    "source_onnx": str(source), "source_onnx_sha256": sha256(source),
                    "logical_kv_tokens": logical, "internal_kv_tokens": padded, "padding_tokens": padded - logical,
                    "query_prefill_hidden_and_exported_kv_unchanged": True,
                    "learned_tensor_metadata_and_bytes_unchanged": True, "changes": changes,
                    "source_node_types": dict(source_types), "external_data_files": external,
                    "extra_flops_formula": "layers * 4 * batch * query_heads * query_tokens * padding_tokens * head_dim",
                    "useful_tflops_numerator_includes_padding": False,
                    "status": "candidate_unmeasured", "npu_execution_attempted_by_builder": False})
    # Keep unrelated stages at their original file paths and avoid their copies
    # or recompilation. The smoke manifest already marks those ungenerated.
    for stage in ("vision", "expert"):
        original_path = source_dir / manifest["stages"][stage]["file"]
        if original_path.is_file():
            manifest["stages"][stage]["file"] = os.path.relpath(original_path, out)
    manifest["generated_at_utc"] = datetime.now(timezone.utc).isoformat()
    manifest["logical_flops_per_block"] = manifest.get("flops_per_block")
    manifest["executed_padding_extra_gemm_flops_per_block"] = None if args.smoke else extra
    manifest["executed_gemm_flops_per_block"] = (None if args.smoke else manifest["flops_per_block"] + extra)
    manifest["padding_candidate_of"] = {"manifest": str(source_manifest), "sha256": sha256(source_manifest),
                                        "status": "unmeasured", "performance_gain": None}
    tmp_json = out / "manifest.json.building"
    tmp_json.write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + "\n")
    tmp_json.replace(out / "manifest.json")
    print(json.dumps({"model": str(target), "layers": layers, "final_path_full_check": "passed",
                      "logical_flops_per_call": useful, "extra_executed_padding_gemm_flops": extra,
                      "npu_run": False}, ensure_ascii=False))


if __name__ == "__main__":
    main()
