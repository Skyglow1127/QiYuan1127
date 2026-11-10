#!/usr/bin/env python3
"""Build deterministic, static, independent-weight pi0.5 shape proxies.

This is a performance workload, not a policy or an accuracy reproduction. Image
patch extraction, text lookup, preprocessing and robot I/O are excluded. Every
learned matrix is independently generated; no downloaded/shared/zero matrices.
FP32 is the portable graph reference for the Vitis AI automatic BF16 path. This
builder does not claim native INT8, P5 quantization, or zero CPU fallback.

Examples:
  python build_proxy.py --stage all
  python build_proxy.py --stage vision --layers 1  # labelled smoke subdirectory
  python build_proxy.py --stage all --manifest-only
"""
from __future__ import annotations

import argparse
from collections import Counter
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import sys
import time


SOURCES = {
    "gemma": "https://raw.githubusercontent.com/Physical-Intelligence/openpi/main/src/openpi/models/gemma.py",
    "siglip": "https://raw.githubusercontent.com/Physical-Intelligence/openpi/main/src/openpi/models/siglip.py",
    "pi0": "https://raw.githubusercontent.com/Physical-Intelligence/openpi/main/src/openpi/models/pi0.py",
    "local_card": "/home/aup/具身论文及技术/papers/P02-pi0.5.md",
}
STAGES = ("vision", "backbone", "expert")


def io(name, shape, dtype="float32", **extra):
    return dict(name=name, shape=list(shape), dtype=dtype, **extra)


def spec(stage: str, layers: int | None = None):
    """Arithmetic numerator: conventional algorithmic GEMM FLOPs, 2 per MAC."""
    l = layers if layers is not None else (27 if stage == "vision" else 18)
    if stage == "vision":
        b, t, d, h, hd, f = 3, 256, 1152, 16, 72, 4304
        terms = {
            "qkv_projections": l * 6 * b * t * d * d,
            "attention_output_projection": l * 2 * b * t * d * d,
            "attention_qk_and_pv": l * 4 * b * h * t * t * hd,
            "mlp_up_and_down": l * 4 * b * t * d * f,
            "vision_to_backbone_projection": 2 * b * t * d * 2048,
        }
        inputs = [io("image_tokens", [b, t, d])]
        outputs = [io("vision_hidden", [b, t, 2048])]
        shapes = dict(batch=b, seq=t, width=d, heads=h, kv_heads=h,
                      head_dim=hd, mlp_dim=f, layers=l)
    else:
        b, t, d, h, hd, f = (1, 968, 2048, 8, 256, 16384) if stage == "backbone" else (1, 50, 1024, 8, 256, 4096)
        s = t if stage == "backbone" else 968 + t
        q = h * hd  # Expert Q width is 2048 although its residual width is 1024.
        terms = {
            "query_projection": l * 2 * b * t * d * q,
            "key_and_value_projections": l * 4 * b * t * d * hd,
            "attention_output_projection": l * 2 * b * t * q * d,
            "attention_qk_and_pv": l * 4 * b * h * t * s * hd,
            "geglu_gate_up_down": l * 6 * b * t * d * f,
        }
        shapes = dict(batch=b, seq=t, kv_seq=s, width=d, heads=h,
                      kv_heads=1, head_dim=hd, mlp_dim=f, layers=l)
        if stage == "backbone":
            inputs = [io("tokens", [b, t, d])]
            outputs = [io("hidden", [b, t, d])]
            for i in range(l):
                outputs += [io(f"kv_k_{i}", [b, 1, t, hd]), io(f"kv_v_{i}", [b, 1, t, hd])]
        else:
            terms.update({
                "action_input_and_output": 4 * b * t * 32 * d,
                "timestep_mlp": 4 * b * d * d,
                "adaptive_rms_modulation_per_layer": l * 12 * b * d * d,
                "final_adaptive_rms_scale_shift": 4 * b * d * d,
            })
            inputs = [io("x_t", [b, t, 32]), io("timestep", [b], fill=1.0)]
            for i in range(l):
                inputs += [io(f"kv_k_{i}", [b, 1, 968, hd]), io(f"kv_v_{i}", [b, 1, 968, hd])]
            inputs += [io("positions", [b, t], "int64", value=[list(range(968, 968 + t))]),
                       io("attention_bias", [t, s], fill=0.0),
                       io("step_size", [1], fill=0.1)]
            outputs = [io("next_x", [b, t, 32])]
    return dict(file=f"{stage}_proxy.onnx", inputs=inputs, outputs=outputs,
                shapes=shapes, flops_per_call=sum(terms.values()), flop_terms=terms,
                generated=False)


class Graph:
    """Write each initializer to external data immediately, keeping RAM bounded."""

    def __init__(self, stage, directory, seed):
        import numpy as np
        import onnx
        self.np, self.onnx = np, onnx
        self.h = onnx.helper
        self.stage = stage
        self.directory = directory
        self.data_name = f"{stage}_proxy.onnx.data"
        self.data_path = directory / self.data_name
        self.data_tmp = self.data_path.with_suffix(".data.building")
        self.f = self.data_tmp.open("wb")
        self.digest = hashlib.sha256()
        self.rng = np.random.default_rng(seed)
        self.nodes, self.initializers = [], []
        self.serial = 0
        self.matmul_flops = 0
        self.matrix_shapes = []
        self.random_parameter_count = 0
        self.constant_count = 0
        self.weight_bytes = 0

    def unique(self, stem):
        self.serial += 1
        return f"{stem}_{self.serial}"

    def node(self, kind, inputs, stem, outputs=None, **attrs):
        outputs = outputs or [self.unique(stem)]
        if isinstance(outputs, str):
            outputs = [outputs]
        self.nodes.append(self.h.make_node(kind, list(inputs), outputs,
                                           name=self.unique(f"node_{stem}"), **attrs))
        return outputs[0] if len(outputs) == 1 else outputs

    def constant(self, value, stem, dtype=None):
        np = self.np
        arr = np.asarray(value, dtype=dtype or np.float32)
        name = self.unique(stem)
        self.initializers.append(self.onnx.numpy_helper.from_array(arr, name))
        self.constant_count += 1
        return name

    def parameter(self, shape, stem, *, scale=None, mean=0.0):
        """Independent float32 normal values in <= 16 MiB chunks; never shared."""
        np, onnx = self.np, self.onnx
        name = self.unique(stem)
        shape = tuple(shape)
        count = math.prod(shape)
        scale = scale if scale is not None else 1.0 / math.sqrt(shape[0])
        # Align tensor starts so external-data consumers can mmap efficiently.
        offset = self.f.tell()
        padding = (-offset) % 64
        if padding:
            buf = bytes(padding)
            self.f.write(buf)
            self.digest.update(buf)
        offset = self.f.tell()
        remaining = count
        while remaining:
            n = min(remaining, 4 * 1024 * 1024)
            values = self.rng.standard_normal(n, dtype=np.float32)
            values *= np.float32(scale)
            if mean:
                values += np.float32(mean)
            # Very unlikely exact normal zeros are replaced rather than creating
            # the prohibited zero weights; this does not tie parameter values.
            values[values == 0] = np.finfo(np.float32).tiny
            raw = values.tobytes()
            self.f.write(raw)
            self.digest.update(raw)
            remaining -= n
        tensor = onnx.TensorProto(name=name, data_type=onnx.TensorProto.FLOAT,
                                  dims=shape, data_location=onnx.TensorProto.EXTERNAL)
        for key, value in (('location', self.data_name), ('offset', str(offset)), ('length', str(count * 4))):
            e = tensor.external_data.add()
            e.key, e.value = key, value
        self.initializers.append(tensor)
        self.random_parameter_count += count
        self.weight_bytes += count * 4
        return name

    def reshape(self, x, shape, stem):
        s = self.constant(shape, f"{stem}_shape", self.np.int64)
        return self.node("Reshape", [x, s], stem)

    def unsqueeze(self, x, axes, stem):
        a = self.constant(axes, f"{stem}_axes", self.np.int64)
        return self.node("Unsqueeze", [x, a], stem)

    def split(self, x, sizes, stem, axis=-1):
        s = self.constant(sizes, f"{stem}_sizes", self.np.int64)
        outputs = [self.unique(f"{stem}_part{i}") for i in range(len(sizes))]
        return self.node("Split", [x, s], stem, outputs=outputs, axis=axis)

    def linear(self, x, rows, cols, tokens, stem, bias=False):
        w = self.parameter([rows, cols], f"{stem}_weight")
        y = self.node("MatMul", [x, w], stem)
        self.matmul_flops += 2 * tokens * rows * cols
        self.matrix_shapes.append(dict(name=stem, m=tokens, k=rows, n=cols,
                                       flops=2 * tokens * rows * cols))
        if bias:
            b = self.parameter([cols], f"{stem}_bias", scale=1e-6)
            y = self.node("Add", [y, b], f"{stem}_bias_add")
        return y

    def gelu(self, x, stem):
        # JAX/Flax nn.gelu defaults to the tanh approximation.
        three = self.constant(3.0, f"{stem}_three")
        cubic = self.node("Pow", [x, three], f"{stem}_cube")
        coefficient = self.constant(0.044715, f"{stem}_cubic_coefficient")
        cubic = self.node("Mul", [cubic, coefficient], f"{stem}_scaled_cube")
        y = self.node("Add", [x, cubic], f"{stem}_polynomial")
        k = self.constant(math.sqrt(2 / math.pi), f"{stem}_sqrt2pi")
        y = self.node("Mul", [y, k], f"{stem}_tanh_argument")
        y = self.node("Tanh", [y], f"{stem}_tanh")
        one = self.constant(1.0, f"{stem}_one")
        y = self.node("Add", [y, one], f"{stem}_plus1")
        half = self.constant(0.5, f"{stem}_half")
        y = self.node("Mul", [y, half], f"{stem}_half_scale")
        return self.node("Mul", [x, y], stem)

    def swish(self, x, stem):
        s = self.node("Sigmoid", [x], f"{stem}_sigmoid")
        return self.node("Mul", [x, s], stem)

    def layer_norm(self, x, d, stem):
        scale = self.parameter([d], f"{stem}_scale", mean=1.0, scale=0.01)
        bias = self.parameter([d], f"{stem}_bias", scale=1e-6)
        return self.node("LayerNormalization", [x, scale, bias], stem,
                         axis=-1, epsilon=1e-6, stash_type=1)

    def rms_norm(self, x, d, stem, cond=None, final=False):
        square = self.node("Mul", [x, x], f"{stem}_square")
        var = self.node("ReduceMean", [square], f"{stem}_variance", axes=[-1], keepdims=1)
        eps = self.constant(1e-6, f"{stem}_epsilon")
        var = self.node("Add", [var, eps], f"{stem}_regularize")
        root = self.node("Sqrt", [var], f"{stem}_sqrt")
        y = self.node("Div", [x, root], f"{stem}_normalize")
        if cond is None:
            scale = self.parameter([d], f"{stem}_scale", mean=1.0, scale=0.01)
            return self.node("Mul", [y, scale], stem), None
        # Final norm does not use the gate. Build its two live projections only.
        count = 2 if final else 3
        mod = self.linear(cond, d, count * d, 1, f"{stem}_modulation", bias=True)
        mod = self.unsqueeze(mod, [1], f"{stem}_modulation_axis")
        pieces = self.split(mod, [d] * count, f"{stem}_modulation_split")
        one = self.constant(1.0, f"{stem}_one")
        scale = self.node("Add", [pieces[0], one], f"{stem}_scale_plus1")
        y = self.node("Mul", [y, scale], f"{stem}_scale")
        y = self.node("Add", [y, pieces[1]], f"{stem}_shift")
        return y, None if final else pieces[2]

    def rope_angles(self, positions, t, hd, stem, constant=False):
        np = self.np
        inv = 10000.0 ** (-2.0 * np.arange(hd // 2, dtype=np.float32) / hd)
        if constant:
            radians = np.arange(t, dtype=np.float32)[None, None, :, None] * inv
            return (self.constant(np.cos(radians), f"{stem}_cos"),
                    self.constant(np.sin(radians), f"{stem}_sin"))
        positions = self.node("Cast", [positions], f"{stem}_positions_float", to=self.onnx.TensorProto.FLOAT)
        positions = self.unsqueeze(positions, [-1], f"{stem}_positions_axis")
        frequencies = self.constant(inv, f"{stem}_inverse_frequencies")
        radians = self.node("Mul", [positions, frequencies], f"{stem}_radians")
        radians = self.unsqueeze(radians, [1], f"{stem}_head_axis")
        return self.node("Cos", [radians], f"{stem}_cos"), self.node("Sin", [radians], f"{stem}_sin")

    def rope(self, x, angles, hd, stem):
        x1, x2 = self.split(x, [hd // 2, hd // 2], f"{stem}_halves")
        cos, sin = angles
        a = self.node("Mul", [x1, cos], f"{stem}_x1cos")
        b = self.node("Mul", [x2, sin], f"{stem}_x2sin")
        left = self.node("Sub", [a, b], f"{stem}_left")
        a = self.node("Mul", [x2, cos], f"{stem}_x2cos")
        b = self.node("Mul", [x1, sin], f"{stem}_x1sin")
        right = self.node("Add", [a, b], f"{stem}_right")
        return self.node("Concat", [left, right], stem, axis=-1)

    def attention(self, x, *, b, t, d, h, kh, hd, stem, angles=None, cache=None, bias=None, projection_bias=False):
        heads = []
        for label, n in (("q", h), ("k", kh), ("v", kh)):
            a = self.linear(x, d, n * hd, b * t, f"{stem}_{label}", bias=projection_bias)
            a = self.reshape(a, [b, t, n, hd], f"{stem}_{label}_reshape")
            heads.append(self.node("Transpose", [a], f"{stem}_{label}_heads", perm=[0, 2, 1, 3]))
        q, k, v = heads
        if angles:
            q = self.rope(q, angles, hd, f"{stem}_q_rope")
            k = self.rope(k, angles, hd, f"{stem}_k_rope")
        own_k, own_v = k, v
        s = t
        if cache:
            k = self.node("Concat", [cache[0], k], f"{stem}_cache_k", axis=2)
            v = self.node("Concat", [cache[1], v], f"{stem}_cache_v", axis=2)
            s += 968
        scale = self.constant(hd ** -0.5, f"{stem}_q_scale")
        q = self.node("Mul", [q, scale], f"{stem}_q_scaled")
        kt = self.node("Transpose", [k], f"{stem}_key_transposed", perm=[0, 1, 3, 2])
        scores = self.node("MatMul", [q, kt], f"{stem}_qk")
        if bias:
            scores = self.node("Add", [scores, bias], f"{stem}_attention_bias")
        probs = self.node("Softmax", [scores], f"{stem}_softmax", axis=-1)
        context = self.node("MatMul", [probs, v], f"{stem}_pv")
        self.matmul_flops += 4 * b * h * t * s * hd
        self.matrix_shapes.extend([
            dict(name=f"{stem}_qk", batch=b * h, m=t, k=hd, n=s, flops=2 * b * h * t * s * hd),
            dict(name=f"{stem}_pv", batch=b * h, m=t, k=s, n=hd, flops=2 * b * h * t * s * hd),
        ])
        context = self.node("Transpose", [context], f"{stem}_context_transposed", perm=[0, 2, 1, 3])
        context = self.reshape(context, [b, t, h * hd], f"{stem}_context_reshape")
        y = self.linear(context, h * hd, d, b * t, f"{stem}_output", bias=projection_bias)
        return y, own_k, own_v

    def mlp(self, x, d, f, tokens, stem, gated):
        up = self.linear(x, d, f, tokens, f"{stem}_up", bias=not gated)
        if gated:
            gate = self.linear(x, d, f, tokens, f"{stem}_gate")
            gate = self.gelu(gate, f"{stem}_gelu")
            up = self.node("Mul", [up, gate], f"{stem}_gated")
        else:
            up = self.gelu(up, f"{stem}_gelu")
        return self.linear(up, f, d, tokens, f"{stem}_down", bias=not gated)

    def residual(self, x, y, gate, stem):
        if gate is not None:
            y = self.node("Mul", [y, gate], f"{stem}_gate")
        return self.node("Add", [x, y], stem)

    def finish(self, stage_spec):
        onnx = self.onnx
        self.f.flush()
        self.f.close()
        self.data_tmp.replace(self.data_path)
        info = lambda x: self.h.make_tensor_value_info(x["name"],
                onnx.TensorProto.INT64 if x["dtype"] == "int64" else onnx.TensorProto.FLOAT, x["shape"])
        graph = self.h.make_graph(self.nodes, f"pi05_{self.stage}_shape_proxy",
                [info(x) for x in stage_spec["inputs"]], [info(x) for x in stage_spec["outputs"]],
                initializer=self.initializers)
        model = self.h.make_model(graph, producer_name="codex_pi05_shape_proxy",
                                 producer_version="1.0", opset_imports=[self.h.make_opsetid("", 17)])
        model.ir_version = 10  # ORT 1.27 supports IR10; avoids an accidental newer-IR dependency.
        model.doc_string = "Static FP32 independent random weights; preembedded-token shape proxy, not policy inference."
        self.h.set_model_props(model, {"stage": self.stage, "weight_generation": "independent_random_normal",
                 "flops_convention": "2_per_GEMM_MAC; no elementwise/transcendental FLOPs",
                 "precision": "FP32 reference; automatic BF16 compilation requires verification"})
        output = self.directory / stage_spec["file"]
        onnx.save_model(model, str(output))
        # Path-based validation resolves external tensors without reading the
        # multi-GB data file into the Python protobuf heap.
        onnx.checker.check_model(str(output), full_check=True)
        if self.matmul_flops != stage_spec["flops_per_call"]:
            raise AssertionError(f"GEMM graph count {self.matmul_flops} != formula {stage_spec['flops_per_call']}")
        return dict(generated=True, nodes=len(self.nodes), node_types=dict(Counter(n.op_type for n in self.nodes)),
                    random_parameters=self.random_parameter_count, random_weight_bytes=self.weight_bytes,
                    external_data_file=self.data_name, external_data_bytes=self.data_path.stat().st_size,
                    external_data_sha256=self.digest.hexdigest(), onnx_bytes=output.stat().st_size,
                    onnx_sha256=hashlib.sha256(output.read_bytes()).hexdigest(),
                    gemm_ledger=self.matrix_shapes, validation="onnx.checker.check_model(path, full_check=True)")


def build(stage, directory, seed, layers=None):
    stage_spec = spec(stage, layers)
    s = stage_spec["shapes"]
    b, t, d, h, hd, f, l = (s[k] for k in ("batch", "seq", "width", "heads", "head_dim", "mlp_dim", "layers"))
    g = Graph(stage, directory, seed + STAGES.index(stage) * 1000003)
    if stage == "vision":
        p = g.parameter([1, t, d], "vision_position_embedding", scale=d ** -0.5)
        x = g.node("Add", ["image_tokens", p], "vision_add_positions")
        for i in range(l):
            n = g.layer_norm(x, d, f"layer{i}_pre_attn_norm")
            a, _, _ = g.attention(n, b=b, t=t, d=d, h=h, kh=h, hd=hd,
                    stem=f"layer{i}_attn", projection_bias=True)
            x = g.residual(x, a, None, f"layer{i}_attn_residual")
            n = g.layer_norm(x, d, f"layer{i}_pre_mlp_norm")
            a = g.mlp(n, d, f, b * t, f"layer{i}_mlp", gated=False)
            x = g.residual(x, a, None, f"layer{i}_mlp_residual")
        x = g.layer_norm(x, d, "final_norm")
        x = g.linear(x, d, 2048, b * t, "vision_to_backbone", bias=True)
        g.node("Identity", [x], "vision_output", outputs="vision_hidden")
    else:
        cond = None
        if stage == "backbone":
            x = "tokens"
            angles = g.rope_angles(None, t, hd, "backbone_rope", constant=True)
        else:
            x = g.linear("x_t", 32, d, b * t, "action_input", bias=True)
            # Official pi0.py posemb_sincos: log-spaced periods 4e-3..4,
            # then two d->d swish time MLPs, feeding each adaptive RMSNorm.
            periods = 0.004 * (4.0 / 0.004) ** g.np.linspace(0, 1, d // 2, dtype=g.np.float32)
            frequencies = g.constant(2 * math.pi / periods, "timestep_frequencies")
            time_axis = g.unsqueeze("timestep", [-1], "timestep_axis")
            radians = g.node("Mul", [time_axis, frequencies], "timestep_radians")
            sine = g.node("Sin", [radians], "timestep_sin")
            cosine = g.node("Cos", [radians], "timestep_cos")
            cond = g.node("Concat", [sine, cosine], "timestep_embedding", axis=-1)
            for i in range(2):
                cond = g.linear(cond, d, d, b, f"timestep_mlp{i}", bias=True)
                cond = g.swish(cond, f"timestep_swish{i}")
            angles = g.rope_angles("positions", t, hd, "expert_rope")
        for i in range(l):
            n, gate = g.rms_norm(x, d, f"layer{i}_pre_attn_norm", cond=cond)
            cache = None if stage == "backbone" else (f"kv_k_{i}", f"kv_v_{i}")
            a, k, v = g.attention(n, b=b, t=t, d=d, h=h, kh=1, hd=hd,
                    stem=f"layer{i}_attn", angles=angles, cache=cache,
                    bias="attention_bias" if stage == "expert" else None)
            if stage == "backbone":
                g.node("Identity", [k], f"layer{i}_key_output", outputs=f"kv_k_{i}")
                g.node("Identity", [v], f"layer{i}_value_output", outputs=f"kv_v_{i}")
            x = g.residual(x, a, gate, f"layer{i}_attn_residual")
            n, gate = g.rms_norm(x, d, f"layer{i}_pre_mlp_norm", cond=cond)
            a = g.mlp(n, d, f, b * t, f"layer{i}_mlp", gated=True)
            x = g.residual(x, a, gate, f"layer{i}_mlp_residual")
        x, _ = g.rms_norm(x, d, "final_norm", cond=cond, final=True)
        if stage == "backbone":
            g.node("Identity", [x], "backbone_output", outputs="hidden")
        else:
            velocity = g.linear(x, d, 32, b * t, "action_output", bias=True)
            delta = g.node("Mul", [velocity, "step_size"], "euler_delta")
            g.node("Sub", ["x_t", delta], "euler_update", outputs="next_x")
    return stage_spec | g.finish(stage_spec)


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--stage", choices=(*STAGES, "all"), default="all")
    p.add_argument("--output-dir", type=Path, default=Path(__file__).resolve().parent / "proxy")
    p.add_argument("--layers", type=int, help="Reduced-layer smoke only; writes a labelled subdirectory")
    p.add_argument("--seed", type=int, default=20261030)
    p.add_argument("--manifest-only", action="store_true", help="Write arithmetic plan without loading numpy/onnx")
    p.add_argument("--force", action="store_true", help="Replace an existing model/data pair")
    args = p.parse_args()
    if args.layers is not None and not 1 <= args.layers <= 18:
        p.error("--layers must be 1..18 and is only a compatibility smoke workload")
    directory = args.output_dir.resolve()
    if args.layers is not None:
        directory /= f"smoke_{args.layers}layer"
    directory.mkdir(parents=True, exist_ok=True)
    manifest_path = directory / "manifest.json"
    if manifest_path.exists():
        manifest = json.loads(manifest_path.read_text())
        if manifest.get("seed") != args.seed:
            p.error("Output manifest has a different seed; choose a separate output directory")
    else:
        manifest = dict(schema_version="1.0", seed=args.seed,
            workload="pi05_preembedded_shape_proxy" if args.layers is None else "compatibility_smoke_only",
            benchmark_eligible=args.layers is None, expert_steps=10,
            reference_dtype="float32", runtime_dtype="must_be_measured_from_EP_compiler_report",
            layout="BTD activations; BHTD KV; row-major [K,N] matrices",
            sources=SOURCES,
            source_accessed_utc=datetime.now(timezone.utc).isoformat(),
            flops_convention={"name": "algorithmic_GEMM_FLOPs", "multiply_add_flops": 2,
                "definition": "Sum 2*batch*M*N*K for learned linear projections and QK/PV MatMul nodes; expert is multiplied by 10 per block.",
                "excluded": ["RMSNorm/LayerNorm", "RoPE", "GELU/Swish", "Softmax", "elementwise/bias/residual", "Euler update", "memory movement"],
                "no_TOPS_conversion": True},
            scope={"included": ["SigLIP transformer and projection", "bidirectional Gemma prefix with 18-layer KV", "expert adaptive RMSNorm, self+prefix attention, GeGLU, action projection, Euler step"],
                "excluded": ["224x224 image patch extraction", "text tokenization and embedding lookup", "image preprocessing", "robot or camera I/O", "learned policy accuracy"],
                "input_tokens_are_preembedded": True,
                "expert_41_inputs": "x_t + timestep + 36 layer KV tensors + positions + additive attention_bias + step_size",
                "expert_attention": "50 current action KV concatenated with 968 cached prefix KV; 1018 keys; no pi0 state token",
                "precision_status": "FP32 reference; INT8 native quantization and output cosine >=0.99 have not been established"},
            stages={s: spec(s, args.layers) for s in STAGES})
    selected = STAGES if args.stage == "all" else (args.stage,)
    for stage in selected:
        model_path = directory / f"{stage}_proxy.onnx"
        if model_path.exists() and not args.force and not args.manifest_only:
            p.error(f"Existing model: {model_path}; use --force to replace")
        if not args.manifest_only:
            started = time.perf_counter()
            print(f"Building {stage}, {manifest['stages'][stage]['shapes']['layers']} layers ...", flush=True)
            manifest["stages"][stage] = build(stage, directory, args.seed, args.layers)
            manifest["stages"][stage]["generation_seconds"] = time.perf_counter() - started
        manifest["generated_at_utc"] = datetime.now(timezone.utc).isoformat()
        manifest["flops_per_block"] = sum(manifest["stages"][s]["flops_per_call"] * (10 if s == "expert" else 1) for s in STAGES)
        manifest["complete"] = all(manifest["stages"][s]["generated"] for s in STAGES)
        temporary = manifest_path.with_suffix(".json.building")
        temporary.write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + "\n")
        temporary.replace(manifest_path)
        stage_info = manifest["stages"][stage]
        print(json.dumps({"stage": stage, "generated": stage_info["generated"],
                          "flops_per_call": stage_info["flops_per_call"], "manifest": str(manifest_path)}, ensure_ascii=False), flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
