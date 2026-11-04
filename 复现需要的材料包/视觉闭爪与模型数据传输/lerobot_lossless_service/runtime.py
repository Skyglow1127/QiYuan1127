"""Strict complete-model B inference shared by local benchmarks and HTTP service.

The model and its own processor files are read-only. No PEFT or robot imports.
PyTorch and LeRobot are imported only when Runtime is instantiated.
"""
from __future__ import annotations

import hashlib
import importlib.metadata
import json
import os
from pathlib import Path
import time

PROTOCOL = "pi05-expert-inference/v1"
UNITS = ["degree"] * 6 + ["metre"]
REPRESENTATION = "absolute joint targets and gripper width"
CAMERAS = ("observation.images.global", "observation.images.wrist")
REQUIRED_PIPELINE = {"config.json", "policy_preprocessor.json", "policy_postprocessor.json",
                     "policy_preprocessor_step_3_normalizer_processor.safetensors",
                     "policy_postprocessor_step_0_unnormalizer_processor.safetensors"}


def sha256_file(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(8 * 1024**2), b""):
            digest.update(block)
    return digest.hexdigest()


def file_identity(path):
    info = Path(path).stat()
    return {"path": str(Path(path).resolve()), "inode": info.st_ino, "device": info.st_dev,
            "size": info.st_size, "mtime_ns": info.st_mtime_ns, "ctime_ns": info.st_ctime_ns}


def verify_weights(path, expected_hash, expected_size, cache_path=None):
    identity = file_identity(path)
    if identity["size"] != expected_size:
        raise RuntimeError("Complete model file size differs from checkpoint receipt")
    cached = None
    if cache_path and cache_path.exists():
        try:
            cached = json.loads(cache_path.read_text())
        except (OSError, ValueError):
            pass
    reused = bool(cached and cached.get("file") == identity and cached.get("sha256") == expected_hash)
    digest = cached["sha256"] if reused else sha256_file(path)
    if digest != expected_hash or file_identity(path) != identity:
        raise RuntimeError("Complete model SHA256 mismatch or changed during verification")
    if cache_path and not reused:
        cache_path.parent.mkdir(parents=True, exist_ok=True)
        temporary = cache_path.with_suffix(".tmp")
        temporary.write_text(json.dumps({"file": identity, "sha256": digest}) + "\n")
        temporary.replace(cache_path)
    return {"sha256": digest, "bytes": identity["size"], "cache_reused": reused,
            "file_identity": identity}


def verify_manifest(folder, expected, required):
    if not isinstance(expected, dict) or not required.issubset(expected):
        raise RuntimeError(f"Missing pinned asset hashes: {sorted(required - set(expected or {}))}")
    identities = {}
    for name, digest in expected.items():
        if Path(name).name != name or not isinstance(digest, str) or len(digest) != 64:
            raise RuntimeError("Expected plain asset filenames and SHA256 digests")
        path = folder / name
        identity = file_identity(path)
        if sha256_file(path) != digest or file_identity(path) != identity:
            raise RuntimeError(f"Processor/tokenizer SHA256 mismatch: {path}")
        identities[name] = identity
    return identities


class Runtime:
    """Runtime(settings), predict(sample, noise=None) -> (actions[50,7], timing_s)."""

    def __init__(self, settings):
        started = time.perf_counter()
        self.settings = dict(settings)
        for key in ("HF_HUB_OFFLINE", "TRANSFORMERS_OFFLINE", "HF_DATASETS_OFFLINE",
                    "HF_HUB_DISABLE_TELEMETRY", "PYTHONNOUSERSITE"):
            os.environ[key] = "1"
        os.environ["TOKENIZERS_PARALLELISM"] = "false"
        os.environ.setdefault("TORCHINDUCTOR_COMPILE_THREADS", "4")
        os.environ.setdefault("MAX_JOBS", "4")
        model_dir = Path(settings["model"]).resolve(strict=True)
        tokenizer_dir = Path(settings["tokenizer"]).resolve(strict=True)
        if any((model_dir / name).exists() for name in ("adapter_config.json", "adapter_model.safetensors")):
            raise RuntimeError("Expert model B must be a complete policy with no adapter files")
        cache = Path(settings["hash_cache_dir"]).resolve() / "model_hash_cache.json" \
            if settings.get("hash_cache_dir") else None
        identity = verify_weights(model_dir / "model.safetensors", settings["expected_model_sha256"],
                                  settings["expected_model_bytes"], cache)
        pipeline_expected = settings["expected_pipeline_sha256"]
        tokenizer_expected = settings["expected_tokenizer_sha256"]
        pipeline_ids = verify_manifest(model_dir, pipeline_expected, REQUIRED_PIPELINE)
        tokenizer_ids = verify_manifest(tokenizer_dir, tokenizer_expected, {"tokenizer.json", "tokenizer_config.json"})
        saved_config = json.loads((model_dir / "config.json").read_text())
        if saved_config.get("use_peft") is not False:
            raise RuntimeError("Saved model B config must explicitly disable PEFT")

        import numpy as np
        import torch
        from lerobot.configs import PreTrainedConfig
        from lerobot.policies import make_pre_post_processors
        from lerobot.policies.pi05.modeling_pi05 import PI05Policy
        from pi05_low_memory import load_base_streaming

        if importlib.metadata.version("lerobot") != "0.6.1":
            raise RuntimeError("This runtime requires inspected LeRobot 0.6.1")
        self.torch, self.np = torch, np
        self.device = torch.device(settings.get("device", "cuda"))
        if self.device.type not in ("cuda", "cpu"):
            raise RuntimeError("Use CPU or cuda API (NVIDIA CUDA / AMD ROCm)")
        if self.device.type == "cuda" and not torch.cuda.is_available():
            raise RuntimeError("Requested GPU is unavailable; no implicit CPU fallback")
        fraction = settings.get("gpu_memory_fraction")
        if fraction is not None:
            if not isinstance(fraction, (int, float)) or isinstance(fraction, bool) or not 0 < fraction <= 1:
                raise RuntimeError("gpu_memory_fraction must be in (0,1]")
            if self.device.type == "cuda":
                device_index = self.device.index if self.device.index is not None else torch.cuda.current_device()
                torch.cuda.set_per_process_memory_fraction(float(fraction), device_index)
        torch.set_num_threads(int(settings.get("num_threads", 8)))
        precision = settings.get("float32_matmul_precision", "high")
        if precision not in ("highest", "high", "medium"):
            raise RuntimeError("Unsupported float32_matmul_precision")
        torch.set_float32_matmul_precision(precision)
        cfg = PreTrainedConfig.from_pretrained(str(model_dir), local_files_only=True,
            cli_overrides=[f"--device={self.device}", "--dtype=bfloat16",
                           "--gradient_checkpointing=false", "--compile_model=false", "--push_to_hub=false"])
        expected = {"chunk_size": 50, "num_inference_steps": 10, "n_obs_steps": 1,
                    "use_relative_actions": False, "use_peft": False, "max_action_dim": 32}
        for key, value in expected.items():
            if getattr(cfg, key) != value:
                raise RuntimeError(f"Unexpected complete policy configuration: {key}")
        if set(cfg.input_features) != {"observation.state", *CAMERAS} \
                or tuple(cfg.input_features["observation.state"].shape) != (7,) \
                or set(cfg.output_features) != {"action"} or tuple(cfg.output_features["action"].shape) != (7,):
            raise RuntimeError("Unexpected model B input/output features")
        cfg.pretrained_path = str(model_dir)
        self.preprocessor, self.postprocessor = make_pre_post_processors(
            policy_cfg=cfg, pretrained_path=str(model_dir),
            preprocessor_overrides={"tokenizer_processor": {"tokenizer_name": str(tokenizer_dir)},
                                   "device_processor": {"device": str(self.device)}},
            postprocessor_overrides={"device_processor": {"device": "cpu"}})
        load_started = time.perf_counter()
        requested_io = settings.get("streaming_io", "auto")
        if requested_io not in ("auto", "mmap", "file"):
            raise RuntimeError("streaming_io must be auto, mmap or file")
        selected_io = ("file" if torch.version.hip else "mmap") if requested_io == "auto" else requested_io
        self.policy, self.load_proof = load_base_streaming(model_dir, cfg, require_native=True,
                                                          read_strategy=selected_io)
        if type(self.policy) is not PI05Policy or hasattr(self.policy, "peft_config") \
                or any("lora_" in name for name, _ in self.policy.named_parameters()):
            raise RuntimeError("Expected a complete unwrapped PI05Policy without LoRA")
        self.policy.eval().requires_grad_(False)
        if file_identity(model_dir / "model.safetensors") != identity["file_identity"]:
            raise RuntimeError("Complete model changed while loading")
        for folder, assets in ((model_dir, pipeline_ids), (tokenizer_dir, tokenizer_ids)):
            if any(file_identity(folder / name) != proof for name, proof in assets.items()):
                raise RuntimeError("Processor/tokenizer file changed while loading")
        self.compile_inference = settings.get("compile_inference", False)
        if type(self.compile_inference) is not bool:
            raise RuntimeError("compile_inference requires an explicit boolean")
        compile_target = settings.get("compile_target", "denoise_step")
        compile_mode = None
        if self.compile_inference:
            torch._dynamo.config.suppress_errors = False
            if compile_target == "denoise_step":
                compile_mode = "reduce-overhead"
                self.policy.model.denoise_step = torch.compile(self.policy.model.denoise_step, mode=compile_mode)
            elif compile_target == "sample_actions":
                compile_mode = cfg.compile_mode
                self.policy.model.sample_actions = torch.compile(self.policy.model.sample_actions, mode=compile_mode)
            else:
                raise RuntimeError("Unsupported compile target; no fallback attempted")
        self.sync()
        self.metadata = {"model_id": settings["model_id"], "model_sha256": identity["sha256"],
            "model_format": "full-native-PI05", "model_verification": identity,
            "model_directory": str(model_dir), "peft_present": False, "adapter_tensors": 0,
            "pipeline_files_sha256": dict(pipeline_expected), "tokenizer_files_sha256": dict(tokenizer_expected),
            "full_loading": self.load_proof, "device_api": str(self.device),
            "streaming_io": selected_io, "streaming_io_requested": requested_io,
            "device": torch.cuda.get_device_name(self.device) if self.device.type == "cuda" else "CPU",
            "torch_cuda": torch.version.cuda, "torch_hip": torch.version.hip,
            "chunk_size": 50, "action_shape": [50, 7], "units": UNITS, "representation": REPRESENTATION,
            "num_inference_steps": 10, "batch_size": 1, "gradient_checkpointing": False,
            "compile_model": self.compile_inference, "compile_target": compile_target if self.compile_inference else None,
            "compile_mode": compile_mode, "float32_matmul_precision": torch.get_float32_matmul_precision(),
            "gpu_memory_fraction": fraction, "weights_loading_seconds": time.perf_counter() - load_started,
            "loading_seconds": time.perf_counter() - started, "robot_io": False,
            "warmup_predictions": 0, "warmup_seconds": 0,
            "versions": {name: importlib.metadata.version(name) for name in
                         ("torch", "lerobot", "transformers", "accelerate", "safetensors")}}

    def sync(self):
        if self.device.type == "cuda":
            self.torch.cuda.synchronize(self.device)

    def predict(self, sample, noise=None):
        torch = self.torch
        self.sync()
        begin = time.perf_counter()
        if sample["task"] != self.settings["task"]:
            raise ValueError("Task differs from the trained red-cube task")
        state = torch.as_tensor(sample["observation.state"], dtype=torch.float32, device="cpu")
        if tuple(state.shape) != (7,) or not torch.isfinite(state).all().item():
            raise ValueError("Expected finite state[7] in degree/metre units")
        clean = {"task": sample["task"], "observation.state": state}
        for key in CAMERAS:
            image = torch.as_tensor(sample[key], device="cpu")
            if tuple(image.shape) != (3, 224, 224):
                raise ValueError(f"Expected canonical RGB CHW image[3,224,224]: {key}")
            if image.dtype == torch.uint8:
                image = image.to(torch.float32) / 255.0
            elif image.is_floating_point():
                if not torch.isfinite(image).all().item() or image.min().item() < 0 or image.max().item() > 1:
                    raise ValueError("Floating RGB must be finite [0,1]")
                image = image.to(torch.float32)
            else:
                raise ValueError("RGB must be uint8 or floating [0,1]")
            clean[key] = image
        batch = self.preprocessor(clean)
        kwargs = {}
        if noise is not None:
            noise = torch.as_tensor(noise, dtype=torch.float32, device=self.device)
            if tuple(noise.shape) != (1, 50, 32) or not torch.isfinite(noise).all().item():
                raise ValueError("Expected finite initial noise[1,50,32]")
            kwargs["noise"] = noise
        self.sync()
        pre_end = time.perf_counter()
        self.policy.reset()
        with torch.inference_mode():
            prediction = self.policy.predict_action_chunk(batch, **kwargs)
        self.sync()
        model_end = time.perf_counter()
        with torch.inference_mode():
            actions = self.postprocessor(prediction).detach().cpu()
        self.sync()
        post_end = time.perf_counter()
        if tuple(actions.shape) != (1, 50, 7) or not torch.isfinite(actions).all().item():
            raise RuntimeError("Model produced invalid actions; no clipping is performed")
        return actions[0].tolist(), {"preprocess": pre_end - begin, "model": model_end - pre_end,
                                    "postprocess": post_end - model_end}

    def warmup(self, count=None):
        count = self.settings.get("warmup_predictions", 2) if count is None else count
        if type(count) is not int or count < 1:
            raise ValueError("At least one warmup prediction is required")
        started = time.perf_counter()
        sample = {"task": self.settings["task"], "observation.state": self.torch.zeros(7),
                  **{key: self.torch.zeros(3, 224, 224) for key in CAMERAS}}
        for _ in range(count):
            self.predict(sample)
        self.metadata.update(warmup_predictions=count, warmup_seconds=time.perf_counter() - started)
