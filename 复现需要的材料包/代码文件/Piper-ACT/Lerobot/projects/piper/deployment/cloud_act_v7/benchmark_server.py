#!/usr/bin/env python3
"""Isolated ACT fp32 inference benchmark, with no robot/camera/network objects.

Use through a persistent SSH stdin/stdout pipe; do not allocate an SSH TTY.
Each frame is a four-byte unsigned big-endian size followed by that many bytes.
The server first emits a UTF-8 JSON ``type=ready`` frame. Requests are NPZ files
with exactly observation.state (little-endian float32[7], degrees/metres) and
observation.images.global / observation.images.wrist (RGB uint8[480,640,3]).
Alternatively use observation.images.global.jpeg / observation.images.wrist.jpeg
(uint8[N] JPEG bytes, each <=1MiB, decoded RGB size exactly 640x480).
Responses are JSON frames with ``predictions`` [100,7], in degrees/metres after
the saved checkpoint postprocessor. They are predictions only, never commands.
Bad NPZ requests produce an error and preserve framing; bad/truncated framing
produces one fatal error and exits. Clean EOF exits. Stdout is protocol-only.

Example: python benchmark_server.py --checkpoint /path/pretrained_model
Self-test: add --self-test --warmups 3 --samples 20 (plain JSON, no framing).
Only local weights are accepted, and no source/model/default file is written.
"""

from __future__ import annotations

import argparse
import ast
from contextlib import redirect_stdout
import hashlib
import importlib.metadata
import io
import json
import os
from pathlib import Path
import struct
import sys
import time
import zipfile
import zlib


PROTOCOL_VERSION = 1
MAX_FRAME_BYTES = 4 * 1024 * 1024
MAX_HEADER_BYTES = 4096
MAX_JPEG_BYTES = 1024 * 1024
MAX_JPEG_HEADER_BYTES = 64 * 1024
STATE_KEY = "observation.state"
CAMERA_KEYS = ("observation.images.global", "observation.images.wrist")
JPEG_KEYS = tuple(key + ".jpeg" for key in CAMERA_KEYS)
INPUT_LAYOUT = {
    STATE_KEY: ((7,), "<f4", 7 * 4),
    **{key: ((480, 640, 3), "|u1", 480 * 640 * 3) for key in CAMERA_KEYS},
}
ACTION_NAMES = [f"joint_{index}.deg" for index in range(1, 7)] + ["gripper.m"]


class ProtocolError(ValueError):
    """The peer's framing cannot be recovered without a new connection."""


class InvalidInput(ValueError):
    """An entire frame was read, but its contents are not an observation."""


def read_exact(stream, length: int) -> bytes:
    chunks = []
    remaining = length
    while remaining:
        chunk = stream.read(remaining)
        if not chunk:
            raise ProtocolError(f"Truncated frame: missing {remaining} bytes")
        chunks.append(chunk)
        remaining -= len(chunk)
    return b"".join(chunks)


def read_frame(stream, max_bytes: int = MAX_FRAME_BYTES) -> bytes | None:
    first = stream.read(1)
    if first == b"":
        return None
    header = first + read_exact(stream, 3)
    size = struct.unpack(">I", header)[0]
    if size == 0 or size > max_bytes:
        raise ProtocolError(f"Frame size {size} outside 1..{max_bytes}")
    return read_exact(stream, size)


def write_json_frame(stream, value: dict) -> None:
    payload = json.dumps(value, ensure_ascii=False, allow_nan=False,
                         separators=(",", ":")).encode("utf-8")
    if len(payload) > MAX_FRAME_BYTES:
        raise ProtocolError("Response exceeds frame size bound")
    frame = struct.pack(">I", len(payload)) + payload
    offset = 0
    while offset < len(frame):
        written = stream.write(memoryview(frame)[offset:])
        if written is None or written <= 0:
            raise ProtocolError('Response stream did not accept a complete frame')
        offset += written
    stream.flush()


def _validate_jpeg_header(data: bytes, key: str) -> None:
    """Read bounded JPEG marker headers before an image decoder can allocate."""
    if not 0 < len(data) <= MAX_JPEG_BYTES or data[:2] != b"\xff\xd8":
        raise InvalidInput(f"{key}: require JPEG bytes <=1MiB")
    position = 2
    while position < min(len(data), MAX_JPEG_HEADER_BYTES):
        if data[position] != 0xff:
            raise InvalidInput(f"{key}: malformed JPEG marker")
        while position < min(len(data), MAX_JPEG_HEADER_BYTES) and data[position] == 0xff:
            position += 1
        if position >= min(len(data), MAX_JPEG_HEADER_BYTES):
            break
        marker = data[position]
        position += 1
        if marker in (0x00, 0x01, 0xd8, 0xd9, 0xda) or 0xd0 <= marker <= 0xd7:
            raise InvalidInput(f"{key}: JPEG frame header missing/invalid")
        if position + 2 > len(data):
            raise InvalidInput(f"{key}: truncated JPEG segment")
        size = struct.unpack(">H", data[position:position + 2])[0]
        end = position + size
        if size < 2 or end > len(data) or end > MAX_JPEG_HEADER_BYTES:
            raise InvalidInput(f"{key}: truncated/oversized JPEG header")
        if marker in (0xc0, 0xc1, 0xc2):
            body = data[position + 2:end]
            if len(body) < 6:
                raise InvalidInput(f"{key}: truncated JPEG frame")
            height, width = struct.unpack(">HH", body[1:5])
            components = body[5]
            if (body[0] != 8 or (width, height) != (640, 480)
                    or components not in (1, 3) or size != 8 + 3 * components):
                raise InvalidInput(f"{key}: require 8-bit 640x480 JPEG with 1/3 components")
            return
        # Other JPEG coding modes are outside the decoder/input contract.
        if 0xc0 <= marker <= 0xcf and marker not in (0xc4, 0xc8, 0xcc):
            raise InvalidInput(f"{key}: unsupported JPEG coding mode")
        position = end
    raise InvalidInput(f"{key}: JPEG frame header missing or exceeds64KiB")


def _validate_npy(raw: bytes, key: str) -> None:
    """Check shape/size before numpy can allocate from an untrusted header."""
    if raw[:6] != b"\x93NUMPY" or len(raw) < 10:
        raise InvalidInput(f"{key}: invalid NPY magic/header")
    version = tuple(raw[6:8])
    if version == (1, 0):
        offset = 10
        header_size = struct.unpack("<H", raw[8:10])[0]
    elif version in ((2, 0), (3, 0)) and len(raw) >= 12:
        offset = 12
        header_size = struct.unpack("<I", raw[8:12])[0]
    else:
        raise InvalidInput(f"{key}: unsupported NPY version")
    if not 0 < header_size <= MAX_HEADER_BYTES or offset + header_size > len(raw):
        raise InvalidInput(f"{key}: invalid/oversized NPY header")
    try:
        header = ast.literal_eval(raw[offset:offset + header_size].decode("latin1"))
    except (ValueError, SyntaxError, UnicodeError, RecursionError) as exc:
        raise InvalidInput(f"{key}: invalid NPY header dictionary") from exc
    if not isinstance(header, dict) or set(header) != {"descr", "fortran_order", "shape"}:
        raise InvalidInput(f"{key}: invalid NPY header fields")
    if key in JPEG_KEYS:
        expected_shape, expected_dtype = header["shape"], "|u1"
        if (not isinstance(expected_shape, tuple) or len(expected_shape) != 1
                or type(expected_shape[0]) is not int or not 1 <= expected_shape[0] <= MAX_JPEG_BYTES):
            raise InvalidInput(f"{key}: require uint8 JPEG vector of1..1MiB bytes")
        expected_bytes = expected_shape[0]
    else:
        expected_shape, expected_dtype, expected_bytes = INPUT_LAYOUT[key]
    if (header["shape"] != expected_shape or header["descr"] != expected_dtype
            or header["fortran_order"] is not False):
        raise InvalidInput(f"{key}: require C-order {expected_dtype}{expected_shape}")
    if len(raw) != offset + header_size + expected_bytes:
        raise InvalidInput(f"{key}: NPY byte count does not match fixed shape")
    if key in JPEG_KEYS:
        _validate_jpeg_header(raw[offset + header_size:], key)


def validate_npz(payload: bytes) -> str:
    """Bound both compressed and expanded inputs without extracting files."""
    if not 0 < len(payload) <= MAX_FRAME_BYTES:
        raise InvalidInput("NPZ payload exceeds frame size bound")
    raw_names = {key + ".npy" for key in INPUT_LAYOUT}
    jpeg_names = {key + ".npy" for key in (STATE_KEY, *JPEG_KEYS)}
    try:
        with zipfile.ZipFile(io.BytesIO(payload)) as archive:
            members = archive.infolist()
            names = {item.filename for item in members}
            if len(members) != 3 or names not in (raw_names, jpeg_names):
                raise InvalidInput("NPZ must contain exactly the three documented observation keys")
            codec = "raw_rgb_uint8" if names == raw_names else "jpeg"
            for member in members:
                key = member.filename[:-4]
                max_size = (MAX_JPEG_BYTES if key in JPEG_KEYS else INPUT_LAYOUT[key][2]) + MAX_HEADER_BYTES + 12
                if (member.flag_bits & 1 or member.file_size > max_size
                        or member.compress_type not in (zipfile.ZIP_STORED, zipfile.ZIP_DEFLATED)):
                    raise InvalidInput(f"{key}: unsupported/oversized NPZ member")
                _validate_npy(archive.read(member), key)
    except (zipfile.BadZipFile, zipfile.LargeZipFile, OSError, EOFError,
            RuntimeError, NotImplementedError, zlib.error) as exc:
        raise InvalidInput(f"Invalid NPZ: {exc}") from exc
    return codec


def decode_observation(payload: bytes, np, jpeg_decoder: str = "pillow") -> tuple[dict, str, str]:
    """Decode a validated request and fingerprint the actual model input bytes."""
    if jpeg_decoder not in ("pillow", "opencv"):
        raise ValueError(f"Unsupported JPEG decoder: {jpeg_decoder}")
    codec = validate_npz(payload)
    keys = INPUT_LAYOUT if codec == "raw_rgb_uint8" else (STATE_KEY, *JPEG_KEYS)
    try:
        with np.load(io.BytesIO(payload), allow_pickle=False) as archive:
            arrays = {key: archive[key] for key in keys}
    except (ValueError, OSError, EOFError) as exc:
        raise InvalidInput(f"Cannot decode NPZ: {exc}") from exc
    if not np.isfinite(arrays[STATE_KEY]).all():
        raise InvalidInput("observation.state must be finite")
    if codec == "jpeg":
        if jpeg_decoder == "pillow":
            from PIL import Image
        else:
            import cv2
        for key in CAMERA_KEYS:
            encoded = arrays.pop(key + ".jpeg")
            if jpeg_decoder == "pillow":
                try:
                    with Image.open(io.BytesIO(encoded.tobytes())) as image:
                        if image.format != "JPEG" or image.size != (640, 480):
                            raise InvalidInput(f"{key}: JPEG format/dimensions mismatch")
                        arrays[key] = np.array(image.convert("RGB"), dtype=np.uint8)
                except (ValueError, OSError, SyntaxError, Image.DecompressionBombError) as exc:
                    raise InvalidInput(f"{key}: cannot decode JPEG: {exc}") from exc
            else:
                try:
                    # Pillow does not apply EXIF orientation implicitly. Preserve
                    # that input contract rather than rotating the model image.
                    image = cv2.imdecode(encoded, cv2.IMREAD_COLOR | cv2.IMREAD_IGNORE_ORIENTATION)
                    if (image is None or image.shape != (480, 640, 3)
                            or image.dtype != np.uint8):
                        raise InvalidInput(f"{key}: decoded JPEG shape/dtype mismatch")
                    arrays[key] = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
                except (ValueError, cv2.error) as exc:
                    raise InvalidInput(f"{key}: cannot decode JPEG: {exc}") from exc
            if arrays[key].shape != (480, 640, 3) or arrays[key].dtype != np.uint8:
                raise InvalidInput(f"{key}: decoded JPEG shape mismatch")
            arrays[key] = np.ascontiguousarray(arrays[key])
    digest = hashlib.sha256()
    # Fixed lengths remove ambiguity: float32[7], then two HWC uint8 RGB images.
    for key in (STATE_KEY, *CAMERA_KEYS):
        digest.update(arrays[key].tobytes(order="C"))
    return arrays, codec, digest.hexdigest()


def serve_stream(reader, writer, infer, metadata: dict) -> int:
    """A synchronous request/response loop; no background thread or socket."""
    write_json_frame(writer, {"ok": True, "type": "ready", **metadata})
    sequence = 0
    while True:
        try:
            payload = read_frame(reader)
        except ProtocolError as exc:
            write_json_frame(writer, {"ok": False, "type": "error", "fatal": True,
                                      "sequence": sequence, "error": str(exc)})
            return 2
        if payload is None:
            return 0
        sequence += 1
        try:
            result = infer(payload)
            write_json_frame(writer, {"ok": True, "type": "prediction",
                                      "sequence": sequence, **result})
        except InvalidInput as exc:
            write_json_frame(writer, {"ok": False, "type": "error", "fatal": False,
                                      "sequence": sequence, "error": str(exc)})
        except Exception as exc:
            # A numerical/CUDA/model failure is not a reason to keep serving.
            write_json_frame(writer, {"ok": False, "type": "error", "fatal": True,
                                      "sequence": sequence,
                                      "error": f"{type(exc).__name__}: {exc}"})
            return 1


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def checkpoint_files(checkpoint: Path) -> dict:
    names = {"config.json", "model.safetensors", "policy_preprocessor.json",
             "policy_postprocessor.json"}
    if (checkpoint / "train_config.json").is_file():
        names.add("train_config.json")
    for name in ("policy_preprocessor.json", "policy_postprocessor.json"):
        config = json.loads((checkpoint / name).read_text())
        for step in config.get("steps", []):
            if step.get("state_file"):
                names.add(step["state_file"])
    result = {}
    for name in sorted(names):
        path = (checkpoint / name).resolve()
        if not path.is_relative_to(checkpoint) or not path.is_file():
            raise ValueError(f"Missing/unsafe local checkpoint file: {name}")
        result[name] = {"sha256": sha256_file(path), "bytes": path.stat().st_size}
    return result


def precision_metadata(torch) -> dict:
    """Report actual process settings, including PyTorch's implicit TF32 gates."""
    matmul_tf32 = bool(torch.backends.cuda.matmul.allow_tf32)
    cudnn_tf32 = bool(torch.backends.cudnn.allow_tf32)
    return {"dtype": "float32", "amp": False, "use_amp": False,
            "tf32": matmul_tf32 or cudnn_tf32,
            "cuda_matmul_allow_tf32": matmul_tf32,
            "cudnn_allow_tf32": cudnn_tf32,
            "float32_matmul_precision": torch.get_float32_matmul_precision(),
            "cudnn_benchmark": bool(torch.backends.cudnn.benchmark)}


class BenchmarkEngine:
    def __init__(self, checkpoint: Path, device: str, threads: int, expected_sha256: str | None,
                 jpeg_decoder: str = "pillow"):
        if jpeg_decoder not in ("pillow", "opencv"):
            raise ValueError(f"Unsupported JPEG decoder: {jpeg_decoder}")
        self.jpeg_decoder = jpeg_decoder
        started = time.perf_counter()
        files = checkpoint_files(checkpoint)
        model_hash = files["model.safetensors"]["sha256"]
        if expected_sha256 and model_hash != expected_sha256:
            raise ValueError(f"Model SHA256 mismatch: {model_hash}")
        # Process-local offline settings; no downloads, installs or cache writes.
        os.environ["HF_HUB_OFFLINE"] = "1"
        os.environ["HF_DATASETS_OFFLINE"] = "1"
        os.environ["HF_HUB_DISABLE_TELEMETRY"] = "1"
        os.environ["OMP_NUM_THREADS"] = str(threads)
        os.environ["MKL_NUM_THREADS"] = str(threads)
        import numpy as np
        import torch
        from lerobot.configs import PreTrainedConfig
        from lerobot.policies import make_pre_post_processors
        from lerobot.policies.act.modeling_act import ACTPolicy

        if device == "cuda" and not torch.cuda.is_available():
            raise RuntimeError("CUDA requested but unavailable; no automatic CPU fallback")
        torch.set_num_threads(threads)
        torch.set_num_interop_threads(1)
        self.opencv_threads = None
        if jpeg_decoder == 'opencv':
            import cv2
            # Small 640x480 pairs do not need the entire25-vCPU pool, which
            # otherwise competes with torch preprocessing/kernel dispatch.
            cv2.setNumThreads(1)
            self.opencv_threads = cv2.getNumThreads()
        torch.manual_seed(0)
        torch.set_float32_matmul_precision("highest")
        torch.backends.cuda.matmul.allow_tf32 = False
        torch.backends.cudnn.allow_tf32 = False
        torch.backends.cudnn.benchmark = False
        self.np, self.torch, self.device = np, torch, device
        config = PreTrainedConfig.from_pretrained(
            str(checkpoint), local_files_only=True,
            cli_overrides=[f"--device={device}", "--use_amp=false",
                           "--pretrained_backbone_weights=null"],
        )
        if (config.type != "act" or config.use_peft or config.n_obs_steps != 1
                or config.chunk_size != 100
                or set(config.input_features) != set(INPUT_LAYOUT)
                or tuple(config.input_features[STATE_KEY].shape) != (7,)
                or any(tuple(config.image_features[key].shape) != (3, 480, 640) for key in CAMERA_KEYS)
                or tuple(config.output_features["action"].shape) != (7,)):
            raise ValueError("Checkpoint is not the fixed global/wrist ACT100x7 layout")
        config.pretrained_path = checkpoint
        self.policy = ACTPolicy.from_pretrained(
            str(checkpoint), config=config, local_files_only=True, strict=True,
        ).to(device=device, dtype=torch.float32)
        self.policy.eval().requires_grad_(False)
        self.pre, self.post = make_pre_post_processors(
            policy_cfg=config, pretrained_path=str(checkpoint),
            preprocessor_overrides={
                "device_processor": {"device": device, "float_dtype": "float32"},
                "normalizer_processor": {"device": device}},
            postprocessor_overrides={
                "device_processor": {"device": "cpu", "float_dtype": "float32"},
                "unnormalizer_processor": {"device": device}},
        )
        self.sync()
        self.metadata = {
            "protocol_version": PROTOCOL_VERSION, "max_frame_bytes": MAX_FRAME_BYTES,
            "response_framing": "coalesced_header_body",
            "checkpoint": str(checkpoint), "model_sha256": model_hash,
            "checkpoint_files": files, "device": device,
            **precision_metadata(torch), "threads": threads,
            "chunk_size": 100, "action_names": ACTION_NAMES,
            "action_units": ["deg"] * 6 + ["m"],
            "versions": {name: importlib.metadata.version(name)
                         for name in ("lerobot", "torch", "torchvision", "numpy")},
            "cuda_runtime": torch.version.cuda,
            "gpu": torch.cuda.get_device_name(0) if device == "cuda" else None,
            "load_ms": (time.perf_counter() - started) * 1000,
            "timing_scope": "synchronized stages; decode_npz includes optional JPEG decode and decoded input hash; excludes stdin/stdout/SSH and JSON response encoding",
            "accepted_input_codecs": ["raw_rgb_uint8", "jpeg"],
            "jpeg_decoder": jpeg_decoder,
            "opencv_threads": self.opencv_threads,
            "decoded_observation_hash_order": [STATE_KEY, *CAMERA_KEYS],
            "decoded_observation_hash_encoding": "concatenated C-order bytes: little-endian float32[7], RGB uint8[480,640,3], RGB uint8[480,640,3]",
            "hardware_objects_created": 0, "hardware_commands_sent": 0,
        }

    def sync(self):
        if self.device == "cuda":
            self.torch.cuda.synchronize()

    def synthetic_payload(self) -> bytes:
        np = self.np
        rng = np.random.default_rng(0)
        output = io.BytesIO()
        np.savez(output, **{
            STATE_KEY: np.array([-10, 60, -50, 0, 10, 0, .06], dtype=np.float32),
            **{key: rng.integers(0, 256, (480, 640, 3), dtype=np.uint8) for key in CAMERA_KEYS}})
        return output.getvalue()

    def infer_npz(self, payload: bytes) -> dict:
        np, torch = self.np, self.torch
        started = time.perf_counter()
        arrays, codec, decoded_hash = decode_observation(payload, np, self.jpeg_decoder)
        decoded = time.perf_counter()
        observation = {STATE_KEY: torch.from_numpy(arrays[STATE_KEY].copy())}
        for key in CAMERA_KEYS:
            # Same RGB HWC uint8 -> CHW float32 [0,1] as the local live adapter.
            observation[key] = torch.from_numpy(arrays[key].copy()).permute(2, 0, 1).float().div(255).contiguous()
        tensorized = time.perf_counter()
        self.policy.reset()
        self.pre.reset()
        self.post.reset()
        with torch.inference_mode():
            self.sync()
            pre_started = time.perf_counter()
            batch = self.pre(observation)
            self.sync()
            pre_done = time.perf_counter()
            normalized = self.policy.predict_action_chunk(batch)
            self.sync()
            forward_done = time.perf_counter()
            prediction = self.post(normalized).detach().cpu().contiguous()
            self.sync()
            post_done = time.perf_counter()
        array = prediction.numpy()
        if array.shape != (1, 100, 7) or not np.isfinite(array).all():
            raise RuntimeError(f"Invalid/nonfinite model output: {array.shape}")
        array = array[0]
        result = {
            "predictions": array.tolist(), "prediction_shape": [100, 7],
            "prediction_sha256": hashlib.sha256(array.tobytes()).hexdigest(),
            "input_sha256": hashlib.sha256(payload).hexdigest(),
            "input_codec": codec,
            "decoded_observation_sha256": decoded_hash,
            "timings_ms": {
                "decode_npz": (decoded - started) * 1000,
                "tensorize": (tensorized - decoded) * 1000,
                "reset_and_sync": (pre_started - tensorized) * 1000,
                "preprocess": (pre_done - pre_started) * 1000,
                "forward": (forward_done - pre_done) * 1000,
                "postprocess_cpu": (post_done - forward_done) * 1000,
                "pipeline": (post_done - started) * 1000,
            },
        }
        return result


def quantiles(values: list[float]) -> dict:
    values = sorted(values)
    def percentile(fraction):
        position = (len(values) - 1) * fraction
        lower = int(position)
        upper = min(lower + 1, len(values) - 1)
        return values[lower] + (values[upper] - values[lower]) * (position - lower)
    return {"min": values[0], "p50": percentile(.5), "p90": percentile(.9),
            "p95": percentile(.95), "max": values[-1], "mean": sum(values) / len(values)}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--checkpoint", required=True, type=Path)
    parser.add_argument("--device", choices=("cuda", "cpu"), default="cuda")
    parser.add_argument("--threads", type=int, default=8)
    parser.add_argument("--warmups", type=int, default=3)
    parser.add_argument("--self-test", action="store_true")
    parser.add_argument("--samples", type=int, default=20)
    parser.add_argument("--expected-sha256")
    parser.add_argument("--jpeg-decoder", choices=("pillow", "opencv"), default="pillow",
                        help="JPEG decoder; input shape/byte bounds are checked before either decoder")
    args = parser.parse_args()
    if not 1 <= args.threads <= 256 or not 1 <= args.warmups <= 100 or not 1 <= args.samples <= 10000:
        parser.error("threads/warmups/samples outside 1..256 / 1..100 / 1..10000")
    if args.expected_sha256 and (len(args.expected_sha256) != 64
            or any(char not in "0123456789abcdef" for char in args.expected_sha256)):
        parser.error("--expected-sha256 must be 64 lowercase hex characters")
    protocol_out = sys.stdout.buffer
    # Some third-party imports/model loaders print to stdout. Protect framing.
    with redirect_stdout(sys.stderr):
        engine = BenchmarkEngine(args.checkpoint.expanduser().resolve(), args.device,
                                 args.threads, args.expected_sha256, args.jpeg_decoder)
        payload = engine.synthetic_payload()
        warmup_results = [engine.infer_npz(payload) for _ in range(args.warmups)]
        engine.metadata["warmups"] = args.warmups
        engine.metadata["warmup_pipeline_ms"] = [entry["timings_ms"]["pipeline"] for entry in warmup_results]
        if not args.self_test:
            return serve_stream(sys.stdin.buffer, protocol_out, engine.infer_npz, engine.metadata)
        results = [engine.infer_npz(payload) for _ in range(args.samples)]
        first = engine.np.asarray(results[0]["predictions"])
        report = {
            "ok": True, "type": "self_test", **engine.metadata,
            "input_kind": "fixed_synthetic_rgb_and_state_not_task_validation",
            "samples": args.samples, "input_bytes": len(payload),
            "input_sha256": hashlib.sha256(payload).hexdigest(),
            "input_codec": results[0]["input_codec"],
            "decoded_observation_sha256": results[0]["decoded_observation_sha256"],
            "quantiles_ms": {stage: quantiles([entry["timings_ms"][stage] for entry in results])
                             for stage in results[0]["timings_ms"]},
            "sample_timings_ms": [entry["timings_ms"] for entry in results],
            "prediction_sha256": results[0]["prediction_sha256"],
            "predictions": results[0]["predictions"],
            "max_abs_difference_from_first": max(float(engine.np.max(engine.np.abs(
                engine.np.asarray(entry["predictions"]) - first))) for entry in results),
        }
    sys.stdout.write(json.dumps(report, ensure_ascii=False, allow_nan=False, indent=2) + "\n")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (BrokenPipeError, KeyboardInterrupt):
        raise SystemExit(1)
    except Exception as exc:
        print(f"{type(exc).__name__}: {exc}", file=sys.stderr, flush=True)
        raise SystemExit(1)
