#!/usr/bin/env python3
"""B1: compare official training video decode/preprocessing with the new builder.

Example (existing environment; no camera, robot or downloads):
  /home/aup/Lerobot/envs/piper-cpu/bin/python measure/check_preprocess.py --output measure/preprocess_check

The official decoder independently produces training float tensors and raw RGB
uint8 frames. The new deployment builder starts with the latter, not with the
training tensor. Two separately loaded saved processor instances are compared.
"""
from __future__ import annotations
import argparse
import hashlib
from importlib import metadata
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "sw"))


def digest(path):
    with Path(path).open("rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()


def recorded_pair(dataset_root, episode, frame):
    """Use actual episode video offsets, never assume a frame starts at file zero."""
    import pyarrow.parquet as pq
    root = Path(dataset_root)
    info = json.loads((root / "meta/info.json").read_text())
    matches = []
    for path in sorted((root / "meta/episodes").rglob("*.parquet")):
        matches.extend(row for row in pq.read_table(path).to_pylist() if row["episode_index"] == episode)
    if len(matches) != 1:
        raise ValueError("episode must identify exactly one metadata row")
    ep = matches[0]
    if not 0 <= frame < ep["length"]:
        raise ValueError("frame outside episode")
    data_path = root / info["data_path"].format(chunk_index=ep["data/chunk_index"] , file_index=ep["data/file_index"])
    rows = pq.read_table(data_path, filters=[("episode_index", "=", episode), ("frame_index", "=", frame)]).to_pylist()
    if len(rows) != 1:
        raise ValueError("episode/frame must identify exactly one data row")
    item = rows[0]
    images = {}
    for role in ("global", "wrist"):
        key = f"observation.images.{role}"
        video_path = root / info["video_path"].format(video_key=key,
            chunk_index=ep[f"videos/{key}/chunk_index"], file_index=ep[f"videos/{key}/file_index"])
        images[role] = (video_path, ep[f"videos/{key}/from_timestamp"] + float(item["timestamp"]))
    return item, images, data_path


def run_check(contract_path, output, episode=41, frames=(0, 313)):
    import numpy as np
    import torch
    from PIL import Image
    from lerobot.datasets.video_utils import decode_video_frames_torchcodec
    from rt_adapter.act import ObservationBuilder, load_processors
    contract = json.loads(Path(contract_path).read_text())
    if contract["preprocessing"]["video_backend"] != "torchcodec" or contract["preprocessing"]["augmentation_enabled"]:
        raise ValueError("check does not match saved training decoder/augmentation contract")
    output = Path(output)
    output.mkdir(parents=True, exist_ok=False)
    torch.set_num_threads(8)
    builder = ObservationBuilder(contract["checkpoint"])
    _, official_preprocessor, _ = load_processors(contract["checkpoint"])
    comparisons, source_hashes = [], {}
    for frame in frames:
        row, video_sources, data_path = recorded_pair(contract["dataset_root"], episode, frame)
        raw_images = {}
        training = {"observation.state": torch.tensor(row["observation.state"], dtype=torch.float32)}
        for role, (video_path, timestamp) in video_sources.items():
            key = f"observation.images.{role}"
            # Independent calls: training path returns official float32 CHW;
            # deployment path starts from uint8 HWC and performs its own conversion.
            training[key] = decode_video_frames_torchcodec(video_path, [timestamp], 1e-4, return_uint8=False)[0]
            raw_chw = decode_video_frames_torchcodec(video_path, [timestamp], 1e-4, return_uint8=True)[0]
            raw_images[role] = raw_chw.permute(1, 2, 0).contiguous().numpy().copy()
            source_hashes[str(video_path)] = digest(video_path)
        source_hashes[str(data_path)] = digest(data_path)
        trained = official_preprocessor(training)
        deployed = builder.build(raw_images, row["observation.state"])
        deployed_raw = builder.raw_observation(raw_images, row["observation.state"])
        for role, (video_path, timestamp) in video_sources.items():
            key = f"observation.images.{role}"
            raw_delta = abs(training[key] - deployed_raw[key])
            normalized_delta = abs(trained[key] - deployed[key])
            saved = contract["normalization"]["checkpoint_stats_float32"][key]
            std = torch.tensor(saved["std"], dtype=torch.float32)
            input_lsb_delta = normalized_delta.squeeze(0) * std * 255
            train_rgb = np.clip(np.rint(training[key].permute(1, 2, 0).numpy()*255), 0, 255).astype(np.uint8)
            deploy_rgb = raw_images[role]
            difference = np.abs(train_rgb.astype(np.int16)-deploy_rgb.astype(np.int16)).astype(np.uint8)
            display = np.concatenate((train_rgb, deploy_rgb, np.minimum(difference.astype(np.uint16)*32, 255).astype(np.uint8)), axis=1)
            png = output / f"ep{episode}_frame{frame:04d}_{role}_training_builder_diff32.png"
            Image.fromarray(display).save(png)
            raw_png = output / f"ep{episode}_frame{frame:04d}_{role}_rawRGB.png"
            Image.fromarray(deploy_rgb).save(raw_png)
            comparisons.append({"episode": episode, "frame": frame, "camera": role,
                "video_path": str(video_path), "video_timestamp_s": timestamp,
                "raw_rgb_sha256": hashlib.sha256(deploy_rgb.tobytes()).hexdigest(),
                "raw_float_max_difference": float(raw_delta.max()),
                "raw_max_difference_lsb": float(raw_delta.max()*255),
                "normalized_tensor_max_difference": float(normalized_delta.max()),
                "normalized_difference_input_referred_lsb": float(input_lsb_delta.max()),
                "passed": bool(raw_delta.max()*255 <= 1+1e-6 and input_lsb_delta.max() <= 1+1e-6),
                "comparison_png": str(png.resolve()), "raw_png_sha256": digest(raw_png)})
        if not torch.equal(trained["observation.state"], deployed["observation.state"]):
            raise ValueError("state preprocessing differs")
    result = {"schema_version": 1, "stage": "B1", "passed": all(item["passed"] for item in comparisons),
        "scope": "same recorded video-decoded RGB frame -> official saved normalization; independent training float decode vs new uint8 HWC builder",
        "contract_sha256": digest(contract_path), "comparisons": comparisons, "source_sha256": source_hashes,
        "training_side": "installed official decode_video_frames_torchcodec(return_uint8=False) then independently loaded saved preprocessor",
        "deployment_side": "separate official raw uint8 decode -> new ObservationBuilder RGB HWC conversion -> second saved preprocessor",
        "current_live_path": {"verified_live": False, "camera_bytes": "current camera service forwards SDK MJPEG; adapter OpenCV-decodes BGR then converts RGB",
            "training_storage": "camera RGB was H264 yuv420p CRF18 encoded then training torchcodec decoded",
            "conclusion": "same post-decode preprocessing aligned; original live MJPEG versus training H264 RGB is not proven pixel-identical and no live acquisition was performed",
            "no_old_deployment_modification": True},
        "versions": {name: metadata.version(name) for name in ("lerobot", "torch", "torchvision", "torchcodec", "numpy", "pillow")},
        "hardware_io": False, "downloads": False,
    }
    (output / "result.json").write_text(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False) + "\n")
    (output / "README.md").write_text("# Preprocessing comparison\n\nEach PNG shows training RGB, builder RGB, and absolute difference amplified 32 times.\n\nThe comparison begins with the same recorded video frame. It verifies RGB/channel order, dimensions, scaling and saved normalization. It does not claim that live SDK MJPEG and H264 training storage preserve identical pixels. No live camera or robot was accessed. Exact results and source SHA256 are in result.json.\n")
    return result


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--contract", type=Path, default=ROOT / "spec/act_contract.json")
    p.add_argument("--output", type=Path, required=True, help="new evidence directory; refuses overwrite")
    p.add_argument("--episode", type=int, default=41)
    p.add_argument("--frames", default="0,313", help="comma-separated recorded episode frame indices")
    args = p.parse_args(argv)
    try:
        frames = tuple(int(value) for value in args.frames.split(","))
        if not frames or len(set(frames)) != len(frames):
            raise ValueError("frames must be unique")
        result = run_check(args.contract, args.output, args.episode, frames)
        print(json.dumps({"passed": result["passed"], "comparisons": len(result["comparisons"]), "result": str(args.output / "result.json")}))
        return 0 if result["passed"] else 1
    except (OSError, ValueError, KeyError, ImportError) as exc:
        p.exit(2, str(exc) + "\n")


if __name__ == "__main__":
    raise SystemExit(main())
