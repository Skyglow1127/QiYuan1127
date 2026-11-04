#!/usr/bin/env python3
"""Read the actual saved ACT contract offline. No robot/policy is instantiated.

Example: /home/aup/Lerobot/envs/piper-cpu/bin/python measure/read_act_contract.py --output spec/act_contract.json --markdown spec/ACT_CONTRACT.md
Verification: .../bin/python measure/read_act_contract.py --verify spec/act_contract.json
"""
from __future__ import annotations
import argparse
import hashlib
from importlib import metadata
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
PROFILE = ROOT.parent / "deployment/act_launch_contact_repair.json"
DATASET = Path("/home/aup/Lerobot/datasets/rad_place_merged_v5")


def digest(path):
    with Path(path).open("rb") as handle:
        return hashlib.file_digest(handle, "sha256").hexdigest()


def read_contract(profile=PROFILE, dataset_root=DATASET):
    from safetensors.torch import load_file
    profile, dataset_root = Path(profile).resolve(), Path(dataset_root).resolve()
    runtime = json.loads(profile.read_text())
    checkpoint = Path(runtime["checkpoint"]).resolve()
    config = json.loads((checkpoint / "config.json").read_text())
    training = json.loads((checkpoint / "train_config.json").read_text())
    info = json.loads((dataset_root / "meta/info.json").read_text())
    stats = json.loads((dataset_root / "meta/stats.json").read_text())
    names = info["features"]["action"]["names"]
    if names != [f"joint_{i}.deg" for i in range(1, 7)] + ["gripper.m"]:
        raise ValueError("actual action semantic names do not match the reviewed adapter")
    if info["features"]["observation.state"]["names"] != names:
        raise ValueError("state/action axes differ")
    if config["type"] != "act" or config["chunk_size"] < runtime["n_action_steps"]:
        raise ValueError("invalid ACT/runtime action count")
    norm = load_file(str(checkpoint / "policy_preprocessor_step_3_normalizer_processor.safetensors"))
    keys = list(config["input_features"]) + ["action"]
    saved_stats = {key: {field: norm[key + "." + field].tolist() for field in ("mean", "std")} for key in keys}
    selected_dataset_stats = {key: {field: stats[key][field] for field in ("mean", "std", "min", "max", "count")} for key in keys}
    evidence_paths = [profile, *(checkpoint / name for name in (
        "config.json", "train_config.json", "model.safetensors", "policy_preprocessor.json", "policy_postprocessor.json",
        "policy_preprocessor_step_3_normalizer_processor.safetensors", "policy_postprocessor_step_0_unnormalizer_processor.safetensors")),
        dataset_root / "meta/info.json", dataset_root / "meta/stats.json",
        Path("/home/aup/Lerobot/train_act_rad_place_v5.sh"),
        ROOT.parent / "official_adapter/rollout_act_piper.py",
        ROOT.parent / "official_adapter/lerobot_robot_piper_shared/protocol.py",
        ROOT.parent / "official_adapter/lerobot_robot_piper_shared/devices.py"]
    import lerobot
    installed = Path(lerobot.__file__).parent
    evidence_paths.extend(installed / path for path in (
        "policies/act/configuration_act.py", "policies/act/modeling_act.py", "policies/act/processor_act.py",
        "datasets/video_utils.py", "processor/normalize_processor.py"))
    hashes = {str(path): digest(path) for path in evidence_paths if path.is_file()}
    if hashes[str(checkpoint / "model.safetensors")] != runtime["model_sha256"]:
        raise ValueError("profile model SHA256 does not match actual original model")
    return {
        "schema_version": 1, "checkpoint": str(checkpoint), "runtime_profile": str(profile),
        "dataset_root": str(dataset_root), "fps_dataset": info["fps"], "chunk_size": config["chunk_size"],
        "saved_n_action_steps": config["n_action_steps"], "runtime_n_action_steps": runtime["n_action_steps"],
        "temporal_ensemble_coeff": config["temporal_ensemble_coeff"], "n_obs_steps": config["n_obs_steps"],
        "input_features": config["input_features"], "output_features": config["output_features"],
        "camera_order": [key for key in config["input_features"] if key.startswith("observation.images.")],
        "action_names": names, "state_names": names, "action_units": ["deg"] * 6 + ["m"],
        "action_space": "absolute joint angles and absolute gripper width; mixed physical units, no IK",
        "normalization": {"mapping": config["normalization_mapping"], "eps": 1e-8,
                          "checkpoint_stats_float32": saved_stats, "dataset_stats_original": selected_dataset_stats,
                          "image_stats_source": "ImageNet override saved in processor, not the dataset image means"},
        "preprocessing": {"raw": "RGB uint8 HWC 480x640x3", "tensor": "float32 CHW /255 then saved preprocessor adds batch and normalizes",
                          "resize": None, "augmentation_enabled": training["dataset"]["image_transforms"]["enable"],
                          "use_imagenet_stats": training["dataset"]["use_imagenet_stats"],
                          "video_backend": training["dataset"]["video_backend"], "dataset_video": info["features"]["observation.images.global"]["info"]},
        "robot": {"dataset_type": info["robot_type"], "joint_count": 6, "gripper_travel_m": [0.0, 0.1],
                  "gripper_travel_source": "current deployment bounds, not an inferred training robot specification",
                  "link_geometry": "not encoded in this ACT checkpoint; no link geometry inferred"},
        "deployment_bounds": runtime["policy"]["limits"]["bounds"],
        "dataset": {"repo_id": training["dataset"]["repo_id"], "total_episodes": info["total_episodes"], "total_frames": info["total_frames"],
                    "data_path": info["data_path"], "video_path": info["video_path"], "all_used_for_training": True},
        "training": {"saved_root_cloud": training["dataset"]["root"], "steps": training["steps"], "batch_size": training["batch_size"],
                     "device": training["policy"]["device"], "seed": training["seed"],
                     "local_train_script_is_not_actual_cloud_training_config": True},
        "runtime": {"device_for_offline_adapter": "cpu", "joint_smoothing_alpha_existing": runtime["policy_joint_smoothing_alpha"],
                    "release_extra_steps_existing": runtime["policy_release_chunk_extension"]["extra_steps"],
                    "no_existing_runtime_changes": True},
        "versions": {name: metadata.version(name) for name in ("lerobot", "torch", "torchvision", "numpy", "safetensors")},
        "f_exec_hz": None, "hardware_validation": False, "evidence_sha256": hashes,
    }


def markdown(c):
    rows = [
        ("Checkpoint", "runtime profile", c["checkpoint"]),
        ("Input images / order", "saved config input_features", str(c["camera_order"]) + "; each 3×480×640 RGB"),
        ("State / action", "config + dataset meta/info.json", "7 / 7; " + ", ".join(c["action_names"])),
        ("chunk_size", "saved config", str(c["chunk_size"])),
        ("Execution consumption", "saved config / current repair profile", f"saved {c['saved_n_action_steps']}; runtime {c['runtime_n_action_steps']} (conditional extra {c['runtime']['release_extra_steps_existing']})"),
        ("Temporal ensemble", "saved config", str(c["temporal_ensemble_coeff"])),
        ("Dataset fps", "dataset meta/info.json", str(c["fps_dataset"])),
        ("Action space / units", "dataset feature names + passive decoder", c["action_space"]),
        ("Training robot", "dataset metadata", f"{c['robot']['dataset_type']}; six joints; gripper deployment envelope 0–0.1 m; checkpoint has no link geometry"),
        ("Normalization", "saved processor state", "MEAN_STD; exact float32 mean/std below; image ImageNet override"),
        ("Image transforms", "actual saved cloud train_config", "disabled; no resize; RGB CHW float32 /255 before saved normalization"),
        ("Training data", "dataset metadata", f"{c['dataset']['total_episodes']} episodes / {c['dataset']['total_frames']} frames, all training data"),
        ("f_exec", "unset deployment parameter", "unknown; offline rate scenarios do not select a physical execution rate"),
    ]
    text = ["# Actual ACT contract", "", "Generated by `measure/read_act_contract.py`; no model, data or existing deployment configuration was changed.", "", "| Item | Read from | Actual value |", "|---|---|---|"]
    text += [f"| {a} | {b} | {v} |" for a, b, v in rows]
    text += ["", "## Saved normalization values", "", "These tensors, rather than rounded values copied from dataset JSON, are consumed by the official saved pipelines.", "", "```json", json.dumps(c["normalization"]["checkpoint_stats_float32"], indent=2), "```", "", "Dataset original mean/std/min/max, complete input shapes, versions, source paths and SHA256 are in `act_contract.json`. Dataset image statistics differ from the saved ImageNet image statistics by design. Dataset gripper values include a negative tail; no implicit correction is hidden in normalization.", "", "`ActionPostprocessor.denormalize()` is the unchanged numerical path. Bounds rejection, explicit optional projection with correction flags, and Q16.16 encoding are separate. The six degree axes and metre gripper retain their own units in Q16.16.", "", "The local training shell script describes a CPU preparation path; the saved train_config records the actual cloud CUDA batch-8 training. Neither saved CUDA configuration nor filename implies that offline execution uses a GPU.", "", "This contract establishes software semantics. It does not establish physical command acceptance, placement success or collision safety.", ""]
    text += ["Q16.16 integer codes are constrained to `ceil(lower × 65536)` through `floor(upper × 65536)`. Non-grid endpoints round inward and can have an error approaching one LSB; intervals with no representable int32 code are rejected. A 0.1 m upper endpoint encodes as 6553, never 6554.", ""]
    return "\n".join(text)


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--profile", type=Path, default=PROFILE)
    p.add_argument("--dataset-root", type=Path, default=DATASET)
    p.add_argument("--output", type=Path, default=ROOT / "spec/act_contract.json")
    p.add_argument("--markdown", type=Path)
    p.add_argument("--verify", type=Path, help="re-read sources and compare with existing contract; writes nothing")
    args = p.parse_args(argv)
    try:
        c = read_contract(args.profile, args.dataset_root)
        if args.verify:
            if c != json.loads(args.verify.read_text()):
                raise ValueError("contract differs from current read-only evidence")
            print(json.dumps({"verified": True, "contract": str(args.verify), "model_sha256": c["evidence_sha256"][str(Path(c["checkpoint"])/"model.safetensors")]}))
        else:
            for target in (args.output, args.markdown):
                if target and target.exists():
                    raise ValueError(f"refusing to overwrite {target}")
            args.output.parent.mkdir(parents=True, exist_ok=True)
            args.output.write_text(json.dumps(c, ensure_ascii=False, indent=2, allow_nan=False) + "\n")
            if args.markdown:
                args.markdown.parent.mkdir(parents=True, exist_ok=True)
                args.markdown.write_text(markdown(c))
            print(json.dumps({"contract": str(args.output), "fps_dataset": c["fps_dataset"], "runtime_n_action_steps": c["runtime_n_action_steps"]}))
        return 0
    except (OSError, ValueError, KeyError) as exc:
        p.exit(2, str(exc) + "\n")


if __name__ == "__main__":
    raise SystemExit(main())
