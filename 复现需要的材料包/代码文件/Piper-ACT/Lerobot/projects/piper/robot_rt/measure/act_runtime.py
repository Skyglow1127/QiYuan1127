"""Local recorded-input ACT helpers. No robot, camera, transport or SDK imports."""
from __future__ import annotations
import hashlib
import json
import math
import os
from pathlib import Path
import platform
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
PROJECT = ROOT.parent
DEFAULT_PROFILE = PROJECT / "deployment/act_launch_contact_repair.json"
DEFAULT_INPUTS = PROJECT / "logs/act_cloud_inference_v7/inputs"


def sha256(path):
    with Path(path).open("rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()


def process_inventory():
    result = subprocess.run(["ps", "-eo", "pid,etimes,pcpu,comm,args"], capture_output=True, text=True, check=True)
    lines = result.stdout.splitlines()
    markers = ("run_act_piper", "rollout_act_piper.py", "launch_act.py", "act_piper_launch", "piper_local_executor", "run_policy_executor",
               "lerobot-record", "lerobot_record", "piper_policy_executor")
    active, safe_processes = [], []
    for line in lines[1:]:
        columns = line.split(None, 4)
        if len(columns) != 5 or int(columns[0]) == os.getpid():
            continue
        script = None
        if "python" in columns[3]:
            script = next((Path(token).name for token in columns[4].split() if token.endswith(".py")), None)
        safe = {"pid": int(columns[0]), "elapsed_s": int(columns[1]),
                "cpu_percent": float(columns[2]), "comm": columns[3], "script_basename": script}
        safe_processes.append(safe)
        # Never mistake this check's shell or an editor/document containing a name for a rollout.
        if ("python" in columns[3] or columns[3] in ("bash", "sh")) and any(marker in columns[4] for marker in markers):
            active.append(safe)
    return {"captured_monotonic_s": time.monotonic(), "processes": safe_processes,
            "active_rollouts": active, "pid_namespace_caveat": "ps sees this execution namespace; run on host for a host-wide check"}


def require_idle_host():
    inventory = process_inventory()
    if inventory["active_rollouts"]:
        raise RuntimeError("Resource blocked: active physical rollout found; nothing was stopped: " + repr(inventory["active_rollouts"]))
    return inventory


def load_recordings(directory):
    import numpy as np
    records = []
    for path in sorted(Path(directory).glob("*.npz")):
        with np.load(path, allow_pickle=False) as raw:
            images = {role: raw["observation.images." + role].copy() for role in ("global", "wrist")}
            state = raw["observation.state"].copy()
        if state.shape != (7,) or not np.isfinite(state).all():
            raise ValueError(f"Invalid recorded state: {path}")
        for role, value in images.items():
            if value.shape != (480, 640, 3) or value.dtype != np.uint8:
                raise ValueError(f"Invalid RGB uint8 recorded image {role}: {path}")
        records.append({"path": str(path.resolve()), "sha256": sha256(path), "images": images, "state": state})
    if not records:
        raise ValueError("No recorded NPZ inputs found; no live capture fallback")
    return records


def load_runtime(profile_path=DEFAULT_PROFILE, input_dir=DEFAULT_INPUTS, device="cpu", threads=8):
    os.environ.setdefault("HF_HUB_OFFLINE", "1")
    os.environ.setdefault("TRANSFORMERS_OFFLINE", "1")
    import torch
    sys.path.insert(0, str(ROOT / "sw"))
    from rt_adapter.act import load_processors, ObservationBuilder, ActionPostprocessor
    from lerobot.policies.act.modeling_act import ACTPolicy
    profile = json.loads(Path(profile_path).read_text())
    checkpoint = Path(profile["checkpoint"])
    if sha256(checkpoint / "model.safetensors") != profile["model_sha256"]:
        raise ValueError("Checkpoint SHA256 differs from current original profile")
    if threads < 1:
        raise ValueError("threads must be positive")
    torch.set_num_threads(threads)
    torch.set_num_interop_threads(1)
    if device != "cpu" and (not torch.cuda.is_available() or not device.startswith("cuda")):
        raise ValueError("Requested accelerator is unavailable in installed Torch runtime")
    cfg, preprocessor, official_post = load_processors(checkpoint, device=device)
    cfg.n_action_steps = int(profile["n_action_steps"])
    if cfg.temporal_ensemble_coeff is not None:
        raise ValueError("This queue alignment/benchmark requires the unchanged no-ensemble profile")
    policy = ACTPolicy.from_pretrained(checkpoint, config=cfg, local_files_only=True, strict=True)
    policy.eval()
    builder = ObservationBuilder(checkpoint, device=device)
    postprocessor = ActionPostprocessor(checkpoint, device=device)
    records = load_recordings(input_dir)
    return dict(torch=torch, cfg=cfg, policy=policy, builder=builder, postprocessor=postprocessor,
                official_post=official_post, official_pre=preprocessor, records=records,
                profile=profile, checkpoint=checkpoint, device=device, threads=threads,
                hashes={"profile": sha256(profile_path), "checkpoint_files": {
                    path.name: sha256(path) for path in sorted(checkpoint.iterdir()) if path.is_file()},
                    "recordings": [{"path": record["path"], "sha256": record["sha256"]} for record in records]})


def sync(runtime):
    if runtime["device"].startswith("cuda"):
        runtime["torch"].cuda.synchronize()


def telemetry(runtime):
    frequencies = []
    for path in Path("/sys/devices/system/cpu").glob("cpu[0-9]*/cpufreq/scaling_cur_freq"):
        try:
            frequencies.append(float(path.read_text()) / 1000)
        except (OSError, ValueError):
            pass
    temperatures = {}
    for zone in Path("/sys/class/thermal").glob("thermal_zone*"):
        try:
            temperatures[zone.name + ":" + (zone / "type").read_text().strip()] = float((zone / "temp").read_text()) / 1000
        except (OSError, ValueError):
            pass
    rss = None
    try:
        for line in Path("/proc/self/status").read_text().splitlines():
            if line.startswith("VmRSS:"):
                rss = int(line.split()[1]) * 1024
    except OSError:
        pass
    return {"monotonic_s": time.monotonic(), "rss_bytes": rss,
            "cpu_scaling_frequency_mhz": {"min": min(frequencies), "mean": sum(frequencies)/len(frequencies), "max": max(frequencies)} if frequencies else None,
            "thermal_zone_c": temperatures, "gpu_allocated_bytes": runtime["torch"].cuda.memory_allocated() if runtime["device"].startswith("cuda") else None}


def environment(runtime):
    torch = runtime["torch"]
    cpu = next((line.split(":", 1)[1].strip() for line in Path("/proc/cpuinfo").read_text().splitlines() if line.startswith("model name")), platform.processor())
    return {"python": sys.version, "platform": platform.platform(), "cpu": cpu,
            "torch": torch.__version__, "torch_cuda": torch.version.cuda,
            "torch_hip": torch.version.hip, "cuda_available": torch.cuda.is_available(),
            "device": runtime["device"], "threads": torch.get_num_threads(),
            "interop_threads": torch.get_num_interop_threads(), "dtype": "float32", "amp": False,
            "chunk_size": runtime["cfg"].chunk_size, "n_action_steps": runtime["cfg"].n_action_steps,
            "recorded_input_only": True, "camera_can_robot_connections": False}


def write_json(path, content):
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    with Path(path).open("x", encoding="utf-8") as stream:
        json.dump(content, stream, indent=2, ensure_ascii=False, allow_nan=False)
        stream.write("\n")
