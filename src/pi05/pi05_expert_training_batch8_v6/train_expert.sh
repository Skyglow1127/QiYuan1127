#!/usr/bin/env bash
# Model B batch 8 only: original pi05_base -> full action expert, frozen VLM, no PEFT.
# Offline official LeRobot training; this script never accesses a robot.
set -euo pipefail

PI05_ENTRY_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if [[ $# -lt 2 || $# -gt 3 ]]; then
    printf 'Usage: bash %s DATASET_ROOT NEW_RUN_DIR [STEPS]\n' "$0" >&2
    exit 2
fi
PI05_DATA_ROOT="$(realpath "$1")"
PI05_RUN_DIR="$(realpath -m "$2")"
PI05_STEPS="${3:-20000}"
: "${PI05_BASE_MODEL_PATH:?Set original local pi05_base directory}"
: "${PI05_TOKENIZER_PATH:?Set verified local tokenizer directory}"
: "${PI05_EXPERIMENT_ROOT:?Set isolated model B experiment root}"
PI05_PYTHON="${PI05_PYTHON:-python}"
export PI05_BASE_MODEL_PATH PI05_TOKENIZER_PATH PI05_EXPERIMENT_ROOT
export HF_HUB_OFFLINE=1 HF_DATASETS_OFFLINE=1 TOKENIZERS_PARALLELISM=false
export PYTHONUNBUFFERED=1 PYTHONDONTWRITEBYTECODE=1
export PI05_KEEP_CHECKPOINTS="${PI05_KEEP_CHECKPOINTS:-1}"
if [[ ! "$PI05_STEPS" =~ ^[0-9]+$ ]] || (( PI05_STEPS < 1 )); then
    printf 'STEPS must be a positive integer.\n' >&2
    exit 2
fi
if [[ -e "$PI05_RUN_DIR" ]]; then
    printf 'Choose a new B output directory; refusing existing path: %s\n' "$PI05_RUN_DIR" >&2
    exit 2
fi
for PI05_FILE in train_expert_checked.py pi05_low_memory.py; do
    [[ -f "$PI05_ENTRY_DIR/$PI05_FILE" ]] || { printf 'Missing companion: %s\n' "$PI05_FILE" >&2; exit 2; }
done
"$PI05_PYTHON" - "$PI05_DATA_ROOT" <<'PY'
from importlib.metadata import version
import json
import math
import os
from pathlib import Path
import sys
import torch
root = Path(sys.argv[1])
info = json.loads((root / "meta/info.json").read_text())
stats = json.loads((root / "meta/stats.json").read_text())
names = [f"joint_{i}.deg" for i in range(1, 7)] + ["gripper.m"]
for key in ("observation.state", "action"):
    if info["features"][key]["shape"] != [7] or info["features"][key]["names"] != names:
        raise SystemExit(f"Unexpected {key} semantics: require absolute J1..6 deg, gripper m")
    low, high = stats[key]["q01"], stats[key]["q99"]
    if len(low) != 7 or len(high) != 7 or not all(math.isfinite(a) and math.isfinite(b) and b > a
                                                    for a, b in zip(low, high)):
        raise SystemExit(f"Invalid {key} quantile statistics")
for camera in ("observation.images.global", "observation.images.wrist"):
    if camera not in info["features"]:
        raise SystemExit(f"Missing camera {camera}")
if (info["total_episodes"], info["total_frames"], info["fps"]) != (52, 20488, 20):
    raise SystemExit("Expected original 52-episode / 20,488-frame / 20 Hz dataset")
base = Path(os.environ["PI05_BASE_MODEL_PATH"])
for name in ("config.json", "model.safetensors", "policy_preprocessor.json", "policy_postprocessor.json"):
    if not (base / name).is_file():
        raise SystemExit(f"Missing original base asset {name}")
if (base / "adapter_config.json").exists():
    raise SystemExit("Original B source must not contain an adapter config")
if version("lerobot") != "0.6.1" or not torch.cuda.is_available() or not torch.cuda.is_bf16_supported():
    raise SystemExit("Require LeRobot 0.6.1 and CUDA with BF16 support")
print(json.dumps({"dataset": str(root), "torch": torch.__version__, "cuda": torch.version.cuda,
                  "gpu": torch.cuda.get_device_name(0), "visible_devices": os.environ.get("CUDA_VISIBLE_DEVICES"),
                  "method": "frozen VLM + full original action expert; PEFT disabled", "batch": 8,
                  "seed": 1000}, ensure_ascii=False), flush=True)
PY

exec "$PI05_PYTHON" "$PI05_ENTRY_DIR/train_expert_checked.py" \
    --dataset.repo_id=local/piper_task \
    --dataset.root="$PI05_DATA_ROOT" \
    --dataset.video_backend=pyav \
    --dataset.eval_split=0.1 \
    --policy.type=pi05 \
    --policy.pretrained_path="$PI05_BASE_MODEL_PATH" \
    --policy.device=cuda \
    --policy.dtype=bfloat16 \
    --policy.normalization_mapping='{"VISUAL":"IDENTITY","STATE":"QUANTILES","ACTION":"QUANTILES"}' \
    --policy.use_relative_actions=false \
    --policy.n_obs_steps=1 \
    --policy.chunk_size=50 \
    --policy.n_action_steps=10 \
    --policy.num_inference_steps=10 \
    --policy.gradient_checkpointing=true \
    --policy.compile_model=false \
    --policy.freeze_vision_encoder=true \
    --policy.train_expert_only=true \
    --policy.use_peft=false \
    --policy.push_to_hub=false \
    --policy.optimizer_lr=2.5e-5 \
    --policy.scheduler_decay_lr=2.5e-6 \
    --policy.scheduler_warmup_steps=1000 \
    --policy.scheduler_decay_steps=30000 \
    --batch_size=8 \
    --seed=1000 \
    --num_workers=4 \
    --steps="$PI05_STEPS" \
    --save_freq="${PI05_SAVE_FREQ:-2000}" \
    --eval_steps="${PI05_EVAL_STEPS:-2000}" \
    --max_eval_samples=0 \
    --env_eval_freq=0 \
    --log_freq="${PI05_LOG_FREQ:-50}" \
    --output_dir="$PI05_RUN_DIR" \
    --job_name=piper_pi05_expert_only_B \
    --wandb.enable=false
