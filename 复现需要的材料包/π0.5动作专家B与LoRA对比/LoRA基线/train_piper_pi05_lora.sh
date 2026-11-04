#!/usr/bin/env bash
# LeRobot 0.6.1 / pi05_base / frozen base + LoRA.
# Run inside the prepared Python environment.
# Usage: bash train_piper_pi05_lora.sh DATASET_ROOT NEW_RUN_DIR [STEPS]
# DATASET_ROOT must contain a compatible LeRobot dataset, including synchronized
# images, observation.state, action, tasks, and q01/q99 state/action statistics.
# The model's output uses the action representation stored in this dataset.
# This script trains offline; it does not connect to or control a robot.
# Verified dataset: rad_place_merged_v5, LeRobot v3.0, 52 episodes,
# 20,488 frames at 20 Hz. State/action order: J1..J6 in degrees, gripper width
# reading in metres. Actions are absolute targets; keep their recorded units.
set -euo pipefail

PI05_SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if [[ ! -f "$PI05_SCRIPT_DIR/train_pi05_checked.py" ]]; then
    printf 'Missing companion file: %s/train_pi05_checked.py\n' "$PI05_SCRIPT_DIR" >&2
    exit 2
fi

if [[ $# -lt 2 || $# -gt 3 ]]; then
    printf 'Usage: bash %s DATASET_ROOT NEW_RUN_DIR [STEPS]\n' "$0" >&2
    exit 2
fi

PI05_DATASET_ROOT="$(realpath "$1")"
PI05_RUN_DIR="$(realpath -m "$2")"
PI05_TRAIN_STEPS="${3:-20000}"
PI05_BATCH_SIZE="${PI05_BATCH_SIZE:-2}"
PI05_BASE_MODEL_PATH="${PI05_BASE_MODEL_PATH:-lerobot/pi05_base}"

if [[ -d "$PI05_BASE_MODEL_PATH" ]]; then
    for PI05_ASSET in config.json model.safetensors policy_preprocessor.json policy_postprocessor.json; do
        if [[ ! -f "$PI05_BASE_MODEL_PATH/$PI05_ASSET" ]]; then
            printf 'Missing base-model asset: %s/%s\n' "$PI05_BASE_MODEL_PATH" "$PI05_ASSET" >&2
            exit 2
        fi
    done
fi

if [[ ! "$PI05_TRAIN_STEPS" =~ ^[0-9]+$ ]] || (( PI05_TRAIN_STEPS < 1 )); then
    printf 'STEPS must be a positive integer.\n' >&2
    exit 2
fi
if [[ ! "$PI05_BATCH_SIZE" =~ ^[0-9]+$ ]] || (( PI05_BATCH_SIZE < 1 )); then
    printf 'PI05_BATCH_SIZE must be a positive integer.\n' >&2
    exit 2
fi
if [[ ! -f "$PI05_DATASET_ROOT/meta/info.json" ]]; then
    printf 'Missing %s/meta/info.json: convert your data to LeRobot format first.\n' "$PI05_DATASET_ROOT" >&2
    exit 2
fi
if [[ -e "$PI05_RUN_DIR" ]]; then
    printf 'Run directory already exists; choose a new directory: %s\n' "$PI05_RUN_DIR" >&2
    exit 2
fi

python - "$PI05_DATASET_ROOT" <<'PY'
from importlib.metadata import version
import json
import math
from pathlib import Path
import sys
import torch

root = Path(sys.argv[1])
info = json.loads((root / "meta/info.json").read_text())
stats = json.loads((root / "meta/stats.json").read_text())
expected_names = [f"joint_{i}.deg" for i in range(1, 7)] + ["gripper.m"]
for key in ("observation.state", "action"):
    feature = info["features"][key]
    if feature["shape"] != [7] or feature["names"] != expected_names:
        raise SystemExit(f"Unexpected {key} schema; check dimension order and units first.")
    lo, hi = stats[key]["q01"], stats[key]["q99"]
    if len(lo) != 7 or len(hi) != 7 or not all(
        math.isfinite(a) and math.isfinite(b) and b > a for a, b in zip(lo, hi)
    ):
        raise SystemExit(f"Invalid {key} quantile statistics.")
for key in ("observation.images.global", "observation.images.wrist"):
    if key not in info["features"]:
        raise SystemExit(f"Missing camera feature: {key}")
if info["fps"] != 20:
    raise SystemExit("This configuration expects the verified 20 Hz dataset.")
print("Dataset:", root)
print("Episodes:", info["total_episodes"], "Frames:", info["total_frames"], "FPS:", info["fps"])
print("Action: absolute J1..J6 targets (deg), gripper width-mode value (m).")

installed = version("lerobot")
if installed != "0.6.1":
    raise SystemExit(f"This script targets lerobot==0.6.1; installed: {installed}")
if not torch.cuda.is_available():
    raise SystemExit("PyTorch cannot access the GPU.")
print("torch:", torch.__version__, "CUDA:", torch.version.cuda)
print("GPU:", torch.cuda.get_device_name(0))
if not torch.cuda.is_bf16_supported():
    raise SystemExit("The selected GPU/runtime does not report BF16 support.")
PY

# Full non-adapter parameters are frozen by PEFT. Explicit [] prevents opting
# any additional modules into full-parameter training. Explicit targets adapt
# action-expert attention and action/time projections, leaving the VLM frozen.
# The 10% split is per task, using complete episodes; for a single task with
# 52 episodes this means 46 train episodes and 6 validation episodes.
# The verified dataset has 18,231 train frames and 2,257 validation frames.
# max_eval_samples=0 evaluates all validation frames rather than an early prefix.
# Batch size 2 and 20,000 steps are starting points, not a hardware/performance
# guarantee. For a smoke test pass 100; use a fresh run directory for training.
python "$PI05_SCRIPT_DIR/train_pi05_checked.py" \
    --dataset.repo_id=local/piper_task \
    --dataset.root="$PI05_DATASET_ROOT" \
    --dataset.video_backend=pyav \
    --dataset.eval_split=0.1 \
    --policy.type=pi05 \
    --policy.pretrained_path="$PI05_BASE_MODEL_PATH" \
    --policy.device=cuda \
    --policy.dtype=bfloat16 \
    --policy.normalization_mapping='{"VISUAL":"IDENTITY","STATE":"QUANTILES","ACTION":"QUANTILES"}' \
    --policy.use_relative_actions=false \
    --policy.chunk_size=50 \
    --policy.n_action_steps=10 \
    --policy.gradient_checkpointing=true \
    --policy.compile_model=false \
    --policy.freeze_vision_encoder=true \
    --policy.train_expert_only=true \
    --policy.push_to_hub=false \
    --peft.method_type=LORA \
    --peft.r=16 \
    --peft.lora_alpha=16 \
    --peft.full_training_modules='[]' \
    --peft.target_modules='(.*\.gemma_expert\..*\.self_attn\.(q|v)_proj|model\.(action_in_proj|action_out_proj|time_mlp_in|time_mlp_out))' \
    --policy.optimizer_lr=1e-4 \
    --policy.scheduler_decay_lr=1e-5 \
    --batch_size="$PI05_BATCH_SIZE" \
    --num_workers=4 \
    --steps="$PI05_TRAIN_STEPS" \
    --save_freq=2000 \
    --eval_steps=2000 \
    --max_eval_samples=0 \
    --env_eval_freq=0 \
    --log_freq=50 \
    --output_dir="$PI05_RUN_DIR" \
    --job_name=piper_pi05_lora \
    --wandb.enable=false
