#!/usr/bin/env bash
# This launcher calls the unmodified official LeRobot training entry point.
set -euo pipefail
piper_act_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "$piper_act_root/env.sh"
piper_act_mode="${1:---dry-run}"
piper_act_steps=20000
piper_act_log_freq=100
piper_act_save_freq=2000
piper_act_kind=act_rad_place
case "$piper_act_mode" in
  --dry-run|--run) ;;
  --benchmark)
    piper_act_steps=100
    piper_act_log_freq=10
    piper_act_save_freq=100
    piper_act_kind=benchmark_act
    ;;
  --resume)
    : "${2:?Provide the absolute path to checkpoint pretrained_model/train_config.json}"
    piper_act_resume_steps="${3:-20000}"
    [[ "$piper_act_resume_steps" =~ ^[1-9][0-9]*$ ]] || { echo 'Total steps must be a positive integer.' >&2; exit 2; }
    piper_act_tag="$(TZ=Asia/Singapore date +%Y%m%d_%H%M%S)"
    mkdir -p "$piper_act_root/logs"
    lerobot-train --config_path="$2" --resume=true --steps="$piper_act_resume_steps" \
      2>&1 | tee "$piper_act_root/logs/resume_act_${piper_act_tag}.log"
    exit 0
    ;;
  *) printf 'Usage: bash %s [--dry-run|--benchmark|--run|--resume /ABS/train_config.json [total_steps]]\n' "$0" >&2; exit 2 ;;
esac
piper_act_data="$piper_act_root/datasets/rad_place_merged_v5"
python - "$piper_act_data" <<'PY'
import json,sys,torch
from pathlib import Path
info=json.loads((Path(sys.argv[1])/'meta/info.json').read_text())
assert info['total_episodes']==52 and info['total_frames']==20488
assert info['fps']==20 and info['features']['action']['shape']==[7]
for camera in ['global','wrist']:
    assert info['features'][f'observation.images.{camera}']['shape']==[480,640,3]
assert torch.cuda.is_available(), 'CUDA is unavailable; stopping.'
print('52 episodes / 20488 frames, all used for training, no validation split.',flush=True)
print('GPU:',torch.cuda.get_device_name(0),flush=True)
PY
piper_act_tag="$(TZ=Asia/Singapore date +%Y%m%d_%H%M%S)"
piper_act_output="$piper_act_root/outputs/${piper_act_kind}_${piper_act_tag}"
piper_act_command=(
  lerobot-train
  --dataset.repo_id=local/rad_place_merged_v5
  --dataset.root="$piper_act_data"
  --dataset.video_backend=torchcodec
  --dataset.eval_split=0.0
  --policy.type=act
  --policy.device=cuda
  --policy.use_amp=false
  --policy.push_to_hub=false
  --batch_size=8
  --num_workers=0
  --persistent_workers=false
  --steps="$piper_act_steps"
  --eval_steps=0
  --env_eval_freq=0
  --save_freq="$piper_act_save_freq"
  --log_freq="$piper_act_log_freq"
  --wandb.enable=false
  --job_name="$piper_act_kind"
  --output_dir="$piper_act_output"
)
printf '%q ' "${piper_act_command[@]}"; printf '\n'
if [[ "$piper_act_mode" == --dry-run ]]; then exit 0; fi
mkdir -p "$piper_act_root/logs"
printf '%s\n' "$piper_act_output" > "$piper_act_root/logs/latest_${piper_act_kind}_output.txt"
"${piper_act_command[@]}" 2>&1 | tee "$piper_act_root/logs/${piper_act_kind}_${piper_act_tag}.log"
