#!/usr/bin/env bash
# Official LeRobot merge + ACT training. Default mode only prints the commands.
set -euo pipefail

case "${1:---dry-run}" in
  --run) piper_act_execute=1 ;;
  --dry-run) piper_act_execute=0 ;;
  *) printf '用法：bash %s [--dry-run|--run]\n' "$0" >&2; exit 2 ;;
esac

source /home/aup/Lerobot/activate.sh
export OMP_NUM_THREADS=8
export MKL_NUM_THREADS=8
export OPENBLAS_NUM_THREADS=1

piper_act_data=/home/aup/Lerobot/datasets/rad_place_merged_v5
piper_act_tag="$(TZ=Asia/Singapore date +%Y%m%d_%H%M%S)"
piper_act_output="/home/aup/Lerobot/outputs/train/act_rad_place_${piper_act_tag}"

piper_act_merge=(
  lerobot-edit-dataset
  --new_repo_id local/rad_place_merged_v5
  --new_root "$piper_act_data"
  --operation.type merge
  --operation.repo_ids '["local/rad_place_031302", "local/rad_place_035859_clean"]'
  --operation.roots '["/home/aup/Lerobot/datasets/rad_place_v5_r03", "/home/aup/Lerobot/datasets/rad_place_v5_r04_clean"]'
  --operation.concatenate_videos false
  --operation.concatenate_data false
  --push_to_hub false
)

piper_act_train=(
  lerobot-train
  --dataset.repo_id local/rad_place_merged_v5
  --dataset.root "$piper_act_data"
  --dataset.video_backend torchcodec
  --dataset.eval_split 0.0
  --policy.type act
  --policy.device cpu
  --policy.push_to_hub false
  --batch_size 4
  --num_workers 0
  --steps 20000
  --eval_steps 0
  --env_eval_freq 0
  --save_freq 2000
  --log_freq 100
  --wandb.enable false
  --job_name act_rad_place_cpu
  --output_dir "$piper_act_output"
)

printf 'CPU ACT：8线程，batch=4，双路640×480，20000步。\n'
printf '全部52条用于训练，不预留验证集；使用默认ImageNet ResNet18骨干，首次会下载约45MB权重。\n'
printf '合并目录：%s\n训练输出：%s\n' "$piper_act_data" "$piper_act_output"
if (( ! piper_act_execute )); then
  printf '\n官方合并命令（新目录不存在时执行）：\n'
  printf '%q ' "${piper_act_merge[@]}"; printf '\n'
  printf '\n官方训练命令：\n'
  printf '%q ' "${piper_act_train[@]}"; printf '\n'
  printf '\n当前仅预览。开始合并及训练：bash %q --run\n' "$0"
  exit 0
fi

if [[ ! -e "$piper_act_data" ]]; then
  "${piper_act_merge[@]}"
fi

# Check expected inputs before reusing an existing merge. Do not overwrite data.
python - "$piper_act_data" <<'PY'
import json
from pathlib import Path
import sys

root = Path(sys.argv[1])
info = json.loads((root / 'meta/info.json').read_text())
assert info['total_episodes'] == 52 and info['total_frames'] == 20488, \
    '合并数据量与已核对的52条/20488帧不符，停止训练，请检查目录。'
assert info['fps'] == 20 and info['robot_type'] == 'piper_shared_observer'
assert info['features']['action']['shape'] == [7]
for role in ('global', 'wrist'):
    assert info['features'][f'observation.images.{role}']['shape'] == [480, 640, 3]
print('合并数据检查通过：52条、20488帧；全部用于训练，不划分验证集。', flush=True)
PY

if [[ -e "$piper_act_output" ]]; then
  printf '训练输出目录已存在，停止以避免覆盖：%s\n' "$piper_act_output" >&2
  exit 2
fi
mkdir -p "$(dirname -- "$piper_act_output")"
exec "${piper_act_train[@]}"
