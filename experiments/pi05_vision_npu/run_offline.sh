#!/usr/bin/env bash
set -euo pipefail
TASK_ROOT=/home/aup/NPU/pi05_vision_npu
export PYTHONDONTWRITEBYTECODE=1 HF_HUB_OFFLINE=1 TRANSFORMERS_OFFLINE=1
export TMPDIR="$TASK_ROOT/cache/tmp"
export XDG_CACHE_HOME="$TASK_ROOT/cache/xdg"
mkdir -p "$TMPDIR" "$XDG_CACHE_HOME"
exec /home/aup/Lerobot/envs/pi05-rocm-7.14.1/bin/python "$TASK_ROOT/code/run_offline.py" "$@"
