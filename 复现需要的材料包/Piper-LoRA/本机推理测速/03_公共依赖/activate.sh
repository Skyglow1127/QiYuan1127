#!/usr/bin/env bash
# Source this file: source /home/aup/Lerobot/projects/pi05_rocm/activate.sh
source /home/aup/Lerobot/envs/pi05-rocm-7.14.1/bin/activate
export PYTHONNOUSERSITE=1
# The desktop shell includes ROS packages through PYTHONPATH. Keep this venv isolated.
unset PYTHONPATH
export HF_HOME=/home/aup/Lerobot/cache/pi05_rocm/huggingface
export TORCH_HOME=/home/aup/Lerobot/cache/pi05_rocm/torch
export TOKENIZERS_PARALLELISM=false
mkdir -p "$HF_HOME" "$TORCH_HOME"
