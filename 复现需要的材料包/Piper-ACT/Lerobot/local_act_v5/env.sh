#!/usr/bin/env bash
# Source the existing local LeRobot environment and keep data/caches on this host.
export PIPER_ACT_LOCAL_ROOT="${PIPER_ACT_LOCAL_ROOT:-/home/aup/Lerobot}"
export PIPER_ACT_DEVICE="${PIPER_ACT_DEVICE:-cpu}"
source "$PIPER_ACT_LOCAL_ROOT/activate.sh"
unset PYTHONPATH PYTHONHOME
export PYTHONNOUSERSITE=1
export HF_HOME="$PIPER_ACT_LOCAL_ROOT/cache/huggingface"
export TORCH_HOME="$PIPER_ACT_LOCAL_ROOT/cache/torch"
export HF_LEROBOT_HOME="$PIPER_ACT_LOCAL_ROOT/datasets"
export WANDB_MODE=disabled
export OMP_NUM_THREADS=4
export MKL_NUM_THREADS=4
export OPENBLAS_NUM_THREADS=1
export TOKENIZERS_PARALLELISM=false
unset ACCELERATE_MIXED_PRECISION
