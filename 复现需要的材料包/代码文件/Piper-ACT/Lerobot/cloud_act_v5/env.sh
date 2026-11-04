#!/usr/bin/env bash
# Source in each cloud terminal. The image's CUDA PyTorch is reused read-only.
_piper_act_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "$_piper_act_root/.venv/bin/activate"
unset PYTHONPATH PYTHONHOME
export PYTHONNOUSERSITE=1
export HF_HOME="$_piper_act_root/cache/huggingface"
export TORCH_HOME="$_piper_act_root/cache/torch"
export HF_LEROBOT_HOME="$_piper_act_root/datasets"
export WANDB_MODE=disabled
export OMP_NUM_THREADS=4
export MKL_NUM_THREADS=4
export OPENBLAS_NUM_THREADS=1
export TOKENIZERS_PARALLELISM=false
unset ACCELERATE_MIXED_PRECISION
unset _piper_act_root
