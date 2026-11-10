#!/usr/bin/env bash
set -euo pipefail
unset PYTHONPATH PYTHONHOME LD_LIBRARY_PATH VIRTUAL_ENV XILINX_XRT
unset XILINX_VITIS XILINX_VITIS_AIETOOLS RYZEN_AI_INSTALLATION_PATH
export PATH=/home/aup/D1T_旧资料归档/2026-10-28_from_Lerobot/envs/smolvla-d1-cpu/bin:/usr/local/bin:/usr/bin:/bin
export PYTHONNOUSERSITE=1
export HF_HOME='/home/aup/D1T_旧资料归档/2026-10-28_from_Lerobot/storage/smolvla_d1_storage/cache/huggingface'
export HF_HUB_DISABLE_TELEMETRY=1
export TOKENIZERS_PARALLELISM=false
export OMP_NUM_THREADS=4
export MKL_NUM_THREADS=4
export OPENBLAS_NUM_THREADS=4
export WANDB_MODE=disabled
exec "$@"
