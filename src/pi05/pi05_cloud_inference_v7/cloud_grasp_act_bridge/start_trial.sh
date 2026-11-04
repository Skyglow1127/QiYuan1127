#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec env -u PYTHONPATH -u LD_LIBRARY_PATH \
  PYTHONNOUSERSITE=1 HF_HUB_OFFLINE=1 HF_DATASETS_OFFLINE=1 \
  HF_HUB_DISABLE_TELEMETRY=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=8 MKL_NUM_THREADS=8 \
  /home/aup/Lerobot/envs/piper-cpu/bin/python -B "$SCRIPT_DIR/start_trial.py" "$@"
