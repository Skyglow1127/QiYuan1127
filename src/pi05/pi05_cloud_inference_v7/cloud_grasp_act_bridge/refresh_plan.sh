#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
env -u PYTHONPATH -u LD_LIBRARY_PATH \
  PYTHONNOUSERSITE=1 PYTHONDONTWRITEBYTECODE=1 HF_HUB_OFFLINE=1 HF_DATASETS_OFFLINE=1 \
  /home/aup/Lerobot/envs/piper-cpu/bin/python -B "$SCRIPT_DIR/prepare_trial.py" --refresh "$@"
exec bash "$SCRIPT_DIR/start_trial.sh"
