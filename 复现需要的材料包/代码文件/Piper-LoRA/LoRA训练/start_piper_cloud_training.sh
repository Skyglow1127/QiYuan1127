#!/usr/bin/env bash
# Run on the configured 5090 instance:
# bash start_piper_cloud_training.sh DATASET_ROOT NEW_RUN_DIR STEPS
set -euo pipefail
source /root/autodl-tmp/piper_pi05/bin/piper_cloud_env.sh
exec bash /root/autodl-tmp/piper_pi05/bin/train_piper_pi05_lora.sh "$@"
