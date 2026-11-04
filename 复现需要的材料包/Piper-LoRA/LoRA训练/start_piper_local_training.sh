#!/usr/bin/env bash
# Start local training: bash start_piper_local_training.sh DATASET_ROOT NEW_RUN_DIR STEPS
set -euo pipefail
PI05_ENTRY_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec bash "$PI05_ENTRY_DIR/train_piper_pi05_lora.sh" "$@"
