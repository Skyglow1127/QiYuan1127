#!/usr/bin/env bash
set -euo pipefail
source /home/aup/Lerobot/projects/pi05_rocm/activate.sh
benchmark_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ulimit -c 0
exec env -u LD_LIBRARY_PATH HF_HUB_OFFLINE=1 HF_DATASETS_OFFLINE=1 \
  python -u "$benchmark_dir/supervise.py"
