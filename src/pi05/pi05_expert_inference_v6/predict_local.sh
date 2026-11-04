#!/usr/bin/env bash
# Recorded-frame inference only. The persistent ROCm service performs inference.
set -euo pipefail
task_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
task_output="$task_dir/predictions/$(TZ=Asia/Singapore date +%Y%m%d_%H%M%S_%N)"
exec env -u PYTHONPATH -u LD_LIBRARY_PATH PYTHONNOUSERSITE=1 \
  /home/aup/Lerobot/envs/piper-cpu/bin/python "$task_dir/client.py" \
  --url http://127.0.0.1:18807 --config "$task_dir/local_config.json" \
  --input "$task_dir/canonical_input.npz" --output "$task_output" \
  --warmup 0 --repeats 1 "$@"
