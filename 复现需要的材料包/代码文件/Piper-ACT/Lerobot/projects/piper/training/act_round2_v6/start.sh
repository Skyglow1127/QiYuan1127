#!/usr/bin/env bash
set -euo pipefail
piper_round2_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source /root/autodl-tmp/lerobot_act/env.sh
exec python "$piper_round2_root/run_training.py" "${1:---check}"
