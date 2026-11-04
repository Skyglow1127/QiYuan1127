#!/usr/bin/env bash
set -euo pipefail
piper_round2_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
piper_local_env="$(cd -- "$piper_round2_root/../../../.." && pwd)/local_act_v5/env.sh"
source "$piper_local_env"
exec python "$piper_round2_root/run_training.py" "${1:---check}"
