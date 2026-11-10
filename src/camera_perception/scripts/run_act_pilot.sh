#!/usr/bin/env bash
set -euo pipefail
# Offline only: no device commands, no ROS launch, no power/enable operation.
cd "$(dirname "$0")/.."
act_python="$PWD/.venv-models/bin/python"
act_output="${1:-$PWD/output/act_pilot_$(date +%Y%m%d_%H%M%S)}"
if [[ -e "$act_output" ]]; then
  echo 'Output already exists; choose a new directory to preserve the previous run.' >&2
  exit 1
fi
"$act_python" -m scripts.policies.prepare --config configs/policies/act_lift_pilot.json --output "$act_output/data"
"$act_python" -m scripts.policies.train --dataset "$act_output/data" --output "$act_output/train" --target-space delta_from_q --steps 300 --seed 42
"$act_python" -m scripts.policies.shadow --dataset "$act_output/data" --checkpoint "$act_output/train/best.pt" --output "$act_output/shadow" --count 20
