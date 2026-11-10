#!/usr/bin/env bash
# Repeat the frozen offline experiment using saved evidence. No hardware APIs.
set -euo pipefail
if [[ $# -ne 1 ]]; then
  echo "Usage: bash scripts/run_preaction_sequence_pilot.sh NEW_OUTPUT_DIRECTORY" >&2
  exit 2
fi
pilot_workspace=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
cd -- "$pilot_workspace"
pilot_output=$1
if [[ -e "$pilot_output" ]]; then
  echo "Output must be a new directory: $pilot_output" >&2
  exit 2
fi
pilot_evidence=output/preaction_sequence_round1_20261025
pilot_protocol="$pilot_evidence/protocol_global.json"
pilot_inventory="$pilot_evidence/inventory.json"
pilot_legacy=output/preaction_unheld_round2_20261023/forecast/best.pt
pilot_python=.venv-models/bin/python
mkdir -p -- "$pilot_output"
"$pilot_python" -m scripts.preaction.prepare_sequence_data \
  --inventory "$pilot_inventory" --protocol "$pilot_protocol" \
  --output "$pilot_output/data" 2>&1 | tee "$pilot_output/data.log"
for pilot_mode in sequence endpoint; do
  "$pilot_python" -m scripts.preaction.train_sequence \
    --data "$pilot_output/data" --protocol "$pilot_protocol" \
    --legacy "$pilot_legacy" --mode "$pilot_mode" \
    --output "$pilot_output/train_$pilot_mode" 2>&1 | tee "$pilot_output/train_$pilot_mode.log"
done
"$pilot_python" -m scripts.preaction.evaluate_sequence \
  --data "$pilot_output/data" --protocol "$pilot_protocol" \
  --sequence "$pilot_output/train_sequence/best.pt" \
  --endpoint "$pilot_output/train_endpoint/best.pt" \
  --output "$pilot_output/evaluation" 2>&1 | tee "$pilot_output/evaluation.log"
