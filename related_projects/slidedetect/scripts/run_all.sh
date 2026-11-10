#!/usr/bin/env bash
set -euo pipefail
ROOT_DIR=$(cd "$(dirname "$0")/.." && pwd)
cd "$ROOT_DIR"
python3 model/reference.py generate
python3 model/extended.py generate
bash scripts/run_unit.sh
for PROFILE in small stride1 stride64 extended1 extended64; do
 bash scripts/run_replay.sh "$PROFILE"
done
for PROFILE in small full decimated precision precision_offset; do
 bash scripts/run_end_to_end.sh "$PROFILE"
done
python3 model/reference.py report
python3 model/extended.py report
bash scripts/run_implementation.sh
bash scripts/run_postroute.sh
bash scripts/capture_wave.sh
mkdir -p build/project_arithmetic
VIVADO_ROOT=${VIVADO_ROOT:-/home/aup/Xilinx/2025.2/Vivado}
"$VIVADO_ROOT/bin/vivado" -mode batch -nojournal -source scripts/check_project.tcl -log build/project_arithmetic/vivado.log >build/project_arithmetic/simulate.log 2>&1
python3 scripts/summarize.py
