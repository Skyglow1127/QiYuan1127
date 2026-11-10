#!/usr/bin/env bash
set -euo pipefail
ROOT_DIR=$(cd "$(dirname "$0")/.." && pwd)
VIVADO_ROOT=${VIVADO_ROOT:-/home/aup/Xilinx/2025.2/Vivado}
export XILINX_VIVADO="$VIVADO_ROOT"
cd "$ROOT_DIR/build/e2e_full"
"$VIVADO_ROOT/bin/xsim" e2e -tclbatch "$ROOT_DIR/scripts/wave.tcl" -wdb "$ROOT_DIR/reports/end_to_end.wdb" -testplusarg FREQUENCY=350000 -testplusarg RUN_MS=115 -testplusarg "RESULTS=$ROOT_DIR/reports/wave_results.csv" >wave.log 2>&1
rg '^PASS end_to_end' wave.log
