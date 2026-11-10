#!/usr/bin/env bash
set -euo pipefail
ROOT_DIR=$(cd "$(dirname "$0")/.." && pwd)
VIVADO_ROOT=${VIVADO_ROOT:-/home/aup/Xilinx/2025.2/Vivado}
export XILINX_VIVADO="$VIVADO_ROOT"
export PATH="$VIVADO_ROOT/bin:$PATH"
for TEST in divider faults; do
 mkdir -p "$ROOT_DIR/build/$TEST"
 cd "$ROOT_DIR/build/$TEST"
 xvlog --sv "$ROOT_DIR/rtl/udiv_round.sv" "$ROOT_DIR/rtl/sfls_core.sv" "$ROOT_DIR/sim/tb_$TEST.sv" >compile.log 2>&1
 xelab "tb_$TEST" -s unit -mt 2 >elaborate.log 2>&1
 xsim unit -runall -onerror quit -onfinish quit >simulate.log 2>&1
 if ! rg -q "^PASS $TEST" simulate.log || rg -q 'Fatal:|FATAL|ERROR:' simulate.log; then tail -50 simulate.log;exit 1;fi
 rg '^PASS' simulate.log
done
