#!/usr/bin/env bash
set -euo pipefail
ROOT_DIR=$(cd "$(dirname "$0")/.." && pwd)
VIVADO_ROOT=${VIVADO_ROOT:-/home/aup/Xilinx/2025.2/Vivado}
export XILINX_VIVADO="$VIVADO_ROOT"
export PATH="$VIVADO_ROOT/bin:$PATH"
PROFILE=${1:-full}
case "$PROFILE" in
 full) LOG2=15; STRIDE=1; FREQ=350000; MS=115 ;;
 small) LOG2=7; STRIDE=1; FREQ=70000; MS=8 ;;
 decimated) LOG2=7; STRIDE=64; FREQ=350000; MS=30 ;;
 precision) LOG2=15; STRIDE=64; FREQ=350000; MS=6020 ;;
 precision_offset) LOG2=15; STRIDE=64; FREQ=349999.999; MS=6020 ;;
 *) exit 2 ;;
esac
mkdir -p "$ROOT_DIR/build/e2e_$PROFILE"
cd "$ROOT_DIR/build/e2e_$PROFILE"
xvlog --sv "$ROOT_DIR/rtl/udiv_round.sv" "$ROOT_DIR/rtl/sfls_core.sv" "$ROOT_DIR/rtl/lamor_frequency_meter.sv" "$ROOT_DIR/sim/tb_end_to_end.sv" >compile.log 2>&1
xelab tb_end_to_end -s e2e -debug typical -mt 2 -generic_top "WINDOW_LOG2=$LOG2" -generic_top "EDGE_STRIDE=$STRIDE" >elaborate.log 2>&1
xsim e2e -runall -onerror quit -onfinish quit -testplusarg "FREQUENCY=$FREQ" -testplusarg "RUN_MS=$MS" -testplusarg "RESULTS=$ROOT_DIR/build/e2e_$PROFILE/results.csv" >simulate.log 2>&1
if ! rg -q '^PASS end_to_end' simulate.log || rg -q 'Fatal:|FATAL|ERROR:' simulate.log; then tail -50 simulate.log;exit 1;fi
rg '^PASS end_to_end' simulate.log
