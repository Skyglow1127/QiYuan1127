#!/usr/bin/env bash
set -euo pipefail
ROOT_DIR=$(cd "$(dirname "$0")/.." && pwd)
VIVADO_ROOT=${VIVADO_ROOT:-/home/aup/Xilinx/2025.2/Vivado}
export XILINX_VIVADO="$VIVADO_ROOT"
export PATH="$VIVADO_ROOT/bin:$PATH"
PROFILE=${1:-small}
case "$PROFILE" in
  small) LOG2=7; STRIDE=1 ;;
  stride1) LOG2=15; STRIDE=1 ;;
  stride64) LOG2=15; STRIDE=64 ;;
  extended1) LOG2=15; STRIDE=1 ;;
  extended64) LOG2=15; STRIDE=64 ;;
  *) echo 'Usage: run_replay.sh small|stride1|stride64' >&2; exit 2 ;;
esac
mkdir -p "$ROOT_DIR/build/$PROFILE"
cd "$ROOT_DIR/build/$PROFILE"
xvlog --sv "$ROOT_DIR/rtl/udiv_round.sv" "$ROOT_DIR/rtl/sfls_core.sv" "$ROOT_DIR/sim/tb_replay.sv" >compile.log 2>&1
xelab tb_replay -s replay -debug typical -mt 2 -generic_top "WINDOW_LOG2=$LOG2" -generic_top "EDGE_STRIDE=$STRIDE" >elaborate.log 2>&1
xsim replay -runall -onerror quit -onfinish quit -testplusarg "VECTORS=$ROOT_DIR/sim/vectors/$PROFILE.txt" -testplusarg "RESULTS=$ROOT_DIR/build/$PROFILE/results.csv" >simulate.log 2>&1
if ! rg -q '^PASS replay' simulate.log || rg -q 'Fatal:|FATAL|ERROR:' simulate.log; then
  tail -50 simulate.log; exit 1
fi
rg '^PASS replay' simulate.log
