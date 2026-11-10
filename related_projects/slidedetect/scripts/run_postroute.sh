#!/usr/bin/env bash
set -euo pipefail
ROOT_DIR=$(cd "$(dirname "$0")/.." && pwd)
VIVADO_ROOT=${VIVADO_ROOT:-/home/aup/Xilinx/2025.2/Vivado}
export XILINX_VIVADO="$VIVADO_ROOT"
export PATH="$VIVADO_ROOT/bin:$PATH"
mkdir -p "$ROOT_DIR/build/postroute"
cd "$ROOT_DIR/build/postroute"
# The generated netlist annotates the matching SDF by its basename.
ln -sf ../implementation/meter_timesim.sdf meter_timesim.sdf
xvlog "$ROOT_DIR/build/implementation/meter_timesim.v" "$VIVADO_ROOT/data/verilog/src/glbl.v" >compile.log 2>&1
xvlog --sv "$ROOT_DIR/sim/tb_postroute.sv" >>compile.log 2>&1
xelab tb_postroute glbl -s postroute -L simprims_ver -L unisims_ver -L secureip -mt 2 -debug typical -transport_int_delays -pulse_r 0 -pulse_int_r 0 >elaborate.log 2>&1
xsim postroute -runall -onerror quit -onfinish quit >simulate.log 2>&1
if ! rg -q '^PASS postroute' simulate.log || rg -q 'Fatal:|FATAL|ERROR:|Timing Violation' simulate.log; then tail -70 simulate.log;exit 1;fi
rg '^PASS postroute' simulate.log
