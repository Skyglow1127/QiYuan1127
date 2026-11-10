#!/usr/bin/env bash
set -euo pipefail
ROOT_DIR=$(cd "$(dirname "$0")/.." && pwd)
VIVADO_ROOT=${VIVADO_ROOT:-/home/aup/Xilinx/2025.2/Vivado}
mkdir -p "$ROOT_DIR/build/implementation"
cd "$ROOT_DIR/build/implementation"
"$VIVADO_ROOT/bin/vivado" -mode batch -nojournal -source "$ROOT_DIR/scripts/create_project.tcl" -log create_project.log >create_project.console.log 2>&1
"$VIVADO_ROOT/bin/vivado" -mode batch -nojournal -source "$ROOT_DIR/scripts/implement.tcl" -log implement.log >implement.console.log 2>&1
rg 'ROUTED_WNS=' implement.console.log
"$VIVADO_ROOT/bin/vivado" -mode batch -nojournal -source "$ROOT_DIR/scripts/implement.tcl" -tclargs 64 >implement_precision.console.log 2>&1
rg 'ROUTED_WNS=|ROUTED_WHS=' implement_precision.console.log
