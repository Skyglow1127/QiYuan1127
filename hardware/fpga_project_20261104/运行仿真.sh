#!/usr/bin/env bash
set -euo pipefail
package_dir=$(CDPATH= cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
if [[ -z ${XILINX_VIVADO:-} ]]; then
  if command -v vivado >/dev/null 2>&1; then
    vivado_exe=$(readlink -f -- "$(command -v vivado)")
    export XILINX_VIVADO="$(dirname "$(dirname "$vivado_exe")")"
  else
    export XILINX_VIVADO=/home/aup/Xilinx/2025.2/Vivado
  fi
fi
if [[ ! -x "$XILINX_VIVADO/bin/vivado" ]]; then
  printf '找不到 Vivado，请设置 XILINX_VIVADO 指向 Vivado 2025.2 安装目录。\n' >&2
  exit 1
fi
suite=${1:-all}
results_dir="$package_dir/重新仿真_$(date +%Y%m%d_%H%M%S)"
exec /usr/bin/python3 "$package_dir/工程源码/sim/run_hdl.py" --suite "$suite" \
  --vivado-bin "$XILINX_VIVADO/bin" --output-dir "$results_dir" \
  --clock-hz 49999500 --random-math-cases 512
