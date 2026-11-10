#!/usr/bin/env bash
set -euo pipefail
TASK_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
NPU_VENV="${NPU_VENV:-$TASK_ROOT/work/ryzenai-venv}"
NPU_SITE="$NPU_VENV/lib/python3.12/site-packages"
unset PYTHONPATH PYTHONHOME LD_PRELOAD
export LD_LIBRARY_PATH="/opt/xilinx/xrt/lib:$NPU_SITE/flexml/flexml_extras/lib:$NPU_SITE/onnxruntime/capi:$NPU_SITE/voe/lib:$NPU_SITE/lnx64.o/tools/peano/lib"
export PATH="$NPU_VENV/bin:/opt/xilinx/xrt/bin:/usr/local/bin:/usr/bin:/bin"
export XILINX_XRT=/opt/xilinx/xrt
export RYZEN_AI_INSTALLATION_PATH="$NPU_VENV"
export XILINX_VITIS="$NPU_SITE"
export XILINX_VITIS_AIETOOLS="$NPU_SITE"
exec "$@"
