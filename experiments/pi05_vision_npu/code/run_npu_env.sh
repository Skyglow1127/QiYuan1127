#!/usr/bin/env bash
set -euo pipefail
TASK_ROOT=/home/aup/NPU/pi05_vision_npu
NPU_VENV=${NPU_VENV:-/home/aup/Documents/Codex/2026-10-30/w/work/ryzenai-venv}
NPU_SITE="$NPU_VENV/lib/python3.12/site-packages"
unset PYTHONPATH PYTHONHOME LD_PRELOAD
export LD_LIBRARY_PATH="/opt/xilinx/xrt/lib:$NPU_SITE/flexml/flexml_extras/lib:$NPU_SITE/onnxruntime/capi:$NPU_SITE/voe/lib:$NPU_SITE/lnx64.o/tools/peano/lib"
export PATH="$NPU_VENV/bin:/opt/xilinx/xrt/bin:/usr/local/bin:/usr/bin:/bin"
export XILINX_XRT=/opt/xilinx/xrt
export RYZEN_AI_INSTALLATION_PATH="$NPU_VENV"
export XILINX_VITIS="$NPU_SITE"
export XILINX_VITIS_AIETOOLS="$NPU_SITE"
export PYTHONDONTWRITEBYTECODE=1
export TMPDIR="$TASK_ROOT/cache/tmp"
export XDG_CACHE_HOME="$TASK_ROOT/cache/xdg"
mkdir -p "$TMPDIR" "$XDG_CACHE_HOME"
# VAIML also writes fixed-name compilation files in CWD. Isolate invocations.
NPU_RUN_CWD=$(mktemp -d "$TASK_ROOT/cache/work.XXXXXXXX")
cd "$NPU_RUN_CWD"
exec "$@"
