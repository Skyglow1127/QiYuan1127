#!/usr/bin/env bash
set -euo pipefail
camera_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if [[ ! -x "$camera_root/.venv-models/bin/python" ]]; then
    echo "Missing .venv-models. See QWEN_GRASP_TECHNICAL_NOTE.md." >&2
    exit 1
fi
cmake -S "$camera_root" -B "$camera_root/build" -DCMAKE_BUILD_TYPE=Release
cmake --build "$camera_root/build" -j4
exec "$camera_root/.venv-models/bin/python" "$camera_root/scripts/run_qwen_grasp.py" "$@"
