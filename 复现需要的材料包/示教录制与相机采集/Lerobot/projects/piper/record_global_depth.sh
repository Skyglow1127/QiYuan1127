#!/usr/bin/env bash
set -euo pipefail
DEPTH_RECORDER_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
DEPTH_RECORDER_PYTHON="${PIPER_VIDEO_PYTHON:-/home/aup/Lerobot/envs/piper-cpu/bin/python}"
exec "$DEPTH_RECORDER_PYTHON" -B "$DEPTH_RECORDER_ROOT/camera_preview/record_global_depth.py" "$@"
