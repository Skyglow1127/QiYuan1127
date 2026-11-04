#!/usr/bin/env bash
set -euo pipefail
RGBD_RECORDER_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
RGBD_RECORDER_PYTHON="${PIPER_VIDEO_PYTHON:-/home/aup/Lerobot/envs/piper-cpu/bin/python}"
exec "$RGBD_RECORDER_PYTHON" -B "$RGBD_RECORDER_ROOT/camera_preview/record_rgbd.py" "$@"
