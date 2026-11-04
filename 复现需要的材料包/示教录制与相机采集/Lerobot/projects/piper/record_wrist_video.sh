#!/usr/bin/env bash
set -euo pipefail
RECORDER_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
RECORDER_PYTHON="${PIPER_VIDEO_PYTHON:-/home/aup/Lerobot/envs/piper-cpu/bin/python}"
exec "$RECORDER_PYTHON" -B "$RECORDER_ROOT/camera_preview/record_wrist_video.py" "$@"
