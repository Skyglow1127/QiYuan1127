#!/usr/bin/env bash
set -euo pipefail
pi05_task_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec /usr/bin/python3 "$pi05_task_dir/service_control.py" start --config "$pi05_task_dir/cloud_config.json" "$@"
