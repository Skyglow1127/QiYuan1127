#!/usr/bin/env bash
set -euo pipefail
d1_timed_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "$d1_timed_root/runtime/Control/ros2_ws/env.sh"
export PYTHONPATH="$d1_timed_root:${PYTHONPATH:-}"
export PYTHONDONTWRITEBYTECODE=1
export ROS_DOMAIN_ID=79
export ROS_LOCALHOST_ONLY=1
export PYTHONUNBUFFERED=1
cd "$d1_timed_root"
if [ "$#" -eq 0 ]; then set -- --help; fi
exec /usr/bin/python3 -m app.timed_runner "$@"
