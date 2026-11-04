#!/usr/bin/env bash
set -e
D1_CLOSED_LOOP_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "$D1_CLOSED_LOOP_DIR/runtime/Control/ros2_ws/env.sh"
export PYTHONPATH="$D1_CLOSED_LOOP_DIR:${PYTHONPATH:-}"
export PYTHONDONTWRITEBYTECODE=1
export ROS_DOMAIN_ID=79
export ROS_LOCALHOST_ONLY=1
export PYTHONUNBUFFERED=1
cd "$D1_CLOSED_LOOP_DIR"
if [ "$#" -eq 0 ]; then set -- --help; fi
exec /usr/bin/python3 -m app.closed_loop_runner "$@"
