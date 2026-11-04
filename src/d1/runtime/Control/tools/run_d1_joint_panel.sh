#!/usr/bin/env bash
set -euo pipefail
tool_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
control_dir="$(dirname -- "$tool_dir")"
if [[ ! -x "$control_dir/build/d1_joint_monitor" ]]; then
  echo 'Missing read-only reader. Run tools/build_d1_joint_monitor.sh first.' >&2
  exit 2
fi
exec python3 "$tool_dir/d1_joint_server.py" --nic enp99s0 --port 18763 "$@"
