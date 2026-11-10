#!/usr/bin/env bash
set -euo pipefail
camera_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
role="${1:-wrist}"
if (( $# > 1 )); then echo '用法: run_kr260_live.sh [wrist|global]' >&2; exit 2; fi
case "$role" in
  wrist) port=18760; label='腕部相机'; lock=/tmp/d1-handeye-camera.lock ;;
  global) port=18761; label='全局相机'; lock=/tmp/d1-global-camera.lock ;;
  -h|--help) echo '用法: run_kr260_live.sh [wrist|global]，默认腕部相机'; exit 0 ;;
  *) echo '请选择 wrist（腕部）或 global（全局）' >&2; exit 2 ;;
esac
board_host="${KR260_HOST:-ubuntu@10.42.0.55}"
board_key="${KR260_KEY:-$HOME/.ssh/id_ed25519_kr260}"
ssh_options=(-i "$board_key" -o BatchMode=yes -o StrictHostKeyChecking=yes -o ConnectTimeout=5 -o ServerAliveInterval=5 -o ServerAliveCountMax=3)
scp "${ssh_options[@]}" "$camera_root/scripts/kr260_live_server.py" "$camera_root/scripts/kr260_live.html" "$camera_root/configs/camera_roles.json" "$board_host:/home/ubuntu/kr260-camera/"
echo "$label：http://127.0.0.1:$port （按 Ctrl+C 停止本路预览）"
# role, port and lock above are fixed allowlisted values, not raw shell input.
exec ssh "${ssh_options[@]}" -tt -o ExitOnForwardFailure=yes -L "127.0.0.1:$port:127.0.0.1:$port" "$board_host" "cd /tmp && flock -n $lock env LD_LIBRARY_PATH=/home/ubuntu/kr260-camera/lib python3 -u /home/ubuntu/kr260-camera/kr260_live_server.py --role $role --port $port"
