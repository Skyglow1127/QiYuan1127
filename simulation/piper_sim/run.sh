#!/usr/bin/env bash
set -euo pipefail
PIPER_DIR="$(cd "$(dirname "$0")" && pwd)"
PIPER_PORT="${PIPER_PORT:-$(python3 - "$PIPER_DIR/config/runtime.json" <<'PYPORT'
import json, sys
print(json.load(open(sys.argv[1])).get('simulation_port', 8768))
PYPORT
)}"
if [[ ! "$PIPER_PORT" =~ ^[0-9]+$ ]] || (( 10#$PIPER_PORT < 1 || 10#$PIPER_PORT > 65535 )); then
 echo 'PIPER_PORT 必须是 1–65535 的端口号。' >&2; exit 1
fi
PIPER_PORT="$((10#$PIPER_PORT))"
export PIPER_PORT
exec 9>"${XDG_RUNTIME_DIR:-/tmp}/piper-simulation-$UID.lock"
flock -w 30 9
start_unit() {
 local unit="$1"; shift
 if systemctl --user is-active --quiet "$unit.service"; then return; fi
 # A transient unit may briefly remain loaded after stopping.
 for attempt in {1..50}; do
  local state
  state="$(systemctl --user show "$unit.service" -p LoadState --value)"
  if [[ "$state" == not-found ]]; then break; fi
  systemctl --user reset-failed "$unit.service" 2>/dev/null || true
  sleep 0.1
 done
 if [[ "$(systemctl --user show "$unit.service" -p LoadState --value)" == loaded ]]; then
  systemctl --user start "$unit.service"
 else
  systemd-run --user --unit="$unit" --collect --property=TimeoutStopSec=4 --setenv="PIPER_PORT=$PIPER_PORT" /bin/bash "$@"
 fi
}
start_unit piper-sim "$PIPER_DIR/scripts/start.sh" --headless
start_unit piper-gui "$PIPER_DIR/scripts/gui.sh"
printf '松灵 PiPER控制台：http://127.0.0.1:%s/\n' "$PIPER_PORT"
