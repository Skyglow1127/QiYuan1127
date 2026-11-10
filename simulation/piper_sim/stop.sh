#!/usr/bin/env bash
set -euo pipefail
exec 9>"${XDG_RUNTIME_DIR:-/tmp}/piper-simulation-$UID.lock"
flock -w 30 9
# Stop only this project’s user units; do not kill by port or process name.
for unit in piper-gui.service piper-sim.service; do
 if [[ "$(systemctl --user show "$unit" -p LoadState --value)" != not-found ]]; then
  systemctl --user stop "$unit"
 fi
done
echo '仿真已关闭。'
