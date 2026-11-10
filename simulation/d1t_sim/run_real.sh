#!/usr/bin/env bash
set -euo pipefail
D1T_DIR="$(cd "$(dirname "$0")" && pwd)"
exec 9>"${XDG_RUNTIME_DIR:-/tmp}/d1t-readonly-$UID.lock"
flock -w 30 9
if ! systemctl --user is-active --quiet d1t-real-monitor.service; then
 cmake -S "$D1T_DIR/hardware" -B "$D1T_DIR/hardware/build" -DCMAKE_BUILD_TYPE=Release
 cmake --build "$D1T_DIR/hardware/build" -j2
 if [[ "$(systemctl --user show d1t-real-monitor.service -p LoadState --value)" == loaded ]]; then
  systemctl --user restart d1t-real-monitor.service
 else
  systemd-run --user --unit=d1t-real-monitor --collect --property=TimeoutStopSec=5 /usr/bin/python3 "$D1T_DIR/scripts/real_monitor.py"
 fi
fi
echo '实机只读监视：http://127.0.0.1:8766/'
