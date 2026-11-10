#!/usr/bin/env bash
set -euo pipefail
exec 9>"${XDG_RUNTIME_DIR:-/tmp}/d1t-readonly-$UID.lock"
flock -w 30 9
if [[ "$(systemctl --user show d1t-real-monitor.service -p LoadState --value)" != not-found ]]; then
 systemctl --user stop d1t-real-monitor.service
fi
echo '已断开实机监视。机械臂供电与使能状态未改变。'
