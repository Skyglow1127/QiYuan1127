#!/usr/bin/env bash
set -e
source "$(dirname "$0")/env.sh"
mkdir -p "$PIPER_ROOT/../../work/piper_run"
PIPER_RUN="$(cd "$PIPER_ROOT/../../work/piper_run" && pwd)"
PIPER_PORT="${PIPER_PORT:-$(python3 - "$PIPER_ROOT/config/runtime.json" <<'PYPORT'
import json, sys
print(json.load(open(sys.argv[1])).get('simulation_port', 8768))
PYPORT
)}"
if [[ ! "$PIPER_PORT" =~ ^[0-9]+$ ]] || (( 10#$PIPER_PORT < 1 || 10#$PIPER_PORT > 65535 )); then
 echo 'PIPER_PORT 必须是 1–65535 的端口号。' >&2; exit 1
fi
PIPER_PORT="$((10#$PIPER_PORT))"
export PIPER_PORT
# JSON alone is not sufficient: another local service may expose /api/state.
if python3 - "$PIPER_PORT" <<'PYCHECK'
import json, socket, sys, urllib.request
port = int(sys.argv[1])
try:
    with urllib.request.urlopen(f'http://127.0.0.1:{port}/api/state', timeout=1) as response:
        state = json.load(response)
    if (isinstance(state, dict) and isinstance(state.get('connected'), bool)
            and isinstance(state.get('positions'), dict)
            and isinstance(state.get('targets'), dict)
            and 'joint1' in state['targets']
            and isinstance(state.get('mode'), str) and state['mode'].startswith('Gazebo')
            and isinstance(state.get('sim_time'), (int, float))):
        sys.exit(0)
except (OSError, ValueError):
    pass
try:
    with socket.create_connection(('127.0.0.1', port), timeout=.5):
        print(f'端口 {port} 已被其他服务占用；请设置 PIPER_PORT 后重试。', file=sys.stderr)
        sys.exit(2)
except OSError:
    sys.exit(1)
PYCHECK
then
 printf '控制台已运行：http://127.0.0.1:%s/\n' "$PIPER_PORT"; exit 0
else
 status="$?"
 if [[ "$status" != 1 ]]; then exit "$status"; fi
fi
python3 "$PIPER_ROOT/scripts/build.py"
nohup gz sim -s -r --headless-rendering "$PIPER_ROOT/worlds/lab.sdf" -v 3 >"$PIPER_RUN/gazebo.log" 2>&1 & echo $! > "$PIPER_RUN/server.pid"
# The create client waits for Gazebo's world service before submitting the URDF.
if ! timeout 40 ros2 run ros_gz_sim create -world piper_lab -file "$PIPER_ROOT/generated/piper.urdf" -name piper; then
 echo "Spawn failed; inspect $PIPER_RUN/gazebo.log"; exit 1
fi
nohup ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:="$PIPER_ROOT/generated/bridge.yaml" >"$PIPER_RUN/bridge.log" 2>&1 & echo $! > "$PIPER_RUN/bridge.pid"
nohup ros2 run robot_state_publisher robot_state_publisher --ros-args --params-file "$PIPER_ROOT/generated/robot_state.yaml" >"$PIPER_RUN/tf.log" 2>&1 & echo $! > "$PIPER_RUN/tf.pid"
nohup python3 "$PIPER_ROOT/scripts/control_server.py" --port "$PIPER_PORT" >"$PIPER_RUN/control.log" 2>&1 & echo $! > "$PIPER_RUN/control.pid"
if [[ "${1:-}" != '--headless' ]]; then
 nohup gz sim -g --gui-config "$PIPER_ROOT/config/gui.config" >"$PIPER_RUN/gui.log" 2>&1 & echo $! > "$PIPER_RUN/gui.pid"
fi
printf '控制台：http://127.0.0.1:%s/\n日志：%s\n' "$PIPER_PORT" "$PIPER_RUN"
trap 'kill $(jobs -pr) 2>/dev/null || true' EXIT INT TERM
wait
