#!/usr/bin/env bash
set -e
source "$(dirname "$0")/env.sh"
mkdir -p "$D1T_ROOT/../../work/run"
D1T_RUN="$(cd "$D1T_ROOT/../../work/run" && pwd)"
D1T_PORT="${D1T_PORT:-$(python3 - "$D1T_ROOT/config/runtime.json" <<'PYPORT'
import json, sys
print(json.load(open(sys.argv[1])).get('simulation_port', 8767))
PYPORT
)}"
if [[ ! "$D1T_PORT" =~ ^[0-9]+$ ]] || (( 10#$D1T_PORT < 1 || 10#$D1T_PORT > 65535 )); then
 echo 'D1T_PORT 必须是 1–65535 的端口号。' >&2; exit 1
fi
D1T_PORT="$((10#$D1T_PORT))"
export D1T_PORT
# JSON alone is not sufficient: another local service may expose /api/state.
if python3 - "$D1T_PORT" <<'PYCHECK'
import json, socket, sys, urllib.request
port = int(sys.argv[1])
try:
    with urllib.request.urlopen(f'http://127.0.0.1:{port}/api/state', timeout=1) as response:
        state = json.load(response)
    if (isinstance(state, dict) and isinstance(state.get('connected'), bool)
            and isinstance(state.get('positions'), dict)
            and isinstance(state.get('targets'), dict)
            and 'arm_Joint1' in state['targets']
            and isinstance(state.get('mode'), str) and state['mode'].startswith('Gazebo')
            and isinstance(state.get('sim_time'), (int, float))):
        sys.exit(0)
except (OSError, ValueError):
    pass
try:
    with socket.create_connection(('127.0.0.1', port), timeout=.5):
        print(f'端口 {port} 已被其他服务占用；请设置 D1T_PORT 后重试。', file=sys.stderr)
        sys.exit(2)
except OSError:
    sys.exit(1)
PYCHECK
then
 printf '控制台已运行：http://127.0.0.1:%s/\n' "$D1T_PORT"; exit 0
else
 status="$?"
 if [[ "$status" != 1 ]]; then exit "$status"; fi
fi
python3 "$D1T_ROOT/scripts/build.py"
nohup gz sim -s -r --headless-rendering "$D1T_ROOT/worlds/lab.sdf" -v 3 >"$D1T_RUN/gazebo.log" 2>&1 & echo $! > "$D1T_RUN/server.pid"
# The create client waits for Gazebo's world service before submitting the URDF.
if ! timeout 40 ros2 run ros_gz_sim create -world d1t_lab -file "$D1T_ROOT/generated/d1t.urdf" -name d1t; then
 echo "Spawn failed; inspect $D1T_RUN/gazebo.log"; exit 1
fi
nohup ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:="$D1T_ROOT/generated/bridge.yaml" >"$D1T_RUN/bridge.log" 2>&1 & echo $! > "$D1T_RUN/bridge.pid"
nohup ros2 run robot_state_publisher robot_state_publisher --ros-args --params-file "$D1T_ROOT/generated/robot_state.yaml" >"$D1T_RUN/tf.log" 2>&1 & echo $! > "$D1T_RUN/tf.pid"
nohup python3 "$D1T_ROOT/scripts/control_server.py" --port "$D1T_PORT" >"$D1T_RUN/control.log" 2>&1 & echo $! > "$D1T_RUN/control.pid"
if [[ "${1:-}" != '--headless' ]]; then
 nohup gz sim -g --gui-config "$D1T_ROOT/config/gui.config" >"$D1T_RUN/gui.log" 2>&1 & echo $! > "$D1T_RUN/gui.pid"
fi
printf '控制台：http://127.0.0.1:%s/\n日志：%s\n' "$D1T_PORT" "$D1T_RUN"
trap 'kill $(jobs -pr) 2>/dev/null || true' EXIT INT TERM
wait
