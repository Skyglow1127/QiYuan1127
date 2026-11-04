# Source this file from bash; dependencies are local, system ROS is unchanged.
_D1_WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/jazzy/setup.bash
_D1_DEP="$_D1_WS/.deps/root"
export AMENT_PREFIX_PATH="$_D1_DEP/opt/ros/jazzy:${AMENT_PREFIX_PATH:-}"
export LD_LIBRARY_PATH="$_D1_DEP/opt/ros/jazzy/lib:$_D1_DEP/opt/ros/jazzy/lib/x86_64-linux-gnu:$_D1_DEP/usr/lib/x86_64-linux-gnu:$_D1_DEP/usr/lib/llvm-18/lib:${LD_LIBRARY_PATH:-}"
export PYTHONPATH="$_D1_DEP/opt/ros/jazzy/lib/python3.12/site-packages:${PYTHONPATH:-}"
if [ -f "$_D1_WS/install/local_setup.bash" ]; then source "$_D1_WS/install/local_setup.bash"; fi
export AMENT_PREFIX_PATH="$_D1_WS/overlay:${AMENT_PREFIX_PATH:-}"
export PYTHONPATH="$_D1_WS/src/d1_pick_place:$_D1_WS/src/d1_camera_bridge:${PYTHONPATH:-}"
unset _D1_WS _D1_DEP
