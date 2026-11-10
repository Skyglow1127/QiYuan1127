#!/usr/bin/env bash
D1T_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ "${D1T_USE_SYSTEM:-0}" == 1 ]]; then
 source /opt/ros/jazzy/setup.bash
 export D1T_ROOT GZ_PARTITION=d1t_codex_sim ROS_DOMAIN_ID=61 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
 export GZ_SIM_RESOURCE_PATH="$D1T_ROOT/models:$D1T_ROOT:$D1T_ROOT/worlds:${GZ_SIM_RESOURCE_PATH:-}"
 for D1T_VENDOR in /opt/ros/jazzy/opt/*; do
  if [[ -d "$D1T_VENDOR/lib/python" ]]; then export PYTHONPATH="$D1T_VENDOR/lib/python:${PYTHONPATH:-}"; fi
 done
 return 0
fi
D1T_RUNTIME="${D1T_RUNTIME:-$(cd "$D1T_ROOT/../../work/runtime" && pwd)}"
source /opt/ros/jazzy/setup.bash
export PATH="$D1T_RUNTIME/usr/bin:$D1T_RUNTIME/opt/ros/jazzy/bin:$PATH"
export LD_LIBRARY_PATH="$D1T_RUNTIME/usr/lib/x86_64-linux-gnu:$D1T_RUNTIME/opt/ros/jazzy/lib:$LD_LIBRARY_PATH"
export PYTHONPATH="$D1T_RUNTIME/opt/ros/jazzy/lib/python3.12/site-packages:$PYTHONPATH"
export AMENT_PREFIX_PATH="$D1T_RUNTIME/opt/ros/jazzy:$AMENT_PREFIX_PATH"
export RUBYLIB="$D1T_RUNTIME/usr/lib/ruby/3.2.0:$D1T_RUNTIME/usr/lib/x86_64-linux-gnu/ruby/3.2.0:$D1T_RUNTIME/usr/lib/ruby/vendor_ruby"
for D1T_VENDOR in "$D1T_RUNTIME"/opt/ros/jazzy/opt/*; do
 export PATH="$D1T_VENDOR/bin:$PATH"
 export LD_LIBRARY_PATH="$D1T_VENDOR/lib:$LD_LIBRARY_PATH"
 export GZ_CONFIG_PATH="$D1T_VENDOR/share/gz:${GZ_CONFIG_PATH:-}"
 export GZ_SIM_SYSTEM_PLUGIN_PATH="$D1T_VENDOR/lib:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"
 export GZ_GUI_PLUGIN_PATH="$D1T_VENDOR/lib/gz-gui-8/plugins:${GZ_GUI_PLUGIN_PATH:-}"
 export GZ_SIM_RESOURCE_PATH="$D1T_VENDOR/share:${GZ_SIM_RESOURCE_PATH:-}"
done
export GZ_PARTITION=d1t_codex_sim
export ROS_DOMAIN_ID=61
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export GZ_SIM_RESOURCE_PATH="$D1T_ROOT/models:$D1T_ROOT:$D1T_ROOT/worlds:$GZ_SIM_RESOURCE_PATH"
export QT_QPA_PLATFORM=xcb
export D1T_ROOT D1T_RUNTIME
export GZ_SIM_SYSTEM_PLUGIN_PATH="$D1T_RUNTIME/opt/ros/jazzy/opt/gz_sim_vendor/lib/gz-sim-8/plugins:$GZ_SIM_SYSTEM_PLUGIN_PATH"
export GZ_GUI_PLUGIN_PATH="$D1T_RUNTIME/opt/ros/jazzy/opt/gz_sim_vendor/lib/gz-sim-8/plugins/gui:$GZ_GUI_PLUGIN_PATH"
export GZ_SIM_PHYSICS_ENGINE_PATH="$D1T_RUNTIME/opt/ros/jazzy/opt/gz_physics_vendor/lib/gz-physics-7/engine-plugins"
export GZ_SIM_SERVER_CONFIG_PATH="$D1T_RUNTIME/opt/ros/jazzy/opt/gz_sim_vendor/share/gz/gz-sim8/server.config"
export GZ_RENDERING_RESOURCE_PATH="$D1T_RUNTIME/opt/ros/jazzy/opt/gz_rendering_vendor/share/gz/gz-rendering8"
export GZ_RENDERING_PLUGIN_PATH="$D1T_RUNTIME/opt/ros/jazzy/opt/gz_rendering_vendor/lib/gz-rendering-8/engine-plugins"
export QML2_IMPORT_PATH="$D1T_RUNTIME/usr/lib/x86_64-linux-gnu/qt5/qml:${QML2_IMPORT_PATH:-}"
export QML2_IMPORT_PATH="$D1T_RUNTIME/opt/ros/jazzy/opt/gz_sim_vendor/lib/gz-sim-8/plugins/gui:$QML2_IMPORT_PATH"
export OGRE2_RESOURCE_PATH="$D1T_RUNTIME/opt/ros/jazzy/opt/gz_ogre_next_vendor/lib/OGRE-Next"

export PYTHONPATH="$D1T_ROOT/../../work/mesh_deps:$PYTHONPATH"

# Project-local Gazebo Python transport and its packaged protobuf dependency.
export PYTHONPATH="$D1T_RUNTIME/usr/lib/python3/dist-packages:$D1T_RUNTIME/opt/ros/jazzy/opt/gz_transport_vendor/lib/python:$D1T_RUNTIME/opt/ros/jazzy/opt/gz_msgs_vendor/lib/python:$PYTHONPATH"
