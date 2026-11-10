#!/usr/bin/env bash
PIPER_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ "${PIPER_USE_SYSTEM:-0}" == 1 ]]; then
 source /opt/ros/jazzy/setup.bash
 export PIPER_ROOT GZ_PARTITION=piper_codex_sim ROS_DOMAIN_ID=62 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
 export GZ_SIM_RESOURCE_PATH="$PIPER_ROOT/models:$PIPER_ROOT:$PIPER_ROOT/worlds:${GZ_SIM_RESOURCE_PATH:-}"
 for PIPER_VENDOR in /opt/ros/jazzy/opt/*; do
  if [[ -d "$PIPER_VENDOR/lib/python" ]]; then export PYTHONPATH="$PIPER_VENDOR/lib/python:${PYTHONPATH:-}"; fi
 done
 return 0
fi
PIPER_RUNTIME="${PIPER_RUNTIME:-$(cd "$PIPER_ROOT/../../work/runtime" && pwd)}"
source /opt/ros/jazzy/setup.bash
export PATH="$PIPER_RUNTIME/usr/bin:$PIPER_RUNTIME/opt/ros/jazzy/bin:$PATH"
export LD_LIBRARY_PATH="$PIPER_RUNTIME/usr/lib/x86_64-linux-gnu:$PIPER_RUNTIME/opt/ros/jazzy/lib:$LD_LIBRARY_PATH"
export PYTHONPATH="$PIPER_RUNTIME/opt/ros/jazzy/lib/python3.12/site-packages:$PYTHONPATH"
export AMENT_PREFIX_PATH="$PIPER_RUNTIME/opt/ros/jazzy:$AMENT_PREFIX_PATH"
export RUBYLIB="$PIPER_RUNTIME/usr/lib/ruby/3.2.0:$PIPER_RUNTIME/usr/lib/x86_64-linux-gnu/ruby/3.2.0:$PIPER_RUNTIME/usr/lib/ruby/vendor_ruby"
for PIPER_VENDOR in "$PIPER_RUNTIME"/opt/ros/jazzy/opt/*; do
 export PATH="$PIPER_VENDOR/bin:$PATH"
 export LD_LIBRARY_PATH="$PIPER_VENDOR/lib:$LD_LIBRARY_PATH"
 export GZ_CONFIG_PATH="$PIPER_VENDOR/share/gz:${GZ_CONFIG_PATH:-}"
 export GZ_SIM_SYSTEM_PLUGIN_PATH="$PIPER_VENDOR/lib:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"
 export GZ_GUI_PLUGIN_PATH="$PIPER_VENDOR/lib/gz-gui-8/plugins:${GZ_GUI_PLUGIN_PATH:-}"
 export GZ_SIM_RESOURCE_PATH="$PIPER_VENDOR/share:${GZ_SIM_RESOURCE_PATH:-}"
done
export GZ_PARTITION=piper_codex_sim
export ROS_DOMAIN_ID=62
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export GZ_SIM_RESOURCE_PATH="$PIPER_ROOT/models:$PIPER_ROOT:$PIPER_ROOT/worlds:$GZ_SIM_RESOURCE_PATH"
export QT_QPA_PLATFORM=xcb
export PIPER_ROOT PIPER_RUNTIME
export GZ_SIM_SYSTEM_PLUGIN_PATH="$PIPER_RUNTIME/opt/ros/jazzy/opt/gz_sim_vendor/lib/gz-sim-8/plugins:$GZ_SIM_SYSTEM_PLUGIN_PATH"
export GZ_GUI_PLUGIN_PATH="$PIPER_RUNTIME/opt/ros/jazzy/opt/gz_sim_vendor/lib/gz-sim-8/plugins/gui:$GZ_GUI_PLUGIN_PATH"
export GZ_SIM_PHYSICS_ENGINE_PATH="$PIPER_RUNTIME/opt/ros/jazzy/opt/gz_physics_vendor/lib/gz-physics-7/engine-plugins"
export GZ_SIM_SERVER_CONFIG_PATH="$PIPER_RUNTIME/opt/ros/jazzy/opt/gz_sim_vendor/share/gz/gz-sim8/server.config"
export GZ_RENDERING_RESOURCE_PATH="$PIPER_RUNTIME/opt/ros/jazzy/opt/gz_rendering_vendor/share/gz/gz-rendering8"
export GZ_RENDERING_PLUGIN_PATH="$PIPER_RUNTIME/opt/ros/jazzy/opt/gz_rendering_vendor/lib/gz-rendering-8/engine-plugins"
export QML2_IMPORT_PATH="$PIPER_RUNTIME/usr/lib/x86_64-linux-gnu/qt5/qml:${QML2_IMPORT_PATH:-}"
export QML2_IMPORT_PATH="$PIPER_RUNTIME/opt/ros/jazzy/opt/gz_sim_vendor/lib/gz-sim-8/plugins/gui:$QML2_IMPORT_PATH"
export OGRE2_RESOURCE_PATH="$PIPER_RUNTIME/opt/ros/jazzy/opt/gz_ogre_next_vendor/lib/OGRE-Next"

export PYTHONPATH="$PIPER_ROOT/../../work/mesh_deps:$PYTHONPATH"

# Project-local Gazebo Python transport and its packaged protobuf dependency.
export PYTHONPATH="$PIPER_RUNTIME/usr/lib/python3/dist-packages:$PIPER_RUNTIME/opt/ros/jazzy/opt/gz_transport_vendor/lib/python:$PIPER_RUNTIME/opt/ros/jazzy/opt/gz_msgs_vendor/lib/python:$PYTHONPATH"
