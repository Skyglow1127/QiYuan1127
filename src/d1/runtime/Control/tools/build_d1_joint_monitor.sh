#!/usr/bin/env bash
set -euo pipefail
tool_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
control_dir="$(dirname -- "$tool_dir")"
g++ -O2 -std=c++17 -Wall -Wextra -I"$control_dir" -isystem /usr/local/include/ddscxx \
  "$tool_dir/d1_joint_monitor.cpp" "$control_dir/msg/ArmString_.cpp" \
  -L/usr/local/lib -Wl,-rpath,/usr/local/lib /usr/local/lib/libunitree_sdk2.a \
  -lrt -lddsc -lddscxx -pthread -o "$control_dir/build/d1_joint_monitor"
