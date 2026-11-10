#!/usr/bin/env bash
# Optional, tested Ubuntu 24.04 backend. Downloads/extracts tools locally; no sudo.
set -euo pipefail
camera_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$camera_root"
if [[ ! -r /usr/include/vulkan/vulkan.h ]]; then
    echo 'Vulkan development headers are missing; install libvulkan-dev before this optional build.' >&2
    exit 1
fi
camera_commit="$(git -C experiments/llama.cpp rev-parse HEAD)"
if [[ "$camera_commit" != 2a667032dc788c3a071d538e0b572e45a3f12731 ]]; then
    echo 'Prepare the pinned llama.cpp CPU runtime with scripts/setup_qwen_grasp.py first.' >&2
    exit 1
fi
if [[ -n "$(git -C experiments/llama.cpp status --porcelain)" ]]; then
    echo 'Existing llama.cpp source has edits; preserve them before rebuilding.' >&2
    exit 1
fi
mkdir -p experiments/vulkan-tools/packages
camera_packages=(
    'glslc=2023.8-1build1'
    'libshaderc1=2023.8-1build1'
    'spirv-headers=1.6.1+1.4.309.0-1~ubuntu0.24.04.2'
)
for camera_package in "${camera_packages[@]}"; do
    camera_name="${camera_package%%=*}"
    camera_version="${camera_package#*=}"
    if ! compgen -G "experiments/vulkan-tools/packages/${camera_name}_${camera_version}_*.deb" > /dev/null; then
        (cd experiments/vulkan-tools/packages && apt download "$camera_package")
    fi
done
for camera_deb in experiments/vulkan-tools/packages/*.deb; do
    dpkg-deb -x "$camera_deb" experiments/vulkan-tools/root
done
camera_tools="$camera_root/experiments/vulkan-tools/root/usr"
export LD_LIBRARY_PATH="$camera_tools/lib/x86_64-linux-gnu${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
cmake -S experiments/llama.cpp -B experiments/llama.cpp/build-vulkan \
    -DCMAKE_BUILD_TYPE=Release -DGGML_VULKAN=ON -DGGML_NATIVE=ON -DLLAMA_BUILD_TESTS=OFF \
    "-DVulkan_GLSLC_EXECUTABLE=$camera_tools/bin/glslc" "-DCMAKE_PREFIX_PATH=$camera_tools" \
    "-DCMAKE_CXX_FLAGS=-I$camera_tools/include"
cmake --build experiments/llama.cpp/build-vulkan --target llama-server -j4
