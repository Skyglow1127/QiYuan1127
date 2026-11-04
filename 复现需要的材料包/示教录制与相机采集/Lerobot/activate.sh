#!/usr/bin/env bash
# Main local PiPER environment. Source this file in a new terminal.
_lerobot_workspace="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
source "$_lerobot_workspace/envs/piper-cpu/bin/activate"
unset PYTHONPATH PYTHONHOME
export PYTHONNOUSERSITE=1
export LEROBOT_WORKSPACE="$_lerobot_workspace"
_lerobot_ffmpeg_lib="$_lerobot_workspace/envs/piper-cpu/ffmpeg-runtime/usr/lib/x86_64-linux-gnu"
case ":${LD_LIBRARY_PATH:-}:" in
  *":$_lerobot_ffmpeg_lib:"*) ;;
  *) export LD_LIBRARY_PATH="$_lerobot_ffmpeg_lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}" ;;
esac
export HF_HOME="$_lerobot_workspace/cache/huggingface"
export HF_HUB_CACHE="$HF_HOME/hub"
export HF_LEROBOT_HOME="$_lerobot_workspace/datasets"
export TORCH_HOME="$_lerobot_workspace/cache/torch"
export UV_CACHE_DIR="$_lerobot_workspace/cache/uv"
export TOKENIZERS_PARALLELISM=false
unset _lerobot_ffmpeg_lib _lerobot_workspace
