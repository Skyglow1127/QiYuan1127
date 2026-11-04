#!/usr/bin/env bash
# Compatibility entry point; canonical entry is /home/aup/Lerobot/activate.sh.
_piper_lerobot_project="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
source "$_piper_lerobot_project/../../lerobot_env.sh"
unset _piper_lerobot_project
