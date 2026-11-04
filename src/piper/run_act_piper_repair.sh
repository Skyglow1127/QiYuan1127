#!/usr/bin/env bash
set -euo pipefail
source /home/aup/Lerobot/activate.sh
exec /home/aup/Lerobot/envs/piper-cpu/bin/python -B /home/aup/Lerobot/projects/piper/deployment/launch_act_contact_repair.py "$@"
