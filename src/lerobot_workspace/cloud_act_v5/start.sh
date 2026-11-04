#!/usr/bin/env bash
# Start the official 20k-step ACT run in a detached tmux session.
set -euo pipefail
piper_act_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if tmux has-session -t '=act_train' 2>/dev/null; then
  printf 'act_train session already exists. View: tmux attach -t act_train\n'
  exit 0
fi
printf -v piper_act_tmux_command 'bash %q --run' "$piper_act_root/train.sh"
tmux new-session -d -s act_train -c "$piper_act_root" "$piper_act_tmux_command"
printf 'Started ACT training. View: tmux attach -t act_train\n'
printf 'Detach without stopping: Ctrl+B, then D. Logs: %s/logs/\n' "$piper_act_root"
