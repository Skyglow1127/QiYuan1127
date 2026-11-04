#!/usr/bin/env bash
set -euo pipefail
task_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# Runtime duration belongs to the pending physical plan, never to the inference
# client. Parse and validate it before invalidating any existing review.
task_duration=0
task_preview_args=()
while [[ $# -gt 0 ]]; do
  case "$1" in
    --duration)
      if [[ $# -lt 2 ]]; then
        printf '%s\n' '--duration requires0(unlimited) or finite seconds in[1,300].' >&2
        exit 2
      fi
      task_duration="$2"
      shift 2 ;;
    --duration=*) task_duration="${1#--duration=}"; shift ;;
    *) task_preview_args+=("$1"); shift ;;
  esac
done
env -u PYTHONPATH -u LD_LIBRARY_PATH PYTHONNOUSERSITE=1 PYTHONDONTWRITEBYTECODE=1 \
  /home/aup/Lerobot/envs/piper-cpu/bin/python -B "$task_dir/plan_workflow.py" --validate-duration "$task_duration"
set -- "${task_preview_args[@]}"
if [[ $# -eq 0 ]]; then
  task_output="$task_dir/previews/$(date +%Y%m%d_%H%M%S_%N)"
  printf '只读预览；不会发送动作。结果目录：%s\n' "$task_output"
  set -- --live --count 5 --output "$task_output"
fi
task_live=false
task_output=""
task_args=("$@")
for ((task_i=0; task_i<${#task_args[@]}; task_i++)); do
  case "${task_args[$task_i]}" in
    --live) task_live=true ;;
    --output) task_output="${task_args[$((task_i+1))]:-}" ;;
    --output=*) task_output="${task_args[$task_i]#--output=}" ;;
  esac
done
if "$task_live"; then
  if [[ -z "$task_output" ]]; then
    printf '实时预览必须指定新 --output 目录。\n' >&2
    exit 2
  fi
  env -u PYTHONPATH -u LD_LIBRARY_PATH PYTHONNOUSERSITE=1 PYTHONDONTWRITEBYTECODE=1 \
    /home/aup/Lerobot/envs/piper-cpu/bin/python -B "$task_dir/plan_workflow.py" --begin-preview "$task_output"
fi
# Finish even when the latest preview fails: its diagnostic must invalidate an
# earlier successful review, rather than leaving the old plan selected.
task_status=0
bash "$task_dir/run_preview.sh" "$@" || task_status=$?
if "$task_live"; then
  bash "$task_dir/refresh_plan.sh" --duration "$task_duration" || exit $?
fi
exit "$task_status"
