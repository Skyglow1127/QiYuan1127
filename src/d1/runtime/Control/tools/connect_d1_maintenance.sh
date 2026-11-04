#!/usr/bin/env bash
# Restore the existing controller maintenance SSH session only. No robot commands.
set -euo pipefail
d1_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
d1_runtime="${XDG_RUNTIME_DIR:-/tmp}/d1-maint-${UID}"
d1_socket="$d1_runtime/board.sock"
d1_known="$d1_root/logs/gripper_recovery_20261023_201607/ssh_known_hosts"
umask 077
mkdir -p -- "$d1_runtime"
if [[ ! -O "$d1_runtime" || -L "$d1_runtime" ]]; then
  printf '维护连接目录不属于当前用户或是符号链接，停止：%s\n' "$d1_runtime" >&2
  exit 1
fi
chmod 700 "$d1_runtime"
d1_log_dir="${D1_MAINTENANCE_LOG_DIR:-$d1_root/../../validation/native_probe_20261027/maintenance}"
mkdir -p -- "$d1_log_dir"
d1_log="$d1_log_dir/maintenance_connect_$(TZ=Asia/Singapore date +%Y%m%dT%H%M%S)_$$.log"
log_phase() {
  # Never redirect/record SSH's interactive authentication output or input.
  printf '%s phase=%s exit=%s\n' "$(date -Is)" "$1" "$2" >> "$d1_log"
}
if [[ ! -f "$d1_known" ]]; then
  log_phase known_hosts_missing 1
  printf '缺少已核验的主机密钥文件，未尝试连接：%s\n日志：%s\n' "$d1_known" "$d1_log" >&2
  exit 1
fi
if ssh -S "$d1_socket" -O check ubuntu@192.168.123.100 2>/dev/null; then
  log_phase existing_master_check 0
  printf 'D1 维护连接已存在且通过复用检查；没有发送机械臂指令。\nSocket：%s\n日志：%s\n' "$d1_socket" "$d1_log"
  exit 0
else
  log_phase existing_master_check "$?"
fi
printf '%s\n' '目标：D1 机载系统 ubuntu@192.168.123.100（不是 KR260）。仅尝试一次密码认证；密码只在终端输入。'
if ssh -MNf -S "$d1_socket" -o ControlPersist=1800 -o ConnectTimeout=5 \
  -o PreferredAuthentications=password -o PubkeyAuthentication=no \
  -o KbdInteractiveAuthentication=no -o PasswordAuthentication=yes \
  -o NumberOfPasswordPrompts=1 -o IdentityAgent=none \
  -o UserKnownHostsFile="$d1_known" -o StrictHostKeyChecking=yes ubuntu@192.168.123.100; then
  log_phase create_master 0
else
  d1_rc=$?
  log_phase create_master "$d1_rc"
  printf 'D1 维护连接未建立（SSH 退出码 %s）。未自动重试；若仍为 Permission denied，请核对 D1 机载 ubuntu 账号的密码，勿继续重复尝试或在聊天发送密码。\n日志：%s\n' "$d1_rc" "$d1_log" >&2
  exit "$d1_rc"
fi
# A successful ssh -MNf alone does not prove that its ControlPath is usable.
# For example, a failed control socket bind may leave an unshareable session.
if ssh -S "$d1_socket" -O check ubuntu@192.168.123.100; then
  log_phase new_master_check 0
  printf 'D1 维护连接已恢复且通过复用检查；没有部署、重启或发送机械臂指令。\nSocket：%s\n日志：%s\n' "$d1_socket" "$d1_log"
else
  d1_rc=$?
  log_phase new_master_check "$d1_rc"
  printf 'SSH 登录命令已结束，但指定维护连接无法复用（检查退出码 %s），不能认定恢复成功。\nSocket：%s\n日志：%s\n未自动删除 socket、重试连接或停止进程；请保留上面的终端提示。\n' "$d1_rc" "$d1_socket" "$d1_log" >&2
  exit "$d1_rc"
fi
