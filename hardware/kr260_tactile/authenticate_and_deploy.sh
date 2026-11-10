#!/usr/bin/env bash
set -u
cd -- "$(dirname -- "$0")"
printf '\nKR260 压力采集实机部署\n'
printf '将核对固件，安装采集依赖，加载已核实双路输入设计，再读取真实 GPIO。\n'
printf '下面需要 KR260 上 ubuntu 用户的 sudo 密码；输入时不会显示字符。\n'
printf '密码只交给板卡 sudo，不记录到程序或聊天。\n\n'
ssh -F ./ssh_config -o BatchMode=yes -o StrictHostKeyChecking=yes -o ConnectTimeout=6 -tt pressure-kr260 'cd /home/ubuntu/kr260-pressure-20261025 && sudo -p "[KR260 ubuntu sudo 密码] " python3 -u deploy_live.py'
result=$?
printf '\n部署命令退出码：%s\n' "$result"
printf '请返回聊天查看实测进度。按回车关闭此终端。\n'
read -r _
exit "$result"
