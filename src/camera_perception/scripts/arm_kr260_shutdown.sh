#!/usr/bin/env bash
# One-time authorization for this Qwen experiment only. Run in a user terminal.
# The board waits for our completion marker; an expired wait does not power off.
set -euo pipefail
exec ssh -t -o HostName=192.168.28.2 -o BatchMode=yes -o StrictHostKeyChecking=yes kr260 \
  'test ! -e /home/ubuntu/kr260-camera/qwen-20261016-work-complete && sudo systemd-run --unit=camera-qwen-finish-20261016 --property=RuntimeMaxSec=10800 /bin/sh -c '\''while [ ! -f /home/ubuntu/kr260-camera/qwen-20261016-work-complete ]; do sleep 5; done; /usr/sbin/poweroff'\'''
