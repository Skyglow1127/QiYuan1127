#!/usr/bin/env bash
source "$(dirname "$0")/env.sh"
exec gz sim -g --gui-config "$PIPER_ROOT/config/gui.config" -v 3
