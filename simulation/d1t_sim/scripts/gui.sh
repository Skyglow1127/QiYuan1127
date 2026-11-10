#!/usr/bin/env bash
source "$(dirname "$0")/env.sh"
exec gz sim -g --gui-config "$D1T_ROOT/config/gui.config" -v 3
