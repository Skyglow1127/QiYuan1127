#!/usr/bin/env bash
set -euo pipefail
D1T_DIR="$(cd "$(dirname "$0")" && pwd)"
exec /usr/bin/python3 "$D1T_DIR/scripts/console.py" "$@"
