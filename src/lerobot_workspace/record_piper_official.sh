#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
source "$ROOT/activate.sh"
exec python "$ROOT/projects/piper/official_adapter/record_piper.py" "$@"
